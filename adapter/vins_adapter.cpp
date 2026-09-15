// Offline VINS-Fusion adapter.
//
// License note: this file links against VINS-Fusion (GPLv3, HKUST Aerial
// Robotics Group) and is therefore distributed under GPLv3 as well. It is
// built locally inside the vins-adapter Docker image; see the LICENSE file.
//
// Contract:
//
//     vins_adapter <config.yaml> <output.tum>
//
// The flat config yaml contains:
//
//     imu: 0|1                     # imu_csv present
//     num_of_cam: 1|2              # right_dir present
//     estimate_extrinsic: 0|1|2    # 2 = refine camera/IMU extrinsics online
//     max_features: int
//     fx, fy, cx, cy, k1, k2, p1, p2: float
//     left_dir: <dir of left frames>
//     right_dir: <dir of right frames>          (optional)
//     imu_csv: <timestamp,gx,gy,gz,ax,ay,az csv>  (optional; gyro first)
//     frame_times_csv: <index,timestamp_ns csv>   (optional; per-frame clock)
//     T_cam0_body: [[R00 R01 R02 t0], [R10 R11 R12 t1], [R20 R21 R22 t2]]
//     T_cam1_body: ...                          (optional, stereo)
//
// On success writes one TUM line per optimized frame to <output.tum>:
//
//     timestamp tx ty tz qx qy qz qw        (seconds, body pose in world)
//
// TUM timestamps are the frame timestamps when frame_times_csv is given,
// otherwise a relative clock (frame_index / 30 fps). As in
// upstream VINS-Fusion, poses are only emitted once the estimator reaches the
// NON_LINEAR state (sliding window initialized) -- short or near-static
// episodes legitimately produce fewer poses than frames.
//
// Exit codes: 0 ok, 2 usage error, 1 runtime failure (message on stderr).

#include <ros/ros.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>

#include "estimator/estimator.h"  // Estimator + the parameters.h globals

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr double kSyntheticFps = 30.0;  // relative clock when no frame_times_csv

bool is_image_file(const fs::path &p) {
  std::string ext = p.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp" ||
         ext == ".tif" || ext == ".tiff";
}

std::vector<fs::path> list_images(const std::string &dir, std::string *err) {
  std::vector<fs::path> images;
  std::error_code ec;
  fs::directory_iterator it(dir, ec), end;
  if (ec) {
    *err = "cannot read image directory " + dir + ": " + ec.message();
    return images;
  }
  for (; it != end; it.increment(ec)) {
    if (ec) break;
    if (it->is_regular_file() && is_image_file(it->path()))
      images.push_back(it->path());
  }
  std::sort(images.begin(), images.end());
  if (images.empty())
    *err = "no images under " + dir;
  return images;
}

// Left/right frames are paired by sort order (the caller materializes them in
// lockstep), the same convention as the KITTI sequences VINS consumes.
bool image_at(const std::vector<fs::path> &files, size_t i, cv::Mat *out) {
  *out = cv::imread(files[i].string(), cv::IMREAD_GRAYSCALE);
  return !out->empty();
}

void write_camodocal_pinhole(const fs::path &path, const std::string &name,
                             int width, int height, double fx, double fy,
                             double cx, double cy, double k1, double k2,
                             double p1, double p2) {
  std::ofstream f(path);
  // The %YAML:1.0 header is required: OpenCV >= 4.13 no longer auto-detects
  // bare YAML in FileStorage::FORMAT_AUTO and throws "Input file is invalid".
  f << "%YAML:1.0\n---\n"
    << "model_type: PINHOLE\n"
    << "camera_name: " << name << "\n"
    << "image_width: " << width << "\n"
    << "image_height: " << height << "\n"
    << "distortion_parameters:\n"
    << "   k1: " << k1 << "\n"
    << "   k2: " << k2 << "\n"
    << "   p1: " << p1 << "\n"
    << "   p2: " << p2 << "\n"
    << "projection_parameters:\n"
    << "   fx: " << fx << "\n"
    << "   fy: " << fy << "\n"
    << "   cx: " << cx << "\n"
    << "   cy: " << cy << "\n";
}

void read_cam_transform(const YAML::Node &node, Eigen::Matrix3d *R,
                        Eigen::Vector3d *t, bool *ok) {
  *R = Eigen::Matrix3d::Identity();
  *t = Eigen::Vector3d::Zero();
  if (!node || !node.IsSequence() || node.size() != 3) {
    *ok = false;
    return;
  }
  for (int i = 0; i < 3; ++i) {
    const YAML::Node &row = node[i];
    if (!row.IsSequence() || row.size() != 4) {
      *ok = false;
      return;
    }
    for (int j = 0; j < 3; ++j) (*R)(i, j) = row[j].as<double>();
    (*t)(i) = row[3].as<double>();
  }
}

struct ImuSample {
  double t;
  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
};

// IMU csv convention: header line, then
// timestamp,gx,gy,gz,ax,ay,az -- seconds, gyro rad/s, accel m/s^2.
std::vector<ImuSample> read_imu_csv(const std::string &path) {
  std::vector<ImuSample> samples;
  std::ifstream f(path);
  std::string line;
  bool first = true;
  while (std::getline(f, line)) {
    if (first) {  // skip the header row
      first = false;
      continue;
    }
    if (line.empty()) continue;
    std::stringstream ss(line);
    double gx, gy, gz, ax, ay, az, ts;
    char comma;
    if (ss >> ts >> comma >> gx >> comma >> gy >> comma >> gz >> comma >> ax >>
        comma >> ay >> comma >> az)
      samples.push_back({ts, Eigen::Vector3d(ax, ay, az),
                         Eigen::Vector3d(gx, gy, gz)});
  }
  return samples;
}

// frame_times_csv: header line, then index,timestamp_ns.
std::vector<double> read_frame_times_csv(const std::string &path,
                                         size_t n_frames) {
  std::vector<double> times;
  std::ifstream f(path);
  std::string line;
  bool first = true;
  while (std::getline(f, line)) {
    if (first) {
      first = false;
      continue;
    }
    if (line.empty()) continue;
    std::stringstream ss(line);
    long long index, ts_ns;
    char comma;
    if (ss >> index >> comma >> ts_ns) {
      if (index >= 0 && static_cast<size_t>(index) >= times.size())
        times.resize(static_cast<size_t>(index) + 1, -1.0);
      times[static_cast<size_t>(index)] = ts_ns / 1e9;
    }
  }
  if (times.size() < n_frames) times.resize(n_frames, -1.0);
  return times;
}

}  // namespace

int main(int argc, char **argv) {
  // ROS plumbing is compiled into the estimator's visualization layer; the
  // publishers are never registered, so publish() calls no-op and no master
  // is required. ros::init only seeds the node name for logging.
  ros::init(argc, argv, "vins_adapter",
            ros::init_options::AnonymousName | ros::init_options::NoRosout);

  if (argc != 3) {
    std::cerr << "usage: vins_adapter <config.yaml> <output.tum>\n";
    return 2;
  }
  const std::string config_path = argv[1];
  const std::string output_path = argv[2];

  YAML::Node cfg;
  try {
    cfg = YAML::LoadFile(config_path);
  } catch (const std::exception &e) {
    std::cerr << "vins_adapter: cannot read config " << config_path << ": "
              << e.what() << "\n";
    return 1;
  }

  const bool use_imu = cfg["imu"] && cfg["imu"].as<int>(0) == 1 &&
                       cfg["imu_csv"] && !cfg["imu_csv"].as<std::string>().empty();
  const bool stereo = cfg["num_of_cam"] && cfg["num_of_cam"].as<int>(1) == 2 &&
                      cfg["right_dir"];
  const int num_of_cam = stereo ? 2 : 1;
  const int estimate_extrinsic = cfg["estimate_extrinsic"]
                                     ? cfg["estimate_extrinsic"].as<int>(2)
                                     : 2;
  const double fx = cfg["fx"] ? cfg["fx"].as<double>() : 500.0;
  const double fy = cfg["fy"] ? cfg["fy"].as<double>() : 500.0;
  const double cx = cfg["cx"] ? cfg["cx"].as<double>() : 320.0;
  const double cy = cfg["cy"] ? cfg["cy"].as<double>() : 240.0;
  const double k1 = cfg["k1"] ? cfg["k1"].as<double>() : 0.0;
  const double k2 = cfg["k2"] ? cfg["k2"].as<double>() : 0.0;
  const double p1 = cfg["p1"] ? cfg["p1"].as<double>() : 0.0;
  const double p2 = cfg["p2"] ? cfg["p2"].as<double>() : 0.0;
  const std::string left_dir =
      cfg["left_dir"] ? cfg["left_dir"].as<std::string>() : "";

  std::string err;
  std::vector<fs::path> left_images = list_images(left_dir, &err);
  if (left_images.empty()) {
    std::cerr << "vins_adapter: " << err << "\n";
    return 1;
  }
  std::vector<fs::path> right_images;
  if (stereo) {
    right_images = list_images(cfg["right_dir"].as<std::string>(), &err);
    if (right_images.empty()) {
      std::cerr << "vins_adapter: " << err << "\n";
      return 1;
    }
    if (right_images.size() < left_images.size()) {
      std::cerr << "vins_adapter: stereo mismatch: " << left_images.size()
                << " left frames but " << right_images.size()
                << " right frames\n";
      return 1;
    }
  }
  const size_t n_frames = left_images.size();

  std::vector<ImuSample> imu;
  if (use_imu) imu = read_imu_csv(cfg["imu_csv"].as<std::string>());

  std::vector<double> times(n_frames, -1.0);
  if (cfg["frame_times_csv"]) {
    times = read_frame_times_csv(cfg["frame_times_csv"].as<std::string>(),
                                 n_frames);
  }
  for (size_t i = 0; i < n_frames; ++i)
    if (times[i] < 0.0) times[i] = static_cast<double>(i) / kSyntheticFps;

  // First frame fixes the image size the camodocal calib files are written
  // with (the feature tracker also re-derives row/col from each image).
  cv::Mat probe;
  if (!image_at(left_images, 0, &probe)) {
    std::cerr << "vins_adapter: cannot decode first left frame "
              << left_images[0].string() << "\n";
    return 1;
  }
  const int width = probe.cols;
  const int height = probe.rows;

  fs::path tmp_dir = fs::temp_directory_path() /
                     ("vins_adapter_" + std::to_string(static_cast<long>(getpid())));
  std::error_code ec;
  fs::create_directories(tmp_dir, ec);
  if (ec) {
    std::cerr << "vins_adapter: cannot create temp dir " << tmp_dir.string()
              << ": " << ec.message() << "\n";
    return 1;
  }

  fs::path cam0_yaml = tmp_dir / "cam0.yaml";
  fs::path cam1_yaml = tmp_dir / "cam1.yaml";
  write_camodocal_pinhole(cam0_yaml, "cam0", width, height, fx, fy, cx, cy,
                          k1, k2, p1, p2);
  if (stereo)
    write_camodocal_pinhole(cam1_yaml, "cam1", width, height, fx, fy, cx, cy,
                            k1, k2, p1, p2);

  // Populate the estimator's global parameters directly (no ROS parameter
  // server). RIC/TIC follow the VINS convention: the camera pose in the body
  // frame, i.e. the config's T_cam{i}_body read as-is.
  MAX_CNT = cfg["max_features"] ? cfg["max_features"].as<int>() : 200;
  MAX_CNT = std::max(50, MAX_CNT);
  MIN_DIST = 20;
  F_THRESHOLD = 1.0;
  FLOW_BACK = 1;
  SHOW_TRACK = 0;
  MULTIPLE_THREAD = 0;  // synchronous inputImage(): deterministic offline runs
  USE_IMU = use_imu ? 1 : 0;
  STEREO = stereo ? 1 : 0;
  NUM_OF_CAM = num_of_cam;
  // Online camera/IMU extrinsic refinement needs IMU preintegration: without
  // IMU, pre_integrations[] stay null and processImage() would dereference
  // pre_integrations[frame_count]->delta_q (estimator.cpp, ESTIMATE_EXTRINSIC
  // == 2 branch). Force it off for vision-only runs.
  ESTIMATE_EXTRINSIC = use_imu ? estimate_extrinsic : 0;
  ACC_N = 0.1;
  ACC_W = 0.001;
  GYR_N = 0.01;
  GYR_W = 0.0001;
  G = Eigen::Vector3d(0.0, 0.0, 9.8);
  TD = 0.0;
  ESTIMATE_TD = 0;
  INIT_DEPTH = 5.0;
  MIN_PARALLAX = 10.0 / FOCAL_LENGTH;
  SOLVER_TIME = 0.04;
  NUM_ITERATIONS = 8;
  ROLLING_SHUTTER = 0;
  ROW = height - 1;
  COL = width - 1;
  VINS_RESULT_PATH = "";
  OUTPUT_FOLDER = "";
  FISHEYE_MASK = "";

  bool ok = true;
  Eigen::Matrix3d R0;
  Eigen::Vector3d t0;
  read_cam_transform(cfg["T_cam0_body"], &R0, &t0, &ok);
  if (!ok) {
    std::cerr << "vins_adapter: bad or missing T_cam0_body in config\n";
    return 1;
  }
  RIC.assign(num_of_cam, Eigen::Matrix3d::Identity());
  TIC.assign(num_of_cam, Eigen::Vector3d::Zero());
  RIC[0] = R0;
  TIC[0] = t0;
  if (stereo) {
    Eigen::Matrix3d R1;
    Eigen::Vector3d t1;
    read_cam_transform(cfg["T_cam1_body"], &R1, &t1, &ok);
    if (!ok) {
      std::cerr << "vins_adapter: bad or missing T_cam1_body in config\n";
      return 1;
    }
    RIC[1] = R1;
    TIC[1] = t1;
  }
  CAM_NAMES.assign(1, cam0_yaml.string());
  if (stereo) CAM_NAMES.push_back(cam1_yaml.string());

  FILE *out = std::fopen(output_path.c_str(), "w");
  if (out == nullptr) {
    std::cerr << "vins_adapter: cannot open output " << output_path << ": "
              << std::strerror(errno) << "\n";
    return 1;
  }

  Estimator estimator;
  estimator.setParameter();

  const char *run_error = nullptr;
  size_t imu_idx = 0;
  for (size_t i = 0; i < n_frames; ++i) {
    const double t = times[i];

    cv::Mat left;
    if (!image_at(left_images, i, &left)) {
      run_error = "cannot decode left frame";
      break;
    }
    if (stereo) {
      cv::Mat right;
      if (!image_at(right_images, i, &right)) {
        run_error = "cannot decode right frame";
        break;
      }
      if (USE_IMU) {
        while (imu_idx < imu.size() && imu[imu_idx].t <= t + 1e-9) {
          estimator.inputIMU(imu[imu_idx].t, imu[imu_idx].acc,
                             imu[imu_idx].gyro);
          ++imu_idx;
        }
      }
      estimator.inputImage(t, left, right);
    } else {
      if (USE_IMU) {
        while (imu_idx < imu.size() && imu[imu_idx].t <= t + 1e-9) {
          estimator.inputIMU(imu[imu_idx].t, imu[imu_idx].acc,
                             imu[imu_idx].gyro);
          ++imu_idx;
        }
      }
      estimator.inputImage(t, left);
    }

    // inputImage() ran processMeasurements() synchronously (MULTIPLE_THREAD=0),
    // so the newest window frame carries this image's pose.
    estimator.mProcess.lock();
    if (estimator.solver_flag == Estimator::NON_LINEAR &&
        estimator.frame_count == WINDOW_SIZE) {
      const Eigen::Quaterniond q(estimator.Rs[estimator.frame_count]);
      const Eigen::Vector3d &p = estimator.Ps[estimator.frame_count];
      std::fprintf(out, "%.9f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n", t, p.x(),
                   p.y(), p.z(), q.x(), q.y(), q.z(), q.w());
    }
    estimator.mProcess.unlock();
  }

  std::fclose(out);
  fs::remove_all(tmp_dir, ec);

  if (run_error != nullptr) {
    std::cerr << "vins_adapter: " << run_error << "\n";
    return 1;
  }
  return 0;
}
