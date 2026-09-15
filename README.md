# VINS-Fusion Adapter

A self-contained CLI that runs [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)'s estimator offline — feed it frame directories plus an optional IMU csv, get a TUM trajectory back, with no ROS runtime, roscore, or bag playback.

```
vins_adapter <config.yaml> <output.tum>
```

---

## Why this exists

VINS-Fusion is an excellent visual-inertial odometry system, but it is written
to run **online, inside ROS**: you start a `roscore`, launch
`vins_node`, play a bag or republish images/IMU on topics, and read the result
from a `vio.csv` that the node writes to a hard-coded output directory. That
model fits a robot; it does not fit a batch data pipeline.

We need VIO trajectories as one stage of an offline pipeline (episodes already
materialized on disk as frame directories + csv, many of them, on headless
Linux workers). Requiring a ROS master per episode means extra processes,
non-deterministic timing, and a per-recording cleanup step.

This repo adds a thin offline front-end to the upstream estimator:

- **No ROS runtime.** No `roscore`, no topics, no bags. The estimator's ROS
  publishers are compiled in but never registered, so `publish()` no-ops.
- **Files in, file out.** Frame directories + CSVs on disk, one TUM file per
  run.
- **Deterministic and stateless.** One process per episode with a freshly
  constructed `Estimator`, single-threaded (`MULTIPLE_THREAD=0`), so no
  sliding-window or marginalisation state leaks between recordings.
- **Self-contained artifact.** The built binary ships with its bundled shared
  libraries and runs on a bare Linux host that has only glibc — no ROS, no
  conda, no build toolchain.

The adapter does not re-implement or fork the estimator. It links the upstream
`Estimator` (built as `vins_lib` in the vendored checkout) and drives it
directly: decode a frame, push any IMU samples up to that timestamp, call
`inputImage()`, read the optimized pose off the sliding window.

### The two conversion traps at the boundary

VINS-Fusion's own `vio.csv` is easy to misread, so the adapter does both
conversions exactly once, at the boundary:

```
vio.csv:  ts_ns, px,py,pz, qw,qx,qy,qz, vx,vy,vz     (nanoseconds, qw FIRST)
TUM:      ts_s,  tx,ty,tz, qx,qy,qz,qw               (seconds,     q  LAST)
```

---

## Build

Requires only Docker on the build host. Everything else (ROS1 Noetic via
RoboStack, Ceres, OpenCV, the VINS-Fusion build) happens inside the image.

```bash
./build_vins_adapter.sh                     # -> vins_adapter/, vins_adapter-linux-<arch>.tar.gz
./build_vins_adapter.sh --remote user@host  # build on a native docker daemon over ssh
./build_vins_adapter.sh --force             # ignore the docker layer cache
./build_vins_adapter.sh --check             # verify an already-copied adapter
./build_vins_adapter.sh --help              # all options
```

The build target follows the cpu arch of the machine that compiles, so the
ELF runs natively there instead of under qemu emulation:

- x86_64 host → `linux/amd64`
- arm64/aarch64 host → `linux/arm64` (Apple Silicon included — a Rosetta shell
  that reports `x86_64` is still detected as `arm64`)
- `--remote <host>` → the remote docker daemon's arch (queried via
  `docker version`)

Pass `--platform linux/<arch>` to cross-build explicitly (e.g. amd64 on an
arm64 mac). On Apple Silicon, `--remote <linux-host>` is strongly recommended
over cross-building amd64 locally: the emulation path is 10-30x slower and
memory hungry.

The build always runs the regression suite at the end; `--skip-tests` opts out.

### Artifacts

| path | what |
| --- | --- |
| `vins_adapter/vins_adapter` | the Linux ELF binary (`x86_64` or `aarch64`) |
| `vins_adapter/lib/` | bundled shared-library closure (resolved via `$ORIGIN/lib`) |
| `vins_adapter/run_vins_adapter.sh` | launcher — use this |
| `vins_adapter/BUILDINFO.json` | provenance: source commit, image id, platform, arch, build time |
| `vins_adapter-linux-x86_64.tar.gz` (+ `.md5`) | amd64 bundle for shipping to workers |
| `vins_adapter-linux-aarch64.tar.gz` (+ `.md5`) | arm64 bundle for shipping to workers |

The arch is recorded in the tarball file name, its `.md5` file name/content,
and in `BUILDINFO.json` inside, so x86_64 and aarch64 bundles can coexist and
be told apart without unpacking. Verify a transferred bundle with
`md5sum -c vins_adapter-linux-x86_64.tar.gz.md5` (or the aarch64 name).

---

## Run

```bash
./vins_adapter/run_vins_adapter.sh config.yaml trajectory.tum
```

Unpacked anywhere on a Linux host of the same arch it was built for
(x86_64 for the amd64 bundle, aarch64 for the arm64 bundle); it needs nothing
but glibc. It is a Linux ELF — do not run it on macOS directly (run it inside
the built image instead).

### Config yaml

A **flat** yaml (not the upstream VINS config):

```yaml
imu: 1                      # 1 = use imu_csv
num_of_cam: 2               # 2 = stereo, requires right_dir
estimate_extrinsic: 0       # 0 trust, 1 refine around guess, 2 refine online
max_features: 200           # per-frame feature cap (floor 50)

# pinhole intrinsics + radial/tangential distortion
fx: 458.65
fy: 457.30
cx: 367.21
cy: 248.38
k1: -0.2834
k2: 0.0739
p1: 0.0002
p2: -0.0001

left_dir:  /data/ep001/cam0       # frames: .png/.jpg/.jpeg/.bmp/.tif/.tiff
right_dir: /data/ep001/cam1       # optional, stereo only
imu_csv:   /data/ep001/imu.csv    # optional
frame_times_csv: /data/ep001/times.csv   # optional

# 3x4 [R|t] row-major, camera pose in the body frame (upstream RIC/TIC convention)
T_cam0_body: [[1,0,0, 0.0],
              [0,1,0, 0.0],
              [0,0,1, 0.0]]
T_cam1_body: [[1,0,0,-0.12],
              [0,1,0, 0.0],
              [0,0,1, 0.0]]      # optional, stereo only
```

| key | required | notes |
| --- | --- | --- |
| `left_dir` | yes | sorted by filename; extension-filtered |
| `right_dir` | stereo | paired with left by sort order; may not be shorter than left |
| `imu` / `imu_csv` | no | both needed for visual-inertial; header row, then `timestamp,gx,gy,gz,ax,ay,az` (seconds, gyro rad/s **first**, accel m/s²) |
| `frame_times_csv` | no | header row, then `index,timestamp_ns` (per-frame clock) |
| `num_of_cam` | no | `2` + `right_dir` = stereo (default 1) |
| `estimate_extrinsic` | no | default 2; **forced to 0** on vision-only runs, since online refinement needs IMU preintegration |
| `max_features` | no | default 200, min 50 |
| `fx fy cx cy k1 k2 p1 p2` | no | defaults 500/500/320/240 and zero distortion |
| `T_cam0_body` | yes | 3 rows of 4 numbers |
| `T_cam1_body` | stereo | same shape |

### Output

One TUM line per optimized frame:

```
timestamp tx ty tz qx qy qz qw
```

Seconds, body pose in world. Timestamps come from `frame_times_csv` when
given; otherwise a relative clock of `frame_index / 30 fps`.

As in upstream VINS-Fusion, poses are emitted only once the estimator reaches
`NON_LINEAR` (sliding window initialized) — **short or near-static episodes
legitimately produce fewer rows than input frames.** That is expected, not a
failure.

### Exit codes

| code | meaning |
| --- | --- |
| `0` | ok |
| `1` | runtime failure (message on stderr: bad config, unreadable frames, stereo mismatch, …) |
| `2` | usage error (wrong argv) |

---

## Tests

```bash
python3 tests/run_tests.py                              # against vins_adapter/run_vins_adapter.sh
python3 tests/run_tests.py --adapter /path/to/binary
python3 tests/run_tests.py --sources-only               # no build needed
```

Two layers: **source guards** (the fixes that keep the adapter running
standalone must not be reverted — camodocal `%YAML:1.0` header, no
`TransformBroadcaster` in `pubTF`, `-DNDEBUG`, zero-initialized estimator state,
`ESTIMATE_EXTRINSIC` forced off without IMU) and **functional runs** (argv
contract, config error paths, undecodable frames, temp-dir cleanup, TUM output
validity, clean exit on untrackable input). Standard library only.

---

## How the build works

`ubuntu:22.04` + Miniforge → ROS1 Noetic from [RoboStack](https://robostack.github.io/)
(official ROS1 stops at 20.04 and the third-party jammy apt ports are gone) →
VINS-Fusion from the **local checkout** in `VINS-Fusion/` (no network clone;
update that checkout to build a different snapshot) → the adapter added to the
`vins_estimator` package so the unexported `vins_lib` target is directly
linkable.

Pinned because the fork is c++11 and the toolchain is not: Ceres 2.1 (2.2
removed `ceres::LocalParameterization`), Eigen 3.4 (Ceres 2.1's
`find_package(Eigen3 3.4.0)` rejects the Eigen 5 version scheme), CMake <4.
Source fixes for gcc-11 / OpenCV 4.13 (c++14, `CV_*` → `cv::*`) are applied
with `sed` **inside the image**, so the vendored checkout stays pristine.

Bundling is iterative, not a single `ldd` pass: conda's `blas`/`cblas` are
symlink aliases of `libopenblas` and the loader dedupes by SONAME, so those
names never appear in `ldd` of the binary itself. The build `ldd`s the binary
and every already-bundled lib until the set stops growing.

## Repo layout

```
adapter/vins_adapter.cpp    the offline front-end (contract doc is in its header comment)
adapter/adapter.cmake       appended to vins_estimator/CMakeLists.txt by the Dockerfile
build_vins_adapter.sh       build + extract + verify + bundle + test
Dockerfile                  pinned ROS1/VINS-Fusion toolchain
tests/run_tests.py          regression suite
VINS-Fusion/                vendored upstream checkout (GPLv3)
```

---

## Acknowledgments

This repo is a thin wrapper around **[VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion)**
by Tong Qin, Shaozu Cao, Jie Pan, Peiliang Li and Shaojie Shen of the
[Aerial Robotics Group](http://uav.ust.hk/), HKUST. All of the state
estimation — feature tracking, IMU preintegration, sliding-window
optimization — is their work; this project only adds an offline entry point
and an artifact packaging step. Please credit and cite upstream, not this
adapter.

VINS-Fusion is an extension of [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
and is the top open-sourced stereo algorithm on the
[KITTI Odometry Benchmark](http://www.cvlibs.net/datasets/kitti/eval_odometry.php).

If you use it in academic work, please cite:

```bibtex
@misc{qin2019a,
  Author = {Tong Qin and Jie Pan and Shaozu Cao and Shaojie Shen},
  Title = {A General Optimization-based Framework for Local Odometry Estimation with Multiple Sensors},
  Year = {2019},
  Eprint = {arXiv:1901.03638}
}

@article{qin2017vins,
  title={VINS-Mono: A Robust and Versatile Monocular Visual-Inertial State Estimator},
  author={Qin, Tong and Li, Peiliang and Shen, Shaojie},
  journal={IEEE Transactions on Robotics},
  year={2018},
  volume={34},
  number={4},
  pages={1004-1020}
}
```

Upstream's own acknowledgements apply here too: it uses
[Ceres Solver](http://ceres-solver.org/) for non-linear optimization,
[DBoW2](https://github.com/dorian3d/DBoW2) for loop detection, the generic
[camera model](https://github.com/hengli/camodocal) from camodocal, and
[GeographicLib](https://geographiclib.sourceforge.io/).

Thanks as well to [RoboStack](https://robostack.github.io/), which is the only
reason ROS1 Noetic is installable on a maintained Ubuntu LTS.

### License

VINS-Fusion is **GPLv3**, and the adapter links against it, so this project is
distributed under **GPLv3** as well (see `LICENSE`, and
`VINS-Fusion/LICENSE` for upstream). The binary is built locally and never
redistributed; it carries the GPL with it.
