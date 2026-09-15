# Builds the offline VINS adapter in a pinned container.
#
#   1. prebuilt ubuntu:22.04 + Miniforge base image
#      (registry.cn-hangzhou.aliyuncs.com/lacogito/conda-forge:ubuntu2204,
#      conda/mamba live under /opt/conda)
#   2. ROS1 Noetic via RoboStack (conda-forge) -- the only maintained binary
#      distribution of ROS1 for jammy: official ROS1 apt support stops at
#      Ubuntu 20.04 and the third-party jammy apt ports have been taken down.
#   3. VINS-Fusion from source (local checkout at vins-adapter/VINS-Fusion,
#      built with catkin) with the adapter main added to its vins_estimator
#      package.
#   4. The adapter binary plus its non-glibc shared libraries land in /out/
#      for build_vins_adapter.sh to copy out of the image onto the host.
#
# License: VINS-Fusion is GPLv3. It is compiled locally here and never
# redistributed; the adapter binary carries the GPL with it.
#
# Build (see build_vins_adapter.sh):
#   docker build --platform linux/amd64 \
#     --build-arg JOBS=4 \
#     -t vins-adapter:noetic-jammy .

FROM registry.cn-hangzhou.aliyuncs.com/lacogito/conda-forge:ubuntu2204

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    ROS_DISTRO=noetic

# RUN defaults to /bin/sh (dash on ubuntu), which has no `source` builtin;
# the conda activation steps below need bash.
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG JOBS=
# JOBS empty -> resolved to the number of CPUs visible inside the container.

# The base image only ships Miniforge (/opt/conda); add the build toolchain.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates \
        wget \
        git \
        build-essential \
        bzip2 \
        patchelf \
    && rm -rf /var/lib/apt/lists/*

# ROS1 Noetic on Ubuntu 22.04, via RoboStack's conda-forge packages. ceres is
# pinned to 2.1 (2.2 removed ceres::LocalParameterization which VINS uses),
# eigen to 3.4 (ceres 2.1's find_package(Eigen3 3.4.0) rejects the Eigen 5
# version scheme) and cmake to <4 (CMake 4 dropped compatibility with the
# fork's minimum version).
RUN /opt/conda/bin/conda create -y -p /opt/ros1 \
        -c robostack-staging -c conda-forge \
        ros-noetic-ros-base \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-tf \
        catkin_tools \
        "cmake<4" \
        libstdcxx \
        ceres-solver=2.1.0 \
        yaml-cpp \
        eigen=3.4.0 \
    && /opt/conda/bin/conda clean -afy

WORKDIR /ws/src

# Copy the local VINS-Fusion checkout into the image and patch it for gcc-11
# (c++14 keeps the fork's c++11 sources compatible with the ceres 2.1
# headers). .git is excluded via .dockerignore.
COPY VINS-Fusion/ /ws/src/VINS-Fusion/
# Patch for the build toolchain: c++14 (gcc-11 + ceres 2.1) and modern OpenCV
# constants (4.13 dropped the C-style CV_* macros used by the calibration
# tools in camera_models and the file readers in vins_estimator).
RUN sed -i 's/-std=c++11/-std=c++14/g' \
    VINS-Fusion/vins_estimator/CMakeLists.txt \
    VINS-Fusion/camera_models/CMakeLists.txt \
    VINS-Fusion/global_fusion/CMakeLists.txt \
    VINS-Fusion/loop_fusion/CMakeLists.txt \
    && find VINS-Fusion/camera_models/src VINS-Fusion/vins_estimator/src \
        -name '*.cc' -o -name '*.cpp' \
    | xargs sed -i \
        -e 's/\bCV_GRAY2BGR\b/cv::COLOR_GRAY2BGR/g' \
        -e 's/\bCV_GRAY2RGB\b/cv::COLOR_GRAY2RGB/g' \
        -e 's/\bCV_BGR2GRAY\b/cv::COLOR_BGR2GRAY/g' \
        -e 's/\bCV_CALIB_CB_ADAPTIVE_THRESH\b/cv::CALIB_CB_ADAPTIVE_THRESH/g' \
        -e 's/\bCV_CALIB_CB_NORMALIZE_IMAGE\b/cv::CALIB_CB_NORMALIZE_IMAGE/g' \
        -e 's/\bCV_CALIB_CB_FILTER_QUADS\b/cv::CALIB_CB_FILTER_QUADS/g' \
        -e 's/\bCV_CALIB_CB_FAST_CHECK\b/cv::CALIB_CB_FAST_CHECK/g' \
        -e 's/\bCV_CHAIN_APPROX_SIMPLE\b/cv::CHAIN_APPROX_SIMPLE/g' \
        -e 's/\bCV_RETR_CCOMP\b/cv::RETR_CCOMP/g' \
        -e 's/\bCV_ADAPTIVE_THRESH_MEAN_C\b/cv::ADAPTIVE_THRESH_MEAN_C/g' \
        -e 's/\bCV_SHAPE_CROSS\b/cv::MORPH_CROSS/g' \
        -e 's/\bCV_SHAPE_RECT\b/cv::MORPH_RECT/g' \
        -e 's/\bCV_TERMCRIT_ITER\b/cv::TermCriteria::MAX_ITER/g' \
        -e 's/\bCV_TERMCRIT_EPS\b/cv::TermCriteria::EPS/g' \
        -e 's/\bCV_THRESH_BINARY_INV\b/cv::THRESH_BINARY_INV/g' \
        -e 's/\bCV_THRESH_BINARY\b/cv::THRESH_BINARY/g' \
        -e 's/\bCV_AA\b/cv::LINE_AA/g' \
        -e 's/\bCV_LOAD_IMAGE_GRAYSCALE\b/cv::IMREAD_GRAYSCALE/g' \
        -e 's/\bCV_LOAD_IMAGE_COLOR\b/cv::IMREAD_COLOR/g'

# Add the adapter to the fork's vins_estimator package (same package, so the
# unexported vins_lib target is directly linkable) and wire it into the build.
COPY adapter/vins_adapter.cpp VINS-Fusion/vins_estimator/src/adapter_main.cpp
COPY adapter/adapter.cmake VINS-Fusion/vins_estimator/adapter.cmake
RUN echo "include(adapter.cmake)" >> VINS-Fusion/vins_estimator/CMakeLists.txt

# Build only the packages the adapter needs: camera_models + vins (estimator
# core). loop_fusion / global_fusion are not required by the adapter contract.
# The conda-forge ROS/boost/OpenCV libs are built against a newer libstdc++
# (GLIBCXX >= 3.4.31) than the system g++-11 ships, so the linker gets the
# conda libstdc++ via -rpath-link; the bundled /out keeps it for runtime.
RUN source /opt/conda/etc/profile.d/conda.sh && conda activate /opt/ros1 \
    && cd /ws \
    && catkin init \
    && catkin config --cmake-args \
        -DEigen3_DIR=/opt/ros1/share/eigen3/cmake \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_FLAGS=-DNDEBUG \
        -DCMAKE_EXE_LINKER_FLAGS=-Wl,-rpath-link,/opt/ros1/lib,--allow-shlib-undefined \
    && catkin build --no-status -j"${JOBS:-$(nproc)}" vins

# Smoke test (usage error = exit 2 proves the binary links and runs without a
# roscore: the estimator's publishers are never registered, so publish()
# no-ops), then bundle the binary with its non-glibc shared libraries so the
# copied-out adapter runs on any Linux host without a ROS installation.
# Bundle the binary with its complete shared-library closure so the copy runs
# standalone on any Linux host without a ROS/conda install. A single ldd pass
# is NOT enough: conda's blas/cblas are symlink aliases of libopenblas and the
# loader dedupes by SONAME, so those names never show up in ldd of the binary.
# Instead, iterate: ldd the binary and every bundled lib until the /out/lib
# set stops growing (loop passes == max dep depth).
RUN source /opt/conda/etc/profile.d/conda.sh && conda activate /opt/ros1 \
    && /ws/devel/lib/vins/vins_adapter 2>/dev/null; test $? -eq 2 \
    && mkdir -p /out/lib \
    && cp /ws/devel/lib/vins/vins_adapter /out/vins_adapter \
    && export LD_LIBRARY_PATH=/out/lib:/opt/ros1/lib \
    && for pass in 1 2 3 4 5 6; do \
        { ldd /out/vins_adapter; ldd /out/lib/*; } 2>/dev/null \
            | awk '/=> \// && $3 ~ /^\// && $3 !~ /^\/out\// \
                   && $3 !~ /\/(ld-linux|ld-2|ld\.so|libc|libm|libpthread|libdl|librt|libnsl|libnss|libresolv|libutil|libanl)[-._]/ {print $3}' \
            | sort -u \
            | xargs -r cp -L -t /out/lib/; \
       done \
    && unset LD_LIBRARY_PATH \
    && patchelf --force-rpath --set-rpath '$ORIGIN/lib' /out/vins_adapter \
    && for f in /out/lib/*; do patchelf --set-rpath '$ORIGIN' "$f"; done
