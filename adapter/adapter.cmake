# Appended to the fork's vins_estimator/CMakeLists.txt by the Dockerfile.
# Runs in that package's scope: catkin_* / OpenCV / Ceres variables and the
# vins_lib target are already defined there.

find_package(yaml-cpp REQUIRED)

# conda-forge's yaml-cpp 0.8 exports the namespaced target yaml-cpp::yaml-cpp,
# while the legacy YAML_CPP_LIBRARIES variable holds the bare name "yaml-cpp";
# passing that to the linker fails because the conda lib lives in
# /opt/ros1/lib, not a default ld search path. Prefer the real target, fall
# back to locating the library file explicitly.
if(TARGET yaml-cpp::yaml-cpp)
  set(VINS_YAML_CPP yaml-cpp::yaml-cpp)
elseif(TARGET yaml-cpp)
  set(VINS_YAML_CPP yaml-cpp)
else()
  find_library(VINS_YAML_CPP NAMES yaml-cpp
    HINTS ${CMAKE_PREFIX_PATH} /opt/ros1
    PATH_SUFFIXES lib lib64
    REQUIRED)
endif()

add_executable(vins_adapter src/adapter_main.cpp)
set_property(TARGET vins_adapter PROPERTY CXX_STANDARD 17)
set_property(TARGET vins_adapter PROPERTY CXX_STANDARD_REQUIRED ON)
target_link_libraries(vins_adapter
  vins_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CERES_LIBRARIES}
  ${VINS_YAML_CPP}
)
