cmake_minimum_required(VERSION 3.16)
project(rtctrl VERSION 0.1.0 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(RTCTRL_BUILD_TESTS "Build rtctrl tests" ON)

find_package(MiLib REQUIRED COMPONENTS roki roki-fd dzco liw)
find_package(Threads REQUIRED)

add_subdirectory(third_party/DynamixelSDK/c++ EXCLUDE_FROM_ALL)

include(FetchContent)
FetchContent_Declare(tomlplusplus
  GIT_REPOSITORY https://github.com/marzer/tomlplusplus.git
  GIT_TAG v3.4.0)
FetchContent_MakeAvailable(tomlplusplus)

add_library(rtctrl
  src/version.cpp
  src/milib_cpp_compat.cpp
  src/model/chain_model.cpp
  src/model/joint_map.cpp
  src/model/trajectory.cpp
  src/model/zvs_writer.cpp
  src/model/ik_solver.cpp
  src/arm/sim_arm.cpp)
add_library(rtctrl::rtctrl ALIAS rtctrl)
target_compile_features(rtctrl PUBLIC cxx_std_17)
target_compile_definitions(rtctrl PRIVATE RTCTRL_VERSION="${PROJECT_VERSION}")
target_include_directories(rtctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(rtctrl PUBLIC
  MiLib::roki
  MiLib::roki-fd
  MiLib::dzco
  MiLib::liw
  dynamixel_sdk
  tomlplusplus::tomlplusplus
  Threads::Threads)

# DynamixelSDK ships per-model register data as .model files; dxl_inspect
# reads them from the submodule, not from the SDK's install-time
# CONTROL_TABLE_PATH (which is invalid when the SDK is not installed).
set(RTCTRL_DXL_MODEL_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/third_party/DynamixelSDK/control_table")

add_subdirectory(examples)

if(RTCTRL_BUILD_TESTS)
  enable_testing()
  FetchContent_Declare(Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.1)
  FetchContent_MakeAvailable(Catch2)
  list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
  add_subdirectory(tests)
endif()
