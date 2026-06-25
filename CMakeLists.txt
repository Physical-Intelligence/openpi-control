cmake_minimum_required(VERSION 3.22)
project(openpi_control LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "openpi-control requires Linux and SocketCAN")
endif()

option(OPENPI_CONTROL_BUILD_NATIVE "Build the physical-arm native runtime" ON)
option(OPENPI_CONTROL_BUILD_TESTING "Build the native unit tests" OFF)

# A source build keeps native dependencies private to this project.
# `build_deps.sh` installs the pinned dependencies here, and this CMake
# configuration exposes them without requiring shell activation.
set(_openpi_control_default_deps_dir "${CMAKE_CURRENT_SOURCE_DIR}/.deps")
if(DEFINED ENV{OPENPI_CONTROL_DEPS_DIR})
  set(_openpi_control_default_deps_dir "$ENV{OPENPI_CONTROL_DEPS_DIR}")
endif()
set(OPENPI_CONTROL_DEPS_DIR "${_openpi_control_default_deps_dir}" CACHE PATH
    "Directory containing openpi-control build dependencies")
set(OPENPI_CONTROL_PINOCCHIO_PREFIX "${OPENPI_CONTROL_DEPS_DIR}/pinocchio/install" CACHE PATH
    "Pinocchio installation prefix used by openpi-control")
set(OPENPI_CONTROL_CPPZMQ_PREFIX "${OPENPI_CONTROL_DEPS_DIR}/cppzmq/install" CACHE PATH
    "cppzmq installation prefix used by openpi-control")

if(EXISTS "${OPENPI_CONTROL_PINOCCHIO_PREFIX}/lib/pkgconfig/pinocchio.pc")
  list(PREPEND CMAKE_PREFIX_PATH "${OPENPI_CONTROL_PINOCCHIO_PREFIX}")
  if(DEFINED ENV{PKG_CONFIG_PATH} AND NOT "$ENV{PKG_CONFIG_PATH}" STREQUAL "")
    set(ENV{PKG_CONFIG_PATH}
        "${OPENPI_CONTROL_PINOCCHIO_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
  else()
    set(ENV{PKG_CONFIG_PATH} "${OPENPI_CONTROL_PINOCCHIO_PREFIX}/lib/pkgconfig")
  endif()
endif()

if(EXISTS "${OPENPI_CONTROL_CPPZMQ_PREFIX}/include/zmq.hpp")
  list(PREPEND CMAKE_PREFIX_PATH "${OPENPI_CONTROL_CPPZMQ_PREFIX}")
  set(OPENPI_CONTROL_CPPZMQ_INCLUDE_DIR "${OPENPI_CONTROL_CPPZMQ_PREFIX}/include")
endif()

# Sanitizer plumbing for the test targets. Valid values: "" (off), "address,undefined",
# "thread". Applied globally so the exercised pi_control sources are instrumented too.
set(OPENPI_CONTROL_SANITIZER "" CACHE STRING "Comma-separated -fsanitize= list for test builds")
if(OPENPI_CONTROL_SANITIZER)
  add_compile_options(-fsanitize=${OPENPI_CONTROL_SANITIZER} -fno-sanitize-recover=all -g -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${OPENPI_CONTROL_SANITIZER})
endif()

if(OPENPI_CONTROL_BUILD_TESTING)
  enable_testing()
  add_subdirectory(native/pi_control/tests)
endif()

if(OPENPI_CONTROL_BUILD_NATIVE)
  set(NO_ROS ON CACHE BOOL "Build without ROS" FORCE)
  add_subdirectory(native/pi_control)
endif()
