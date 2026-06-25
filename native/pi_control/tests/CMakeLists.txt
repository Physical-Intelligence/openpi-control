# Unit and SocketCAN transport tests that do not require Pinocchio or ZMQ.
# The project is Linux-only; syscall wrappers exercise the transport without
# opening physical CAN hardware.
include(FetchContent)
set(_googletest_find_package_args)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  list(APPEND _googletest_find_package_args FIND_PACKAGE_ARGS NAMES GTest)
endif()
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/releases/download/v1.16.0/googletest-1.16.0.tar.gz
  URL_HASH SHA256=78c676fc63881529bf97bf9d45948d905a66833fbfa5318ea2cd7478cb98f399
  ${_googletest_find_package_args}
)
unset(_googletest_find_package_args)
FetchContent_MakeAvailable(googletest)

set(PI_CONTROL_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Boost is only needed for header-only circular_buffer. Prefer the CMake config
# (Homebrew, apt libboost-*-dev) but fall back to a bare include-path probe so a
# headers-only install is enough.
find_package(Boost CONFIG QUIET)
if(NOT Boost_FOUND)
  find_path(OPENPI_BOOST_INCLUDE_DIR boost/circular_buffer.hpp REQUIRED)
endif()

add_executable(pi_control_tests
  test_math_ops.cpp
  test_filter.cpp
  test_stall_detect.cpp
  test_hold_checker.cpp
  test_current_estimation.cpp
  test_device_config.cpp
  test_driver_can.cpp
  test_posix_syscall_hooks.cpp
  test_servo_dm_status.cpp
  test_driver_base.cpp
  ${PI_CONTROL_ROOT}/src/can/math_ops.cpp
  ${PI_CONTROL_ROOT}/src/pi_filter.cpp
  ${PI_CONTROL_ROOT}/src/pi_stall_detect.cpp
  ${PI_CONTROL_ROOT}/src/pi_current_estimation.cpp
  ${PI_CONTROL_ROOT}/src/pi_hold_checker.cpp
  ${PI_CONTROL_ROOT}/src/pi_device_config.cpp
  ${PI_CONTROL_ROOT}/src/pi_driver_can.cpp
  ${PI_CONTROL_ROOT}/src/pi_servo_dm_status.cpp
  ${PI_CONTROL_ROOT}/src/pi_info.cpp
)
target_include_directories(pi_control_tests PRIVATE ${PI_CONTROL_ROOT}/include)
target_compile_features(pi_control_tests PRIVATE cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(pi_control_tests PRIVATE -Wall -Wextra)
endif()
if(TARGET Boost::headers)
  target_link_libraries(pi_control_tests PRIVATE Boost::headers)
else()
  target_include_directories(pi_control_tests PRIVATE ${OPENPI_BOOST_INCLUDE_DIR})
endif()

target_link_libraries(pi_control_tests PRIVATE GTest::gtest_main)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(pi_control_tests PRIVATE
    -Wl,--wrap=socket
    -Wl,--wrap=ioctl
    -Wl,--wrap=bind
    -Wl,--wrap=setsockopt
    -Wl,--wrap=close
    -Wl,--wrap=select
    -Wl,--wrap=write
    -Wl,--wrap=read)
endif()

include(GoogleTest)
gtest_discover_tests(pi_control_tests DISCOVERY_TIMEOUT 60)
