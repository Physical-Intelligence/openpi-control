cmake_minimum_required(VERSION 3.20)
project(OpenPI_control_native)
set(OPENPI_CONTROL_TARGET pi_control_node)

# Boost: avoid deprecated FindBoost (CMake >= 3.30 warns; CMP0167 NEW uses BoostConfig.cmake).
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# C++17 is required by the native control implementation.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for clangd and other tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(ENABLE_ALGO_PINO)
add_compile_options(-Wall -Wextra -Wpedantic)
# -Wno-psabi is GCC-only (silences the ARM struct-passing ABI warning).
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-psabi)
endif()

# Find bundled shared libraries next to the packaged Linux executable.
set(CMAKE_INSTALL_RPATH "$ORIGIN/../libs")

# Boost.System is header-only on supported Linux distributions.
find_package(Boost CONFIG REQUIRED COMPONENTS program_options)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
if(NOT TARGET Eigen3::Eigen3 AND TARGET Eigen3::Eigen)
  add_library(Eigen3::Eigen3 ALIAS Eigen3::Eigen)
endif()
find_package(fmt REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(pinocchio REQUIRED)
find_package(Franka 0.21.3 EXACT CONFIG REQUIRED)
pkg_check_modules(PC_ZMQ REQUIRED libzmq)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)
pkg_check_modules(TINYXML2 REQUIRED IMPORTED_TARGET tinyxml2)
add_library(TinyXML2::TinyXML2 ALIAS PkgConfig::TINYXML2)

find_library(OPENPI_CONTROL_MODBUS_STATIC_LIBRARY NAMES libmodbus.a
  PATHS "${OPENPI_CONTROL_LIBMODBUS_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)
add_library(openpi_control_modbus STATIC IMPORTED)
set_target_properties(openpi_control_modbus PROPERTIES
  IMPORTED_LOCATION "${OPENPI_CONTROL_MODBUS_STATIC_LIBRARY}"
  INTERFACE_INCLUDE_DIRECTORIES "${OPENPI_CONTROL_LIBMODBUS_PREFIX}/include/modbus")

# Trossen iNerve controller SDK (DriverTrossen). Provisioned by
# scripts/build_deps.sh from the pinned TrossenRobotics/trossen_arm release
# (prebuilt static lib + headers); provides the imported STATIC target
# `libtrossen_arm`.
find_package(libtrossen_arm REQUIRED)

# Let CMake's runtime dependency scanner resolve local build dependencies
# without hard-coding those directories into the installed executable.
set(CMAKE_BUILD_RPATH ${PINOCCHIO_LIBRARY_DIRS} ${PC_ZMQ_LIBRARY_DIRS})

# Include directories. Do not add /usr/local here: a host-wide Pinocchio (e.g. a
# different version installed by another project) would shadow the pinned .deps
# build and break the link with an ABI mismatch. pkg-config supplies every
# required -I/-L for the pinned dependencies.
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${PC_ZMQ_INCLUDE_DIRS}
  ${OPENPI_CONTROL_CPPZMQ_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
)

# pkg-config gives -L in *_LIBRARY_DIRS; plain -l flags in target_link_libraries need those dirs here.
if(PINOCCHIO_LIBRARY_DIRS)
  link_directories(${PINOCCHIO_LIBRARY_DIRS})
endif()
if(PC_ZMQ_LIBRARY_DIRS)
  link_directories(${PC_ZMQ_LIBRARY_DIRS})
endif()

# Define the common source files
set(COMMON_SOURCES
  src/can/math_ops.cpp
  src/pi_control_node.cpp
  src/pi_command_line_args.cpp
  src/pi_info.cpp
  src/pi_topic.cpp
  src/pi_topic_zmq.cpp
  src/pi_driver.cpp
  src/pi_driver_can_mit.cpp
  src/pi_driver_arx_encoder.cpp
  src/pi_driver_can.cpp
  src/pi_driver_controller.cpp
  src/pi_driver_trossen.cpp
  src/pi_driver_fr3.cpp
  src/pi_fr3_controller.cpp
  src/pi_robotiq.cpp
  # Serial bus-servo stack (protocol-neutral DriverSerial + FeeTech; a future
  # Dynamixel port adds pi_driver_dxl.cpp / pi_servo_dxl.cpp here).
  src/pi_driver_serial.cpp
  src/pi_driver_ft.cpp
  src/pi_servo_ft.cpp
  src/pi_joint.cpp
  src/pi_servo.cpp
  src/pi_servo_controller.cpp
  src/pi_servo_dm.cpp
  src/pi_servo_dm_status.cpp
  src/pi_servo_can_encoder.cpp
  src/pi_device.cpp
  src/pi_device_arm.cpp
  src/pi_device_arm_can.cpp
  src/pi_device_arm_serial.cpp
  src/pi_device_fr3.cpp
  src/pi_device_effector.cpp
  src/pi_device_effector_can.cpp
  src/pi_device_effector_controller.cpp
  src/pi_device_effector_serial.cpp
  src/pi_device_config.cpp
  src/pi_algo.cpp
  src/pi_algo_pino.cpp
  src/pi_stall_detect.cpp
  src/pi_force_feedback.cpp
  src/pi_filter.cpp
  src/pi_current_estimation.cpp
  src/pi_hold_checker.cpp
  src/pi_safe_mode.cpp

)

# ---------------------------------------------------------------------------
# libpi_trossen_shim: isolation boundary around the prebuilt libtrossen_arm.a.
# The vendor archive statically bundles its own pinocchio/urdfdom/tinyxml2;
# linking it into the node directly makes those symbols interpose with the
# node's shared pinocchio and mixes two pinocchio builds in one process
# (SIGSEGV in ModelTpl::addJoint during TrossenArmDriver::configure). The
# archive is therefore linked into this dedicated shared library and its
# symbols are hidden from the dynamic symbol table, so only the
# TrossenArmShim API (compiled here, not from the archive) is exported.
# ---------------------------------------------------------------------------
add_library(pi_trossen_shim SHARED src/pi_trossen_shim.cpp)
target_include_directories(pi_trossen_shim PRIVATE include)
target_link_libraries(pi_trossen_shim PRIVATE libtrossen_arm pthread)
# Mark every symbol pulled from static archives (i.e. libtrossen_arm.a and
# its bundled third-party copies) as LOCAL in the .so's dynamic symbol
# table. Internal references bind at link time; nothing leaks out.
target_link_options(pi_trossen_shim PRIVATE "LINKER:--exclude-libs,ALL")

add_executable(${OPENPI_CONTROL_TARGET} ${COMMON_SOURCES})

target_link_libraries(${OPENPI_CONTROL_TARGET}
  ${PC_ZMQ_LDFLAGS}
  ${PINOCCHIO_LDFLAGS}
  Boost::program_options
  pi_trossen_shim
  Franka::Franka
  openpi_control_modbus
)
target_link_options(${OPENPI_CONTROL_TARGET} PRIVATE "LINKER:--exclude-libs,ALL")

# Use a transitive DT_RPATH instead of DT_RUNPATH. Libraries copied beside the
# executable by the runtime-dependency install rule have no reason to know the
# wheel layout themselves; the node's $ORIGIN/../libs path must therefore also
# apply to their dependencies.
target_link_options(${OPENPI_CONTROL_TARGET} PRIVATE -Wl,--disable-new-dtags)

if(OPENPI_CONTROL_BUILD_TESTING)
  if(NOT TARGET GTest::gtest_main)
    find_package(GTest REQUIRED)
  endif()
  include(GoogleTest)

  set(PI_TOPIC_ZMQ_TEST_SOURCES ${COMMON_SOURCES})
  list(REMOVE_ITEM PI_TOPIC_ZMQ_TEST_SOURCES src/pi_control_node.cpp)

  add_executable(pi_topic_zmq_tests
    tests/test_command_line_args.cpp
    tests/test_driver_can_mit.cpp
    tests/test_topic_zmq.cpp
    ${PI_TOPIC_ZMQ_TEST_SOURCES}
  )
  target_compile_definitions(pi_topic_zmq_tests PRIVATE OPENPI_CONTROL_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  target_link_libraries(pi_topic_zmq_tests
    GTest::gtest_main
    ${PC_ZMQ_LDFLAGS}
    ${PINOCCHIO_LDFLAGS}
    Boost::program_options
    pi_trossen_shim
    Franka::Franka
    openpi_control_modbus
  )
  gtest_discover_tests(
    pi_topic_zmq_tests
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS native-integration)
endif()

install(TARGETS ${OPENPI_CONTROL_TARGET}
  RUNTIME_DEPENDENCY_SET openpi_control_runtime_dependencies
  RUNTIME DESTINATION openpi_control/bin)

# The shim is a build product (not scanned as an external runtime dependency),
# so bundle it explicitly next to the other shared libraries; the node's
# $ORIGIN/../libs RPATH resolves it after wheel installation.
install(TARGETS pi_trossen_shim LIBRARY DESTINATION openpi_control/libs)

# Ship the node's complete non-system shared-library closure. The executable's
# $ORIGIN/../libs RPATH above resolves these files after wheel installation;
# a binary consumer therefore does not need Pinocchio, ZeroMQ, or Boost from
# apt. Keep the Ubuntu base runtime libraries external.
install(RUNTIME_DEPENDENCY_SET openpi_control_runtime_dependencies
  PRE_EXCLUDE_REGEXES
    "^linux-vdso\\.so.*"
    "^ld-linux.*"
  POST_EXCLUDE_REGEXES
    ".*/libc\\.so\\..*"
    ".*/libdl\\.so\\..*"
    ".*/libgcc_s\\.so\\..*"
    ".*/libm\\.so\\..*"
    ".*/libpthread\\.so\\..*"
    ".*/libresolv\\.so\\..*"
    ".*/librt\\.so\\..*"
    ".*/libstdc\\+\\+\\.so\\..*"
  LIBRARY DESTINATION openpi_control/libs
  RUNTIME DESTINATION openpi_control/libs)

# libFuzzer harnesses for the wire-facing CAN status parsers. Compiles the
# runtime sources (minus the node's main) with fuzzer + ASan instrumentation so
# arbitrary CAN frames can be replayed through the reception-thread parsers.
option(OPENPI_CONTROL_BUILD_FUZZERS "Build libFuzzer harnesses (requires Clang)" OFF)
if(OPENPI_CONTROL_BUILD_FUZZERS)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "OPENPI_CONTROL_BUILD_FUZZERS requires the Clang compiler")
  endif()

  set(FUZZ_RUNTIME_SOURCES ${COMMON_SOURCES})
  list(REMOVE_ITEM FUZZ_RUNTIME_SOURCES src/pi_control_node.cpp)

  foreach(fuzz_name fuzz_servo_status_parsers)
    add_executable(${fuzz_name}
      tests/fuzz/${fuzz_name}.cpp
      ${FUZZ_RUNTIME_SOURCES}
    )
    target_compile_options(${fuzz_name} PRIVATE
      -fsanitize=fuzzer,address -fno-omit-frame-pointer -g -O1)
    target_link_options(${fuzz_name} PRIVATE -fsanitize=fuzzer,address)
    target_link_libraries(${fuzz_name}
      ${PC_ZMQ_LDFLAGS}
      ${PINOCCHIO_LDFLAGS}
      Boost::program_options
      pi_trossen_shim
      Franka::Franka
      openpi_control_modbus
    )
  endforeach()
endif()
