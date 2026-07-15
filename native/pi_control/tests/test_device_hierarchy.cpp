#include <type_traits>

#include "pi_device_arm.hpp"
#include "pi_device_arm_arx.hpp"
#include "pi_device_effector.hpp"
#include "pi_device_effector_arx.hpp"
#include "pi_device_franka.hpp"
#include "pi_device_robotiq.hpp"

static_assert(std::is_abstract_v<DeviceArm>);
static_assert(std::is_abstract_v<DeviceEffector>);
static_assert(std::is_base_of_v<DeviceArm, DeviceArmArx>);
static_assert(std::is_base_of_v<DeviceArm, DeviceFranka>);
static_assert(std::is_base_of_v<DeviceEffector, DeviceEffectorArx>);
static_assert(std::is_base_of_v<DeviceEffector, DeviceRobotiq>);
static_assert(!std::is_abstract_v<DeviceArmArx>);
static_assert(!std::is_abstract_v<DeviceFranka>);
static_assert(!std::is_abstract_v<DeviceEffectorArx>);
static_assert(!std::is_abstract_v<DeviceRobotiq>);
