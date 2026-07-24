/*!
 * @file pi_device_arm_nello.hpp
 * @brief Defines the DeviceArmNello class for Nello robotic arm device.
 */
#pragma once
#include "pi_device_arm.hpp"

/*!
 * @class DeviceArmNello
 * @brief Nello robotic arm device implementation.
 */
class DeviceArmNello : public DeviceArm {
   public:
    /*!
     * @brief Constructs a new DeviceArmNello instance.
     *
     * @param cla Command-line arguments containing device configuration parameters.
     */
    DeviceArmNello(const CommandLineArgs& cla);

    // Destroys the DeviceArmNello instance.
    ~DeviceArmNello();

    /*!
     * @brief Moves the arm to the ready position using Nello-specific movement sequence.
     *
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode move_to_ready_position() override;

    /*!
     * @brief Sets control mode for Nello arm.
     *
     * Nello: leader and follower require different servo operation modes.
     * - NORMAL_OPERATION: follow the target_role policy.
     * - READY_MOVE_OVERRIDE: force a safe position-based mode so the arm can move to home from current pose.
     */
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

   private:
};

