/*!
 * @file pi_device_arm_serial.hpp
 * @brief Defines the DeviceArmSerial class for serial bus-servo arm device.
 */
#pragma once
#include "pi_device_arm.hpp"

/*!
 * @class DeviceArmSerial
 * @brief serial bus-servo arm device implementation.
 */
class DeviceArmSerial : public DeviceArm {
   public:
    /*!
     * @brief Constructs a new DeviceArmSerial instance.
     *
     * @param cla Command-line arguments containing device configuration parameters.
     */
    DeviceArmSerial(const CommandLineArgs& cla);

    // Destroys the DeviceArmSerial instance.
    ~DeviceArmSerial();

    /*!
     * @brief Moves the arm to the ready position using serial-arm movement sequence.
     *
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode move_to_ready_position() override;

    /*!
     * @brief Sets control mode for serial bus-servo arm.
     *
     * Serial bus-servo devices: leader and follower require different servo operation modes.
     * - NORMAL_OPERATION: follow the target_role policy.
     * - READY_MOVE_OVERRIDE: force a safe position-based mode so the arm can move to home from current pose.
     */
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

   private:
};

