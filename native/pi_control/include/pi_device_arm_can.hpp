/*!
 * @file pi_device_arm_can.hpp
 * @brief Defines the DeviceArmCan class for MIT-mode CAN arm device.
 */
#pragma once
#include "pi_device_arm.hpp"

/*!
 * @class DeviceArmCan
 * @brief Concrete implementation of DeviceArm for MIT-mode CAN arm devices.
 */
class DeviceArmCan : public DeviceArm {
   public:
    /*!
     * @brief Constructs a new DeviceArmCan instance.
     * @param cla Command-line arguments containing device configuration parameters such as
     */
    DeviceArmCan(const CommandLineArgs& cla);

    // Destroys the DeviceArmCan instance.
    ~DeviceArmCan();

    /*!
     * @brief Sets control mode for MIT-mode CAN arm.
     *
     * MIT-mode CAN devices do not require explicit leader/follower mode switching at this level.
     * Keep as a no-op (commands still flow through normal Joint/Servo path).
     */
    virtual ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

    /*!
     * @brief MIT-mode CAN frames carry a torque feed-forward field.
     * @return Always true.
     */
    bool supports_torque_feed_forward() const override { return true; }

   private:
};

