/*!
 * @file pi_device_arm_arx.hpp
 * @brief Defines the DeviceArmArx class for ARX robotic arm device.
 */
#pragma once
#include "pi_device_arm.hpp"

/*!
 * @class DeviceArmArx
 * @brief Concrete implementation of DeviceArm for ARX robotic arm devices.
 */
class DeviceArmArx : public DeviceArm {
   public:
    /*!
     * @brief Constructs a new DeviceArmArx instance.
     * @param cla Command-line arguments containing device configuration parameters such as
     */
    DeviceArmArx(const CommandLineArgs& cla);

    // Destroys the DeviceArmArx instance.
    ~DeviceArmArx();

    /*!
     * @brief Sets control mode for ARX arm.
     *
     * ARX family does not require explicit leader/follower mode switching at this level.
     * Keep as a no-op (commands still flow through normal Joint/Servo path).
     */
    virtual ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

   private:
};

