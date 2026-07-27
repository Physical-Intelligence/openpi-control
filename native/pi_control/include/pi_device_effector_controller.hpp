/*!
 * @file pi_device_effector_controller.hpp
 * @brief DeviceEffectorController class for grippers managed by a whole-arm controller.
 */
#pragma once

#include "pi_device_effector_can.hpp"

/*!
 * @class DeviceEffectorController
 * @brief Effector device for grippers driven by a whole-arm controller (DriverController).
 *
 * Inherits the CAN torque-gripper motion logic (distance-to-torque via
 * apply_torque_with_damping(), which ServoController maps to controller
 * external efforts). Unlike CAN grippers, controller-managed grippers need real
 * leader/follower mode switching (position vs external effort on the vendor
 * controller), so set_control_mode() restores the DeviceEffector base
 * behavior of delegating to Joint::change_control_mode_for_{leader,follower}().
 */
class DeviceEffectorController : public DeviceEffectorCan {
   public:
    /*!
     * @brief Constructor.
     * @param cla Command-line arguments.
     */
    explicit DeviceEffectorController(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DeviceEffectorController() override;

    /*!
     * @brief Delegates mode switching to the joints (DeviceEffector base
     *        behavior), undoing the CAN no-op override.
     * @param target_role Target role (LEADER or FOLLOWER).
     * @param intent Control mode intent.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;
};
