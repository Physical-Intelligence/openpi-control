/*!
 * @file pi_device_effector_can.hpp
 * @brief MIT-mode CAN effector device implementation.
 */

#pragma once
#include "pi_device_effector.hpp"

/*!
 * @brief MIT-mode CAN effector device implementation.
 */
class DeviceEffectorCan : public DeviceEffector {
public:
    /*!
     * @brief Constructor.
     * @param cla Command-line arguments.
     */
    DeviceEffectorCan(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DeviceEffectorCan();

    //
    // Override functions
    //

    /*!
     * @brief Moves a joint using distance-based torque control.
     * @param p_joint Pointer to the joint.
     * @param target_pos Target position (relative radians).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode move_joint_with_torque(Joint *p_joint, float target_pos) override;

    /*!
     * @brief Sets control mode for MIT-mode CAN effector.
     *
     * MIT-mode CAN devices do not require special leader/follower mode switching here; enabling and
     * the regular command path is sufficient. We keep this as a no-op to satisfy Device API.
     */
    virtual ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

private:
    float ramped_target_pos_ = 0.0f;
    bool ramped_target_initialized_ = false;
};
