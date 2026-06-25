/*!
 * @file pi_device_effector_arx.hpp
 * @brief ARX effector device implementation.
 */

#pragma once
#include "pi_device_effector.hpp"

/*!
 * @brief ARX effector device implementation.
 */
class DeviceEffectorArx : public DeviceEffector {
public:
    /*!
     * @brief Constructor.
     * @param cla Command-line arguments.
     */
    DeviceEffectorArx(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DeviceEffectorArx();

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
     * @brief Sets control mode for ARX effector.
     *
     * ARX family does not require special leader/follower mode switching here; enabling and
     * the regular command path is sufficient. We keep this as a no-op to satisfy Device API.
     */
    virtual ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

private:
    float ramped_target_pos_ = 0.0f;
    bool ramped_target_initialized_ = false;
};
