/*!
 * @file pi_device_effector.hpp
 * @brief Base class for robotic effector devices.
 */
#pragma once
#include <map>
#include <boost/circular_buffer.hpp>
#include <vector>

#include "pi_control.hpp"
#include "pi_device.hpp"
#include "pi_device_arm.hpp"
#include "pi_force_feedback.hpp"
#include "pi_joint.hpp"

/*!
 * @brief Effector control modes.
 */
enum class EffectorControlMode {
    TORQUE,    ///< Torque control mode.
    POSITION,  ///< Position control mode.
    UNDEFINED  ///< Undefined control mode.
};

/*!
 * @brief Base class for robotic effector devices.
 */
class DeviceEffector : public Device {
   public:
    /*!
     * @brief Constructor.
     * @param cla Command-line arguments.
     */
    DeviceEffector(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DeviceEffector();

    //
    // Override functions
    //

    /*!
     * @brief Safely parks the effector device before shutdown.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode park_safely() override;

    /*!
     * @brief Initializes the effector device.
     * @param cla Command-line arguments.
     * @param argc Argument count.
     * @param argv Argument values.
     * @param p_topic Topic instance for communication.
     * @param p_driver Driver instance for hardware communication.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode init(const CommandLineArgs& cla, int argc, char** argv, std::shared_ptr<Topic> p_topic,
                            std::shared_ptr<Driver> p_driver) override;

    /*!
     * @brief Gets the current observation of the effector joints.
     * @param msg Joint information message to be populated.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode get_observation(MsgJoints& msg) override;

    /*!
     * @brief Reads current hardware values from all joints and servos.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode read_hardware_values() override;

    /*!
     * @brief Writes command values to all joints and servos.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode write_hardware_values() override;

    /*!
     * @brief Starts the effector device.
     * @param baud_rate Retained driver API parameter; ignored by SocketCAN.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode start(int baud_rate) override;

    /*!
     * @brief Gets a list of all servo IDs.
     * @param servo_ids Output vector to be populated with servo IDs.
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode get_servo_ids(std::vector<int>& servo_ids) override {
        for (auto& p_joint : joints_) {
            ReturnCode return_code = p_joint->get_servo_ids(servo_ids);
            if (return_code != ReturnCode::SUCCESS) return return_code;
        }
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Probes effector joints individually to identify the first servo
     *        whose per-joint read fails. Intended for use by the owning Arm
     *        when its bulk read on the shared bus failed but all arm joints
     *        responded -- the failing servo must then live on the effector.
     * @return The hardware servo id (matches the value printed on the cable)
     *         of the first non-responsive effector joint, or -1 if every joint
     *         responds.
     */
    int probe_failed_servo_id();

    /*!
     * @brief Moves a joint using torque control.
     * @param p_joint Pointer to the joint.
     * @param target_pos Target position (relative radians).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode move_joint_with_torque(Joint* p_joint, float target_pos) = 0;

    //
    // Pure virtual functions
    //

    /*!
     * @brief Moves the effector to the ready position.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode move_to_ready_position() override;

    /*!
     * @brief Operates the effector as a follower device.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode operate_as_follower() override;

    /*!
     * @brief Sets the attached arm device.
     * @param p_arm Pointer to the arm device (can be nullptr).
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode set_arm(DeviceArm* p_arm) {
        p_arm_ = p_arm;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Gets the attached arm device.
     * @return Pointer to the arm device, or nullptr if none.
     */
    DeviceArm* get_arm() const { return p_arm_; }

    /*!
     * @brief Sets the minimum and maximum position limits for all joints.
     * @param min_pos Minimum position limit (relative radians).
     * @param max_pos Maximum position limit (relative radians).
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode set_effector_min_max_pos(float min_pos, float max_pos) {
        ReturnCode first_error = ReturnCode::SUCCESS;
        for (auto& p_joint : joints_) {
            ReturnCode return_code = p_joint->set_pos_min_relative(min_pos);
            if (return_code != ReturnCode::SUCCESS &&
                first_error == ReturnCode::SUCCESS) {
                first_error = return_code;
            }

            return_code = p_joint->set_pos_max_relative(max_pos);
            if (return_code != ReturnCode::SUCCESS &&
                first_error == ReturnCode::SUCCESS) {
                first_error = return_code;
            }
        }
        return first_error;
    }

    //
    // Own functions
    //

    /*!
     * @brief Applies joint commands to the effector.
     * @param msg Joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode apply_action(const MsgJoints& msg) override;

    /*!
     * @brief Processes a received follower joint information message.
     * @param msg_joints Follower joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode process_follower_msg(const MsgJoints& msg_joints) override;

    /*!
     * @brief Operates the effector as a leader device.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode operate_as_leader() override;

    /*!
     * @brief Gets the normalized gripper position from a joint.
     * @param p_joint Pointer to the joint.
     * @return Normalized gripper position [0.0, 1.0].
     */
    virtual float get_normalized_gripper_position(Joint* p_joint);

    /*!
     * @brief Converts normalized gripper position to relative radian joint position.
     * @param normalized_pos Normalized gripper position [0.0, 1.0].
     * @return Joint position in relative radians.
     */
    virtual float get_gripper_pos_rad_relative_from_normalized(float normalized_pos);

    /*!
     * @brief Sets the derivative gain (Kd) for effector position control.
     * @param effector_kd Derivative gain (Kd) value.
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode set_effector_kd(float effector_kd);

    /*!
     * @brief Gets the current control mode.
     * @return Current EffectorControlMode.
     */
    EffectorControlMode get_control_mode() { return control_mode_; }

    /*!
     * @brief Returns the effective control mode used for servo/driver configuration.
     *
     * Why this exists:
     * - `COMMAND_MOVE_TO_READY_POS` needs to temporarily force a safe, position-based behavior while the device
     *   moves to home/ready from its *current* pose.
     * - We intentionally do NOT overwrite the persistent `control_mode_` (which represents the user's configured
     *   behavior) because that would require saving/restoring and risks leaving the device in the wrong mode if
     *   an error/timeout occurs.
     * - Instead, we use an override flag that is applied only during the ready-move window; once cleared, the
     *   device deterministically returns to its normal policy-based behavior (role + configured control_mode_).
     */
    EffectorControlMode get_effective_control_mode() const {
        return ready_move_force_position_mode_ ? EffectorControlMode::POSITION : control_mode_;
    }

    /*!
     * @brief Sets/clears the temporary override used during move-to-ready.
     */
    void set_ready_move_force_position_mode(bool enable) { ready_move_force_position_mode_ = enable; }

    /*!
     * @brief Configures joint/servo control modes for the effector.
     */
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

    /*!
     * @brief Sets the distance-to-torque conversion factor.
     * @param distance_to_torque Conversion factor (Nm/rad).
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode set_distance_to_torque(float distance_to_torque) {
        distance_to_torque_ = distance_to_torque;
        return ReturnCode::SUCCESS;
    }

   protected:
    /*!
     * @brief Resets ready-related state for a move-to-ready re-entry.
     *
     * Base Device reset clears generic counters; effector also clears its per-joint step map.
     */
    void reset_ready_state_for_move_to_ready() override {
        Device::reset_ready_state_for_move_to_ready();
        // Keep override flag unchanged here; it is managed by set_control_mode(intent).
    }

    /*!
     * @brief Clears buffered leader targets and interpolation segments.
     */
    void clear_command_buffers_for_move_to_ready() override;

    ///< @todo If effector DOF is changed, then this should be revisited
    std::vector<float> tele_pos_;  ///< Teleoperation target positions (relative radians).
    std::vector<float> tele_vel_;  ///< Teleoperation target velocities (rad/s).
    std::vector<float> tele_tor_;  ///< Teleoperation target torques (Nm).

    std::vector<std::unique_ptr<ForceFeedback>> force_feedbacks_;  ///< Force feedback objects for haptic teleoperation.

    std::vector<std::unique_ptr<Joint>> joints_;  ///< List of joints in the effector.

    int start_index_joint_ = 0;                   ///< Starting index of effector joints in merged joint array.
    int start_index_servo_ = 0;                   ///< Starting index of effector servos in merged servo array.

    EffectorControlMode control_mode_ = EffectorControlMode::UNDEFINED;  ///< Current control mode.
    bool ready_move_force_position_mode_ = false;  ///< Temporary override for COMMAND_MOVE_TO_READY_POS.

    DeviceArm* p_arm_ = nullptr;                  ///< Pointer to the attached arm device.

    float distance_to_torque_ = 0.0f;             ///< Distance-to-torque conversion factor (Nm/rad).
    float grip_spring_offset_ = 0.0f;             ///< Torque-mode spring offset (rad), subtracted from the position error (monopi ControlFollowGripper "offset"; a per-installation zero trim -- prefer adjusting the servo zero).
    bool open_at_min_ = false;              ///< Open side of the effector: true if open at min position, false if open at max position.
};
