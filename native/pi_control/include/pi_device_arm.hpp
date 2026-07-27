/*!
 * @file pi_device_arm.hpp
 * @brief Base class for robotic arm devices.
 */
#pragma once
#include <chrono>
#include <map>
#include <memory>
#include <unordered_map>

#include "pi_control.hpp"
#include "pi_device.hpp"
#include "pi_filter.hpp"
#include "pi_joint.hpp"

/*!
 * @class DeviceArm
 * @brief Base class for robotic arm devices.
 */
class DeviceArm : public Device {
   public:
    /*!
     * @brief Constructor.
     * @param cla Command-line arguments.
     */
    DeviceArm(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DeviceArm();

    //
    // Override functions
    //

    /*!
     * @brief Safely parks the arm device.
     */
    virtual ReturnCode park_safely() override;

    /*!
     * @brief Applies joint commands to the arm.
     * @param msg Joint information message.
     */
    virtual ReturnCode apply_action(const MsgJoints& msg) override;

    /*!
     * @brief Processes follower joint information message.
     * @param msg_joints Follower joint information message.
     */
    virtual ReturnCode process_follower_msg(const MsgJoints& msg_joints) override;

    /*!
     * @brief Initializes the arm device.
     * @param cla Command-line arguments.
     * @param argc Number of command-line arguments.
     * @param argv Command-line argument strings.
     * @param p_topic Topic instance for communication.
     * @param p_driver Driver instance for hardware communication.
     */
    ReturnCode init(const CommandLineArgs& cla, int argc, char** argv, std::shared_ptr<Topic> p_topic,
                    std::shared_ptr<Driver> p_driver) override;

    /*!
     * @brief Gets current observation of arm joints.
     * @param msg Joint information message.
     */
    ReturnCode get_observation(MsgJoints& msg) override;

    /*!
     * @brief Starts the arm device.
     * @param baud_rate Retained driver API parameter; ignored by SocketCAN.
     */
    ReturnCode start(int baud_rate) override;

    /*!
     * @brief Final pre-loop servo error-state check. Probes every joint by
     *        re-sending its last commanded target so DM servos (which never
     *        broadcast spontaneously) answer with fresh status frames, waits
     *        ``VERIFY_SERVOS_PROBE_WAIT_US`` for the asynchronous parse of
     *        those responses, then runs the per-joint ``verify_operational()``
     *        check (which attempts one re-enable for a latched DM error).
     *        Delegates to the effector afterwards.
     * @return ReturnCode::SUCCESS when every servo is operational.
     */
    ReturnCode verify_servos_operational() override;

    /*!
     * @brief Performs a single step in the control loop.
     */
    ReturnCode step() override;

    /*!
     * @brief Puts the device into sleep mode.
     */
    ReturnCode sleep() override;

    /*!
     * @brief Stops the device.
     */
    ReturnCode stop() override;

    /*!
     * @brief Gets list of all servo IDs in the device.
     * @param servo_ids Output vector for servo IDs.
     */
    virtual ReturnCode get_servo_ids(std::vector<int>& servo_ids) override {
        for (auto& p_joint : joints_) {
            ReturnCode return_code = p_joint->get_servo_ids(servo_ids);
            if (return_code != ReturnCode::SUCCESS) return return_code;
        }
        // PI_INFO("DeviceArm", InfoLevel::FREQUENT_3, "%d joints were added from arm", (int)joints_.size());
        if (p_effector_) {
            ReturnCode return_code = p_effector_->get_servo_ids(servo_ids);
            if (return_code != ReturnCode::SUCCESS) return return_code;
            // PI_INFO("DeviceArm", InfoLevel::FREQUENT_3, "Effector joints were added");
        }

        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Requests a \"move-to-ready\" re-entry for arm and attached effector.
     *
     * Arm owns the effector device instance. To ensure both are moved to ready/home from their current
     * poses, forward the request to the effector too.
     */
    void request_move_to_ready_position(int request_id = 0) override;

    /*!
     * @brief Requests a "move-to-ready-and-stop" for arm and attached effector.
     */
    void request_move_to_ready_and_stop(int request_id = 0) override;

    /*!
     * @brief Configures joint/servo control modes for the arm (and attached effector).
     */
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

    ReturnCode set_runtime_force_feedback(bool enabled, float gain) override;

    ReturnCode set_runtime_force_feedback_gain(float gain) override;

    /*!
     * @brief Follower calibration gravity float (gravity_tune / arm_check float runs).
     *
     * Enable: the arm joints switch to leader control mode (no position PD) and
     * operate_as_follower() applies the model gravity feed-forward alone, matching the
     * leader's disengaged float. The float-entry pose is captured as the runaway
     * baseline; the control loop re-engages HOLD itself the moment any joint drifts
     * beyond ``abort_drift_rad``. Disable (also via runtime_hold() or any move-to-ready
     * path) re-engages follower position control at the current pose.
     */
    ReturnCode set_runtime_gravity_float(bool enabled, float abort_drift_rad) override;

    /*!
     * @brief Runtime per-joint torq_rescale update (calibration tools; no node restart).
     */
    ReturnCode set_runtime_torq_rescale(const std::vector<float>& values) override;

    /*!
     * @brief Publishes the next joint's DEVICE_INFO_SERVO_PARAM message (round-robin).
     *
     * Carries the effective MIT codec ranges and the motor-reported firmware ranges so
     * clients (gravity_tune) can compare two arms' servo parameters before assuming one
     * torq_rescale calibration fits both. One joint per call: the status sockets run
     * with a tiny HWM, so a per-joint burst would be dropped down to its tail.
     */
    ReturnCode publish_next_servo_param() override;

    ReturnCode runtime_hold() override;

    /*!
     * @brief UI-facing progress estimate for any active move-to-ready.
     *
     * Captures each non-failed joint's initial |home - current| at first call (or after
     * ``reset_ready_state_for_move_to_ready()`` clears the cache), then returns
     * ``1 - mean(|home - current_now| / |home - current_start|)``. Failed joints are
     * excluded during emergency recovery. Clamped to ``[0, 1]`` by the caller.
     */
    float get_ready_move_completion_ratio() const override;

    //
    // Own functions
    //

    /*!
     * @brief Enables or disables gravity compensation.
     * @param enable Enable flag.
     */
    virtual ReturnCode enable_gravity_compensation(bool enable) {
        enabled_gravity_compensation_ = enable;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Whether this arm's joints accept a torque feed-forward with position commands.
     *
     * Gravity feed-forward streamed from this node requires it. DeviceArm::init
     * fast-fails a --follower_gravity_compensation request on serial arms
     * (position-only bus); controller arms are accepted without torque streaming
     * because the vendor controller applies its own compensation.
     * @return True only for MIT-mode CAN arms.
     */
    virtual bool supports_torque_feed_forward() const { return false; }

   protected:
    /*!
     * @brief Reads current hardware values from all joints and servos.
     */
    ReturnCode read_hardware_values() override;

    /*!
     * @brief Writes command values to all joints and servos.
     */
    ReturnCode write_hardware_values() override;

    /*!
     * @brief Operates the arm as a leader device.
     */
    virtual ReturnCode operate_as_leader() override;

    /*!
     * @brief Moves the arm to the ready position.
     */
    virtual ReturnCode move_to_ready_position() override;

    /*!
     * @brief Operates the arm as a follower device.
     */
    virtual ReturnCode operate_as_follower() override;

    /*!
     * @brief Resets ready-related state for a move-to-ready re-entry.
     */
    void reset_ready_state_for_move_to_ready() override;

    /*!
     * @brief Clears buffered leader targets and interpolation segments.
     */
    void clear_command_buffers_for_move_to_ready() override;

    /*!
     * @brief Resets synchronized follower slew integration to measured positions.
     */
    void reset_slew_targets_to_current();

    std::vector<float> tele_pos_;  ///< Teleoperation target positions (relative radians).
    std::vector<float> tele_vel_;  ///< Teleoperation target velocities (rad/s).
    std::vector<float> tele_tor_;  ///< Teleoperation target torques (Nm).
    std::vector<float> max_vel_;   ///< Maximum velocity limits (rad/s).

    int servo_param_publish_index_ = 0;  ///< Round-robin cursor of publish_next_servo_param().

    std::vector<float> follower_pos_;         ///< Follower target positions (relative radians).
    std::vector<float> follower_vel_;         ///< Follower target velocities (rad/s).
    std::vector<float> follower_tor_;         ///< Follower target torques (Nm).
    std::vector<float> follower_temperature_;  ///< Follower joint temperatures (degrees Celsius).
    std::vector<float> follower_idc_current_;  ///< Follower input DC currents (Amperes).

    std::vector<Filter> follower_pos_filter_;  ///< Low-pass filters for follower position.
    std::vector<Filter> follower_vel_filter_;  ///< Low-pass filters for follower velocity.
    std::vector<Filter> follower_tor_filter_;  ///< Low-pass filters for follower torque.

    // Pre-allocated vectors to avoid allocations in control loop
    std::vector<float> target_tor_;              ///< Pre-allocated target torque vector.
    std::vector<float> current_motor_positions_;  ///< Pre-allocated current motor positions vector.
    std::vector<float> slew_goal_positions_;      ///< Safe follower goals used to compute one synchronized slew scale.
    std::chrono::steady_clock::time_point follower_slew_last_time_{};  ///< Previous follower slew update time.

    std::vector<int16_t>       joint_init_sequence_;      ///< Sequence of joint IDs for initialization.
    int16_t                    moving_joint_index_ = 0;   ///< Index of currently moving joint.
    std::map<int16_t, Joint*> joint_id_to_pointer_;      ///< Map from joint ID to Joint pointer.

    std::vector<std::unique_ptr<Joint>> joints_;  ///< List of all joints in the arm.

    std::unique_ptr<Device> p_effector_    = nullptr;  ///< Owned attached end effector device (nullable).
    bool    enabled_gravity_compensation_ = false;    ///< Gravity compensation enable flag.

    int  dof_effector_       = 0;     ///< Degrees of freedom of attached effector.
    int  servo_num_effector_ = 0;     ///< Number of servo motors in attached effector.
    bool is_ready_arm_       = false; ///< Flag indicating arm is in ready position.

    bool logged_effector_skip_in_recovery_ = false;  ///< Rate-limit the "skipping effector" log to once per recovery.
    bool logged_per_joint_read_fallback_   = false;  ///< Rate-limit the per-joint read fallback log to once per recovery.

    /*!
     * @brief Per-joint absolute displacement-to-home captured at the start of a ready move.
     *
     * Used by ``get_ready_move_completion_ratio()`` to compute a UI-friendly progress
     * estimate: ``1.0 - mean(remaining / initial)`` over the non-failed joints.
     * Populated lazily on the first call; cleared by ``reset_ready_state_for_move_to_ready()``
     * so each new move-to-ready starts from a fresh baseline. ``mutable`` so the const
     * accessor can populate the cache on first use.
     */
    mutable std::unordered_map<int16_t, float> ready_move_initial_disp_to_home_;


    DeviceConfig   config_effector_model_;      ///< Configuration for effector model.
    DeviceConfig   config_effector_individual_; ///< Configuration for individual effector device.
    CommandLineArgs cla_effector_;              ///< Command-line arguments for effector device.
};
