/*!
 * @file pi_joint.hpp
 * @brief Joint class for managing joint operations.
 */

#pragma once
#include <memory>
#include <vector>

#include "pi_info.hpp"
#include "pi_device.hpp"
#include "pi_driver.hpp"
#include "pi_profile.hpp"
#include "pi_servo.hpp"

#define SPRING_TYPE_BIDIRECTIONAL    2   ///< Bidirectional spring effect.
#define SPRING_TYPE_UNIDIRECTIONAL   1   ///< Unidirectional spring effect.
#define SPRING_TYPE_REVERSE_UNIDIR  -1   ///< Reverse unidirectional spring effect.

#define SPRING_TYPE_BIDIRECTIONAL    2   ///< Bidirectional spring effect.
#define SPRING_TYPE_UNIDIRECTIONAL   1   ///< Unidirectional spring effect.
#define SPRING_TYPE_REVERSE_UNIDIR  -1   ///< Reverse unidirectional spring effect.

/*!
 * @brief Manages joint operations for a single degree of freedom in the robot.
 */
class Joint {
   public:
    int id_;  ///< Unique identifier for the joint.
    float pos_error_margin_;  ///< Acceptable position error margin for position control (rad).
    /*!
     * @brief Asymmetric upper-bound safety margin (rad) for normalized [0, 1] command interpretation.
     *
     * Only consumed by ``DeviceEffector::get_gripper_pos_rad_relative_from_normalized``: when a leader
     * (or AI policy) publishes ``normalized=1.0`` (e.g. a spring-loaded handle resting at "fully open"),
     * the follower's effective target becomes ``pos_max - pos_max_safety_margin_`` instead of the raw
     * ``pos_max`` so that PID overshoot / encoder noise at the steady-state cannot push the actual
     * reading past ``pos_max + pos_error_margin_`` and trip ``Servo::read_hardware_values``'s
     * "position exceeds limits" guard. The mapping stays linear over [0, 1] so AI training data
     * remains a clean linear function of normalized command.
     *
     * Asymmetric on purpose: ``normalized=0.0`` still maps to the raw ``pos_min`` so the gripper
     * can fully close to grip objects. Default 0.0 keeps legacy behavior (no margin).
     */
    float pos_max_safety_margin_ = 0.0f;
    bool pos_max_safety_margin_configured_ = false;
    float normalized_pos_min_ = 0.0f;  ///< Lower relative-radian bound used by normalized mapping.
    float normalized_pos_max_ = 0.0f;  ///< Upper relative-radian bound used by normalized mapping.
    bool normalized_pos_min_configured_ = false;
    bool normalized_pos_max_configured_ = false;
    float pos_rescale_;       ///< Position rescaling factor for coordinate transformation.
    float vel_max_ = 0.0f;      ///< Maximum velocity limit for the joint (rad/sec).
    float follow_vel_max_ = 0.0f;  ///< Velocity limit for synchronized follower slew tracking (rad/sec).
    bool follow_vel_max_configured_ = false;  ///< Whether follow_vel_max was explicitly configured.
    float follow_viscous_damping_ = 0.0f;  ///< Motor-side viscous damping for follower tracking (Nm/(rad/sec)).
    float gravity_comp_factor_ = 1.0f;  ///< Scale applied to this joint's model gravity torque (empirical fit).
    float grip_torque_limit_nm_ = 0.0f;  ///< Symmetric grip torque bound (Nm); position mode converts it to a +/-position-error window, torque mode clips directly.
    float last_grip_requested_target_pos_ = 0.0f;  ///< Last position-mode gripper target before the symmetric force bound.
    float last_grip_applied_target_pos_ = 0.0f;  ///< Last position-mode gripper target after the symmetric force bound.
    bool grip_limiter_active_ = false;  ///< Whether the symmetric force bound changed the last gripper target.
    float accel_max_;           ///< Maximum acceleration limit for the joint (rad/sec²).
    float safe_mode_derating_;  ///< Derating factor (0.0-1.0) applied to maximum velocity and acceleration.
    float torq_min_;       ///< Minimum torque limit for the joint (Nm).
    float torq_max_;       ///< Maximum torque limit for the joint (Nm).
    float torq_rescale_;   ///< Torque rescaling factor for coordinate transformation.
    float safe_torq_min_;  ///< Minimum safe torque limit for the joint (Nm).
    float safe_torq_max_;  ///< Maximum safe torque limit for the joint (Nm).
    float tele_pos_ = 0;  ///< Teleoperation position from the leader device (relative radian).
    float tele_vel_ = 0;  ///< Teleoperation velocity from the leader device (rad/sec).
    float tele_tor_ = 0;  ///< Teleoperation torque from the leader device (Nm).
    float follower_pos_ = 0;          ///< Current position of the follower joint (relative radian).
    /// True once at least one follower state message has populated
    /// follower_pos_. The bilateral leader must not spring toward the
    /// zero-initialized default before the first message arrives (zero is the
    /// YAM home pose, not "no target").
    bool follower_pos_valid_ = false;
    float follower_vel_ = 0;          ///< Current velocity of the follower joint (rad/sec).
    float follower_tor_ = 0;          ///< Current torque of the follower joint (Nm).
    float follower_temperature_ = 0;  ///< Current temperature of the follower joint (°C).
    float follower_idc_current_ = 0;  ///< Input DC current of the follower joint (A).
    float prev_pos_ = 0;  ///< Previous position value for change detection (rad).
    bool spring_invert_ = false;  ///< Flag to invert spring effect direction.
    bool spring_enabled_ = false;               ///< Flag indicating whether spring control mode is active.
    float spring_constant_ = 0;                 ///< Spring constant value (Nm/rad).
    float spring_preload_ = 0;                  ///< Spring preload value (Nm).
    bool spring_force_config_ = false;          ///< Flag indicating whether this joint has spring force configured.
    int spring_type_ = 1;                       ///< Spring type: 1 (unidirectional), -1 (reverse unidirectional), 2 (bidirectional).
    float threshold_angle_change_ = 0;          ///< Angle change threshold (rad) to detect significant movement.
    float threshold_time_sec_ = 0;              ///< Time threshold (sec) to determine joint stability.
    prof_time_t last_significant_change_time_;  ///< Timestamp of the last significant movement.
    float target_pos_ = 0;           ///< Target position for motion control (relative radian).
    float adjusted_target_pos_ = 0;  ///< Target position adjusted with constraints (relative radian).
    float prev_target_pos_ = 0;      ///< Previous target position for change detection (relative radian).
    float target_tor_ = 0;       ///< Target torque for torque control (Nm).
    float prev_target_tor_ = 0;  ///< Previous target torque for change detection (Nm).

    // Ready-move stuck detection. Tracks the joint's measured-position progress while it is
    // commanded toward the ready pose so the ready movement can exit instead of looping
    // forever when a joint is physically stuck. ``reset_stuck_counter()`` initialises the
    // baseline; ``update_and_check_stuck()`` advances the counter and reports whether the
    // joint has crossed READY_MOVE_STUCK_ITER_THRESHOLD. Once crossed, the classification is
    // latched for the remainder of this ready move: a joint that already proved it cannot make
    // progress must not reset the device-level arrival confirmation by twitching past the motion
    // floor (gravity sag oscillation, issue #5). The joint keeps receiving commands while
    // latched, so a physically released joint still catches up and lands within tolerance.
    float stuck_last_pos_rad_ = 0.0f;  ///< Measured position at the previous ready-move iteration.
    int   stuck_iter_count_   = 0;     ///< Consecutive iterations with sub-floor measured motion.
    bool  stuck_latched_      = false; ///< True once stuck was detected during this ready move.

    /*!
     * @brief Constructor.
     * @param p_device Pointer to the device that this joint belongs to.
     * @param p_driver Pointer to the driver object.
     */
    Joint(Device* p_device, Driver* p_driver);

    /*!
     * @brief Destructor.
     */
    ~Joint() = default;

    /*!
     * @brief Safely parks the joint by moving it to a safe position before shutdown.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode park_safely();

    /*!
     * @brief Initializes the joint with configuration from the device model config.
     * @param joint_config_model JSON object containing the joint model configuration.
     * @param p_config Pointer to the device model configuration object.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode init_config_model(const json& joint_config_model, const DeviceConfig* p_config);

    /*!
     * @brief Initializes the joint with configuration from the individual device config.
     * @param joint_config JSON object containing the individual joint configuration.
     * @param p_config Pointer to the individual device configuration object.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode init_config_individual(const json& joint_config, const DeviceConfig* p_config);

    /*!
     * @brief Validates the optional normalized-position range after raw servo limits load.
     * @return ReturnCode::SUCCESS when the range is absent or valid.
     */
    ReturnCode validate_normalized_position_range();

    /*!
     * @brief Reads current sensor values from the hardware servos.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode read_hardware_values();

    /*!
     * @brief Starts the joint hardware and enables control.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode start_hardware();

    /*!
     * @brief Confirms that the reference servo's cached position is fresh
     *        (i.e. populated by an actual hardware response). Delegates to
     *        ``Servo::verify_position_fresh()``.
     * @return ReturnCode::SUCCESS if fresh, otherwise ReturnCode::FAIL.
     */
    ReturnCode verify_position_fresh();

    /*!
     * @brief Confirms every servo of the joint is in a healthy, enabled
     *        state before the control loop starts. Delegates to
     *        ``Servo::verify_operational()`` (which may attempt a one-shot
     *        re-enable for a latched DM error).
     * @return ReturnCode::SUCCESS when every servo is operational,
     *         otherwise the first failing servo's error code.
     */
    ReturnCode verify_operational();

    /*!
     * @brief Commands the joint's reference servo to hold at its currently
     *        measured position. Updates ``prev_target_pos_`` so the next
     *        ``move_to_ready_position()`` iteration starts incremental
     *        stepping from the true current pose, not from a stale 0.
     * @return ReturnCode indicating success or failure of the underlying
     *         servo move().
     */
    ReturnCode hold_at_current_position();

    /*!
     * @brief Resets the ready-move stuck detection counter. Should be called
     *        once just before a fresh ready movement begins (e.g. after
     *        hold_at_current_position()) so the baseline reflects the joint's
     *        current measured position.
     */
    void reset_stuck_counter();

    /*!
     * @brief Updates the stuck-iteration counter for this joint based on the
     *        current measured position. Increments when the joint has moved
     *        less than ``READY_MOVE_STUCK_POS_DELTA_RAD`` since the last call
     *        AND is still further than ``pos_error_margin_`` from the target;
     *        resets otherwise. Returns true once the counter exceeds
     *        ``READY_MOVE_STUCK_ITER_THRESHOLD``.
     * @return True if the joint is currently classified as stuck.
     */
    bool update_and_check_stuck();

    /*!
     * @brief Stops the joint hardware safely.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode stop_hardware() { return park_safely(); }

    /*!
     * @brief Gets the follower's position.
     * @return Follower's position in relative radian.
     */
    float get_follower_pos_rad() { return follower_pos_; }

    /*!
     * @brief Gets the joint's current position in relative radian.
     * @return The joint's current position in relative radian.
     */
    float get_pos_rad_relative() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_pos_rad_relative(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_pos_rad_relative();
    }

    /*!
     * @brief Converts an absolute radian value to relative radian.
     * @param rad_absolute The absolute radian value to convert.
     * @return The converted relative radian value.
     */
    float get_pos_rad_relative(float rad_absolute) {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_pos_rad_relative(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_pos_rad_relative(rad_absolute);
    }

    /*!
     * @brief Gets the joint's current position in absolute radian.
     * @return The joint's current position in absolute radian.
     */
    float get_pos_rad_absolute() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_pos_rad_absolute(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_pos_rad_absolute();
    }

    /*!
     * @brief Converts a relative radian value to absolute radian.
     * @param rad_relative The relative radian value to convert.
     * @return The converted absolute radian value.
     */
    float get_pos_rad_absolute(float rad_relative) {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_pos_rad_absolute(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_pos_rad_absolute(rad_relative);
    }

    /*!
     * @brief Gets the joint's current position as raw servo value.
     * @return The joint's current position as raw servo value.
     */
    float get_pos_servo() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_pos_servo(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_pos_servo();
    }

    /*!
     * @brief Gets the joint's current velocity.
     * @return The joint's current velocity in rad/sec.
     */
    float get_vel_rad_sec() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_vel_rad_sec(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_vel_rad_sec();
    }

    /*!
     * @brief Gets the joint's current torque.
     * @return The joint's current torque in Nm.
     */
    float get_tor_nm() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_tor_nm(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_tor_nm();
    }

    /*!
     * @brief Age of the hardware frame backing the reference servo's position.
     * @return Frame age in milliseconds, or -1 when unknown/untracked.
     */
    prof_time_msec_t get_frame_age_ms() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_frame_age_ms(): Joint%d", id_);
            return -1;
        }
        return servos_[reference_servo_index_]->get_frame_age_ms();
    }

    /*!
     * @brief Gets the joint's maximum position limit.
     * @return Maximum position limit in relative radian.
     */
    float get_pos_max_relative() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_pos_max(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->pos_max_rel_;
    }

    /*!
     * @brief Sets the joint's maximum position limit.
     * @param pos_max Maximum position limit in relative radian.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode set_pos_max_relative(float pos_max) {
        if (reference_servo_index_ < 0 ||
            reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in set_pos_max(): Joint%d", id_);
            return ReturnCode::NOT_INITIALIZED;
        }
        servos_[reference_servo_index_]->pos_max_rel_ = pos_max;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Gets the joint's minimum position limit.
     * @return Minimum position limit in relative radian.
     */
    float get_pos_min_relative() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_pos_min(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->pos_min_rel_;
    }

    bool has_normalized_position_range() const {
        return normalized_pos_min_configured_ && normalized_pos_max_configured_;
    }

    float get_normalized_pos_min_relative() {
        return has_normalized_position_range() ? normalized_pos_min_ : get_pos_min_relative();
    }

    float get_normalized_pos_max_relative() {
        return has_normalized_position_range() ? normalized_pos_max_ : get_pos_max_relative();
    }

    /*!
     * @brief Sets the joint's minimum position limit.
     * @param pos_min Minimum position limit in relative radian.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode set_pos_min_relative(float pos_min) {
        if (reference_servo_index_ < 0 ||
            reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in set_pos_min(): Joint%d", id_);
            return ReturnCode::NOT_INITIALIZED;
        }
        servos_[reference_servo_index_]->pos_min_rel_ = pos_min;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Gets the joint's direction invert flag.
     * @return Direction invert value (typically 1 for normal, -1 for inverted).
     */
    int get_dir_invert() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_dir_invert(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->dir_invert_;
    }

    /*!
     * @brief Gets the servo resolution for this joint.
     * @return Servo resolution (typically in counts per revolution or similar units).
     */
    int get_servo_resolution() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_servo_resolution(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->get_servo_resolution();
    }

    /*!
     * @brief Gets the joint's home position.
     * @return Home position in relative radian.
     */
    float get_home_pos_relative() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_dir_invert(): Joint%d", id_);
        }
        Servo* p_servo_reference = servos_[reference_servo_index_].get();
        return p_servo_reference->home_pos_rel_;
    }

    /*!
     * @brief Gets the joint's spring home position in absolute coordinates.
     * @return Spring home position in absolute radian.
     */
    float get_spring_home_pos_absolute() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_spring_home_absolute(): Joint%d", id_);
        }
        Servo* p_servo_reference = servos_[reference_servo_index_].get();
        float spring_home_relative = p_servo_reference->spring_home_pos_rel_;
        float spring_home_abs = p_servo_reference->get_pos_rad_absolute(spring_home_relative);
        return spring_home_abs;
    }

    /*!
     * @brief Gets the joint's response delay.
     * @return Response delay in seconds.
     */
    float get_response_delay() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_response_delay(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->response_delay_;
    }

    /*!
     * @brief Gets the joint.s current temperature.
     * @return Temperature in degrees Celsius.
     */
    float get_temperature() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_temperature(): Joint%d", id_);
        }
        return servos_[reference_servo_index_]->temperature_;
    }

    /*!
     * @brief Gets the total input DC current for all servos in this joint.
     * @return Total input DC current in amperes (A).
     */
    float get_idc_current() {
        if (servos_.size() == 0 || reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in get_idc_current(): Joint%d", id_);
        }
        float idc_current = 0;
        for (auto& p_servo : servos_) {
            idc_current += p_servo->get_idc_current();
        }
        return idc_current;
    }

    /*!
     * @brief Sets the teleoperation position from the leader device.
     * @param tele_pos Teleoperation position from the leader in relative radian.
     */
    void set_tele_pos_rad(float tele_pos);

    /*!
     * @brief Gets the teleoperation position from the leader device.
     * @return Teleoperation position from the leader in relative radian.
     */
    float get_tele_pos_rad() { return tele_pos_; }

    /*!
     * @brief Gets the safe teleoperation position with safety constraints applied.
     * @return Safe teleoperation position in relative radian.
     */
    float get_safe_tele_pos_rad() { return servos_[reference_servo_index_]->get_safe_tele_pos_rad(tele_pos_); }

    /*!
     * @brief Gets the safe teleoperation position for an explicit command position.
     * @param tele_pos Commanded position to be safety-clamped.
     * @return Safe command position in relative radian.
     */
    float get_safe_tele_pos_rad(float tele_pos) { return servos_[reference_servo_index_]->get_safe_tele_pos_rad(tele_pos); }

    /*!
     * @brief Sets the teleoperation velocity from the leader device.
     * @param tele_vel Teleoperation velocity from the leader in rad/sec.
     */
    void set_tele_vel_rad_sec(float tele_vel) { tele_vel_ = tele_vel; }

    /*!
     * @brief Sets the teleoperation torque from the leader device.
     * @param tele_tor Teleoperation torque from the leader in Nm.
     */
    void set_tele_tor_nm(float tele_tor) { tele_tor_ = tele_tor; }

    /*!
     * @brief Sets the joint's position control derivative (Kd) gain.
     * @param pos_kd The position control Kd gain value.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode set_pos_kd(float pos_kd) {
        if (reference_servo_index_ < 0 ||
            reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Servo is not initialized in set_pos_kd(): Joint%d", id_);
            return ReturnCode::NOT_INITIALIZED;
        }
        servos_[reference_servo_index_]->pos_kd_ = pos_kd;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Collects all servo IDs associated with this joint.
     * @param servo_ids Output vector to store the collected servo IDs.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode get_servo_ids(std::vector<int>& servo_ids) {
        ReturnCode return_code = ReturnCode::SUCCESS;
        for (auto& p_servo : servos_) {
            return_code = p_servo->get_servo_ids(servo_ids);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Joint %d failed to add servos %d to the list", id_, p_servo->id_);
                return return_code;
            }
        }
        return return_code;
    }

    /*!
     * @brief Clamps a position-mode gripper target to a symmetric bounded error.
     *
     * With MIT position control the motor torque is approximately
     * ``kp * (target - measured)``. Clamping both opening and closing error to
     * ``grip_torque_limit_nm_ / kp`` prevents either mechanical stop or bind
     * from driving the gripper at full current. The limit derates linearly to
     * zero over the 60-90 C ramp.
     *
     * @param target_pos The desired target position in relative radian.
     * @return The clamped target position in relative radian.
     */
    float clamp_target_to_grip_torque_limit(float target_pos);

    float get_last_grip_requested_target_pos() const { return last_grip_requested_target_pos_; }
    float get_last_grip_applied_target_pos() const { return last_grip_applied_target_pos_; }
    bool is_grip_limiter_active() const { return grip_limiter_active_; }

    /*!
     * @brief Moves the joint to the target position using trajectory planning.
     * @param target_pos The target position in relative radian.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos);

    /*!
     * @brief Moves the joint to the target position with specified velocity and torque limits.
     * @param target_pos The target position in relative radian.
     * @param target_vel The maximum velocity for the motion in rad/sec.
     * @param target_tor The maximum torque limit for the motion in Nm.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos, float target_vel, float target_tor);

    /*!
     * @brief Applies the specified torque to the joint.
     * @param torque The torque to be applied in Nm.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode apply_torque(float torque);

    /*!
     * @brief Applies torque while retaining configured servo damping.
     * @param torque The torque to be applied in Nm.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode apply_torque_with_damping(float torque);

    /*!
     * @brief Calculates the spring force based on current position.
     * @return Calculated spring force in Nm.
     */
    float calc_spring_force();

    /*!
     * @brief Updates the joint stability status based on movement history.
     * @param now_time Current time for stability evaluation.
     * @return True if the joint is stable, false otherwise.
     */
    bool update_stability(const prof_time_t& now_time);

    /*!
     * @brief Applies spring effect force to the joint.
     * @param base_torque Base torque value to which spring force will be added (Nm).
     * @param commandline_flag Flag indicating whether spring effect is enabled via command line option.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode apply_spring_force(float base_torque, bool commandline_flag);

    /*!
     * @brief Changes the joint control mode to support spring effect.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode change_control_mode_for_spring();

    /*!
     * @brief Changes the joint control mode for leader (teleoperation) operation.
     * @param current_time Current time for timing-dependent mode changes.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode change_control_mode_for_leader(const prof_time_t& current_time);

    /*!
     * @brief Changes the joint control mode for follower operation.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode change_control_mode_for_follower();

    /*!
     * @brief Factory method to create Joint objects from configuration.
     * @param p_config Pointer to the device model configuration.
     * @param p_config_individual Pointer to the individual device configuration.
     * @param joints Output vector to store pointers to the created Joint objects.
     * @param p_device Pointer to the Device object.
     * @param p_driver Pointer to the Driver object.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode new_joints(const DeviceConfig* p_config, const DeviceConfig* p_config_individual,
                                 std::vector<std::unique_ptr<Joint>>& joints, Device* p_device, Driver* p_driver);

    /*!
     * @brief Returns the device type of the device that this joint belongs to.
     * @return Device type of the parent device, or UNKNOWN if device is not set.
     */
    DeviceType get_device_type_belong_to() {
        if (p_device_ == nullptr) return DeviceType::UNKNOWN;
        return p_device_->get_device_type();
    }

    /*!
     * @brief Returns the role of the device that this joint belongs to.
     * @return The role of the parent device, or UNKNOWN if device is not set.
     */
    Role get_device_role_belong_to() {
        if (p_device_ == nullptr) return Role::UNKNOWN;
        return p_device_->get_device_role();
    }

    /*!
     * @brief Clips a value to a specified range.
     * @param value The value to be clipped.
     * @param min The minimum allowable value.
     * @param max The maximum allowable value.
     * @return The clipped value (clamped to [min, max]).
     */
    float clipping(float value, float min, float max) { return (value > max) ? max : ((value < min) ? min : value); }

    /*!
     * @brief Returns a pointer to the reference servo for this joint.
     * @return Pointer to the reference Servo object, or nullptr if index is invalid.
     */
    Servo* get_reference_servo() {
        if (reference_servo_index_ >= (int)servos_.size()) {
            PI_ERROR("Reference index is invalid in get_reference_servo()");
            return nullptr;
        }
        return servos_[reference_servo_index_].get();
    }

    /*!
     * @brief Returns a reference to the vector of servos in this joint.
     * @return Reference to the vector containing pointers to all Servo objects in this joint.
     */
    std::vector<std::unique_ptr<Servo>>& get_servos() { return servos_; }
    const std::vector<std::unique_ptr<Servo>>& get_servos() const { return servos_; }

    /*!
     * @brief Returns the hardware servo id of this joint's reference servo.
     *
     * This is the CAN identifier users see on the physical hardware and is the
     * id reported to the UI when this joint fails.
     * For single-servo joints (the common case) this is simply the only
     * servo's id; for multi-servo joints we use the configured
     * ``reference_servo_index_`` so the joint maps to a single, stable id.
     * Returns -1 if the joint has no servos.
     */
    int16_t reference_servo_id() const {
        if (servos_.empty()) return -1;
        const size_t idx = (reference_servo_index_ >= 0 &&
                            static_cast<size_t>(reference_servo_index_) < servos_.size())
                               ? static_cast<size_t>(reference_servo_index_)
                               : 0;
        if (servos_[idx] == nullptr) return -1;
        return static_cast<int16_t>(servos_[idx]->id_);
    }

    /*!
     * @brief Returns the number of servos in this joint.
     * @return The number of servos associated with this joint.
     */
    int get_servo_num() { return servos_.size(); }

   protected:
    int reference_servo_index_ = 0;  ///< Index of the reference servo in the servos_ vector.
    std::vector<std::unique_ptr<Servo>> servos_;  ///< Vector of unique pointers to Servo objects that make up this joint.
    Driver* p_driver_ = nullptr;  ///< Pointer to the Driver object for hardware communication.
    Device* p_device_ = nullptr;  ///< Pointer to the Device object that this joint belongs to.
    bool parked_ = false;  ///< Flag indicating whether this joint has been safely parked.
    float prev_tele_pos_ = 0;  ///< Previous teleoperation position for change detection (relative radian).
    float tele_pos_moving_dir_ = 0;  ///< Direction of teleoperation position movement.
    float stalled_position_ = 0;  ///< Position where the joint stalled.

};
