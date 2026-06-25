/*!
 * @file pi_device.hpp
 * @brief Device base class for all robotic devices.
 */

#pragma once
#include <algorithm>
#include <chrono>
#include <memory>
#include <mutex>
#include <unordered_set>

#include "pi_algo.hpp"
#include "pi_control.hpp"
#include "pi_device_config.hpp"
#include "pi_driver.hpp"
#include "pi_topic.hpp"

#define DEVICE_ARRIVED_CONFIRM_CNT 30  ///< Number of control loop iterations to confirm joint arrival.

// Heartbeat-based watchdog for the unified move-to-ready and emergency-recovery state machines.
// Each move-to-ready (startup, command-driven, or emergency) is allowed to take as long as it
// needs (a slow ready move on a heavy arm can run for tens of seconds), but every successful
// ``step()`` iteration must refresh ``last_step_heartbeat_``. If no heartbeat arrives for this
// many milliseconds, ``check_emergency_timeout_locked`` flips the state to TIMED_OUT and the
// device is force-parked. This catches the case where the control loop has actually hung
// (deadlock or blocked device I/O) without punishing a legitimately slow recovery.
// Configurable via --emergency_shutdown_timeout_ms.
#define DEVICE_EMERGENCY_TIMEOUT_MS_DEFAULT 2000

class Topic;

/*!
 * @brief Device control methods.
 */
enum class ControlType {
    UNKNOWN,           ///< Control type is unknown.
    SERVO_DIRECT,     ///< Direct servo control.
    FIRMWARE_INDIRECT  ///< Firmware indirect control.
};

/*!
 * @brief High-level intent for switching device control modes.
 *
 * This intentionally expresses *policy* rather than hardware details. Concrete device classes
 * map this intent to joint/servo/driver settings.
 */
enum class ControlModeIntent {
    NORMAL_OPERATION = 0,   ///< Configure for normal behavior (based on role and device settings).
    READY_MOVE_OVERRIDE = 1 ///< Temporary override used only to move to ready/home position safely.
};

/*!
 * @brief Types of robotic devices.
 */
enum class DeviceType {
    UNKNOWN,             ///< Device type is unknown.
    ARM,                 ///< Robotic arm device.
    EFFECTOR             ///< Attached end-effector device.
};

/*!
 * @brief Device operation modes.
 */
enum class DeviceMode {
    INVALID,     ///< Invalid or uninitialized mode.
    NORMAL,      ///< Normal operation mode.
    PASSIVE      ///< Passive mode.
};

/*!
 * @brief Joint movement strategies.
 */
enum class MovingMode {
    INVALID,    ///< Invalid or uninitialized moving mode.
    SEQUENTIAL, ///< Sequential movement.
    PARALLEL    ///< Parallel movement.
};

/*!
 * @brief Abstract base class for all robotic devices.
 */
class Device {
   public:
    /*!
     * @brief Constructor.
     * @param cla Command-line arguments.
     */
    Device(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    virtual ~Device();

    //
    // Pure virtual functions
    //

    /*!
     * @brief Safely parks the device before shutdown.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode park_safely() { return ReturnCode::SUCCESS; };

    /*!
     * @brief Applies joint commands to the device.
     * @param msg Joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode apply_action(const MsgJoints& msg) = 0;

    /*!
     * @brief Gets the current observation of the device joints.
     * @param msg Joint information message to be populated.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode get_observation(MsgJoints& msg) = 0;

    /*!
     * @brief Applies joystick commands to the device.
     * @param msg_joystick Joystick information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode apply_action(const MsgJoystick& msg_joystick);

    /*!
     * @brief Processes a received follower joint information message.
     * @param msg_joints Follower joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode process_follower_msg(const MsgJoints& msg_joints) = 0;

    /*!
     * @brief Reads current hardware values from all joints and servos.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode read_hardware_values() = 0;

    /*!
     * @brief Writes command values to all joints and servos.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode write_hardware_values() = 0;

    /*!
     * @brief Moves the device to the ready position.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode move_to_ready_position() = 0;

    /*!
     * @brief Operates the device as a leader device.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode operate_as_leader() = 0;

    /*!
     * @brief Operates the device as a follower device.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode operate_as_follower() = 0;

    /*!
     * @brief Gets a list of all servo IDs.
     * @param servo_ids Output vector to be populated with servo IDs.
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode get_servo_ids(std::vector<int>& servo_ids) = 0;

    //
    // Virtual functions
    //

    /*!
     * @brief Initializes the device.
     * @param cla Command-line arguments.
     * @param argc Argument count.
     * @param argv Argument values.
     * @param p_topic Topic instance for communication (can be nullptr).
     * @param p_driver Driver instance for hardware communication (can be nullptr).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode init(const CommandLineArgs& cla, int argc, char** argv,
                            std::shared_ptr<Topic> p_topic = nullptr, std::shared_ptr<Driver> p_driver = nullptr);

    /*!
     * @brief Gets the current observation in relative position format.
     * @param profile_data_joint Profile data joint structure to be populated.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    /*!
     * @brief Checks if the device is currently running.
     * @return True if the device is running, false otherwise.
     */
    virtual bool is_running();

    /*!
     * @brief Starts the device.
     * @param baud_rate Retained driver API parameter; ignored by SocketCAN.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode start(int baud_rate);

    /*!
     * @brief Performs a single step in the device's control loop.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode step();

    /*!
     * @brief Puts the device into sleep mode.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode sleep();

    /*!
     * @brief Stops the device.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode stop();

    /*!
     * @brief Publishes device information to the communication topic.
     * @param info_key Information key.
     * @param p_float_data Optional pointer to float data (can be nullptr).
     * @param p_int_data Optional pointer to int data (can be nullptr).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode publish_device_info(int info_key, std::vector<float>* p_float_data = nullptr,
                                           std::vector<int>* p_int_data = nullptr);

    /*!
     * @brief Requests a \"move-to-ready\" re-entry from the current pose.
     *
     * The request is consumed in the control loop (`step()`): the device will temporarily override
     * control modes (see `ControlModeIntent::READY_MOVE_OVERRIDE`), reset ready flags, and reuse the
     * existing `move_to_ready_position()` state machines until completion, then restore normal control modes.
     */
    virtual void request_move_to_ready_position(int request_id = 0) {
        move_to_ready_request_id_ = request_id;
        completed_move_to_ready_request_id_ = 0;
        // Suppress the previous ready announcement before this command's ACK is published.
        is_ready_ = false;
        last_ready_move_progress_publish_ = {};
        move_to_ready_cmd_state_ = MoveToReadyCmdState::REQUESTED;
    }

    int completed_move_to_ready_request_id() const { return completed_move_to_ready_request_id_; }

    /*!
     * @brief Requests a "move-to-ready" followed by automatic stop.
     *
     * Same as `request_move_to_ready_position()` but the device will also set
     * `is_running_` to false (via `p_topic_->stop()`) once the ready position
     * is reached, causing the main loop to exit cleanly.
     */
    virtual void request_move_to_ready_and_stop(int request_id = 0) {
        stop_after_ready_ = true;
        request_move_to_ready_position(request_id);
    }

    /*!
     * @brief Emergency recovery lifecycle states.
     *
     * Triggered from the main loop when ``step()`` reports a joint error. The
     * device reuses the existing move-to-ready state machine but at the slower
     * ERROR velocity (``cla.move_to_ready_vel_rad_s_error``) and skips joints
     * that have become unresponsive. When recovery completes (or times out),
     * the main loop tears the device down via the normal ``stop()`` path.
     */
    enum class EmergencyRecoveryState {
        NONE = 0,        ///< Normal operation; no emergency in progress.
        REQUESTED = 1,   ///< Trigger received; mode-switch + slow ready move pending.
        MOVING_SLOW = 2, ///< Slow move-to-ready in progress.
        COMPLETED = 3,   ///< Ready position reached; main loop should exit.
        TIMED_OUT = 4    ///< Recovery loop stopped progressing (no step heartbeat for ``emergency_shutdown_timeout_ms``).
    };

    /*!
     * @brief Triggers the emergency-recovery state machine for the given error.
     *
     * Idempotent: subsequent calls during an active recovery only update the
     * recorded cause / failed joint id and do not restart the state machine.
     * Publishes ``DEVICE_INFO_ERROR_DETECTED`` on the first call.
     *
     * @param cause            The ReturnCode reported by ``step()``.
     * @param failed_joint_id  The id of the joint that failed, or ``-1`` if
     *                         unknown / multiple joints failed.
     * @return ``ReturnCode::SUCCESS`` if the recovery state machine was armed
     *         (or was already armed). Errors are non-fatal: callers must
     *         continue stepping until ``emergency_recovery_state()`` reports a
     *         terminal value.
     */
    virtual ReturnCode enter_emergency_recovery(ReturnCode cause, int failed_joint_id);

    /*!
     * @brief Whether the device is currently inside the emergency-recovery state machine.
     */
    bool is_in_emergency_recovery() const {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        return emergency_state_ != EmergencyRecoveryState::NONE;
    }

    /*!
     * @brief Whether a ready move would run as part of initial startup.
     *
     * True only before the device first becomes ready, outside the commanded
     * COMMAND_MOVE_TO_READY_POS state machine and outside emergency recovery.
     * Used to scope `--dont_go_to_home_pos` to startup: explicit move-to-ready
     * commands and emergency-recovery homing must always execute.
     */
    bool is_startup_ready_phase() const {
        return move_to_ready_cmd_state_ == MoveToReadyCmdState::IDLE && !is_in_emergency_recovery();
    }

    /*!
     * @brief Snapshot the current emergency-recovery state.
     */
    EmergencyRecoveryState emergency_recovery_state() const {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        return emergency_state_;
    }

    /*!
     * @brief Whether the device is performing ANY kind of move-to-ready right now.
     *
     * True during:
     *   - initial startup ready-move (is_ready_ has not been set true yet),
     *   - commanded MOVE_TO_READY_AND_STOP (move_to_ready_cmd_state_ != IDLE),
     *   - emergency recovery (emergency_state_ != NONE).
     *
     * Safety checks that compare against the leader's cached target (e.g.
     * POS_BEHIND) must skip themselves when this returns true: during
     * move-to-ready the leader is paused (or never spoke yet) and the cached
     * tele_pos would spuriously trip the check as the joint follows the
     * internally generated ready trajectory.
     */
    bool is_in_any_move_to_ready_state() const {
        if (is_ready_ == false) return true;
        if (move_to_ready_cmd_state_ != MoveToReadyCmdState::IDLE) return true;
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        return emergency_state_ != EmergencyRecoveryState::NONE;
    }

    /*!
     * @brief Whether emergency-recovery move-to-ready should use the ERROR velocity.
     *
     * Concrete devices (DeviceArm, DeviceEffector) read this in their
     * ``move_to_ready_position()`` via ``ready_move_step_rad()`` to pick the
     * slower ERROR step size instead of the NORMAL one.
     */
    bool is_slow_move_active() const {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        return slow_move_active_;
    }

    /*!
     * @brief Per-loop position step for the active move-to-ready (radians per loop).
     *
     * Used by every concrete device's ``move_to_ready_position()`` to bound the
     * angular speed regardless of how far the arm is from home. NORMAL speed is
     * used by startup / command-driven ready moves; ERROR speed (~1.5x slower)
     * is used while ``slow_move_active_`` is true (emergency recovery).
     */
    float ready_move_step_rad() const {
        const float vel = is_slow_move_active() ? cla_.move_to_ready_vel_rad_s_error
                                                : cla_.move_to_ready_vel_rad_s_normal;
        const float freq = static_cast<float>(cla_.control_frequency);
        const float dt = (freq > 0.0f) ? (1.0f / freq) : (1.0f / 50.0f);  // fallback to 50 Hz if cla unset
        return vel * dt;
    }

    /*!
     * @brief Returns the failed joint set so concrete devices can skip them.
     *
     * Best-effort policy: joints in this set should not receive new commands
     * during the slow move-to-ready. Concrete devices may add ids here as
     * additional joints fail.
     */
    std::unordered_set<int16_t> failed_joint_ids_snapshot() const {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        return failed_joint_ids_;
    }

    /*!
     * @brief Records a newly observed joint failure during recovery (best-effort).
     */
    void mark_joint_failed_during_recovery(int16_t joint_id) {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        failed_joint_ids_.insert(joint_id);
    }

    /*!
     * @brief Records the joint id that caused the most recent ``read_hardware_values()`` failure.
     *
     * Called from concrete devices' read paths so that ``pi_control_node`` can pass the actual
     * failed joint into ``enter_emergency_recovery()`` instead of a placeholder -1. ``-1`` means
     * "unknown" (e.g. the failure was at the driver/bus level, not a specific joint).
     */
    void set_last_failed_joint_id(int joint_id) {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        last_failed_joint_id_ = joint_id;
    }

    /*!
     * @brief Returns the joint id recorded by ``set_last_failed_joint_id`` (or -1 if none).
     */
    int last_failed_joint_id() const {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        return last_failed_joint_id_;
    }

    /*!
     * @brief Marks the emergency recovery as completed (ready position reached).
     *
     * Concrete devices should call this from their ``move_to_ready_position()``
     * once the arm has reached ready, or when no further joints can be moved.
     * Publishes ``DEVICE_INFO_SHUTDOWN_AFTER_ERROR`` and stops the topic so
     * the main loop exits cleanly.
     */
    virtual void mark_emergency_recovery_completed();

    /*!
     * @brief Configures joint/servo control modes for a given role and intent.
     *
     * - `NORMAL_OPERATION`: devices set their standard leader/follower control modes.
     * - `READY_MOVE_OVERRIDE`: devices should switch to a safe position-based mode suitable for executing
     *   `move_to_ready_position()` from the current pose.
     */
    virtual ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) = 0;

    /*!
     * @brief Moves a joint to the target position.
     * @param p_joint Pointer to the joint.
     * @param target_pos Target position in relative radians.
     * @param target_tor Target torque in Nm.
     * @param safe_mode_derating Derating factor (0.0 to 1.0).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode move(Joint* p_joint, float target_pos, float target_tor, float safe_mode_derating);

    /*!
     * @brief Sets the device operation mode.
     * @param new_mode New operation mode.
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode set_mode(DeviceMode new_mode) {
        mode_ = new_mode;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Gets the current operation mode.
     * @return Current DeviceMode.
     */
    virtual DeviceMode get_mode() { return mode_; }

    /*!
     * @brief Gets the model name.
     * @return Model name string.
     */
    virtual std::string get_model() { return model_; }

    /*!
     * @brief Gets the device ID.
     * @return Device ID string.
     */
    virtual std::string get_id() { return id_; }

    /*!
     * @brief Factory method to create a new device instance.
     * @param cfg_model Device configuration for the model.
     * @param cfg_individual Device configuration for the individual device.
     * @param cla Command-line arguments.
     * @return Pointer to the newly created Device instance.
     */
    static Device* new_device(const DeviceConfig& cfg_model, const DeviceConfig& cfg_individual,
                              const CommandLineArgs& cla);

    /*!
     * @brief Gets the device type.
     * @return DeviceType enum value.
     */
    DeviceType get_device_type() { return type_; }

    /*!
     * @brief Gets the device role in teleoperation.
     * @return Role enum value.
     */
    Role get_device_role() { return role_; }

    /*!
     * @brief Gets the current message type for teleoperation.
     * @return MsgType enum value.
     */
    MsgType get_device_message_type() { return msg_type_; }

    /*!
     * @brief Gets the total degrees of freedom including peripheral devices.
     * @return Total DOF including peripherals.
     */
    int get_dof_total() { return dof_total_; }

    /*!
     * @brief Gets the degrees of freedom of this device (excluding peripherals).
     * @return DOF of this device only.
     */
    int get_dof() { return dof_; }

    /*!
     * @brief Gets the name of the joystick topic.
     * @return Topic name string.
     */
    std::string get_topic_joystick_name() { return topic_joystick_name_; }

    /*!
     * @brief Sets the name of the joystick topic.
     * @param name Topic name string.
     * @return ReturnCode::SUCCESS if successful.
     */
    ReturnCode set_topic_joystick_name(std::string name) {
        topic_joystick_name_ = name;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Gets the total number of servos including peripheral devices.
     * @return Total number of servo motors including peripherals.
     */
    int get_servo_num_total() { return servo_num_total_; }

    /*!
     * @brief Gets the number of servos in this device (excluding peripherals).
     * @return Number of servo motors in this device only.
     */
    int get_servo_num() { return servo_num_; }

    /*!
     * @brief Enables or disables spring effect.
     * @param enable True to enable, false to disable.
     * @return ReturnCode::SUCCESS if successful.
     */
    virtual ReturnCode enable_spring_effect(bool enable) {
        enabled_spring_effect_ = enable;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Gets the current status of spring effect.
     * @return True if enabled, false if disabled.
     */
    virtual bool is_spring_effect_enabled() { return enabled_spring_effect_; }

    /*!
     * @brief Checks if the device is ready for operation.
     * @return True if ready, false otherwise.
     */
    virtual bool is_ready() { return is_ready_; }


    /*!
     * @brief Checks if the device is read only.
     * @return True if read only, false otherwise.
     */
    virtual bool is_read_only() { return is_read_only_; }

    /*!
     * @brief Forces the device to be marked as ready.
     */
    virtual void force_to_be_ready() { is_ready_ = true; }

    /*!
     * @brief Checks if force feedback is enabled.
     * @return True if enabled, false otherwise.
     */
    virtual bool is_force_feedback_enabled() {
        return role_ == Role::LEADER && enabled_force_feedback_ && cla_.force_feedback >= 0.0f;
    }

    virtual ReturnCode set_runtime_force_feedback(bool enabled, float gain);

    virtual ReturnCode set_runtime_force_feedback_gain(float gain);

    virtual ReturnCode runtime_hold();

    /*!
     * @brief Records a client-liveness heartbeat (DEVICE_COMMAND_HEARTBEAT).
     *
     * The watchdog only arms after the first heartbeat is seen, so clients
     * that never send heartbeats (older library versions) keep the previous
     * behavior. Called from the lifecycle command dispatch, which runs on the
     * control thread (Topic::step inside Device::step), same as the check.
     */
    void note_client_heartbeat() {
        client_heartbeat_seen_ = true;
        client_watchdog_tripped_ = false;
        last_client_heartbeat_ = std::chrono::steady_clock::now();
    }

    bool rejects_direct_commands() const {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        return emergency_state_ != EmergencyRecoveryState::NONE ||
               move_to_ready_cmd_state_ != MoveToReadyCmdState::IDLE;
    }

    /*!
     * @brief Gets the trajectory planning type.
     * @return TrajectoryPlanningType enum value.
     */
    virtual TrajectoryPlanningType get_planning_type() { return planning_type_; }

    /*!
     * @brief Gets the control loop frequency in Hz.
     * @return Control frequency in Hz.
     */
    virtual int get_frequency() { return control_frequency_; }

    /*!
     * @brief Gets the command-line arguments.
     * @return Constant reference to the CommandLineArgs object.
     */
    virtual const CommandLineArgs& get_cla() { return cla_; }

    /*!
     * @brief Publishes joystick information.
     * @param msg Joystick information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode publish_joystick(const MsgJoystick& msg) {
        if (p_topic_ == nullptr) {
            PI_ERROR("Device: Topic is not initialized in publish_joystick(const MsgJoystic& msg)");
            return ReturnCode::NOT_INITIALIZED;
        }
        return p_topic_->publish(msg);
    }

    /*!
     * @brief Converts a normalized value to an actual value with clamping.
     * @param normalized_value Normalized value (typically in range [-1.0, 1.0]).
     * @param max Maximum value for the conversion.
     * @return Converted value in the range [-max, max].
     */
    virtual float normalized_to_value(float normalized_value, float max) {
        float converted = max * normalized_value;
        if (fabs(converted) > max) {
            converted = (normalized_value > 0) ? max : -max;
        }
        return converted;
    }

   protected:
    bool is_normal_status_        = true;   ///< Flag indicating normal operational status.
    bool is_ready_                = false;  ///< Flag indicating device is ready for operation.
    bool is_driver_created_by_this_ = false;  ///< Flag indicating driver was created by this device.
    bool is_topic_created_by_this_  = false;  ///< Flag indicating topic was created by this device.
    bool is_read_only_               = false;  ///< Flag indicating device is read only.

    std::shared_ptr<Driver> p_driver_ = nullptr;  ///< Driver instance for hardware communication (can be nullptr).
    std::shared_ptr<Topic> p_topic_   = nullptr;  ///< Topic instance for communication (can be nullptr).
    std::unique_ptr<Algo> p_algo_     = nullptr;  ///< Owned algorithm instance for kinematics and dynamics (nullable).
    const DeviceConfig* p_config_model_      = nullptr;  ///< Device model configuration.
    const DeviceConfig* p_config_individual_ = nullptr;  ///< Device individual configuration.
    const std::string model_;  ///< Model name of the device.
    const std::string id_;     ///< Device ID string.

    Role role_ = Role::UNKNOWN;  ///< Role in teleoperation (LEADER, FOLLOWER, or UNKNOWN).

    DeviceMode mode_ = DeviceMode::NORMAL;  ///< Current device operation mode.
    DeviceType type_ = DeviceType::UNKNOWN;  ///< Type of the device.

    bool enabled_spring_effect_  = false;  ///< Flag indicating spring effect is enabled.
    bool enabled_force_feedback_ = true;   ///< Flag indicating force feedback is enabled.

    int dof_       = 0;  ///< DOF of this device only, excluding peripherals.
    int servo_num_ = 0;  ///< Number of servo motors in this device only, excluding peripherals.

    int dof_total_       = 0;  ///< Total DOF including attached peripheral devices.
    int servo_num_total_ = 0;  ///< Total number of servo motors including peripherals.

    int init_count_  = 0;  ///< Counter tracking control loop iterations during initial movement.
    int reached_cnt_ = 0;  ///< Counter tracking consecutive iterations where device reached desired position.

    long ready_move_budget_total_ = 0;  ///< Iteration budget for the active move-to-ready (0 = uninitialised).
    long ready_move_budget_used_  = 0;  ///< Iterations consumed by the active move-to-ready.

    /*!
     * @brief Initialises the move-to-ready iteration budget from the actual travel distance.
     *
     * Call from the concrete device's move_to_ready_position() init block.
     * Budget = READY_MOVE_BUDGET_TRAVEL_SCALE x (max joint displacement / per-loop step), with a
     * READY_MOVE_BUDGET_MIN_ITERS floor, so it scales with distance and only fires when the move
     * is far beyond its velocity-bounded travel time (e.g. a joint oscillating against gravity
     * resetting the stuck/arrival counters forever -- issue #5).
     */
    void ready_move_budget_init(float max_displacement_rad, float step_rad) {
        const float step = (step_rad > 1e-9f) ? step_rad : 1e-9f;
        const long travel_iters = static_cast<long>(max_displacement_rad / step) + 1;
        ready_move_budget_total_ = std::max(
            static_cast<long>(READY_MOVE_BUDGET_MIN_ITERS),
            READY_MOVE_BUDGET_TRAVEL_SCALE * travel_iters + DEVICE_ARRIVED_CONFIRM_CNT);
        ready_move_budget_used_ = 0;
    }

    /*!
     * @brief Consumes one move-to-ready iteration; true once the budget is exhausted.
     */
    bool ready_move_budget_spend() {
        ++ready_move_budget_used_;
        return ready_move_budget_total_ > 0 && ready_move_budget_used_ > ready_move_budget_total_;
    }

    MovingMode moving_mode_ = MovingMode::INVALID;  ///< Strategy for moving joints (SEQUENTIAL or PARALLEL).
    TrajectoryPlanningType planning_type_;  ///< Trajectory planning type.

    MsgType msg_type_ = MsgType::INVALID;  ///< Message type for teleoperation communication.

    int control_frequency_ = 0;  ///< Control loop frequency in Hz.

    ControlType control_type_ = ControlType::SERVO_DIRECT;  ///< Control method for servo control.

    std::string topic_joystick_name_;  ///< Name of the joystick topic.

    CommandLineArgs cla_;  ///< Command-line arguments.

    /*!
     * @brief Internal state machine for COMMAND_MOVE_TO_READY_POS.
     */
    enum class MoveToReadyCmdState {
        IDLE = 0,
        REQUESTED,
        MOVING,
        RESTORE
    };

    MoveToReadyCmdState move_to_ready_cmd_state_ = MoveToReadyCmdState::IDLE;
    int move_to_ready_request_id_ = 0;            ///< Lifecycle request currently moving to ready.
    int completed_move_to_ready_request_id_ = 0;  ///< Request id attached to READY_NOW completion.

    bool stop_after_ready_ = false;  ///< When true, call p_topic_->stop() once move-to-ready completes.

    /// One-shot log latch for the leader's recovery publish gate in step().
    bool logged_recovery_publish_gate_ = false;

    // --- Client-liveness watchdog (see note_client_heartbeat) ---
    // A client that dies abruptly (kill -9, host crash) leaves the nodes
    // running unsupervised: the leader keeps driving the follower through the
    // live stream indefinitely. Once heartbeats have been observed, losing
    // them for kClientHeartbeatTimeoutMs drops the device to its safe idle:
    // leaders fall back to gravity compensation, followers pause leader
    // listening and hold. All fields are control-thread only.
    static constexpr int kClientHeartbeatTimeoutMs = 5000;
    bool client_heartbeat_seen_ = false;
    bool client_watchdog_tripped_ = false;
    std::chrono::steady_clock::time_point last_client_heartbeat_;

    /// Runs once per step() on the control thread; trips at most once per
    /// silence episode (a fresh heartbeat re-arms).
    void check_client_heartbeat_watchdog();

    // Emergency recovery state machine. Accessed from the main step() thread and from
    // the Topic ZMQ command callback thread, so the fields are guarded by emergency_mutex_.
    mutable std::mutex emergency_mutex_;  ///< Guards all emergency_* / failed_joint_ids_ members.
    EmergencyRecoveryState emergency_state_ = EmergencyRecoveryState::NONE;
    bool slow_move_active_ = false;       ///< When true, ready_move_step_rad() returns the ERROR-speed step size.
    int emergency_cause_ = 0;             ///< ReturnCode value (as int) recorded at trigger time.
    int emergency_failed_joint_id_ = -1;  ///< First failed joint id (-1 if unknown).
    int last_failed_joint_id_ = -1;       ///< Most recent failed joint id from read_hardware_values (-1 if unknown).
    std::unordered_set<int16_t> failed_joint_ids_;  ///< Joints that should be skipped during recovery.
    std::chrono::steady_clock::time_point emergency_start_time_;  ///< Set when entering MOVING_SLOW.
    std::chrono::steady_clock::time_point last_recovery_progress_publish_;  ///< Throttle timer for progress publishes.
    std::chrono::steady_clock::time_point last_step_heartbeat_;  ///< Refreshed at the end of every successful step() in MOVING_SLOW.
    bool emergency_completed_published_ = false;  ///< Guards single publish of SHUTDOWN_AFTER_ERROR.

    /*!
     * @brief Periodic publish of ``DEVICE_INFO_RECOVERY_IN_PROGRESS``.
     *
     * Called from ``step()`` every ``EMERGENCY_PROGRESS_PUBLISH_INTERVAL_MS`` while
     * the device is in ``MOVING_SLOW``. ``progress`` is in ``[0.0, 1.0]``.
     */
    void publish_recovery_progress(float progress);

    /*!
     * @brief Periodic publish of ``DEVICE_INFO_READY_MOVE_IN_PROGRESS``.
     *
     * Called from ``step()`` every ``EMERGENCY_PROGRESS_PUBLISH_INTERVAL_MS`` whenever
     * any move-to-ready is active (startup, command-driven, or emergency recovery).
     * Lets the Python side wait unbounded for ``DEVICE_INFO_READY_NOW`` while still
     * detecting genuine hangs (no heartbeat for a configurable stale threshold).
     *
     * @param source  One of ``READY_MOVE_SOURCE_{STARTUP,COMMAND,EMERGENCY}``.
     * @param is_error true when running at ERROR speed (emergency), false for NORMAL.
     * @param progress Best-effort completion ratio in ``[0.0, 1.0]``.
     */
    void publish_ready_move_progress(int source, bool is_error, float progress);

    std::chrono::steady_clock::time_point last_ready_move_progress_publish_;  ///< Throttle timer for READY_MOVE_IN_PROGRESS publishes.

    /*!
     * @brief Updates ``emergency_state_`` to ``TIMED_OUT`` if the step heartbeat is stale.
     *
     * Heartbeat-based watchdog: as long as ``step()`` keeps refreshing
     * ``last_step_heartbeat_`` we keep waiting; if the gap exceeds
     * ``cla_.emergency_shutdown_timeout_ms`` the control loop has plausibly hung
     * (deadlock or blocked device I/O) and we force-park. Returns true if
     * (and only if) the state transitioned this call.
     */
    bool check_emergency_timeout_locked();

    /*!
     * @brief Best-effort estimate of how far any active move-to-ready has progressed.
     *
     * Returns a value in ``[0.0, 1.0]`` (clamped by callers). The base implementation
     * returns ``0.0`` (indeterminate) -- concrete devices (e.g. DeviceArm) should
     * override with a meaningful ratio derived from the actual joint displacements
     * toward home. Used by both the legacy ``RECOVERY_IN_PROGRESS`` publish and the
     * unified ``READY_MOVE_IN_PROGRESS`` publish; UI-only, has no effect on the
     * heartbeat-based timeout.
     */
    virtual float get_ready_move_completion_ratio() const { return 0.0f; }

    static constexpr int EMERGENCY_PROGRESS_PUBLISH_INTERVAL_MS = 200;  ///< Throttle for recovery progress publishes.

    /*!
     * @brief Resets ready-related state so `move_to_ready_position()` restarts from the current pose.
     *
     * Derived classes should override to reset additional state (e.g., arm sequential index, per-joint step maps).
     */
    virtual void reset_ready_state_for_move_to_ready() {
        is_ready_ = false;
        init_count_ = 0;
        reached_cnt_ = 0;
    }

    /*!
     * @brief Clears teleoperation command buffers to avoid \"jump\" after move-to-ready.
     *
     * Follower devices store the last received leader targets (tele_pos/vel/tor) and may also keep an active
     * interpolation segment (precomputed waypoints). After COMMAND_MOVE_TO_READY_POS finishes and we return to
     * normal operation, these stale targets can immediately pull the robot away from home, causing a sudden jump.
     *
     * Concrete devices override this to:
     * - overwrite tele targets with the current measured position (hold),
     * - and reset any interpolation state.
     */
    virtual void clear_command_buffers_for_move_to_ready() {}
};
