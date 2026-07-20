/*!
 * @file pi_servo.hpp
 * @brief Servo base class and servo type enumeration for the PI Control system.
 */

#pragma once
#include <cstdint>
#include <string>

#include "pi_current_estimation.hpp"
#include "pi_device.hpp"
#include "pi_driver.hpp"
#include "pi_hold_checker.hpp"
#include "pi_profile.hpp"
#include "pi_safe_mode.hpp"

/*!
 * @brief Enumeration of supported servo types and input devices.
 */
enum class ServoType {
    NOT_SUPPORTED = 0,   ///< Unsupported or unknown servo type.

    ENCOS_A4310   = 101, ///< Encos A4310 servo (CAN protocol).

    DM_4340       = 151, ///< Dinamo DM J4340 servo (CAN protocol).
    DM_4310       = 152, ///< Dinamo DM J4310 servo (CAN protocol).

    CAN_PASSIVE_ENCODER = 701 ///< Passive CAN trigger encoder (YAM teaching handle): request/response poll, read-only, never commanded.
};

/*!
 * @brief Parameter container class for servo safety and teleoperation thresholds.
 */
class ServoParam {
   public:
    float tolerable_pos_difference_rad_; ///< Tolerable position error used by safety checks.
    float max_pos_difference_rad_;       ///< Maximum allowed position difference threshold between leader and follower in radians (triggers safe mode).
    float velocity_threshold_rad_sec_;    ///< Velocity threshold in rad/sec for detecting fast movements (used for force feedback gain adjustment).

    /*!
     * @brief Constructor.
     * @param tolerable_pos_difference_rad Tolerable position difference threshold in radians.
     * @param max_pos_difference_rad Maximum position difference threshold in radians.
     * @param velocity_threshold_rad_sec Velocity threshold in rad/sec.
     */
    ServoParam(float tolerable_pos_difference_rad, float max_pos_difference_rad, float velocity_threshold_rad_sec)
        : tolerable_pos_difference_rad_(tolerable_pos_difference_rad),
          max_pos_difference_rad_(max_pos_difference_rad),
          velocity_threshold_rad_sec_(velocity_threshold_rad_sec) {}
};

/*!
 * @brief Abstract base class for managing servo motor operations.
 */
class Servo {
   public:
    std::string servo_model_;                                 ///< Servo model name string (e.g., "DM J4310").
    int id_;                                                  ///< Servo ID (unique identifier, may not be sequential in some configurations).
    int data_index_;                                          ///< Data buffer index for group read/write operations (may differ from servo ID).
    float pos_kp_;                                            ///< Position control proportional gain (Kp).
    float pos_ki_;                                            ///< Position control integral gain (Ki).
    float pos_kd_;                                            ///< Position control derivative gain (Kd).
    float vel_kp_ = -1;                                       ///< Velocity control proportional gain (Kp, -1 means disabled).
    float vel_ki_ = -1;                                       ///< Velocity control integral gain (Ki, -1 means disabled).
    float kt_ = 0;                                            ///< Torque conversion constant: servo_value * kt_ = torque in Nm.
    float kv_ = 0;                                            ///< Velocity conversion constant: servo_value * kv_ = velocity in rad/sec.
    float ka_ = 0;                                            ///< Current conversion constant: servo_value * ka_ = current in mA.
    int dir_invert_ = 1;                                      ///< Direction inversion flag: 1 = normal direction, -1 = inverted direction.
    float zero_pos_abs_ = 0;                                  ///< Zero/home position in absolute radian (reference for relative coordinates).
    float position_wrap_period_ = 0;                           ///< Optional single-turn feedback period; 0 disables startup unwrapping.
    float position_wrap_offset_rel_ = 0;                       ///< Runtime whole-turn offset added in the relative position frame.
    bool position_wrap_initialized_ = false;                   ///< Whether the startup wrap offset has been selected.
    float spring_home_pos_rel_ = 0;                          ///< Spring equilibrium position in relative radian (where spring force is zero).
    float home_pos_rel_ = 0;                                 ///< Home position in relative radian (used for initial positioning and parking).
    float pos_min_rel_;                                       ///< Minimum position limit in relative radian.
    float pos_max_rel_;                                       ///< Maximum position limit in relative radian.
    bool pos_min_configured_ = false;                         ///< Whether pos_min was loaded from any configuration file.
    bool pos_max_configured_ = false;                         ///< Whether pos_max was loaded from any configuration file.
    float curr_pos_abs_ = 0;                                  ///< Current position in absolute radian (raw servo value).
    float curr_vel_ = 0;                                      ///< Current velocity in rad/sec.
    float curr_tor_ = 0;                                      ///< Current torque in Nm.
    float temperature_ = 0.0;                                ///< Servo temperature in degrees Celsius (°C).
    uint8_t motor_error_code_ = 0;                           ///< Native motor/drive error code (DM 4-bit or ENCOS 5-bit).
    float idc_current_ = 0;                                  ///< Estimated input DC current in amperes (A) for power monitoring.
    float response_delay_ = 0;                                ///< Response delay in seconds (accounts for communication latency and servo processing time).
    float pre_pos_abs_ = 0;                                   ///< Previous position in absolute radian (for change detection).
    float prev_pos_ = 0;                                      ///< Previous position in relative radian (for change detection).
    float prev_vel_ = 0;                                      ///< Previous velocity in rad/sec (for change detection).
    float prev_tor_ = 0;                                      ///< Previous torque in Nm (for change detection).
    MotorParams4CurrentEstimation motor_params_current_estimation_; ///< Motor parameters used for estimating input DC current from motor current.

    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device object.
     * @param p_joint Pointer to the Joint object.
     * @param p_driver Pointer to the Driver object for hardware communication.
     */
    Servo(Device* p_device, Joint* p_joint, Driver* p_driver);

    /*!
     * @brief Destructor.
     */
    virtual ~Servo();

    /*!
     * @brief Safely parks the servo before shutdown.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode park_safely() = 0;

    /*!
     * @brief Initializes the servo with model configuration.
     * @param servo_config_model JSON object containing the servo model configuration.
     * @param p_config Pointer to the device model configuration object.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode init_config_model(const json& servo_config_model, const DeviceConfig* p_config);

    /*!
     * @brief Initializes the servo with individual configuration.
     * @param servo_config JSON object containing the individual servo configuration.
     * @param p_config Pointer to the device configuration object.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode init_config_individual(const json& servo_config, const DeviceConfig* p_config);

    /*!
     * @brief Reads current sensor values from the servo hardware.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode read_hardware_values();

    /*!
     * @brief Verifies that the servo's cached position is fresh (i.e. populated
     *        from an actual hardware response, not still at the post-construction
     *        zero-initialised default). Used right after ``start_hardware()`` +
     *        ``read_hardware_values()`` to detect the case where no status frame
     *        ever made it into the cache, which would cause a wrong first command.
     *        Default implementation returns SUCCESS; subclasses that rely on
     *        asynchronous status caches must override.
     * @return ReturnCode::SUCCESS when the cached position is trustworthy,
     *         otherwise ReturnCode::FAIL.
     */
    virtual ReturnCode verify_position_fresh() { return ReturnCode::SUCCESS; }

    /*!
     * @brief Verifies that the servo is in a healthy, enabled state after
     *        ``start_hardware()`` + ``read_hardware_values()`` and before the
     *        control loop starts streaming commands. Subclasses whose
     *        hardware can latch a silent-disable error (e.g. the DM
     *        communication-loss trip 0xD) override this to check the error
     *        state and optionally attempt a one-shot recovery. Default
     *        implementation returns SUCCESS.
     * @return ReturnCode::SUCCESS when the servo is operational,
     *         otherwise an error code.
     */
    virtual ReturnCode verify_operational() { return ReturnCode::SUCCESS; }

    /*!
     * @brief Selects the unique configured whole-turn offset that places the
     *        current position inside the servo's raw relative limits.
     *
     * Single-turn DM feedback can return an equivalent angle one revolution
     * away after a power cycle. This is opt-in via ``position_wrap_period``;
     * when no wrapped candidate lies inside the configured range, the position
     * is left unchanged so the normal position-limit safety check can fire.
     * @return SUCCESS when disabled, initialized, or no candidate exists;
     *         INVALID_PARAM if the configured range is ambiguous.
     */
    ReturnCode initialize_position_wrap();

    /*!
     * @brief Starts the servo hardware and enables control.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode start_hardware() = 0;

    /*!
     * @brief Stops the servo hardware safely.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode stop_hardware() { return park_safely(); }

    /*!
     * @brief Moves the servo to the target position.
     * @param target_pos Target position in relative radian (relative to zero position).
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode move(float target_pos) = 0;

    /*!
     * @brief Moves the servo to the target position with specified velocity and torque.
     * @param target_pos Target position in relative radian (relative to zero position).
     * @param target_vel Target velocity in rad/sec.
     * @param target_tor Target torque limit in Nm.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode move(float target_pos, float target_vel, float target_tor) = 0;

    /*!
     * @brief Applies the specified torque to the servo.
     * @param torque Torque to be applied in Nm.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode apply_torque(float torque) = 0;

    /*!
     * @brief Applies torque while retaining the servo's configured derivative damping.
     *
     * Servo families that do not support a separate damping gain fall back to
     * their normal torque command. This is used by monopi-style gripper
     * control; arm gravity/torque commands continue to use apply_torque().
     *
     * @param torque Torque to be applied in Nm.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode apply_torque_with_damping(float torque) { return apply_torque(torque); }

    /*!
     * @brief Gets the servo's current position in relative radian.
     * @return Current position in relative radian.
     */
    float get_pos_rad_relative() {
        return (curr_pos_abs_ - zero_pos_abs_) * dir_invert_ + position_wrap_offset_rel_;
    }

    /*!
     * @brief Converts an absolute radian value to relative radian.
     * @param rad_absolute The absolute radian value to convert.
     * @return The converted relative radian value.
     */
    float get_pos_rad_relative(float rad_absolute) {
        return (rad_absolute - zero_pos_abs_) * dir_invert_ + position_wrap_offset_rel_;
    }

    /*!
     * @brief Gets the servo's current position in absolute radian.
     * @return Current position in absolute radian.
     */
    float get_pos_rad_absolute() { return curr_pos_abs_; }

    /*!
     * @brief Converts a relative radian value to absolute radian.
     * @param rad_relative The relative radian value to convert.
     * @return The converted absolute radian value.
     */
    float get_pos_rad_absolute(float rad_relative) {
        return (rad_relative - position_wrap_offset_rel_) * dir_invert_ + zero_pos_abs_;
    }

    /*!
     * @brief Gets the servo's current position as raw servo value.
     * @return Current position as raw servo value (units depend on servo type).
     */
    virtual float get_pos_servo() = 0;

    /*!
     * @brief Gets the servo's current velocity.
     * @return Current velocity in rad/sec.
     */
    virtual float get_vel_rad_sec() { return curr_vel_; }

    /*!
     * @brief Gets the servo's current torque.
     * @return Current torque in Nm.
     */
    virtual float get_tor_nm() { return curr_tor_; }

    /*!
     * @brief Gets the servo's current temperature.
     * @return Current temperature in degrees Celsius (°C).
     */
    virtual float get_temperature() { return temperature_; }

    /*!
     * @brief Checks if the servo position difference exceeds the maximum threshold.
     * @return true if position difference exceeds maximum threshold, false otherwise.
     */
    virtual bool is_behind_more_than_max_threshold() {
        float pos_difference = get_pos_rad_relative() - get_tele_pos_rad();
        return fabs(pos_difference) > p_servo_param_->max_pos_difference_rad_;
    }

    /*!
     * @brief Checks if the servo position difference exceeds the tolerable threshold.
     * @return true if position difference exceeds tolerable threshold, false otherwise.
     */
    virtual bool is_behind_more_than_tolerable_threshold() {
        float pos_difference = get_pos_rad_relative() - get_tele_pos_rad();
        return fabs(pos_difference) > p_servo_param_->tolerable_pos_difference_rad_;
    }

    /*!
     * @brief Gets the servo's maximum torque limit.
     * @return Maximum torque limit in Nm.
     */
    virtual float get_tor_max();

    /*!
     * @brief Gets the servo's position error margin.
     * @return Position error margin in radians.
     */
    virtual float get_pos_error_margin();

    /*!
     * @brief Changes the servo control mode for leader (teleoperation) operation.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode change_control_mode_for_leader() { return ReturnCode::SUCCESS; }

    /*!
     * @brief Changes the servo control mode for follower operation.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode change_control_mode_for_follower() { return ReturnCode::SUCCESS; }

    /*!
     * @brief Gets the servo's estimated input DC current.
     * @return Estimated input DC current in amperes (A).
     */
    virtual float get_idc_current() { return idc_current_; }

    /*!
     * @brief Age of the newest hardware feedback frame backing this servo's
     *        cached position.
     *
     * Servo types whose driver keeps a receive-time stamp (ServoDm via the
     * DriverArx cache) override this; the base returns -1 (unknown) so
     * publishers can tell "freshness not tracked" apart from a real age.
     *
     * @return Frame age in milliseconds, or -1 when the servo type does not
     *         track per-frame freshness or no frame was ever received.
     */
    virtual prof_time_msec_t get_frame_age_ms() { return -1; }

    /*!
     * @brief The position kp this servo actually sends in position frames.
     *
     * Subclasses that substitute a non-zero default when pos_kp_ is 0 (ServoDm
     * sends kp=10 in that case) must override this so torque-window logic like
     * Joint::clamp_target_to_grip_torque_limit reasons about the gain that
     * reaches the motor, not the configured value.
     *
     * @return Effective position proportional gain (Kp).
     */
    virtual float get_effective_pos_kp() const { return pos_kp_; }

    /*!
     * @brief Gets the adjusted position proportional gain.
     *
     * Bilateral force feedback weakens the leader's position spring
     * (kp = force_feedback_gain * pos_kp) so the operator can overpower it.
     * Move-to-ready and emergency recovery must NOT inherit that weak spring:
     * those moves have to carry the arm against gravity, and at gain 0.1-0.3
     * the wrist joints get kp 1-3, the move stalls, per-joint stuck detection
     * latches, and the device force-parks mid-recovery and falls. The ARX
     * family has no control-mode switch to escape through
     * (DeviceArmArx::set_control_mode is a no-op), so the escape lives here.
     *
     * @return Adjusted position proportional gain (Kp).
     */
    virtual float get_adjusted_pos_kp() {
        if (p_device_->is_force_feedback_enabled() && !is_device_in_move_to_ready_belong_to()) {
#if 0
            // Dynamic P gain adjustment based on the velocity
            float moved_distance = fabs(get_pos_rad_relative() - prev_pos_);
            float control_period = 1.0 / p_device_->get_frequency();
            float current_velocity = moved_distance / control_period;
            float ratio = (current_velocity >= p_servo_param_->velocity_threshold_rad_sec_)
                             ? 0.0
                             : 1.0 - (current_velocity / p_servo_param_->velocity_threshold_rad_sec_);
            return p_device_->get_cla().force_feedback * pos_kp_ * ratio;
#else
            return p_device_->get_cla().force_feedback * pos_kp_;
#endif

        }
        return pos_kp_;
    }

    /*!
     * @brief Gets the adjusted position derivative gain.
     *
     * i2rt bilateral teleoperation uses a weak position spring with zero
     * derivative gain. Keeping the full configured damping on a leader makes
     * normal hand motion feel much stiffer than the requested bilateral gain
     * implies. Move-to-ready and recovery retain the configured damping.
     *
     * @return Adjusted position derivative gain (Kd).
     */
    virtual float get_adjusted_pos_kd() {
        if (p_device_->is_force_feedback_enabled() && !is_device_in_move_to_ready_belong_to()) {
            return 0.0f;
        }
        return pos_kd_;
    }

    /*!
     * @brief Factory method to create Servo objects from configuration.
     * @param servo_config JSON object containing servo configuration list.
     * @param p_config_model Pointer to the device model configuration.
     * @param servos Output vector to store pointers to the created Servo objects.
     * @param p_device Pointer to the Device object that these servos belong to.
     * @param p_joint Pointer to the Joint object that these servos belong to.
     * @param p_driver Pointer to the Driver object for hardware communication.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode new_servos(const json& servo_config, const DeviceConfig* p_config_model,
                                 std::vector<std::unique_ptr<Servo>>& servos, Device* p_device, Joint* p_joint, Driver* p_driver);

    /*!
     * @brief Factory method to initialize Servo objects with individual configuration.
     * @param servo_config JSON object containing individual servo configuration list.
     * @param p_config_individual Pointer to the individual device configuration.
     * @param servos Vector of Servo pointers to initialize (created by new_servos()).
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode init_config_individual(const json& servo_config, const DeviceConfig* p_config_individual,
                                             std::vector<std::unique_ptr<Servo>>& servos);

    /*!
     * @brief Clips a value to a specified range.
     * @param value The value to be clipped.
     * @param min The minimum allowable value.
     * @param max The maximum allowable value.
     * @return The clipped value (clamped to [min, max]).
     */
    float clipping(float value, float min, float max) { return (value > max) ? max : ((value < min) ? min : value); }

    /*!
     * @brief Returns the device type of the device that this servo belongs to.
     * @return Device type of the parent device, or UNKNOWN if device is not set.
     */
    DeviceType get_device_type_belong_to() {
        if (p_device_ == nullptr) return DeviceType::UNKNOWN;
        return p_device_->get_device_type();
    }

    /*!
     * @brief Returns the role of the device that this servo belongs to.
     * @return Role of the parent device, or UNKNOWN if device is not set.
     */
    Role get_device_role_belong_to() {
        if (p_device_ == nullptr) return Role::UNKNOWN;
        return p_device_->get_device_role();
    }

    /*!
     * @brief Returns the message type of the device that this servo belongs to.
     * @return Message type of the parent device, or INVALID if device is not set.
     */
    MsgType get_device_message_type_belong_to() {
        if (p_device_ == nullptr) return MsgType::INVALID;
        return p_device_->get_device_message_type();
    }

    /*!
     * @brief Returns the device mode of the device that this servo belongs to.
     * @return Device mode of the parent device, or INVALID if device is not set.
     */
    DeviceMode get_device_mode_belong_to() {
        if (p_device_ == nullptr) return DeviceMode::INVALID;
        return p_device_->get_mode();
    }

    /*!
     * @brief Whether the parent device is currently inside any move-to-ready
     * sequence (startup, commanded MOVE_TO_READY_AND_STOP, or emergency
     * recovery).
     *
     * The leader-position based POS_BEHIND safety check uses the cached
     * `tele_pos_` as its reference. During move-to-ready that reference is
     * stale (the leader is paused / has never spoken yet) and would
     * spuriously fire as soon as the joint moved more than a few centimetres
     * away from its frozen tele_pos. Safety checks that depend on tele_pos
     * must skip themselves during move-to-ready and rely on the move-to-ready
     * state machine's own stuck/displacement tracking instead.
     *
     * @return true if the device is doing any kind of move-to-ready, false
     *         otherwise (or if no parent device is attached).
     */
    bool is_device_in_move_to_ready_belong_to() {
        if (p_device_ == nullptr) return false;
        return p_device_->is_in_any_move_to_ready_state();
    }

    /*!
     * @brief Collects servo IDs into a list.
     * @param servo_ids Output vector to store servo IDs.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode get_servo_ids(std::vector<int>& servo_ids) {
        servo_ids.push_back(id_);
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Gets the leader's teleoperation position for this servo.
     * @return Leader's teleoperation position in relative radian.
     */
    virtual float get_tele_pos_rad();

    /*!
     * @brief Gets the servo's encoder resolution.
     * @return Servo resolution (encoder counts per revolution).
     */
    virtual int get_servo_resolution() { return 1; }

    /*!
     * @brief Gets the safe teleoperation position with safety constraints applied.
     * @param tele_pos Requested teleoperation position in relative radian.
     * @return Safe teleoperation position in relative radian (clamped to safe limits if needed).
     */
    virtual float get_safe_tele_pos_rad(float tele_pos) { return safe_mode_.get_safe_tele_pos_rad(this, tele_pos); }

    /*!
     * @brief Gets the servo type.
     * @return Servo type enumeration value.
     */
    ServoType get_servo_type() { return type_; }

   protected:
    /*!
     * @brief Initializes the current estimation system for the servo.
     * @param servo_model Servo model string (e.g., "DM J4310", "XM430-W210").
     * @param p_config Pointer to the device configuration object.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode init_current_estimation(std::string& servo_model, const DeviceConfig* p_config) {
        (void)servo_model;
        (void)p_config;
        return ReturnCode::SUCCESS;
    }

    Driver* p_driver_ = nullptr;                    ///< Pointer to the Driver object for hardware communication.
    Device* p_device_ = nullptr;                   ///< Pointer to the Device object that this servo belongs to.
    Joint* p_joint_ = nullptr;                      ///< Pointer to the Joint object that this servo belongs to.
    bool parked_ = false;                           ///< Flag indicating whether this servo has been safely parked.
    ServoType type_ = ServoType::NOT_SUPPORTED;      ///< Servo type enumeration value identifying the specific servo model.
    HoldChecker checker_pos_exceed_;                 ///< Hold checker for detecting when servo position exceeds minimum or maximum limits.
    HoldChecker checker_vel_exceed_;                 ///< Hold checker for detecting when servo velocity exceeds the limit.
    HoldChecker checker_tor_exceed_;                 ///< Hold checker for detecting when servo torque exceeds the limit.
    bool torque_mode_disabled_warning_active_ = false;  ///< Guards one warning per sustained over-torque excursion.
    HoldChecker checker_pos_difference_exceed_;      ///< Hold checker for detecting when position difference between leader and follower exceeds the limit.
    HoldChecker checker_temperature_exceed_;         ///< Hold checker for detecting when servo temperature exceeds safe limits.
    HoldChecker checker_stall_detection_;            ///< Hold checker for detecting servo stall conditions (motor not moving despite torque command).
    CurrentEstimation current_estimation_;           ///< Current estimation system for calculating input DC current from motor current measurements.
    SafeMode safe_mode_;                             ///< Safe mode management system for handling safety violations and graceful degradation.
    const ServoParam* p_servo_param_ = nullptr;     ///< Constant pointer to the servo parameter object containing safety thresholds.
};
