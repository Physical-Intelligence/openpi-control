/*!
 * @file pi_servo_dm.hpp
 * @brief ServoDm class for managing DM-type servos via CAN interface.
 */

#pragma once
#include "pi_driver_arx.hpp"
#include "pi_servo.hpp"

/*!
 * @brief Parameter container class for DM servo configuration limits.
 */
class ServoDmParam : public ServoParam {
   public:
    float kp_min_;  ///< Minimum allowed proportional gain for position control.
    float kp_max_;  ///< Maximum allowed proportional gain for position control.
    float kd_min_;  ///< Minimum allowed derivative gain for position control.
    float kd_max_;  ///< Maximum allowed derivative gain for position control.
    float pos_min_;  ///< Minimum position limit in radians.
    float pos_max_;  ///< Maximum position limit in radians.
    float vel_min_;  ///< Minimum velocity limit in rad/sec.
    float vel_max_;  ///< Maximum velocity limit in rad/sec.
    float tor_min_;  ///< Minimum torque limit in Nm.
    float tor_max_;  ///< Maximum torque limit in Nm.

    /*!
     * @brief Constructor.
     * @param kp_min Minimum proportional gain.
     * @param kp_max Maximum proportional gain.
     * @param kd_min Minimum derivative gain.
     * @param kd_max Maximum derivative gain.
     * @param pos_min Minimum position limit in radians.
     * @param pos_max Maximum position limit in radians.
     * @param vel_min Minimum velocity limit in rad/sec.
     * @param vel_max Maximum velocity limit in rad/sec.
     * @param tor_min Minimum torque limit in Nm.
     * @param tor_max Maximum torque limit in Nm.
     * @param tolerable_pos_difference_rad Tolerable position difference threshold in radians.
     * @param max_pos_difference_rad Maximum position difference threshold in radians.
     * @param velocity_threshold_rad_sec Velocity threshold in rad/sec.
     */
    ServoDmParam(float kp_min, float kp_max, float kd_min, float kd_max, float pos_min, float pos_max, float vel_min,
                 float vel_max, float tor_min, float tor_max, float tolerable_pos_difference_rad,
                 float max_pos_difference_rad, float velocity_threshold_rad_sec)
        : ServoParam(tolerable_pos_difference_rad, max_pos_difference_rad, velocity_threshold_rad_sec),
          kp_min_(kp_min),
          kp_max_(kp_max),
          kd_min_(kd_min),
          kd_max_(kd_max),
          pos_min_(pos_min),
          pos_max_(pos_max),
          vel_min_(vel_min),
          vel_max_(vel_max),
          tor_min_(tor_min),
          tor_max_(tor_max) {}
};

/*!
 * @brief Manages DM-type servo motors controlled via CAN interface.
 */
class ServoDm : public Servo {
   public:
    /*!
     * @brief Operation modes supported by DM servos.
     */
    enum class OperationMode {
        MIT = 1, ///< Position, velocity, and torque command with PD control.
    };

    /*!
     * @brief Register addresses for DM servo configuration and control.
     */
    enum class RegAddr {
        TIMEOUT       = 9,  ///< CAN communication-loss protection window (uint32, 50 us per count).
        CONTROL_MODE = 10, ///< Register address for setting/reading control mode.
    };

    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device object.
     * @param p_joint Pointer to the Joint object.
     * @param p_driver Pointer to the Driver object (must be a CAN driver).
     */
    ServoDm(Device* p_device, Joint* p_joint, Driver* p_driver);

    /*!
     * @brief Destructor.
     */
    ~ServoDm();

    /*!
     * @brief Safely parks the servo before shutdown.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode park_safely() override;

    /*!
     * @brief Initializes the servo with model configuration.
     * @param joint_config JSON object containing the joint model configuration.
     * @param p_config Pointer to the device model configuration object.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode init_config_model(const json& joint_config, const DeviceConfig* p_config) override;

    /*!
     * @brief Initializes the servo with individual configuration.
     * @param servo_config JSON object containing the individual servo configuration.
     * @param p_config Pointer to the device configuration object.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode init_config_individual(const json& servo_config, const DeviceConfig* p_config) override;

    /*!
     * @brief Gets the servo's current position as raw servo value.
     * @return Current position in absolute radian.
     */
    virtual float get_pos_servo() override { return (float)curr_pos_abs_; }

    /*!
     * @brief The position kp actually sent in position frames: ServoDm
     *        substitutes SERVO_DM_DEFAULT_KP when pos_kp_ is configured 0, so
     *        torque-window logic must see that default, not the raw config.
     * @return Effective position proportional gain (Kp).
     */
    virtual float get_effective_pos_kp() const override;

    /*!
     * @brief Starts the servo hardware and enables control.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode start_hardware() override;

    /*!
     * @brief Age of the newest parsed CAN frame for this servo, from the
     *        DriverArx receive cache (``last_update_perf_``). Note this is
     *        bus liveness, not position freshness: some frame types (ENCOS
     *        config acks, non-position ack statuses) refresh the stamp
     *        without updating the cached position.
     * @return Frame age in milliseconds, or -1 when no frame was ever parsed.
     */
    prof_time_msec_t get_frame_age_ms() override;

    /*!
     * @brief Confirms that ``received_servo_data_`` for this servo has been
     *        populated by at least one parsed status frame. DM motors do not
     *        broadcast status spontaneously, so without a successful
     *        ``DriverArx::enable()`` response parse the cache stays zero and
     *        ``curr_pos_abs_`` would be a stale 0. The check looks at
     *        ``motor_id_`` which is zero only when the cache has never been
     *        touched (real DM IDs are 1+).
     * @return ReturnCode::SUCCESS if the cache holds parsed data,
     *         ReturnCode::FAIL otherwise.
     */
    ReturnCode verify_position_fresh() override;

    /*!
     * @brief Confirms the servo is in a healthy, enabled state right before
     *        the control loop starts. A latched DM error (e.g. 0xD
     *        communication loss left over from a previous session) silently
     *        disables the motor while position feedback keeps flowing, so
     *        the joint would collapse under gravity the moment commands
     *        start. Attempts one re-enable for DM servos before failing.
     * @return ReturnCode::SUCCESS if the servo reports a healthy state,
     *         otherwise an error code.
     */
    ReturnCode verify_operational() override;

    /*!
     * @brief Moves the servo to the target position.
     * @param target_pos Target position in radians.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos) override;

    /*!
     * @brief Moves the servo to the target position with specified velocity and torque.
     * @param target_pos Target position in radians.
     * @param target_vel Target velocity in rad/sec.
     * @param target_torq Target torque limit in Nm.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos, float target_vel, float target_torq) override;

    /*!
     * @brief Applies the specified torque to the servo.
     * @param torque Torque to be applied in Nm.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode apply_torque(float torque) override;

    /*!
     * @brief Applies torque with the configured derivative damping gain.
     * @param torque Torque to be applied in Nm.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode apply_torque_with_damping(float torque) override;

    /*!
     * @brief Reads current sensor values from the servo hardware.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode read_hardware_values() override;

    /*!
     * @brief Parses a DM servo status message from a CAN frame.
     * @param p_frame Pointer to the received CAN frame.
     * @param p_received_servo_data Pointer to the buffer where parsed servo data will be stored.
     * @param p_find_data_index Function pointer to find the data buffer index for a given servo ID.
     * @param p_driver_arx Pointer to the CAN driver object.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode parse_dm_servo_status(DriverCan::can_frame_t* p_frame, ReceivedServoData* p_received_servo_data,
                                            DriverArx::func_find_data_index_t p_find_data_index,
                                            DriverArx* p_driver_arx);

    /*!
     * @brief Parses an Encos servo status message from a CAN frame.
     * @param p_frame Pointer to the received CAN frame.
     * @param p_received_servo_data Pointer to the buffer where parsed servo data will be stored.
     * @param p_find_data_index Function pointer to find the data buffer index for a given servo ID.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode parser_encos_servo_status(DriverCan::can_frame_t* p_frame,
                                                ReceivedServoData* p_received_servo_data,
                                                DriverArx::func_find_data_index_t p_find_data_index);

    /*!
     * @brief Constructs a CAN command frame for DM servo control.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor (0-15).
     * @param kp Proportional gain for position control.
     * @param kd Derivative gain for position control.
     * @param pos Target position in radians.
     * @param spd Target velocity in rad/sec.
     * @param tor Target torque in Nm.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_command_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id, float kp,
                                                    float kd, float pos, float spd, float tor);

    /*!
     * @brief Constructs a CAN frame for writing a register value to DM servo.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor (0-15).
     * @param register_addr The register address to write.
     * @param register_value The value to write to the register.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_write_register_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                           RegAddr register_addr, uint32_t register_value);

    /*!
     * @brief Constructs a CAN frame to enable or disable the motor.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor (0-15).
     * @param enable_flag True to enable the motor, false to disable it.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_enable_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                   bool enable_flag);

    /*!
     * @brief Constructs a CAN frame to reset the motor.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor (0-15).
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_reset_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id);

    /*!
     * @brief Constructs a CAN frame to set the zero position.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor (0-15).
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_set_zero_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id);

    /*!
     * @brief Constructs a CAN frame to set the operation mode.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor (0-15).
     * @param operation_mode The operation mode to set.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_set_operation_mode_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                               OperationMode operation_mode);

    /*!
     * @brief Constructs a CAN command frame for Encos servo control.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor.
     * @param kp Proportional gain for position control.
     * @param kd Derivative gain for position control.
     * @param pos Target position in radians.
     * @param spd Target velocity in rad/sec.
     * @param tor Target torque in Nm.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_command_encos_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id, float kp,
                                                       float kd, float pos, float spd, float tor);

    /*!
     * @brief Constructs an ENCOS config-set frame that writes the CAN
     *        communication-loss heartbeat window.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor.
     * @param timeout_ms Heartbeat window in milliseconds (0 disables the protection).
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_set_can_timeout_encos_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                               uint16_t timeout_ms);

    /*!
     * @brief Constructs an ENCOS config-query frame for the current CAN
     *        communication-loss heartbeat window.
     * @param can_frame Reference to the CAN frame structure to be filled.
     * @param motor_id The CAN ID of the target motor.
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode can_frame_to_get_can_timeout_encos_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id);

    /*!
     * @brief Parses the ACK_QUERY reply to a CAN-timeout query.
     * @param can_frame The received reply frame.
     * @param motor_id Expected motor CAN ID.
     * @param timeout_ms Out: current heartbeat window in milliseconds.
     * @return ReturnCode::SUCCESS when the frame is a matching reply, FAIL otherwise.
     */
    static ReturnCode parse_can_timeout_reply_encos_servo(const DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                          uint16_t& timeout_ms);

   protected:
    /*!
     * @brief Initializes the current estimation system for the DM servo.
     * @param servo_model Servo model string.
     * @param p_config Pointer to the device configuration object.
     * @return ReturnCode indicating success or failure.
     */
    virtual ReturnCode init_current_estimation(std::string& servo_model, const DeviceConfig* p_config) override;

    ReturnCode latch_effector_thermal_fault(uint8_t status_code, const char* description, const char* action,
                                            const char* trigger);
    ReturnCode reject_if_thermal_fault_latched() const;

    DriverArx* p_driver_can_ = nullptr;  ///< Pointer to the CAN driver (cast from base Driver pointer).
    HoldChecker checker_motor_no_response_;  ///< Checker for detecting motor communication failures.
    bool motor_moved_ = false;  ///< Flag indicating whether the motor has moved from its initial position.
    uint8_t last_reported_fault_code_ = 0;  ///< Last DM fault logged, to avoid repeating it every control loop.
    bool thermal_fault_latched_ = false;  ///< Thermal effector fault: output was disabled and must not be re-enabled.
};
