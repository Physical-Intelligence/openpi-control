/*!
 * @file pi_servo_ft.hpp
 * @brief ServoFt class for managing FeeTech SMS/STS serial bus servos (STS3215 class).
 *
 * Covers the SO-ARM100/101 servos: FeeTech STS3215 and the register-compatible
 * Hiwonder HX-30HM / HX-10HM (SO-ARM101 kits), which all share the canonical
 * "FeeTech STS3215" servo model string. Single-turn position control with a
 * hardware Torque Limit register; there is no current control loop, so
 * apply_torque() fast-fails and torque-mode effectors approximate torque
 * limiting via move(pos, vel, tor).
 */

#pragma once
#include <cstdint>

#include "pi_driver_ft.hpp"
#include "pi_servo.hpp"

/*!
 * @brief Parameter container class for FeeTech servo configuration.
 */
class ServoFtParam : public ServoParam {
   public:
    /*!
     * @brief Constructor.
     * @param tolerable_pos_difference_rad Tolerable position difference threshold in radians.
     * @param max_pos_difference_rad Maximum position difference threshold in radians.
     * @param velocity_threshold_rad_sec Velocity threshold in rad/sec.
     */
    ServoFtParam(float tolerable_pos_difference_rad, float max_pos_difference_rad, float velocity_threshold_rad_sec)
        : ServoParam(tolerable_pos_difference_rad, max_pos_difference_rad, velocity_threshold_rad_sec) {}
};

/*!
 * @brief Manages FeeTech SMS/STS-type servo motors controlled via serial interface.
 */
class ServoFt : public Servo {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device object.
     * @param p_joint Pointer to the Joint object.
     * @param p_driver Pointer to the Driver object (must be a DriverFt).
     */
    ServoFt(Device* p_device, Joint* p_joint, Driver* p_driver);

    /*!
     * @brief Destructor.
     */
    ~ServoFt();

    /*!
     * @brief Safely parks the servo before shutdown (torque off, goal aligned to present).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode park_safely() override;

    /*!
     * @brief Initializes the servo with model configuration.
     * @param servo_config JSON object containing the servo model configuration.
     * @param p_config Pointer to the device model configuration object.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode init_config_model(const json& servo_config, const DeviceConfig* p_config) override;

    /*!
     * @brief Starts the servo hardware: position mode, PID gains, acceleration,
     *        goal-to-present sync, then torque on.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode start_hardware() override;

    /*!
     * @brief Gets the servo's current position as raw encoder steps.
     * @return Current position in encoder steps.
     */
    float get_pos_servo() override { return (float)curr_pos_steps_; }

    /*!
     * @brief Moves the servo to the target position.
     * @param target_pos Target position in relative radians.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos) override;

    /*!
     * @brief Moves the servo to the target position with a torque limit.
     *
     * The STS3215 has no current loop; target_tor is approximated by writing
     * the Torque Limit register (0-1000, 0.1% of stall torque) alongside the
     * goal position. target_vel is not consumed (the ready-move stepping and
     * the Acceleration register bound the motion).
     * @param target_pos Target position in relative radians.
     * @param target_vel Target velocity in rad/sec (unused).
     * @param target_tor Target torque limit in Nm.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos, float target_vel, float target_tor) override;

    /*!
     * @brief Direct torque command -- not supported (no current control loop).
     * @param torque Torque in Nm.
     * @return ReturnCode::NOT_SUPPORTED always.
     */
    ReturnCode apply_torque(float torque) override;

    /*!
     * @brief Enables or disables torque output of the servo.
     * @param enable True to enable torque output, false to disable it.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode enable_torque(bool enable) override;

    /*!
     * @brief Changes the servo control mode for leader (teleoperation) operation.
     *
     * The SO-ARM leader is fully passive (LeRobot convention): torque is simply
     * disabled so the operator back-drives the arm. Force feedback requires a
     * current loop the STS3215 does not have, so it fast-fails.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode change_control_mode_for_leader() override;

    /*!
     * @brief Changes the servo control mode for follower operation
     *        (position mode, goal synced to present, torque on).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode change_control_mode_for_follower() override;

    /*!
     * @brief Gets the servo's encoder resolution.
     * @return Servo resolution (encoder counts per revolution).
     */
    int get_servo_resolution() override { return servo_resolution_; }

    /*!
     * @brief Converts a raw FeeTech velocity value (steps/s) to rad/sec.
     * @param servo_value Raw velocity value (sign-magnitude already decoded).
     * @return Velocity in rad/sec.
     */
    float get_vel_rad_sec(int32_t servo_value) { return servo_value * kv_; }

    /*!
     * @brief Converts a raw FeeTech load value (0.1% units) to Nm.
     * @param servo_value Raw load value (sign-magnitude already decoded).
     * @return Torque in Nm.
     */
    float get_tor_nm(int32_t servo_value) { return servo_value * kt_; }

    /*!
     * @brief Converts encoder steps to radians (center step = 0 rad).
     * @param steps Position in encoder steps.
     * @return Position in radians.
     */
    float steps_to_rad(int32_t steps) {
        return (steps - center_offset_) * (2.0 * M_PI) / (float)servo_resolution_;
    }

    /*!
     * @brief Converts radians to encoder steps (0 rad = center step).
     * @param rad_value Position in radians.
     * @return Position in encoder steps.
     */
    int32_t rad_to_steps(float rad_value) {
        return (int32_t)(rad_value * ((float)servo_resolution_ / (2.0 * M_PI)) + center_offset_);
    }

    /*!
     * @brief Converts a torque value in Nm to a Torque Limit register value.
     * @param torque Torque in Nm (magnitude is used; the register is unsigned).
     * @return Torque Limit register value (0-1000, 0.1% of stall torque).
     */
    int32_t torque_to_torque_limit(float torque);

    int32_t curr_pos_steps_ = 0;     ///< Current position in encoder steps (unsigned single-turn).
    uint32_t servo_resolution_ = 0;  ///< Servo encoder resolution (counts per full revolution).
    int32_t center_offset_ = 0;      ///< Encoder step corresponding to 0 rad (calibrated middle, 2048).

   protected:
    DriverFt* p_driver_ft_ = nullptr;  ///< Pointer to the FeeTech driver.
    int prof_accel_ = 0;               ///< Optional Acceleration register value (unit 100 steps/s^2; 0 = ramp off).
};
