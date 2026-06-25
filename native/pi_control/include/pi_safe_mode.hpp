/*!
 * @file pi_safe_mode.hpp
 * @brief SafeMode class for managing safety features and graceful error handling.
 */

#pragma once
#include <cstdint>

#include "pi_command_line_args.hpp"
#include "pi_control.hpp"

class Servo;

/*!
 * @brief Manages safety features and graceful error handling for servo operations.
 */
class SafeMode {
   public:
    /*!
     * @brief Constructor.
     * @param cla Command line arguments.
     */
    SafeMode(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~SafeMode();

    /*!
     * @brief Gets the safe teleoperation position with safety constraints applied.
     * @param p_servo Pointer to the servo.
     * @param tele_pos Requested teleoperation position in relative radian.
     * @return Safe teleoperation position in relative radian.
     */
    float get_safe_tele_pos_rad(Servo* p_servo, float tele_pos);

    /*!
     * @brief Performs graceful management when a safety violation is detected.
     * @param p_servo Pointer to the servo that triggered the safety violation.
     * @param servo_return_code Return code from the servo indicating the type of violation.
     * @return Return code indicating the result of graceful management.
     */
    ReturnCode graceful_management(Servo* p_servo, ReturnCode servo_return_code);

    /*!
     * @brief Exits safe mode when conditions have returned to normal.
     * @param p_servo Pointer to the servo exiting safe mode.
     * @param safe_mode_return_code Return code indicating which safe mode type to exit.
     * @return Return code indicating success or failure.
     */
    ReturnCode exit_safe_mode(Servo* p_servo, ReturnCode safe_mode_return_code);

    /*!
     * @brief Enables the safe mode system.
     */
    void enable_safe_mode() { is_safe_mode_enabled_ = true; }

    /*!
     * @brief Disables the safe mode system.
     */
    void disable_safe_mode() { is_safe_mode_enabled_ = false; }

    /*!
     * @brief Checks if safe mode is active due to position behind violation.
     * @return true if safe mode is active due to position behind, false otherwise.
     */
    bool is_in_safe_mode_behind() { return is_pos_behind_; }

    /*!
     * @brief Checks if safe mode is active due to torque violation.
     * @return true if safe mode is active due to torque violation, false otherwise.
     */
    bool is_in_safe_mode_tor() { return is_tor_; }

    /*!
     * @brief Checks whether sustained measured-torque violations should trigger protective stops.
     * @return true when torque safe mode is enabled, false for warning-only behavior.
     */
    bool is_torque_mode_enabled() const { return is_torque_mode_enabled_; }

    /*!
     * @brief Checks if safe mode is active due to temperature violation.
     * @return true if safe mode is active due to temperature violation, false otherwise.
     */
    bool is_in_safe_mode_temperature() { return is_temperature_; }

    /*!
     * @brief Checks if safe mode is active due to position exceeding maximum limit.
     * @return true if safe mode is active due to position exceed, false otherwise.
     */
    bool is_in_safe_mode_pos_exceed() const { return is_pos_exceed_; }

    /*!
     * @brief Checks if safe mode is active due to abnormal velocity.
     * @return true if safe mode is active due to velocity violation, false otherwise.
     */
    bool is_in_safe_mode_vel() const { return is_vel_; }

    /*!
     * @brief Checks if safe mode is active due to communication signal loss.
     * @return true if safe mode is active due to signal loss, false otherwise.
     */
    bool is_in_safe_mode_no_signal() const { return is_no_signal_; }

    /*!
     * @brief Encodes all six safe-mode flags into one byte for compact CSV logging.
     *
     * Bit layout (LSB-first):
     *   bit 0: is_pos_behind_   (legacy _safe_mode_behind)
     *   bit 1: is_tor_          (legacy _safe_mode_tor)
     *   bit 2: is_temperature_  (legacy _safe_mode_temperature)
     *   bit 3: is_pos_exceed_   (newly exposed)
     *   bit 4: is_vel_          (newly exposed)
     *   bit 5: is_no_signal_    (newly exposed)
     *   bit 6-7: reserved (always 0)
     */
    uint8_t encode_bits() const {
        return static_cast<uint8_t>((is_pos_behind_ ? 0x01 : 0) | (is_tor_ ? 0x02 : 0) |
                                    (is_temperature_ ? 0x04 : 0) | (is_pos_exceed_ ? 0x08 : 0) |
                                    (is_vel_ ? 0x10 : 0) | (is_no_signal_ ? 0x20 : 0));
    }

   private:
    /*!
     * @brief Handles graceful management when position exceeds maximum limit.
     * @param p_servo Pointer to the servo with position exceed violation.
     * @return Return code indicating success or failure.
     */
    ReturnCode graceful_management_pos_exceed(Servo* p_servo);

    /*!
     * @brief Handles graceful management when position falls behind minimum limit.
     * @param p_servo Pointer to the servo with position behind violation.
     * @return Return code indicating success or failure.
     */
    ReturnCode graceful_management_pos_behind(Servo* p_servo);

    /*!
     * @brief Handles graceful management when abnormal velocity is detected.
     * @param p_servo Pointer to the servo with velocity violation.
     * @return Return code indicating safe mode entry.
     */
    ReturnCode graceful_management_vel(Servo* p_servo);

    /*!
     * @brief Handles graceful management when excessive torque is detected.
     * @param p_servo Pointer to the servo with torque violation.
     * @return Return code indicating safe mode entry.
     */
    ReturnCode graceful_management_tor(Servo* p_servo);

    /*!
     * @brief Updates cached servo state values.
     * @param p_servo Pointer to the servo from which to read values.
     * @return Return code indicating success or failure.
     */
    ReturnCode update_servo_values(Servo* p_servo);

    /*!
     * @brief Handles graceful management when communication signal is lost.
     * @param p_servo Pointer to the servo with communication loss.
     * @return Return code indicating safe mode entry.
     */
    ReturnCode graceful_management_sig(Servo* p_servo);

    /*!
     * @brief Handles graceful management when temperature exceeds safe limits.
     * @param p_servo Pointer to the servo with temperature violation.
     * @return Return code indicating safe mode entry.
     */
    ReturnCode graceful_management_temperature(Servo* p_servo);

    bool is_safe_mode_enabled_;     ///< Flag indicating whether the safe mode system is enabled.
    bool is_torque_mode_enabled_;   ///< Flag enabling sustained measured-torque protective stops.
    bool is_pos_exceed_ = false;     ///< Flag indicating safe mode is active due to position exceeding maximum limit.
    bool is_pos_behind_ = false;     ///< Flag indicating safe mode is active due to position falling behind minimum limit.
    bool is_vel_ = false;            ///< Flag indicating safe mode is active due to abnormal velocity detected.
    bool is_tor_ = false;            ///< Flag indicating safe mode is active due to excessive torque detected.
    bool is_no_signal_ = false;      ///< Flag indicating safe mode is active due to communication loss with servo.
    bool is_temperature_ = false;    ///< Flag indicating safe mode is active due to temperature exceeding safe limits.
    float servo_pos_rad_ = 0.0f;    ///< Cached servo position in radians (refreshed every managed cycle).
    float servo_vel_ = 0.0f;        ///< Cached servo velocity in rad/sec (refreshed every managed cycle).
    float servo_tor_ = 0.0f;        ///< Cached servo torque in Nm (refreshed every managed cycle).
    /// Position captured once at the POS_BEHIND latch transition. The hold in
    /// get_safe_tele_pos_rad() anchors to this instead of the live measured
    /// position: a position loop whose target tracks its own measurement has
    /// zero restoring torque, so a loaded joint would sag and ratchet downward
    /// for the entire latch. servo_pos_rad_ cannot serve as the anchor because
    /// the TOR/TEMP graceful handlers refresh it every cycle.
    float pos_behind_hold_pos_rad_ = 0.0f;
};
