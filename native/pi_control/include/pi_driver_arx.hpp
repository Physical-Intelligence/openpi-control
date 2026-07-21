/*!
 * @file pi_driver_arx.hpp
 * @brief DriverArx class for ARX device communication via CAN interface.
 */

#pragma once
#include <atomic>
#include <map>
#include <mutex>

#include "pi_driver.hpp"
#include "pi_driver_can.hpp"

#define MAX_SERVO_INFO_BUF_SIZE 20  ///< Maximum number of servo information entries in the receive buffer.

class ServoDm;

/*!
 * @brief Data structure to store servo feedback data received from CAN messages.
 */
class ReceivedServoData {
   public:
    int motor_id_;            ///< Motor/servo ID.
    float angle_actual_rad_;  ///< Actual current angle in radians.
    float speed_actual_rad_;  ///< Actual current angular velocity in radians per second.
    float current_actual_float_;  ///< Actual current in Amperes.
    uint8_t temperature_;        ///< Current temperature in degrees Celsius.
    uint8_t error_;              ///< Error code or status flags.
    uint8_t digital_inputs_;     ///< Raw digital-inputs byte (passive encoders only; bit 0 = button 0, bit 1 = button 1).
    uint32_t update_count_;      ///< Number of frames parsed into this slot (freshness/silence detection for polled devices).
};

/*!
 * @brief RX route for a passive request/response encoder (YAM teaching handle).
 *
 * The encoder answers on its own CAN id (or id + 1, depending on the firmware
 * receive mode) with a payload that does not match the DM/ENCOS status
 * heuristics, so routes are registered explicitly and checked first in
 * handle_received_message().
 */
class PassiveEncoderRoute {
   public:
    int encoder_id_;   ///< Encoder request CAN id (the report arrives on the map key).
    int data_index_;   ///< Cache slot in received_servo_data_ for this encoder.
    bool firmware_compat_ = false;  ///< Tolerate multiple teaching-handle firmware revisions during EEPROM/frequency setup (config ``passive_encoder_firmware_compat``). Default false = single-format setup.
};

/*!
 * @brief Driver implementation for ARX devices using CAN bus communication.
 */
class DriverArx : public DriverCan {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device instance.
     * @param cla Command-line arguments.
     */
    explicit DriverArx(Device* p_device, const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DriverArx();

    /*!
     * @brief Opens the CAN control port and starts message reception.
     * @param baud_rate Baud rate parameter (unused for CAN).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode open(int baud_rate) override;

    /*!
     * @brief Closes the CAN control port and stops message reception.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode close() override;

    /*!
     * @brief Reads hardware values from a servo.
     * @param p_servo Pointer to the Servo instance.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode read_hardware_values(Servo* p_servo) override;

    /*!
     * @brief Sends a control command to a DM-CAN servo motor.
     * @param p_servo_dm Pointer to the ServoDm instance.
     * @param kp Proportional gain (Kp) for PID position control.
     * @param kd Derivative gain (Kd) for PID position control.
     * @param position Target position in radians.
     * @param velocity Target velocity in radians per second.
     * @param torque Target torque in Newton-meters.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode send_command(ServoDm* p_servo_dm, float kp, float kd, float position, float velocity, float torque);

    /*!
     * @brief Enables or disables a servo motor.
     * @param id The CAN ID of the servo motor.
     * @param type The servo type or model identifier.
     * @param enable_flag True to enable, false to disable (defaults to true).
     * @param defer_effector_thermal_fault True when the effector will emit the
     *        complete thermal-stop record after this call returns.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode enable(int id, int type, bool enable_flag = true, bool defer_effector_thermal_fault = false);

    /*!
     * @brief Sends exactly one disable frame without stopping asynchronous reception.
     * @param id The CAN ID of the servo motor.
     * @param type The servo type or model identifier.
     * @return ReturnCode::SUCCESS if the frame was sent, otherwise an error code.
     */
    ReturnCode send_disable_once(int id, int type);

    /*!
     * @brief Resets the zero position of a servo motor.
     * @param id The CAN ID of the servo motor.
     * @param type The servo type or model identifier.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode reset_zero_position(int id, int type) override;

    int last_enable_fault_status() const { return last_enable_fault_status_.load(); }

    /*!
     * @brief Returns the motor_id stored in ``received_servo_data_[data_index]``.
     *        The cache is zero-initialised, so a return value of 0 means no
     *        status frame for that slot has ever been parsed; real DM/ENCOS
     *        motor IDs are 1 or higher. Used by ``ServoDm::verify_position_fresh()``
     *        to detect a stale-cache after start_hardware().
     * @param data_index The cache slot index (typically ``Servo::data_index_``).
     * @return Cached motor ID, or 0 if the slot has never been populated /
     *         the index is out of range.
     */
    int get_received_motor_id(int data_index) const {
        if (data_index < 0 || data_index >= MAX_SERVO_INFO_BUF_SIZE) return 0;
        std::lock_guard<std::mutex> lock(received_servo_data_mutex_);
        return received_servo_data_[data_index].motor_id_;
    }

    /*!
     * @brief Registers an RX route for a passive request/response encoder so
     *        handle_received_message() can parse its responses. Must be called
     *        during device init (before the reception thread starts processing
     *        encoder frames is fine; the route map is mutex-guarded anyway).
     * @param response_can_id CAN id the encoder answers on.
     * @param encoder_id Encoder request CAN id.
     * @param data_index Cache slot in received_servo_data_ for this encoder.
     * @param firmware_compat Tolerate multiple teaching-handle firmware revisions during
     *        EEPROM/frequency setup (config ``passive_encoder_firmware_compat``; default false).
     * @return ReturnCode::SUCCESS, or INVALID_PARAM for an out-of-range data index.
     */
    ReturnCode register_passive_encoder(int response_can_id, int encoder_id, int data_index,
                                        bool firmware_compat = false);

    /*!
     * @brief Snapshots the cached state of a polled passive encoder.
     * @param data_index The cache slot index (typically ``Servo::data_index_``).
     * @param p_pos_rad Out: trigger position in radians (signed, as reported).
     * @param p_vel_rad_sec Out: trigger velocity in radians per second.
     * @param p_digital_inputs Out: raw digital-inputs byte.
     * @param p_update_count Out: number of frames ever parsed into the slot.
     * @return true on success, false for an out-of-range data index or null output pointer.
     */
    bool get_received_encoder_data(int data_index, float* p_pos_rad, float* p_vel_rad_sec,
                                   uint8_t* p_digital_inputs, uint32_t* p_update_count) const {
        if (data_index < 0 || data_index >= MAX_SERVO_INFO_BUF_SIZE || p_pos_rad == nullptr ||
            p_vel_rad_sec == nullptr || p_digital_inputs == nullptr || p_update_count == nullptr) {
            return false;
        }
        std::lock_guard<std::mutex> lock(received_servo_data_mutex_);
        const ReceivedServoData& data = received_servo_data_[data_index];
        *p_pos_rad = data.angle_actual_rad_;
        *p_vel_rad_sec = data.speed_actual_rad_;
        *p_digital_inputs = data.digital_inputs_;
        *p_update_count = data.update_count_;
        return true;
    }

   private:
    /*!
     * @brief Validates and corrects every registered passive encoder before
     *        asynchronous reception or motor enable starts.
     * @return ReturnCode::SUCCESS when every encoder meets the required configuration.
     */
    ReturnCode configure_passive_encoders();

    /*!
     * @brief Validates firmware and EEPROM frequencies for one passive encoder.
     * @param request_can_id CAN id used for encoder configuration requests.
     * @param firmware_compat Tolerate multiple teaching-handle firmware revisions (default false).
     * @return ReturnCode::SUCCESS when the encoder is ready for passive polling.
     */
    ReturnCode configure_passive_encoder(int request_can_id, bool firmware_compat);

    /*!
     * @brief Sends a configuration request to a passive encoder.
     */
    ReturnCode send_passive_encoder_request(int request_can_id, const uint8_t* p_data, uint8_t data_len);

    /*!
     * @brief Waits synchronously for a matching passive-encoder configuration reply.
     */
    ReturnCode wait_for_passive_encoder_reply(int request_can_id, int expected_device, uint8_t expected_command,
                                              uint8_t expected_len, int timeout_ms, can_frame_t* p_reply);

    /*!
     * @brief Reads one passive-encoder EEPROM byte.
     * @param firmware_compat When true, accept both the <=2.2.x READINGS (0x86, 5-byte) and
     *        >=2.3.x GET_EEPROM (0x87, 3-byte) reply formats; when false, require the single
     *        READINGS format (original behavior).
     */
    ReturnCode read_passive_encoder_eeprom(int request_can_id, uint8_t device, uint8_t offset, uint8_t* p_value,
                                           bool firmware_compat);

    /*!
     * @brief Reads a low/high EEPROM frequency pair.
     * @param firmware_compat When true, fall back to the 8-bit low byte if the 16-bit high-byte
     *        offset is unimplemented (legacy <=2.2.12 EEPROM layout); when false, a missing high
     *        byte is an error (original behavior).
     */
    ReturnCode read_passive_encoder_frequency(int request_can_id, uint8_t device, uint8_t high_offset,
                                              uint8_t low_offset, int* p_frequency, bool firmware_compat);

    /*!
     * @brief Drains frames queued by startup validation before reception starts.
     */
    void drain_startup_frames();

    /*!
     * @brief Callback function to handle received CAN messages from servos.
     * @param p_data_buf Pointer to the data buffer.
     * @param data_buf_size Total size of the data buffer in bytes.
     * @param read_bytes Number of bytes actually read from the CAN bus.
     */
    void handle_received_message(void* p_data_buf, size_t data_buf_size, size_t read_bytes);

    ReceivedServoData received_servo_data_[MAX_SERVO_INFO_BUF_SIZE];  ///< Circular buffer storing servo feedback data.
    /// Explicit RX routes for passive encoders, keyed by response CAN id.
    /// Checked before the DM/ENCOS heuristics in handle_received_message().
    /// Guarded by received_servo_data_mutex_ (written once at init, read on
    /// the CAN reception thread).
    std::map<int, PassiveEncoderRoute> passive_encoder_routes_;
    /// Guards received_servo_data_: the CAN reception thread writes it via
    /// handle_received_message() while the main control loop reads it through
    /// read_hardware_values() / get_received_motor_id(). Uncontended in
    /// practice (sub-microsecond hold times at CAN frame rates).
    mutable std::mutex received_servo_data_mutex_;
    std::mutex transaction_mutex_;  ///< Serializes writes against synchronous enable transactions.
    std::atomic<int> last_enable_fault_status_{-1};  ///< Fault status returned by the most recent enable/disable operation.
};
