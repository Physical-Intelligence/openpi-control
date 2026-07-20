/*!
 * @file pi_driver_arx.hpp
 * @brief DriverArx class for ARX device communication via CAN interface.
 */

#pragma once
#include <atomic>
#include <map>
#include <mutex>
#include <set>

#include "pi_driver.hpp"
#include "pi_driver_can.hpp"
#include "pi_profile.hpp"  // for prof_time_t

#define MAX_SERVO_INFO_BUF_SIZE 20  ///< Maximum number of servo information entries in the receive buffer.

class ServoDm;

/*!
 * @brief Data structure to store servo feedback data received from CAN messages.
 *
 * ``last_update_perf_`` is the wall-clock timestamp of the most recent
 * successful parse (set by ``ServoDm::parse_dm_servo_status`` /
 * ``ServoDm::parser_encos_servo_status`` /
 * ``ServoCanPassiveEncoder::parse_encoder_status``). A default-constructed
 * value (detect via ``Profile::is_zero``) means no frame has ever been
 * parsed for this slot. The control loop reads this via
 * ``DriverArx::get_last_update_perf`` to detect a CAN-dead servo
 * regardless of the cached pos / vel / tor magnitude.
 */
class ReceivedServoData {
   public:
    int motor_id_ = 0;                   ///< Motor/servo ID.
    float angle_actual_rad_ = 0.0f;      ///< Actual current angle in radians.
    float speed_actual_rad_ = 0.0f;      ///< Actual current angular velocity in radians per second.
    float current_actual_float_ = 0.0f;  ///< Actual current in Amperes.
    uint8_t temperature_ = 0;            ///< Current temperature in degrees Celsius.
    uint8_t error_ = 0;                  ///< Error code or status flags.
    uint8_t digital_inputs_ = 0;  ///< Raw digital-inputs byte (passive encoders only; bit 0 = button 0, bit 1 = button 1).
    uint32_t update_count_ = 0;   ///< Number of frames parsed into this slot (freshness/silence detection for polled devices).
    prof_time_t last_update_perf_;  ///< Timestamp of the most recent successful parse. Default-init means never parsed.
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
     * @brief Frame-age based bulk read.
     *
     * Unlike DXL where ``group_read_hardware_values()`` actually probes the
     * bus, the ARX/CAN path receives status frames asynchronously on a
     * background thread (see ``DriverArx::handle_received_message``). This
     * override therefore scans the cached ``last_update_perf_`` of every
     * servo bound to this driver: if any servo's most recent frame is older
     * than the threshold (`ARX_STALE_FRAME_AGE_NORMAL_MS` once any motor has
     * responded at least once, `ARX_STALE_FRAME_AGE_INITIAL_MS` until then),
     * the servo is inserted into ``dead_servo_ids_``,
     * ``last_failed_servo_id_`` is updated to the lowest stale id, and
     * ``FAIL`` is returned. ``DeviceArm::read_hardware_values`` /
     * ``DeviceEffector::read_hardware_values`` already handle ``FAIL`` +
     * dead-set (mark per-joint failed, escalate to emergency recovery), so
     * no Device-side change is needed.
     *
     * @return ``SUCCESS`` when every bound servo's most recent frame is
     *         within the threshold; ``FAIL`` otherwise.
     */
    ReturnCode group_read_hardware_values() override;

    /*!
     * @brief Lowest stale servo id from the most recent
     *        ``group_read_hardware_values()`` call, or -1 if every servo
     *        responded. Surfaced to ``DeviceArm`` so the log can report a
     *        single id.
     */
    int last_failed_servo_id() const override { return last_failed_servo_id_; }

    /*!
     * @brief Set of servo ids currently considered dead (no frame within
     *        the staleness threshold). Cleared on every
     *        ``group_read_hardware_values()`` call so it always reflects
     *        the current cycle.
     */
    std::set<int> dead_servo_ids() const override { return dead_servo_ids_; }

    /*!
     * @brief Asserts the communication-loss policy (DM TIMEOUT register /
     *        ENCOS heartbeat window) for every servo bound to this driver's
     *        device.
     *
     * The policy comes from Device::wants_comm_loss_stop(): the window is
     * armed for velocity/torque-commanded devices (a stale command is a
     * runaway -> stop) and disarmed for position-commanded devices (the last
     * command is the hold pose -> keep holding instead of collapsing
     * detorqued).
     *
     * Runs with reception stopped so each config acknowledgement is drained
     * synchronously instead of being parsed as a status frame. Must be called
     * immediately before the command stream starts. The DM comm-loss counter
     * measures time since the LAST RECEIVED FRAME and the register write arms
     * the comparison immediately (it does not restart the counter), so each
     * DM servo first gets an idempotent enable frame to reset its counter and
     * the TIMEOUT write follows within milliseconds; a servo that then
     * receives no frame within the window (DM_SERVO_CAN_TIMEOUT_MS)
     * auto-disables with a latched communication-loss error (0xD). The ENCOS
     * window is persistent (non-volatile), so it is queried first and only
     * written on mismatch to limit flash wear.
     *
     * @return ReturnCode::SUCCESS if the pass completed (individual write
     *         failures are logged loudly but are non-fatal: the servo stays
     *         controllable, just with an unasserted policy), otherwise an
     *         error code.
     */
    ReturnCode arm_comm_loss_protection() override;

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
     * @brief Returns the ``last_update_perf_`` cached for the given
     *        ``data_index`` slot. Default-constructed when no frame has been
     *        parsed yet (caller should use ``Profile::is_zero``). Read by
     *        ``ServoDm::read_hardware_values`` to detect per-servo CAN dead
     *        by frame age.
     */
    prof_time_t get_last_update_perf(int data_index) const {
        if (data_index < 0 || data_index >= MAX_SERVO_INFO_BUF_SIZE) {
            return prof_time_t{};
        }
        std::lock_guard<std::mutex> lock(received_servo_data_mutex_);
        return received_servo_data_[data_index].last_update_perf_;
    }

    /*!
     * @brief Registers an RX route for a passive request/response encoder so
     *        handle_received_message() can parse its responses. Must be called
     *        during device init (before the reception thread starts processing
     *        encoder frames is fine; the route map is mutex-guarded anyway).
     * @param response_can_id CAN id the encoder answers on.
     * @param encoder_id Encoder request CAN id.
     * @param data_index Cache slot in received_servo_data_ for this encoder.
     * @return ReturnCode::SUCCESS, or INVALID_PARAM for an out-of-range data index.
     */
    ReturnCode register_passive_encoder(int response_can_id, int encoder_id, int data_index);

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
     * @return ReturnCode::SUCCESS when the encoder is ready for passive polling.
     */
    ReturnCode configure_passive_encoder(int request_can_id);

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
     */
    ReturnCode read_passive_encoder_eeprom(int request_can_id, uint8_t device, uint8_t offset, uint8_t* p_value);

    /*!
     * @brief Reads a low/high EEPROM frequency pair.
     */
    ReturnCode read_passive_encoder_frequency(int request_can_id, uint8_t device, uint8_t high_offset,
                                              uint8_t low_offset, int* p_frequency);

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

    /*!
     * @brief Set of servo ids whose most recent CAN frame is older than the
     *        staleness threshold. Populated and cleared once per
     *        ``group_read_hardware_values()`` call. Main-thread only.
     */
    std::set<int> dead_servo_ids_;

    /*!
     * @brief Lowest stale servo id from the most recent
     *        ``group_read_hardware_values()`` call, or -1 when no servo is
     *        stale. Main-thread only.
     */
    int last_failed_servo_id_ = -1;

    /*!
     * @brief Latched flag: ``true`` once any servo on this driver has
     *        produced at least one parsed status frame. Picks between
     *        ``ARX_STALE_FRAME_AGE_INITIAL_MS`` (start-up, longer tolerance)
     *        and ``ARX_STALE_FRAME_AGE_NORMAL_MS`` (steady state, tighter
     *        tolerance) inside ``group_read_hardware_values()``.
     */
    bool any_motor_moved_ = false;

    /*!
     * @brief Warn-only telemetry-stall tracking for
     *        ``group_read_hardware_values()``: servo id -> ``last_update_perf_``
     *        recorded when the stall warning fired. An entry means the stall
     *        PI_WARN was already emitted (edge trigger); it is erased -- with a
     *        recovery PI_WARN reporting the gap length -- once a fresh frame
     *        arrives. Main-thread only.
     */
    std::map<int, prof_time_t> stall_warned_since_;
};
