/*!
 * @file pi_driver_ft.hpp
 * @brief DriverFt -- FeeTech SMS/STS serial bus servo driver (STS3215 class).
 *
 * Speaks the FeeTech UART protocol (0xFF 0xFF header, complement checksum,
 * INST_READ/WRITE plus SYNC READ 0x82 / SYNC WRITE 0x83 bulk transfers) on a
 * plain USB serial port. Mirrors the DriverDxl structure: cached bulk feedback
 * per control cycle, queued group writes flushed once per cycle, and sticky
 * dead-servo tracking for daisy-chain cable breaks.
 *
 * Covers the SO-ARM100/101 arms (FeeTech STS3215; Hiwonder HX-30HM / HX-10HM
 * are register-compatible and use the same model string). SYNC READ requires
 * STS3215 firmware >= 3.9 class; open() verifies it once and fast-fails when
 * the bus does not answer a bulk read (no sequential-read fallback).
 *
 * The Python mirror of this protocol lives in pi_control/servos/ft_serial.py.
 */

#pragma once
#include <cstdint>
#include <set>
#include <vector>

#include "pi_driver_serial.hpp"

class ServoFt;

/*!
 * @brief FeeTech SMS/STS serial bus servo driver.
 */
class DriverFt : public DriverSerial {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device instance.
     * @param cla Command-line arguments.
     */
    DriverFt(Device* p_device, const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DriverFt() override;

    /*!
     * @brief Opens the serial port, sizes the feedback caches, and verifies
     *        SYNC READ support with one bulk read (fast-fail when unsupported).
     * @param baud_rate Baud rate for serial communication (SO-ARM default 1000000).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode open(int baud_rate) override;

    /*!
     * @brief Closes the serial port.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode close() override;

    /*!
     * @brief Copies the cached bulk-read feedback of one servo into its Servo object.
     * @param p_servo Pointer to the Servo instance (must be a ServoFt).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode read_hardware_values(Servo* p_servo) override;

    /*!
     * @brief Reads feedback from all servos with one SYNC READ transaction.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode group_read_hardware_values() override;

    /*!
     * @brief Flushes the queued goal position / torque limit SYNC WRITE packets.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode group_write_hardware_values() override;

    /*!
     * @brief Returns the lowest dead servo id (daisy-chain break point), or -1.
     */
    int last_failed_servo_id() const override { return last_failed_servo_id_; }

    /*!
     * @brief Returns the sticky set of servo ids classified dead on the bus.
     */
    std::set<int> dead_servo_ids() const override { return dead_servo_ids_; }

    /*!
     * @brief Pings a servo (INST_PING) and validates the status packet.
     * @param id Servo ID.
     * @return ReturnCode::SUCCESS when the servo answered, otherwise an error code.
     */
    ReturnCode ping(int id);

    /*!
     * @brief Reads a 1/2-byte register from a servo (INST_READ, little-endian).
     * @param id Servo ID.
     * @param address Register address.
     * @param length Register length in bytes (1 or 2).
     * @param value Output register value (unsigned raw).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode read_register(int id, uint8_t address, uint8_t length, int32_t& value);

    /*!
     * @brief Writes a 1/2-byte register on a servo (INST_WRITE, little-endian).
     *
     * EEPROM registers (address below the SRAM boundary) are automatically
     * wrapped with a Lock Flag unlock/re-lock pair, which the STS3215 firmware
     * requires for the write to take effect.
     * @param id Servo ID.
     * @param address Register address.
     * @param value Register value (unsigned raw; pre-encode sign-magnitude values).
     * @param length Register length in bytes (1 or 2).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode write_register(int id, uint8_t address, int32_t value, uint8_t length);

    /*!
     * @brief Enables or disables torque output (Torque Enable register).
     * @param p_servo Pointer to the ServoFt instance.
     * @param enable True to enable, false to disable.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode enable_torque(ServoFt* p_servo, bool enable);

    /*!
     * @brief Sets the operating mode (0 position / 1 velocity / 2 PWM / 3 step).
     * @param p_servo Pointer to the ServoFt instance.
     * @param mode Operating mode value.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode set_operating_mode(ServoFt* p_servo, uint8_t mode);

    /*!
     * @brief Writes the position loop P/I/D gains (one byte each, EEPROM).
     * @param p_servo Pointer to the ServoFt instance.
     * @param pos_p Position P gain (0-254).
     * @param pos_i Position I gain (0-254).
     * @param pos_d Position D gain (0-254).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode set_position_pid(ServoFt* p_servo, float pos_p, float pos_i, float pos_d);

    /*!
     * @brief Writes the Acceleration register (unit: 100 steps/s^2, 0 disables the ramp).
     * @param p_servo Pointer to the ServoFt instance.
     * @param acceleration Acceleration register value (0-254).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode set_acceleration(ServoFt* p_servo, uint8_t acceleration);

    /*!
     * @brief Writes the goal position of one servo directly (single WRITE).
     * @param p_servo Pointer to the ServoFt instance.
     * @param goal_position Goal position in encoder steps (0-4095).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode set_goal_position_direct(ServoFt* p_servo, int32_t goal_position);

    /*!
     * @brief Queues a goal position for the per-cycle SYNC WRITE flush.
     * @param p_servo Pointer to the ServoFt instance.
     * @param goal_position Goal position in encoder steps (0-4095).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode set_goal_position_group(ServoFt* p_servo, int32_t goal_position);

    /*!
     * @brief Queues a torque limit (0-1000, 0.1% of max torque) for the SYNC WRITE flush.
     * @param p_servo Pointer to the ServoFt instance.
     * @param torque_limit Torque Limit register value (0-1000).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode set_torque_limit_group(ServoFt* p_servo, int32_t torque_limit);

    /*!
     * @brief Discards one servo's queued goal position / torque limit group-write entries.
     *
     * Used when switching that servo to the passive leader mode: goal entries queued
     * earlier in the same control cycle (the final move-to-ready step) must not be
     * flushed after its torque is disabled, because a goal position command can
     * re-engage torque (Hiwonder HX firmware on SO-ARM101 kits).
     * @param p_servo Pointer to the ServoFt instance whose entries are dropped.
     */
    void discard_pending_group_writes(ServoFt* p_servo);

    /*!
     * @brief Reads the Torque Enable register of one servo.
     * @param p_servo Pointer to the ServoFt instance.
     * @param enabled Output raw register value (0 = torque off, 1 = torque on).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode get_torque_enabled(ServoFt* p_servo, int32_t& enabled);

    /*!
     * @brief Reads the present position of one servo (single READ).
     * @param p_servo Pointer to the ServoFt instance.
     * @param present_position Output present position in encoder steps.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode get_present_position(ServoFt* p_servo, int32_t& present_position);

    /*!
     * @brief Reads the present position and writes it as the goal position.
     *
     * Called before enabling torque so the servo holds its current pose instead
     * of jumping to a stale goal register (same rationale as DriverDxl).
     * @param p_servo Pointer to the ServoFt instance.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode sync_goal_position_to_present(ServoFt* p_servo);

    /*!
     * @brief Decodes a FeeTech sign-magnitude value (dedicated sign bit).
     * @param value Raw register value.
     * @param sign_bit Index of the sign bit (velocity: 15, load: 10).
     * @return Signed value.
     */
    static int32_t decode_sign_magnitude(uint32_t value, int sign_bit);

    /*!
     * @brief Encodes a signed value into FeeTech sign-magnitude representation.
     * @param value Signed value.
     * @param sign_bit Index of the sign bit.
     * @return Raw register value.
     */
    static uint32_t encode_sign_magnitude(int32_t value, int sign_bit);

   private:
    /*!
     * @brief Builds a FeeTech instruction packet and sends it.
     * @param id Servo ID (or the broadcast ID).
     * @param instruction Instruction byte.
     * @param p_params Instruction parameter bytes (may be nullptr when length is 0).
     * @param param_length Number of parameter bytes.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode send_instruction(uint8_t id, uint8_t instruction, const uint8_t* p_params, size_t param_length);

    /*!
     * @brief Reads one status packet (FF FF ID LEN ERR PARAMS.. CHK) with a deadline.
     * @param expected_id Servo ID the status packet must carry; pass the broadcast ID (0xFE) to accept any servo
     *        (used for SYNC READ responses, which are matched by the ID they carry).
     * @param p_params Output buffer for the status parameters (may be nullptr when param_length is 0).
     * @param param_length Expected number of status parameter bytes.
     * @param timeout_ms Deadline for the complete packet.
     * @param p_packet_id Optional output for the ID the packet actually carried (may be nullptr).
     * @return ReturnCode::SUCCESS if a valid packet arrived, otherwise an error code.
     */
    ReturnCode receive_status(uint8_t expected_id, uint8_t* p_params, size_t param_length, int timeout_ms,
                              int* p_packet_id);

    /*!
     * @brief Sends an instruction and reads the matching status packet, with retries.
     * @param id Servo ID.
     * @param instruction Instruction byte.
     * @param p_params Instruction parameter bytes.
     * @param param_length Number of instruction parameter bytes.
     * @param p_rsp_params Output buffer for status parameters.
     * @param rsp_param_length Expected number of status parameter bytes.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode txrx_instruction(uint8_t id, uint8_t instruction, const uint8_t* p_params, size_t param_length,
                                uint8_t* p_rsp_params, size_t rsp_param_length);

    /*!
     * @brief Writes a register without the EEPROM unlock wrapper (used by the wrapper itself).
     */
    ReturnCode write_register_raw(int id, uint8_t address, int32_t value, uint8_t length);

    /*!
     * @brief Reads exactly ``length`` bytes from the port before ``deadline_ms`` elapses.
     * @return Number of bytes actually read (may be short on timeout).
     */
    size_t read_exact(uint8_t* p_buf, size_t length, int deadline_ms);

    /*!
     * @brief Discards any stale bytes pending in the receive buffer.
     */
    void flush_input();

    /*!
     * @brief Sends one SYNC WRITE packet for a fixed register window.
     * @param address Register start address.
     * @param data_length Bytes per servo.
     * @param entries (servo id, raw value) pairs; values are little-endian encoded.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode sync_write(uint8_t address, uint8_t data_length, const std::vector<std::pair<int, uint16_t>>& entries);

    /*!
     * @brief Probes unclassified servos with ping() after a bulk-read failure
     *        and moves the silent ones into dead_servo_ids_ (sticky).
     */
    void classify_dead_servos();

    int servo_num_total_ = 0;                       ///< Total number of servos registered on the bus.
    std::vector<int> sync_read_ids_;                ///< Alive servo ids included in the SYNC READ transaction.
    std::vector<int> all_servo_ids_;                ///< All servo ids as reported by the device at open().
    std::set<int> dead_servo_ids_;                  ///< Sticky set of servos classified dead (cable break).
    int last_failed_servo_id_ = -1;                 ///< Lowest dead servo id, or -1 when all alive.

    std::vector<uint16_t> pres_pos_;                ///< Cached present position (raw steps, unsigned single-turn).
    std::vector<int32_t> pres_vel_;                 ///< Cached present velocity (steps/s, sign-magnitude decoded).
    std::vector<int32_t> pres_load_;                ///< Cached present load (0.1% units, sign-magnitude decoded).
    std::vector<uint8_t> pres_temp_;                ///< Cached present temperature (degrees Celsius).
    std::vector<uint8_t> pres_status_;              ///< Cached Servo Status error bits.

    std::vector<std::pair<int, uint16_t>> pending_goal_position_;  ///< Queued (id, steps) for the goal position SYNC WRITE.
    std::vector<std::pair<int, uint16_t>> pending_torque_limit_;   ///< Queued (id, 0.1% units) for the torque limit SYNC WRITE.
};
