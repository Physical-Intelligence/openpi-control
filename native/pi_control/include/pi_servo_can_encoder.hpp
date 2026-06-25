/*!
 * @file pi_servo_can_encoder.hpp
 * @brief ServoCanPassiveEncoder class for the YAM teaching-handle trigger encoder.
 *
 * The encoder is a passive request/response device on the leader's CAN bus
 * (shared with the arm's DM servos). It is read-only: the driver polls it with
 * a 2-byte frame and the encoder answers with trigger position/velocity and
 * two button bits. It must never receive MIT command, enable, or zero frames.
 * Protocol captured from the i2rt reference implementation
 * (``PassiveEncoderReader`` in ``i2rt/motor_drivers/dm_driver.py``); see
 * docs/yam_teaching_handle.md.
 */

#pragma once
#include <chrono>

#include "pi_driver_arx.hpp"
#include "pi_servo.hpp"

#define PASSIVE_ENCODER_RESPONSE_LEN 6      ///< Response payload: device_id (u8) + position (i16) + velocity (i16) + digital_inputs (u8), big-endian.
#define PASSIVE_ENCODER_TICKS_PER_REV 4096  ///< Encoder resolution: radians = ticks * 2*pi / 4096.

/*!
 * @brief Manages the YAM teaching-handle passive trigger encoder over CAN.
 */
class ServoCanPassiveEncoder : public Servo {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device object.
     * @param p_joint Pointer to the Joint object.
     * @param p_driver Pointer to the Driver object (must be a CAN driver).
     */
    ServoCanPassiveEncoder(Device* p_device, Joint* p_joint, Driver* p_driver);

    /*!
     * @brief Destructor.
     */
    ~ServoCanPassiveEncoder();

    /*!
     * @brief Marks the servo parked. The encoder is passive, so no park frame is sent.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode park_safely() override;

    /*!
     * @brief Initializes the servo with model configuration and registers the
     *        encoder's response CAN id with the driver's RX routing.
     * @param servo_config JSON object containing the servo model configuration.
     * @param p_config Pointer to the device model configuration object.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode init_config_model(const json& servo_config, const DeviceConfig* p_config) override;

    /*!
     * @brief Proves the encoder is present by polling until a response is
     *        parsed into the driver cache. Never sends enable/MIT frames.
     * @return ReturnCode::SUCCESS once a response arrived, NO_RESPONSE otherwise.
     */
    ReturnCode start_hardware() override;

    /*!
     * @brief Confirms at least one encoder response has been parsed into the cache.
     * @return ReturnCode::SUCCESS when fresh, ReturnCode::FAIL otherwise.
     */
    ReturnCode verify_position_fresh() override;

    /*!
     * @brief No-op: the encoder is read-only and cannot be commanded.
     */
    ReturnCode move(float target_pos) override;

    /*!
     * @brief No-op: the encoder is read-only and cannot be commanded.
     */
    ReturnCode move(float target_pos, float target_vel, float target_tor) override;

    /*!
     * @brief No-op: the encoder is read-only and cannot be commanded.
     */
    ReturnCode apply_torque(float torque) override;

    /*!
     * @brief Copies the latest cached encoder response into the servo state,
     *        publishes button bits only for fresh responses, and advances the
     *        bounded poll/retry state machine. Returns SAFE_MODE_SIG after
     *        sustained wall-clock silence.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode read_hardware_values() override;

    /*!
     * @brief Gets the servo's current position as raw servo value.
     * @return Current trigger displacement in absolute radian.
     */
    float get_pos_servo() override { return curr_pos_abs_; }

    /*!
     * @brief Parses a passive-encoder response frame into the driver's servo cache.
     *        Caller must hold the driver's ``received_servo_data_mutex_``.
     * @param frame The received CAN frame (on the encoder's response CAN id).
     * @param expected_encoder_id Encoder request CAN id stored in the cache slot.
     * @param slot Cache slot to fill (position/velocity in radians, digital inputs).
     * @return ReturnCode indicating success or failure.
     */
    static ReturnCode parse_encoder_status(const DriverCan::can_frame_t& frame, int expected_encoder_id,
                                           ReceivedServoData& slot);

   private:
    /*!
     * @brief Sends the 2-byte poll request ``[0xFF, 0x02]`` to the encoder's CAN id.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode send_poll_frame();

    /*!
     * @brief Sends the i2rt REQ_RESTART frame ([0xFF, 0x0F]): reboots a wedged
     *        handle MCU. The encoder answers polls again ~8.5 s later.
     */
    ReturnCode send_restart_frame();

    /*!
     * @brief Publishes the encoder's button bits through MsgJoystick on the
     *        device's joystick topic (leader role with --topic_joystick only).
     * @param digital_inputs Raw digital-inputs byte (bit 0 = top, bit 1 = bottom).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode publish_buttons(uint8_t digital_inputs);

    using Clock = std::chrono::steady_clock;

    DriverArx* p_driver_can_ = nullptr;      ///< Pointer to the CAN driver (cast from base Driver pointer).
    int response_can_id_ = -1;               ///< CAN id the encoder answers on (default id + 1).
    int button_num_ = 2;                     ///< Number of buttons carried in the digital-inputs byte.
    uint32_t last_update_count_ = 0;         ///< Cache update counter at the previous read.
    bool publish_joystick_enabled_ = false;  ///< True when the device publishes MsgJoystick.
    bool response_seen_ = false;             ///< True after the first fresh encoder response.
    bool poll_outstanding_ = false;          ///< True while waiting inside the 10 ms response deadline.
    bool silence_warned_ = false;            ///< True after the current silence episode's 2 s warning.
    bool restart_sent_ = false;              ///< True after the current silence episode's one restart request.
    Clock::time_point poll_sent_at_{};        ///< Send time of the most recent poll.
    Clock::time_point next_poll_allowed_at_{};  ///< Earliest time another poll may be sent.
    Clock::time_point last_response_at_{};    ///< Wall-clock time of the most recently consumed response.
};
