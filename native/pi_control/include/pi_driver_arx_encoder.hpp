/*!
 * @file pi_driver_arx_encoder.hpp
 * @brief DriverArxEncoder: read-only CAN driver for ARX joint encoders (leader arm).
 */

#pragma once

#include "pi_driver_arx.hpp"

/*!
 * @brief Read-only CAN driver for an ARX encoder leader arm.
 *
 * Each joint carries a passive encoder that broadcasts a fixed 2-byte
 * mechanical angle at 200 Hz on its own CAN id (no servo, no torque, no
 * enable handshake). This driver inherits ``DriverArx`` to reuse its
 * ``open``/``close`` reception loop, the ``ReceivedServoData`` cache,
 * ``read_hardware_values`` (angle -> ``curr_pos_abs_``) and the staleness
 * scan in ``group_read_hardware_values``. Only the per-frame parse differs
 * (different CAN id range and payload), and all actuation entry points are
 * no-ops so nothing is ever written to the bus.
 */
class DriverArxEncoder : public DriverArx {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device instance.
     * @param cla Command-line arguments.
     */
    explicit DriverArxEncoder(Device* p_device, const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DriverArxEncoder();

    /*!
     * @brief No-op: the encoder cannot be commanded (read-only).
     * @return ReturnCode::SUCCESS always.
     */
    ReturnCode send_command(ServoDm* p_servo_dm, float kp, float kd, float position, float velocity,
                            float torque) override;

    /*!
     * @brief No-op: the encoder has no enable handshake (read-only).
     *
     * Returning SUCCESS lets ``ServoDm::start_hardware()`` proceed; the
     * 200 Hz broadcast populates the cache, and ``verify_position_fresh``
     * passes once the first frame is parsed.
     * @return ReturnCode::SUCCESS always.
     */
    ReturnCode enable(int id, int type, bool enable_flag = true, bool defer_effector_thermal_fault = false) override;

    /*!
     * @brief No-op: the encoder zero is set out-of-band (read-only).
     * @return ReturnCode::SUCCESS always.
     */
    ReturnCode reset_zero_position(int id, int type) override;

   protected:
    /*!
     * @brief Decode a 2-byte encoder feedback frame into the cache slot.
     *
     * Drops frames whose DLC is not ``ARX_ENCODER_FEEDBACK_DLC`` or whose
     * CAN id is not mapped to a configured encoder. Decodes
     * ``raw = (data[0] << 8) | data[1]`` and stores
     * ``(raw - ARX_ENCODER_ZERO_RAW) * PI / ARX_ENCODER_RAD_DIVISOR`` rad.
     */
    void handle_received_message(void* p_data_buf, size_t data_buf_size, size_t read_bytes) override;
};
