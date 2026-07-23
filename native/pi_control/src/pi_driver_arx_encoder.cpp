/*!
 * @file pi_driver_arx_encoder.cpp
 * @brief Implementation of DriverArxEncoder for read-only ARX joint encoders.
 */

#include <unistd.h>
#include <cmath>
#include <cstdint>

#include "pi_control.hpp"
#include "pi_driver_arx_encoder.hpp"
#include "pi_info.hpp"

DriverArxEncoder::DriverArxEncoder(Device* p_device, const CommandLineArgs& cla) : DriverArx(p_device, cla) {}

DriverArxEncoder::~DriverArxEncoder() {}

ReturnCode DriverArxEncoder::send_command(ServoDm* p_servo_dm, float kp, float kd, float position, float velocity,
                                          float torque) {
    // Read-only encoder: never drive the bus. Absorbs gravity-compensation,
    // move-to-ready and park torques computed by the generic arm loop.
    (void)p_servo_dm;
    (void)kp;
    (void)kd;
    (void)position;
    (void)velocity;
    (void)torque;
    return ReturnCode::SUCCESS;
}

ReturnCode DriverArxEncoder::enable(int id, int type, bool enable_flag, bool defer_effector_thermal_fault) {
    (void)type;
    (void)enable_flag;
    (void)defer_effector_thermal_fault;

    // No enable handshake exists; the encoder free-runs at 200 Hz. Confirm the async
    // receive thread has landed at least one frame for this id so the subsequent
    // ServoDm::verify_position_fresh() sees a non-zero motor_id (otherwise startup
    // would race the parser and fail on a perfectly healthy bus).
    const int data_index = find_data_index(id);
    if (data_index < 0) {
        PI_ERROR("DriverArxEncoder::enable: CAN id 0x%X is not a mapped encoder", id);
        return ReturnCode::FAIL;
    }

    for (int attempt = 0; attempt < ARX_ENCODER_WARMUP_MAX_RETRY; ++attempt) {
        if (get_received_motor_id(data_index) != 0) {
            return ReturnCode::SUCCESS;
        }
        usleep(ARX_ENCODER_WARMUP_SLEEP_US);
    }

    // Timed out: leave the canonical failure to verify_position_fresh so the error
    // message and the dead-bus handling stay in one place.
    PI_WARN("DriverArxEncoder::enable: no encoder frame for CAN id 0x%X within warmup window", id);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverArxEncoder::reset_zero_position(int id, int type) {
    (void)id;
    (void)type;
    return ReturnCode::SUCCESS;
}

void DriverArxEncoder::handle_received_message(void* p_data_buf, size_t data_buf_size, size_t read_bytes) {
    (void)data_buf_size;
    (void)read_bytes;

    if (p_data_buf == nullptr) {
        PI_ERROR("Invalid data buffer in DriverArxEncoder::handle_received_message()");
        return;
    }

    can_frame_t* p_frame = (can_frame_t*)p_data_buf;

    // Encoder feedback is a fixed 2-byte angle. Anything else on the bus is not
    // an encoder report (drop silently rather than corrupt a cache slot).
    if (p_frame->can_dlc != ARX_ENCODER_FEEDBACK_DLC) {
        return;
    }

    const int data_index = find_data_index(p_frame->can_id);
    if (data_index < 0) {
        // CAN id not mapped to any configured encoder: not for us.
        return;
    }

    const uint16_t raw = (uint16_t)((p_frame->data[0] << 8) | p_frame->data[1]);
    const float angle_rad =
        ((int)raw - ARX_ENCODER_ZERO_RAW) * (float)M_PI / ARX_ENCODER_RAD_DIVISOR;

    ReturnCode return_code = update_encoder_slot(data_index, (int)p_frame->can_id, angle_rad);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to update encoder slot for CAN id 0x%X (data_index=%d)", p_frame->can_id, data_index);
    }
}
