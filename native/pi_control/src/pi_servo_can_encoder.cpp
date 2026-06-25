/*!
 * @file pi_servo_can_encoder.cpp
 * @brief Implementation of ServoCanPassiveEncoder for the YAM teaching-handle trigger encoder.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "pi_joint.hpp"
#include "pi_servo_can_encoder.hpp"

// Trigger displacement per encoder tick.
static constexpr float PASSIVE_ENCODER_RAD_PER_TICK = 2.0f * (float)M_PI / PASSIVE_ENCODER_TICKS_PER_REV;

// Poll request payload (i2rt PassiveEncoderReader request frame).
#define PASSIVE_ENCODER_POLL_BYTE_0 0xFF
#define PASSIVE_ENCODER_POLL_BYTE_1 0x02

// i2rt encoder_manager REQ_RESTART: [ALL_DEVICE, 0x0F] reboots the handle
// firmware. Measured on yambot hardware: the encoder answers polls again
// ~8.4 s after the restart request.
#define PASSIVE_ENCODER_RESTART_BYTE_1 0x0F

// i2rt's reader runs at 250 Hz. A poll remains outstanding for 10 ms; after
// that deadline the reader backs off for 500 ms before trying again. This
// makes repeated read_hardware_values() calls idempotent even though generic
// effector observation currently reads the joint twice per control loop.
static constexpr auto PASSIVE_ENCODER_POLL_INTERVAL = std::chrono::milliseconds(4);
static constexpr auto PASSIVE_ENCODER_RESPONSE_DEADLINE = std::chrono::milliseconds(10);
static constexpr auto PASSIVE_ENCODER_RETRY_BACKOFF = std::chrono::milliseconds(500);
static constexpr auto PASSIVE_ENCODER_START_TIMEOUT = std::chrono::milliseconds(1500);
static constexpr auto PASSIVE_ENCODER_START_WAIT = std::chrono::milliseconds(1);

// Wall-clock recovery ladder. While the encoder is silent, the cached trigger
// value is held; buttons are not republished until a fresh response arrives.
static constexpr auto PASSIVE_ENCODER_SILENCE_WARN = std::chrono::seconds(2);
static constexpr auto PASSIVE_ENCODER_RESTART_AT = std::chrono::seconds(3);
static constexpr auto PASSIVE_ENCODER_FAILURE_AT = std::chrono::seconds(20);

// Safety thresholds: the encoder drives a read-only joint, so the
// leader/follower difference checks never apply; these only seed the base
// class ServoParam pointer with sane defaults.
static const ServoParam g_servo_can_passive_encoder_param(DEFAULT_TOLERABLE_POS_DIFFERENCE_RAD,
                                                          MAX_POS_DIFFERENCE_RAD,
                                                          DEFAULT_VELOCITY_THRESHOLD_RAD_SEC);

ServoCanPassiveEncoder::ServoCanPassiveEncoder(Device* p_device, Joint* p_joint, Driver* p_driver)
    : Servo(p_device, p_joint, p_driver) {
    p_driver_can_ = dynamic_cast<DriverArx*>(p_driver);
}

ServoCanPassiveEncoder::~ServoCanPassiveEncoder() {}

ReturnCode ServoCanPassiveEncoder::park_safely() {
    // Passive device: nothing to disable, nothing to send.
    parked_ = true;
    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Passive encoder ID %d parked safely", id_);
    return ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::init_config_model(const json& servo_config, const DeviceConfig* p_config) {
    type_ = ServoType::CAN_PASSIVE_ENCODER;
    p_servo_param_ = &g_servo_can_passive_encoder_param;

    ReturnCode return_code = Servo::init_config_model(servo_config, p_config);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (p_driver_can_ == nullptr) {
        PI_ERROR("Passive encoder ID %d requires a CAN driver", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    // Firmware receive mode: the i2rt default ("plus_one") answers on id + 1.
    // An explicit response_can_id in the model config overrides it.
    response_can_id_ = id_ + 1;
    if (servo_config.contains(p_config->fn_servo_response_can_id)) {
        return_code = p_config->get_field_value(servo_config, p_config->fn_servo_response_can_id, response_can_id_);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    if (servo_config.contains(p_config->fn_joystick_button_num)) {
        return_code = p_config->get_field_value(servo_config, p_config->fn_joystick_button_num, button_num_);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
        if (button_num_ < 0 || button_num_ > 8) {
            PI_ERROR("Passive encoder ID %d: joystick_button_num must be in [0, 8], but found %d", id_, button_num_);
            return ReturnCode::INVALID_PARAM;
        }
    }

    return_code = p_driver_can_->register_passive_encoder(response_can_id_, id_, data_index_);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    PI_INFO("Servo", InfoLevel::HELPFUL_1,
            "Passive encoder ID %d: response_can_id=0x%02X button_num=%d", id_, response_can_id_, button_num_);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::start_hardware() {
    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    // MsgJoystick publishing requires a leader-side bound joystick topic;
    // resolve once so the per-cycle read does not spam NOT_INITIALIZED errors
    // when a launcher runs the handle without --topic_joystick.
    const std::string& topic_joystick = p_device_->get_cla().topic_joystick;
    publish_joystick_enabled_ = (get_device_role_belong_to() == Role::LEADER) &&
                                (topic_joystick != OPT_DEFAULT_NONE) && !topic_joystick.empty();
    if (!publish_joystick_enabled_) {
        PI_WARN("Passive encoder ID %d: buttons will not be published "
                "(role is not LEADER or --topic_joystick is unset)",
                id_);
    }

    // Presence probe using the same single-outstanding/deadline/backoff rules
    // as normal reads. DriverArx has already validated the encoder before
    // motor enable; this proves that report polling also works after reception
    // starts.
    const auto startup_deadline = Clock::now() + PASSIVE_ENCODER_START_TIMEOUT;
    next_poll_allowed_at_ = Clock::now();
    int polls_sent = 0;
    while (Clock::now() < startup_deadline) {
        const auto now = Clock::now();
        float pos = 0, vel = 0;
        uint8_t digital = 0;
        uint32_t update_count = 0;
        if (p_driver_can_->get_received_encoder_data(data_index_, &pos, &vel, &digital, &update_count) &&
            update_count > 0) {
            response_seen_ = true;
            poll_outstanding_ = false;
            last_response_at_ = now;
            next_poll_allowed_at_ = std::max(next_poll_allowed_at_, poll_sent_at_ + PASSIVE_ENCODER_POLL_INTERVAL);
            PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                    "Passive encoder ID %d responded after %d poll(s): pos=%.3f rad, digital_inputs=0x%02X",
                    id_, polls_sent, pos, digital);
            return ReturnCode::SUCCESS;
        }

        if (poll_outstanding_ && now - poll_sent_at_ >= PASSIVE_ENCODER_RESPONSE_DEADLINE) {
            poll_outstanding_ = false;
            next_poll_allowed_at_ = now + PASSIVE_ENCODER_RETRY_BACKOFF;
        }
        if (!poll_outstanding_ && now >= next_poll_allowed_at_) {
            ReturnCode return_code = send_poll_frame();
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to send startup poll to passive encoder ID %d", id_);
                return return_code;
            }
            poll_sent_at_ = now;
            poll_outstanding_ = true;
            next_poll_allowed_at_ = now + PASSIVE_ENCODER_POLL_INTERVAL;
            polls_sent++;
        }
        std::this_thread::sleep_for(PASSIVE_ENCODER_START_WAIT);
    }

    PI_ERROR("Passive encoder ID %d did not respond to %d bounded polls on CAN id %d (response id 0x%02X); "
             "is the teaching handle connected?",
             id_, polls_sent, id_, response_can_id_);
    return ReturnCode::NO_RESPONSE;
}

ReturnCode ServoCanPassiveEncoder::verify_position_fresh() {
    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    float pos = 0, vel = 0;
    uint8_t digital = 0;
    uint32_t update_count = 0;
    if (!p_driver_can_->get_received_encoder_data(data_index_, &pos, &vel, &digital, &update_count) ||
        update_count == 0) {
        PI_ERROR("Servo ID %d: passive encoder cache is stale (data_index=%d); no response ever parsed", id_,
                 data_index_);
        return ReturnCode::FAIL;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::move(float target_pos) {
    (void)target_pos;
    return ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::move(float target_pos, float target_vel, float target_tor) {
    (void)target_pos;
    (void)target_vel;
    (void)target_tor;
    return ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::apply_torque(float torque) {
    (void)torque;
    return ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::read_hardware_values() {
    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    prev_vel_ = curr_vel_;
    prev_tor_ = curr_tor_;
    pre_pos_abs_ = curr_pos_abs_;
    prev_pos_ = get_pos_rad_relative();

    float pos = 0, vel = 0;
    uint8_t digital = 0;
    uint32_t update_count = 0;
    if (!p_driver_can_->get_received_encoder_data(data_index_, &pos, &vel, &digital, &update_count)) {
        PI_ERROR("Servo ID %d: invalid data index %d", id_, data_index_);
        return ReturnCode::FAIL;
    }

    // i2rt convention: the trigger reading is the displacement MAGNITUDE
    // (|ticks|), so handles whose encoder counts negative when squeezed map
    // onto the same [0, pos_max] joint range without per-side sign calibration.
    curr_pos_abs_ = fabsf(pos);
    curr_vel_ = vel;
    curr_tor_ = 0;

    const auto now = Clock::now();
    const bool fresh = (update_count != last_update_count_);
    last_update_count_ = update_count;
    if (fresh) {
        const bool response_arrived_before_deadline = poll_outstanding_;
        const double silence_seconds =
            response_seen_ ? std::chrono::duration<double>(now - last_response_at_).count() : 0.0;
        if (silence_warned_ || restart_sent_) {
            PI_WARN("Servo ID %d: passive encoder recovered after %.1f s of silence; trigger was held",
                    id_, silence_seconds);
        }
        response_seen_ = true;
        poll_outstanding_ = false;
        last_response_at_ = now;
        silence_warned_ = false;
        restart_sent_ = false;
        if (response_arrived_before_deadline) {
            next_poll_allowed_at_ =
                std::max(next_poll_allowed_at_, poll_sent_at_ + PASSIVE_ENCODER_POLL_INTERVAL);
        }
        safe_mode_.exit_safe_mode(this, ReturnCode::SAFE_MODE_SIG);
    } else {
        if (poll_outstanding_ && now - poll_sent_at_ >= PASSIVE_ENCODER_RESPONSE_DEADLINE) {
            poll_outstanding_ = false;
            next_poll_allowed_at_ = now + PASSIVE_ENCODER_RETRY_BACKOFF;
        }

        if (response_seen_) {
            const auto silence = now - last_response_at_;
            const double silence_seconds = std::chrono::duration<double>(silence).count();
            if (!silence_warned_ && silence >= PASSIVE_ENCODER_SILENCE_WARN) {
                PI_WARN("Servo ID %d: passive encoder silent for %.1f s; holding last trigger value "
                        "(restart at 3 s, SIG at 20 s)",
                        id_, silence_seconds);
                silence_warned_ = true;
            }
            if (!restart_sent_ && silence >= PASSIVE_ENCODER_RESTART_AT) {
                PI_WARN("Servo ID %d: passive encoder still silent after %.1f s; sending firmware restart "
                        "(reboot takes ~8.5 s)",
                        id_, silence_seconds);
                ReturnCode restart_code = send_restart_frame();
                if (restart_code != ReturnCode::SUCCESS) {
                    PI_WARN("Servo ID %d: failed to send passive encoder firmware restart (rc=%d)", id_,
                            static_cast<int>(restart_code));
                }
                restart_sent_ = true;
                poll_outstanding_ = false;
                next_poll_allowed_at_ = now + PASSIVE_ENCODER_RETRY_BACKOFF;
            }
            if (silence >= PASSIVE_ENCODER_FAILURE_AT) {
                PI_ERROR("Servo ID %d: passive encoder silent for %.1f s (cable disconnected?)", id_,
                         silence_seconds);
                return safe_mode_.graceful_management(this, ReturnCode::SAFE_MODE_SIG);
            }
        }
    }

    if (!poll_outstanding_ && now >= next_poll_allowed_at_) {
        ReturnCode return_code = send_poll_frame();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to send poll frame to passive encoder ID %d", id_);
            return return_code;
        }
        poll_sent_at_ = now;
        poll_outstanding_ = true;
        next_poll_allowed_at_ = now + PASSIVE_ENCODER_POLL_INTERVAL;
    }

    return fresh ? publish_buttons(digital) : ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::parse_encoder_status(const DriverCan::can_frame_t& frame, int expected_encoder_id,
                                                        ReceivedServoData& slot) {
    if (frame.can_dlc < PASSIVE_ENCODER_RESPONSE_LEN) {
        PI_ERROR("Passive encoder response too short: %d bytes (expected %d)", frame.can_dlc,
                 PASSIVE_ENCODER_RESPONSE_LEN);
        return ReturnCode::INVALID_PARAM;
    }

    // Big-endian payload: device_id (u8), position (i16, ticks),
    // velocity (i16, ticks/s), digital_inputs (u8). The device_id byte is NOT
    // validated: YAM handle firmware reports 0 there regardless of the CAN id
    // (observed on hardware; the i2rt reference ignores the byte as well), so
    // routing trusts the registered response CAN id alone.
    const uint8_t* p_data = frame.data;
    const int16_t position_ticks = static_cast<int16_t>((p_data[1] << 8) | p_data[2]);
    const int16_t velocity_ticks = static_cast<int16_t>((p_data[3] << 8) | p_data[4]);

    slot.motor_id_ = expected_encoder_id;
    slot.error_ = 0;
    slot.angle_actual_rad_ = position_ticks * PASSIVE_ENCODER_RAD_PER_TICK;
    slot.speed_actual_rad_ = velocity_ticks * PASSIVE_ENCODER_RAD_PER_TICK;
    slot.current_actual_float_ = 0;
    slot.temperature_ = 0;
    slot.digital_inputs_ = p_data[5];
    slot.update_count_++;

    return ReturnCode::SUCCESS;
}

ReturnCode ServoCanPassiveEncoder::send_poll_frame() {
    DriverCan::can_frame_t frame{};
    frame.can_id = id_;
    frame.can_dlc = 2;
    frame.data[0] = PASSIVE_ENCODER_POLL_BYTE_0;
    frame.data[1] = PASSIVE_ENCODER_POLL_BYTE_1;
    return p_driver_can_->send_frame(&frame, sizeof(frame));
}

ReturnCode ServoCanPassiveEncoder::send_restart_frame() {
    DriverCan::can_frame_t frame{};
    frame.can_id = id_;
    frame.can_dlc = 2;
    frame.data[0] = PASSIVE_ENCODER_POLL_BYTE_0;  // ALL_DEVICE addressing, same as the poll
    frame.data[1] = PASSIVE_ENCODER_RESTART_BYTE_1;
    return p_driver_can_->send_frame(&frame, sizeof(frame));
}

ReturnCode ServoCanPassiveEncoder::publish_buttons(uint8_t digital_inputs) {
    if (!publish_joystick_enabled_) {
        return ReturnCode::SUCCESS;
    }

    MsgJoystick msg;
    // The YAM handle has no LEFT/RIGHT identity; subscribers key on the topic
    // name (one node per arm), so the in-payload side stays at the default.
    msg.side_ = MsgJoystick::Side::LEFT;
    msg.mode_ = 0;
    msg.button_.resize(button_num_);
    for (int i = 0; i < button_num_; i++) {
        msg.button_[i] = static_cast<int8_t>((digital_inputs >> i) & 0x01);
    }

    ReturnCode return_code = p_device_->publish_joystick(msg);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to publish joystick information (servo ID %d)", id_);
        return return_code;
    }
    return ReturnCode::SUCCESS;
}
