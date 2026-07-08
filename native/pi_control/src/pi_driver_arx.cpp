/*!
 * @file pi_driver_arx.cpp
 * @brief Implementation of the DriverArx class for ARX device CAN bus communication and servo control.
 */

#include <unistd.h>

#include <chrono>
#include <mutex>
#include <set>
#include <thread>
#include <tuple>

#include "pi_info.hpp"
#include "pi_servo.hpp"
#include "pi_servo_can_encoder.hpp"
#include "pi_servo_dm.hpp"
#include "pi_servo_dm_status.hpp"
#include "pi_driver_arx.hpp"

namespace {

constexpr uint8_t kPassiveEncoderAllDevices = 0xFF;
constexpr uint8_t kPassiveEncoderReqReportFrequency = 0x01;
constexpr uint8_t kPassiveEncoderReqVersion = 0x03;
constexpr uint8_t kPassiveEncoderReqAdcFrequency = 0x04;
constexpr uint8_t kPassiveEncoderReqReadings = 0x06;
constexpr uint8_t kPassiveEncoderReqGetEeprom = 0x07;
constexpr uint8_t kPassiveEncoderResponseFlag = 0x80;
constexpr uint8_t kPassiveEncoderAdcFrequencyHighOffset = 27;
constexpr uint8_t kPassiveEncoderAdcFrequencyLowOffset = 8;
constexpr uint8_t kPassiveEncoderReportFrequencyHighOffset = 28;
constexpr uint8_t kPassiveEncoderReportFrequencyLowOffset = 25;
constexpr int kPassiveEncoderRequiredAdcFrequency = 255;
constexpr int kPassiveEncoderRequiredReportFrequency = 0;
constexpr int kPassiveEncoderVersionTimeoutMs = 1000;
constexpr int kPassiveEncoderEepromTimeoutMs = 500;
constexpr int kPassiveEncoderDrainTimeoutMs = 50;
constexpr std::tuple<int, int, int> kPassiveEncoderMinimumFirmware{2, 2, 12};

int dm_response_motor_id(const DriverCan::can_frame_t& frame) {
    if (frame.can_dlc < 8) return -1;

    const uint32_t can_id = frame.can_id & 0x7FF;
    const int payload_motor_id = frame.data[0] & 0x0F;
    if (can_id == 0x00) {
        return payload_motor_id;
    }

    // YAM DM servos use the P16 response layout: motor N replies on CAN ID N + 0x10.
    if (can_id >= 0x11 && can_id <= 0x1F && static_cast<int>(can_id - 0x10) == payload_motor_id) {
        return payload_motor_id;
    }

    return -1;
}

}  // namespace

DriverArx::DriverArx(Device* p_device, const CommandLineArgs& cla) : DriverCan(p_device, cla) {
    memset(received_servo_data_, 0, sizeof(received_servo_data_));
}

DriverArx::~DriverArx() {}

ReturnCode DriverArx::open(int baud_rate) {
    (void)baud_rate;

    ReturnCode return_code = DriverCan::open(baud_rate);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to open CAN driver");
        return return_code;
    }

    return_code = configure_passive_encoders();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Passive encoder startup validation failed; refusing to enable motors");
        DriverCan::close();
        return return_code;
    }

    return_code = DriverCan::start_reception([this](void* p_data_buf, size_t data_buf_size, size_t read_bytes) {
        this->handle_received_message(p_data_buf, data_buf_size, read_bytes);
    });
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to start CAN reception");
        DriverCan::close();
        return return_code;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DriverArx::configure_passive_encoders() {
    // request_can_id -> firmware_compat. A single encoder may have several
    // routes; OR their flags so any route opting into compat mode enables it.
    std::map<int, bool> request_firmware_compat;
    {
        std::lock_guard<std::mutex> lock(received_servo_data_mutex_);
        for (const auto& [response_can_id, route] : passive_encoder_routes_) {
            (void)response_can_id;
            request_firmware_compat[route.encoder_id_] =
                request_firmware_compat[route.encoder_id_] || route.firmware_compat_;
        }
    }

    for (const auto& [request_can_id, firmware_compat] : request_firmware_compat) {
        ReturnCode return_code = configure_passive_encoder(request_can_id, firmware_compat);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    if (!request_firmware_compat.empty()) {
        drain_startup_frames();
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverArx::configure_passive_encoder(int request_can_id, bool firmware_compat) {
    const uint8_t version_request[] = {kPassiveEncoderAllDevices, kPassiveEncoderReqVersion};
    ReturnCode return_code =
        send_passive_encoder_request(request_can_id, version_request, sizeof(version_request));
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Passive encoder CAN id 0x%03X: failed to request firmware version", request_can_id);
        return return_code;
    }

    can_frame_t version_reply{};
    return_code = wait_for_passive_encoder_reply(
        request_can_id, -1, kPassiveEncoderReqVersion | kPassiveEncoderResponseFlag, 5,
        kPassiveEncoderVersionTimeoutMs, &version_reply);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Passive encoder CAN id 0x%03X: no valid firmware response", request_can_id);
        return return_code;
    }

    const uint8_t device = version_reply.data[0];
    const std::tuple<int, int, int> firmware{version_reply.data[2], version_reply.data[3], version_reply.data[4]};
    if (firmware < kPassiveEncoderMinimumFirmware) {
        PI_ERROR("Passive encoder CAN id 0x%03X device %u: firmware %u.%u.%u is unsupported; "
                 "required firmware is >=2.2.12",
                 request_can_id, device, version_reply.data[2], version_reply.data[3], version_reply.data[4]);
        return ReturnCode::NOT_SUPPORTED;
    }

    int adc_frequency = -1;
    int report_frequency = -1;
    return_code = read_passive_encoder_frequency(request_can_id, device, kPassiveEncoderAdcFrequencyHighOffset,
                                                 kPassiveEncoderAdcFrequencyLowOffset, &adc_frequency,
                                                 firmware_compat);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Passive encoder CAN id 0x%03X device %u: failed to read ADC frequency", request_can_id, device);
        return return_code;
    }
    return_code = read_passive_encoder_frequency(request_can_id, device, kPassiveEncoderReportFrequencyHighOffset,
                                                 kPassiveEncoderReportFrequencyLowOffset, &report_frequency,
                                                 firmware_compat);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Passive encoder CAN id 0x%03X device %u: failed to read report frequency", request_can_id,
                 device);
        return return_code;
    }

    if (adc_frequency != kPassiveEncoderRequiredAdcFrequency) {
        const uint8_t set_adc_frequency[] = {
            kPassiveEncoderAllDevices, kPassiveEncoderReqAdcFrequency, kPassiveEncoderRequiredAdcFrequency};
        PI_WARN("Passive encoder CAN id 0x%03X device %u: correcting ADC frequency from %d to %d",
                request_can_id, device, adc_frequency, kPassiveEncoderRequiredAdcFrequency);
        return_code = send_passive_encoder_request(request_can_id, set_adc_frequency, sizeof(set_adc_frequency));
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return_code = read_passive_encoder_frequency(request_can_id, device, kPassiveEncoderAdcFrequencyHighOffset,
                                                     kPassiveEncoderAdcFrequencyLowOffset, &adc_frequency,
                                                     firmware_compat);
        if (return_code != ReturnCode::SUCCESS || adc_frequency != kPassiveEncoderRequiredAdcFrequency) {
            PI_ERROR("Passive encoder CAN id 0x%03X device %u: ADC frequency verification failed (read %d, "
                     "expected %d)",
                     request_can_id, device, adc_frequency, kPassiveEncoderRequiredAdcFrequency);
            return (return_code == ReturnCode::SUCCESS) ? ReturnCode::FAIL : return_code;
        }
    }

    if (report_frequency != kPassiveEncoderRequiredReportFrequency) {
        const uint8_t set_report_frequency[] = {
            kPassiveEncoderAllDevices, kPassiveEncoderReqReportFrequency, kPassiveEncoderRequiredReportFrequency};
        PI_WARN("Passive encoder CAN id 0x%03X device %u: correcting report frequency from %d to %d",
                request_can_id, device, report_frequency, kPassiveEncoderRequiredReportFrequency);
        return_code =
            send_passive_encoder_request(request_can_id, set_report_frequency, sizeof(set_report_frequency));
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        return_code = read_passive_encoder_frequency(request_can_id, device,
                                                     kPassiveEncoderReportFrequencyHighOffset,
                                                     kPassiveEncoderReportFrequencyLowOffset, &report_frequency,
                                                     firmware_compat);
        if (return_code != ReturnCode::SUCCESS || report_frequency != kPassiveEncoderRequiredReportFrequency) {
            PI_ERROR("Passive encoder CAN id 0x%03X device %u: report frequency verification failed (read %d, "
                     "expected %d)",
                     request_can_id, device, report_frequency, kPassiveEncoderRequiredReportFrequency);
            return (return_code == ReturnCode::SUCCESS) ? ReturnCode::FAIL : return_code;
        }
    }

    PI_INFO("Driver", InfoLevel::ESSENTIAL_0,
            "Passive encoder CAN id 0x%03X device %u ready: firmware=%u.%u.%u adc_frequency=%d "
            "report_frequency=%d",
            request_can_id, device, version_reply.data[2], version_reply.data[3], version_reply.data[4],
            adc_frequency, report_frequency);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverArx::send_passive_encoder_request(int request_can_id, const uint8_t* p_data, uint8_t data_len) {
    if (p_data == nullptr || data_len == 0 || data_len > 8) {
        return ReturnCode::INVALID_PARAM;
    }

    can_frame_t frame{};
    frame.can_id = request_can_id;
    frame.can_dlc = data_len;
    memcpy(frame.data, p_data, data_len);
    return send_frame(&frame, sizeof(frame));
}

ReturnCode DriverArx::wait_for_passive_encoder_reply(int request_can_id, int expected_device,
                                                     uint8_t expected_command, uint8_t expected_len,
                                                     int timeout_ms, can_frame_t* p_reply) {
    if (p_reply == nullptr) {
        return ReturnCode::INVALID_PARAM;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        can_frame_t frame{};
        ReturnCode return_code = read_frame(&frame, sizeof(frame));
        if (return_code == ReturnCode::NO_RESPONSE) {
            continue;
        }
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
        if (static_cast<int>(frame.can_id & 0x7FF) != request_can_id || frame.can_dlc != expected_len ||
            frame.data[1] != expected_command ||
            (expected_device >= 0 && frame.data[0] != static_cast<uint8_t>(expected_device))) {
            continue;
        }
        *p_reply = frame;
        return ReturnCode::SUCCESS;
    }
    return ReturnCode::NO_RESPONSE;
}

ReturnCode DriverArx::read_passive_encoder_eeprom(int request_can_id, uint8_t device, uint8_t offset,
                                                  uint8_t* p_value, bool firmware_compat) {
    if (p_value == nullptr) {
        return ReturnCode::INVALID_PARAM;
    }

    const uint8_t request[] = {device, kPassiveEncoderReqGetEeprom, offset};
    ReturnCode return_code = send_passive_encoder_request(request_can_id, request, sizeof(request));
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (!firmware_compat) {
        // Original behavior: the encoder replies to GET_EEPROM in the READINGS
        // response format (command 0x86, 5 bytes, big-endian int16 value).
        can_frame_t reply{};
        return_code = wait_for_passive_encoder_reply(
            request_can_id, device, kPassiveEncoderReqReadings | kPassiveEncoderResponseFlag, 5,
            kPassiveEncoderEepromTimeoutMs, &reply);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
        const int16_t value = static_cast<int16_t>((reply.data[2] << 8) | reply.data[3]);
        *p_value = static_cast<uint8_t>(value & 0xFF);
        return ReturnCode::SUCCESS;
    }

    // Compat mode: firmware <=2.2.x replies to GET_EEPROM in the READINGS
    // response format (command 0x86, 5 bytes, big-endian int16 value); firmware
    // >=2.3.x uses a dedicated GET_EEPROM response (command 0x87, 3 bytes,
    // single value byte). Accept both.
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(kPassiveEncoderEepromTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        can_frame_t frame{};
        return_code = read_frame(&frame, sizeof(frame));
        if (return_code == ReturnCode::NO_RESPONSE) {
            continue;
        }
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
        if (static_cast<int>(frame.can_id & 0x7FF) != request_can_id || frame.data[0] != device) {
            continue;
        }
        if (frame.can_dlc == 5 &&
            frame.data[1] == (kPassiveEncoderReqReadings | kPassiveEncoderResponseFlag)) {
            const int16_t value = static_cast<int16_t>((frame.data[2] << 8) | frame.data[3]);
            *p_value = static_cast<uint8_t>(value & 0xFF);
            return ReturnCode::SUCCESS;
        }
        if (frame.can_dlc == 3 &&
            frame.data[1] == (kPassiveEncoderReqGetEeprom | kPassiveEncoderResponseFlag)) {
            *p_value = frame.data[2];
            return ReturnCode::SUCCESS;
        }
    }
    return ReturnCode::NO_RESPONSE;
}

ReturnCode DriverArx::read_passive_encoder_frequency(int request_can_id, uint8_t device, uint8_t high_offset,
                                                     uint8_t low_offset, int* p_frequency, bool firmware_compat) {
    if (p_frequency == nullptr) {
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t high = 0;
    uint8_t low = 0;
    ReturnCode return_code = read_passive_encoder_eeprom(request_can_id, device, high_offset, &high, firmware_compat);
    if (firmware_compat && return_code == ReturnCode::NO_RESPONSE) {
        // Firmware <=2.2.12 exposes a 27-byte EEPROM (offsets 0-26): the 16-bit
        // frequency high bytes at offsets 27/28 do not exist and reads for them
        // get no reply. Treat a silent high byte like the 0xFF "uninitialized"
        // marker and fall back to the 8-bit value in the low byte.
        PI_WARN("Passive encoder CAN id 0x%03X device %u: EEPROM offset %u not implemented "
                "(legacy 8-bit layout); using 8-bit frequency from offset %u",
                request_can_id, device, high_offset, low_offset);
        high = 0xFF;
    } else if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    return_code = read_passive_encoder_eeprom(request_can_id, device, low_offset, &low, firmware_compat);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    *p_frequency = (high == 0xFF) ? low : ((static_cast<int>(high) << 8) | low);
    return ReturnCode::SUCCESS;
}

void DriverArx::drain_startup_frames() {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(kPassiveEncoderDrainTimeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        can_frame_t frame{};
        ReturnCode return_code = read_frame(&frame, sizeof(frame));
        if (return_code == ReturnCode::NO_RESPONSE) {
            return;
        }
        if (return_code != ReturnCode::SUCCESS) {
            PI_WARN("Failed while draining passive-encoder startup frames (rc=%d)", static_cast<int>(return_code));
            return;
        }
    }
}

ReturnCode DriverArx::close() {
    ReturnCode return_code = DriverCan::close();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to close CAN driver");
        return return_code;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DriverArx::send_command(ServoDm* p_servo_dm, float kp, float kd, float position, float velocity,
                                   float torque) {
    if (p_servo_dm == nullptr) {
        PI_ERROR("Invalid servo pointer in send_command()");
        return ReturnCode::INVALID_PARAM;
    }

    std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
    if (is_socket_open()) {
        can_frame_t frame;
        switch ((int)p_servo_dm->get_servo_type()) {
            case (int)ServoType::ENCOS_A4310: {
                ReturnCode return_code = ServoDm::can_frame_to_command_encos_servo(
                    frame, p_servo_dm->id_, kp, kd, position, velocity, torque);
                if (return_code != ReturnCode::SUCCESS) {
                    return return_code;
                }
                return send_frame(&frame, sizeof(frame));
            }
            case (int)ServoType::DM_4340:
            case (int)ServoType::DM_4310: {
                ReturnCode return_code = ServoDm::can_frame_to_command_dm_servo(
                    frame, p_servo_dm->id_, kp, kd, position, velocity, torque);
                if (return_code != ReturnCode::SUCCESS) {
                    return return_code;
                }
                return send_frame(&frame, sizeof(frame));
            }
            default:
                PI_ERROR("Unsupported servo type: %d", (int)p_servo_dm->get_servo_type());
                return ReturnCode::NOT_SUPPORTED;
        }
    } else {
        PI_ERROR("CAN socket is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }
}

ReturnCode DriverArx::enable(int id, int type, bool enable_flag, bool defer_effector_thermal_fault) {
    std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
    last_enable_fault_status_ = -1;
    if (is_socket_open()) {
        can_frame_t frame;
        switch (type) {
            case static_cast<int>(ServoType::ENCOS_A4310): {
                ReturnCode return_code = ServoDm::can_frame_to_command_encos_servo(frame, id, 0, 0, 0, 0, 0);
                if (return_code != ReturnCode::SUCCESS) {
                    return return_code;
                }
                return_code = send_frame(&frame, sizeof(frame));
                if (return_code != ReturnCode::SUCCESS) {
                    return return_code;
                }
                // Give the asynchronous receive thread time to parse the first status frame
                // triggered by this zero-effort enable command into ``received_servo_data_``;
                // otherwise the subsequent ``read_hardware_values()`` may race the parser and
                // return pos=0, causing an abrupt first ready-move command.
                usleep(ENABLE_ENCOS_CACHE_WAIT_US);
                break;
            }

            default:
                ServoDm::can_frame_to_set_operation_mode_dm_servo(frame, id, ServoDm::OperationMode::MIT);
                ReturnCode return_code = send_frame(&frame, sizeof(frame));
                if (return_code != ReturnCode::SUCCESS) {
                    return return_code;
                }

                usleep(100);

                can_frame_t enable_frame;
                ServoDm::can_frame_to_enable_dm_servo(enable_frame, id, enable_flag);

                int normal_code = (enable_flag) ? 0x10 : 0x0;

                stop_reception();

                ReturnCode handshake_code = [&]() -> ReturnCode {
                    // With servos unpowered/disconnected, waiting too long here looks like a hang.
                    // `read_frame()` has a small timeout, so this caps total wait per-servo.
                    int max_retry_count = 10;
                    int max_response_count = MAX_SERVO_INFO_BUF_SIZE;
                    int retry_count = 0;
                    int last_non_normal_status = -1;
                    bool acknowledged = false;
                    bool startup_fault_reset_attempted = false;

                    auto send_reset_sequence = [&](int attempt) -> ReturnCode {
                        can_frame_t reset_frame;
                        ReturnCode ret_code = ServoDm::can_frame_to_reset_dm_servo(reset_frame, id);
                        if (ret_code != ReturnCode::SUCCESS) {
                            PI_ERROR("Failed to construct reset frame for servo id=%d (retry %d/%d)", id,
                                     attempt, max_retry_count);
                            return ret_code;
                        }
                        for (int reset_count = 0; reset_count < 3; reset_count++) {
                            ret_code = send_frame(&reset_frame, sizeof(reset_frame));
                            if (ret_code != ReturnCode::SUCCESS) {
                                PI_ERROR("Failed to send reset frame for servo id=%d "
                                         "(retry %d/%d, reset %d/3)",
                                         id, attempt, max_retry_count, reset_count + 1);
                                return ret_code;
                            }
                            usleep(100);
                        }
                        return ReturnCode::SUCCESS;
                    };

                    for (; retry_count < max_retry_count; retry_count++) {
                        ReturnCode ret_code = send_frame(&enable_frame, sizeof(enable_frame));
                        if (ret_code == ReturnCode::BUSY) {
                            // Socket not writable within timeout -> retry.
                            PI_WARN("Servo id=%d: enable attempt %d/%d: CAN socket not writable; retrying",
                                    id, retry_count + 1, max_retry_count);
                            continue;
                        }
                        if (ret_code != ReturnCode::SUCCESS) {
                            PI_ERROR("Failed to send enable frame for servo id=%d (retry %d/%d)", id, retry_count,
                                     max_retry_count);
                            return ret_code;
                        }

                        // Per-attempt response accounting so a retrying servo names itself in the
                        // log (id, attempt, why each attempt failed) instead of retrying silently
                        // until the final NO_RESPONSE (issue #5).
                        int frames_other_id = 0;
                        int frames_invalid = 0;
                        int last_error_nibble = -1;
                        bool timed_out = false;

                        for (int response_count = 0; response_count < max_response_count; response_count++) {
                            can_frame_t response_frame;
                            ret_code = read_frame(&response_frame, sizeof(response_frame));
                            if (ret_code == ReturnCode::NO_RESPONSE) {
                                // No matching response within timeout -> retry sending enable frame.
                                timed_out = true;
                                break;
                            }
                            if (ret_code != ReturnCode::SUCCESS) {
                                PI_ERROR("Failed to read response frame for servo id=%d (retry %d/%d)", id,
                                         retry_count, max_retry_count);
                                return ret_code;
                            }

                            const int parsed_motor_id = dm_response_motor_id(response_frame);
                            if (parsed_motor_id < 0) {
                                ++frames_invalid;
                                continue;
                            }

                            if (parsed_motor_id != id) {
                                ++frames_other_id;
                                PI_INFO("Driver", InfoLevel::DETAIL_2,
                                        "Ignoring enable response for motor_id=%d while waiting for id=%d",
                                        parsed_motor_id, id);
                                continue;
                            }

                            const int status_code = response_frame.data[0] >> 4;
                            // Preserve the terminal snapshot even when an enable response
                            // is a hardware fault. ServoDm uses this cache to report
                            // measured position, effort, current, and temperature before
                            // latching an effector thermal stop.
                            if (enable_flag) {
                                // Reception is stopped here, but lock anyway so the
                                // cache has a single locking discipline.
                                std::lock_guard<std::mutex> lock(received_servo_data_mutex_);
                                ReturnCode parse_rc = ServoDm::parse_dm_servo_status(
                                    &response_frame, received_servo_data_,
                                    &DriverArx::find_data_index, this);
                                if (parse_rc != ReturnCode::SUCCESS) {
                                    PI_WARN("Failed to parse enable response status for servo id=%d (rc=%d)",
                                            id, static_cast<int>(parse_rc));
                                }
                            }

                            if (status_code == (normal_code >> 4)) {
                                acknowledged = true;
                                break;
                            }

                            last_error_nibble = status_code;
                            last_non_normal_status = status_code;
                            const DmServoStatusInfo& status = dm_servo_status_info(status_code);
                            if (status.is_fault) {
                                if (enable_flag && status.is_resettable_on_enable &&
                                    !startup_fault_reset_attempted) {
                                    startup_fault_reset_attempted = true;
                                    PI_WARN("Servo id=%d: enable attempt %d/%d reported 0x%X (%s); "
                                            "sending one startup reset sequence and retrying",
                                            id, retry_count + 1, max_retry_count,
                                            static_cast<unsigned>(status_code), status.description);
                                    ret_code = send_reset_sequence(retry_count + 1);
                                    if (ret_code != ReturnCode::SUCCESS) {
                                        return ret_code;
                                    }
                                    break;
                                }
                                last_enable_fault_status_ = status_code;
                                if (status.is_thermal_fault && defer_effector_thermal_fault) {
                                    return ReturnCode::HARDWARE_FAULT;
                                }
                                PI_ERROR("HARDWARE FAULT: DM servo id=%d reported status 0x%X (%s) while %s "
                                         "(reported temperature=%u C). Action: %s",
                                         id, static_cast<unsigned>(status_code), status.description,
                                         enable_flag ? "enabling" : "disabling",
                                         static_cast<unsigned>(response_frame.data[7]), status.action);
                                return ReturnCode::HARDWARE_FAULT;
                            }

                            PI_WARN("Servo id=%d: enable attempt %d/%d: status frame reported 0x%X (%s; "
                                    "expected 0x%X); sending reset and retrying",
                                    id, retry_count + 1, max_retry_count, static_cast<unsigned>(status_code),
                                    status.description, static_cast<unsigned>(normal_code >> 4));

                            ret_code = send_reset_sequence(retry_count + 1);
                            if (ret_code != ReturnCode::SUCCESS) {
                                return ret_code;
                            }
                            break;
                        }

                        if (acknowledged) {
                            break;
                        }

                        if (timed_out && frames_other_id == 0 && frames_invalid == 0 && last_error_nibble < 0) {
                            PI_WARN("Servo id=%d: enable attempt %d/%d: no response within timeout; retrying",
                                    id, retry_count + 1, max_retry_count);
                        } else if (last_error_nibble < 0) {
                            // Frames arrived but none was a valid response for this servo -- the
                            // classic shape of a chain member answering with frames that fail
                            // validation. Summarise what was seen so the culprit is identifiable.
                            PI_WARN("Servo id=%d: enable attempt %d/%d: no valid response "
                                    "(%d frame(s) from other ids, %d unparseable frame(s), timed_out=%d); retrying",
                                    id, retry_count + 1, max_retry_count, frames_other_id, frames_invalid,
                                    (int)timed_out);
                        }
                    }

                    if (!acknowledged) {
                        if (last_non_normal_status >= 0) {
                            const DmServoStatusInfo& status = dm_servo_status_info(last_non_normal_status);
                            PI_ERROR("Servo id=%d responded but could not be %s after %d attempts: "
                                     "last status 0x%X (%s; expected 0x%X)",
                                     id, enable_flag ? "enabled" : "disabled", max_retry_count,
                                     static_cast<unsigned>(last_non_normal_status), status.description,
                                     static_cast<unsigned>(normal_code >> 4));
                            return ReturnCode::FAIL;
                        }
                        PI_ERROR("No response while %s servo id=%d after %d retries",
                                 enable_flag ? "enabling" : "disabling", id, max_retry_count);
                        return ReturnCode::NO_RESPONSE;
                    }

                    return ReturnCode::SUCCESS;
                }();

                ReturnCode restart_code =
                    DriverCan::start_reception([this](void* p_data_buf, size_t data_buf_size, size_t read_bytes) {
                        this->handle_received_message(p_data_buf, data_buf_size, read_bytes);
                    });
                if (restart_code != ReturnCode::SUCCESS) {
                    PI_ERROR("Failed to restart CAN reception after changing enable state for servo id=%d", id);
                    return restart_code;
                }
                if (handshake_code != ReturnCode::SUCCESS) {
                    return handshake_code;
                }

                break;
        }

        usleep(100);

    } else {
        PI_ERROR("CAN socket is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverArx::send_disable_once(int id, int type) {
    std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
    if (!is_socket_open()) {
        PI_ERROR("CAN socket is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }
    if (type != static_cast<int>(ServoType::DM_4340) &&
        type != static_cast<int>(ServoType::DM_4310)) {
        PI_ERROR("Unsupported servo type for one-shot disable: %d", type);
        return ReturnCode::NOT_SUPPORTED;
    }

    can_frame_t frame;
    ReturnCode return_code = ServoDm::can_frame_to_enable_dm_servo(frame, id, false);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to construct one-shot disable frame for servo id=%d", id);
        return return_code;
    }
    return_code = send_frame(&frame, sizeof(frame));
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to send one-shot disable frame for servo id=%d", id);
    }
    return return_code;
}

ReturnCode DriverArx::register_passive_encoder(int response_can_id, int encoder_id, int data_index,
                                               bool firmware_compat) {
    if (data_index < 0 || data_index >= MAX_SERVO_INFO_BUF_SIZE) {
        PI_ERROR("Passive encoder id=%d: data_index %d out of range [0, %d)", encoder_id, data_index,
                 MAX_SERVO_INFO_BUF_SIZE);
        return ReturnCode::INVALID_PARAM;
    }

    // The encoder route is checked before the DM/ENCOS heuristics, so a
    // response id inside those ranges would shadow a motor's status frames.
    // Legal (registration is explicit and wins on purpose), but worth a loud
    // warning because it almost certainly means a miswired id assignment.
    const bool inside_dm_range = (response_can_id == 0x00) || (response_can_id >= 0x11 && response_can_id <= 0x1F);
    const bool inside_encos_range = (response_can_id >= 0x01 && response_can_id <= 0x07);
    if (inside_dm_range || inside_encos_range) {
        PI_WARN("Passive encoder id=%d: response CAN id 0x%02X overlaps the %s status-frame range; "
                "encoder routing takes precedence and will shadow that motor",
                encoder_id, response_can_id, inside_dm_range ? "DM" : "ENCOS");
    }

    std::lock_guard<std::mutex> lock(received_servo_data_mutex_);
    passive_encoder_routes_[response_can_id] = PassiveEncoderRoute{encoder_id, data_index, firmware_compat};
    PI_INFO("Driver", InfoLevel::HELPFUL_1,
            "Registered passive encoder route: encoder_id=%d response_can_id=0x%02X data_index=%d firmware_compat=%d",
            encoder_id, response_can_id, data_index, firmware_compat ? 1 : 0);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverArx::reset_zero_position(int id, int type) {
    std::lock_guard<std::mutex> transaction_lock(transaction_mutex_);
    if (is_socket_open()) {
        can_frame_t frame;
        switch (type) {
            case static_cast<int>(ServoType::DM_4340):
            case static_cast<int>(ServoType::DM_4310):
                ServoDm::can_frame_to_set_zero_dm_servo(frame, id);
                {
                    ReturnCode return_code = send_frame(&frame, sizeof(frame));
                    if (return_code != ReturnCode::SUCCESS) {
                        return return_code;
                    }
                }
                break;
            case static_cast<int>(ServoType::ENCOS_A4310):
                ///< @todo Implement zero position reset for ENCOS servos
                break;
            default:
                PI_ERROR("Unsupported servo type for zero position reset: %d", type);
                return ReturnCode::NOT_SUPPORTED;
        }

        usleep(1000);
    } else {
        PI_ERROR("CAN socket is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverArx::read_hardware_values(Servo* p_servo) {
    if (p_servo == nullptr) {
        PI_ERROR("Invalid servo pointer in read_hardware_values()");
        return ReturnCode::FAIL;
    }
    ServoDm* p_servo_dm = dynamic_cast<ServoDm*>(p_servo);
    if (p_servo_dm == nullptr) {
        PI_ERROR("Unsupported servo type in read_hardware_values()");
        return ReturnCode::INVALID_PARAM;
    }
    if (p_servo_dm->data_index_ < 0 || p_servo_dm->data_index_ >= MAX_SERVO_INFO_BUF_SIZE) {
        PI_ERROR("Servo id=%d: data_index %d out of range [0, %d)", p_servo_dm->id_, p_servo_dm->data_index_,
                 MAX_SERVO_INFO_BUF_SIZE);
        return ReturnCode::INVALID_PARAM;
    }

    bool cache_fresh = false;
    {
        // Snapshot under the cache lock so one frame's fields stay coherent —
        // the reception thread may be parsing a newer frame concurrently.
        std::lock_guard<std::mutex> lock(received_servo_data_mutex_);
        const ReceivedServoData& data = received_servo_data_[p_servo_dm->data_index_];
        p_servo_dm->curr_pos_abs_ = data.angle_actual_rad_;
        p_servo_dm->curr_vel_ = data.speed_actual_rad_;
        p_servo_dm->curr_tor_ = data.current_actual_float_;
        p_servo_dm->temperature_ = data.temperature_;
        // Propagate the native motor/drive error nibble parsed by ServoDm::parse_dm_servo_status (DM, 4 bits)
        // or ServoDm::parser_encos_servo_status (ENCOS, 5 bits). Raw value: no decoding so analysis tools can
        // map it directly to the datasheet tables.
        p_servo_dm->motor_error_code_ = data.error_;
        cache_fresh = data.motor_id_ != 0;
    }

    if (cache_fresh) {
        ReturnCode return_code = p_servo_dm->initialize_position_wrap();
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    ///< @note Input DC current is calculated in the base class Driver::read_hardware_values()

    return DriverCan::read_hardware_values(p_servo);
}

void DriverArx::handle_received_message(void* p_data_buf, size_t data_buf_size, size_t read_bytes) {
    if (p_data_buf == nullptr) {
        PI_ERROR("Invalid data buffer in handle_received_message()");
        return;
    }
    if (data_buf_size < sizeof(can_frame_t) || read_bytes < sizeof(can_frame_t)) {
        PI_ERROR("Truncated CAN frame in handle_received_message(): buffer=%zu, read=%zu, expected=%zu",
                 data_buf_size, read_bytes, sizeof(can_frame_t));
        return;
    }

    can_frame_t* p_frame = (can_frame_t*)p_data_buf;

    // DM replies use either master response ID 0x00 or the P16 layout
    // (motor N responds on N + 0x10). ENCOS replies use CAN IDs 0x01..0x07.
    // Drop other bus participants instead of corrupting the servo cache.
    ReturnCode return_code;

    // This runs on the CAN reception thread; the parsers below write the
    // servo cache that the main control loop reads.
    std::lock_guard<std::mutex> lock(received_servo_data_mutex_);

    // Explicitly registered passive-encoder routes win over the DM/ENCOS
    // heuristics: the encoder's response id is configured, not inferred.
    const auto encoder_it = passive_encoder_routes_.find(static_cast<int>(p_frame->can_id & 0x7FF));
    if (encoder_it != passive_encoder_routes_.end()) {
        const PassiveEncoderRoute& route = encoder_it->second;
        return_code = ServoCanPassiveEncoder::parse_encoder_status(*p_frame, route.encoder_id_,
                                                                   received_servo_data_[route.data_index_]);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to parse passive encoder response (CAN ID: 0x%02X)", p_frame->can_id);
        }
        return;
    }

    if (dm_response_motor_id(*p_frame) >= 0) {
        return_code =
            ServoDm::parse_dm_servo_status(p_frame, received_servo_data_, &DriverArx::find_data_index, this);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to parse DM servo status message (CAN ID: 0x%02X)", p_frame->can_id);
        }
        return;
    }

    switch (p_frame->can_id) {
        case 0x01:
        case 0x02:
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x06:
        case 0x07:
            return_code =
                ServoDm::parser_encos_servo_status(p_frame, received_servo_data_, &DriverArx::find_data_index);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to parse ENCOS servo status message (CAN ID: 0x%02X)", p_frame->can_id);
                return;
            }
            break;

        default:
            // Unknown CAN ID: not a servo response channel. Drop silently
            // -- the only known non-servo participant today is the
            // a6_power_control board on 0x80, which would otherwise be
            // mis-parsed as DM status and corrupt joint state.
            return;
    }
}
