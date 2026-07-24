/*!
 * @file pi_driver_ft.cpp
 * @brief Implementation of the DriverFt class for FeeTech SMS/STS serial bus servos.
 */

#include "pi_driver_ft.hpp"

#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>

#include "pi_info.hpp"
#include "pi_servo_ft.hpp"

namespace {

// FeeTech SMS/STS wire protocol (mirrors pi_control/servos/ft_serial.py).
constexpr uint8_t kFtHeaderByte = 0xFF;
constexpr uint8_t kFtBroadcastId = 0xFE;

constexpr uint8_t kFtInstPing = 0x01;
constexpr uint8_t kFtInstRead = 0x02;
constexpr uint8_t kFtInstWrite = 0x03;
constexpr uint8_t kFtInstSyncRead = 0x82;
constexpr uint8_t kFtInstSyncWrite = 0x83;

// SMS/STS control table (subset used by the C++ control loop).
constexpr uint8_t kFtAddrPositionPGain = 21;
constexpr uint8_t kFtAddrPositionDGain = 22;
constexpr uint8_t kFtAddrPositionIGain = 23;
constexpr uint8_t kFtAddrOperatingMode = 33;
constexpr uint8_t kFtAddrTorqueEnable = 40;
constexpr uint8_t kFtAddrAcceleration = 41;
constexpr uint8_t kFtAddrGoalPosition = 42;
constexpr uint8_t kFtAddrTorqueLimit = 48;
constexpr uint8_t kFtAddrLockFlag = 55;
constexpr uint8_t kFtAddrPresentPosition = 56;

// Bulk feedback window: Present Position(2) + Velocity(2) + Load(2) +
// Voltage(1) + Temperature(1) + Async Write Flag(1) + Servo Status(1),
// contiguous at addresses 56..65.
constexpr uint8_t kFtFeedbackLength = 10;

// Registers below this address live in EEPROM on SMS/STS servos and require
// the Lock Flag (addr 55) to be cleared before a write takes effect.
constexpr uint8_t kFtSramStartAddr = 40;

constexpr uint8_t kFtLockFlagLocked = 1;
constexpr uint8_t kFtLockFlagUnlocked = 0;

// Sign-magnitude bit positions (STS3215 feedback encoding).
constexpr int kFtVelocitySignBit = 15;
constexpr int kFtLoadSignBit = 10;

// Per-packet response deadline. At 1 Mbps a 16-byte status packet takes
// ~0.16 ms on the wire; the dominant term is USB latency (up to one 16 ms
// FTDI/CDC latency window), so allow two windows plus margin.
constexpr int kFtResponseTimeoutMs = 40;

// Retry policy for single-register transactions; mirrors DriverDxl's
// kSingleRegMaxAttempts rationale (transient bus glitches on half-duplex
// serial should not abort a control-mode change).
constexpr int kFtSingleRegMaxAttempts = 3;

// Instruction packet: FF FF ID LEN INST [params] CHK.
constexpr size_t kFtInstructionOverhead = 6;
constexpr size_t kFtTxBufCapacity = 64;
constexpr size_t kFtMaxInstructionParams = kFtTxBufCapacity - kFtInstructionOverhead;

// Status packet parameter capacity (largest expected: the feedback window).
constexpr size_t kFtMaxStatusParams = 32;

constexpr int kFtGoalPositionMin = 0;
constexpr int kFtGoalPositionMax = 4095;
constexpr int kFtTorqueLimitMin = 0;
constexpr int kFtTorqueLimitMax = 1000;

uint8_t ft_checksum(const uint8_t* p_data, size_t length) {
    unsigned int sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += p_data[i];
    }
    return static_cast<uint8_t>(~sum & 0xFF);
}

}  // namespace

DriverFt::DriverFt(Device* p_device, const CommandLineArgs& cla) : DriverSerial(p_device, cla) {
    if (p_device == nullptr) {
        PI_ERROR("Device pointer is null in DriverFt constructor");
    }
}

DriverFt::~DriverFt() {}

int32_t DriverFt::decode_sign_magnitude(uint32_t value, int sign_bit) {
    const uint32_t sign_mask = 1u << sign_bit;
    if (value & sign_mask) {
        return -static_cast<int32_t>(value & (sign_mask - 1));
    }
    return static_cast<int32_t>(value);
}

uint32_t DriverFt::encode_sign_magnitude(int32_t value, int sign_bit) {
    if (value < 0) {
        return static_cast<uint32_t>(-value) | (1u << sign_bit);
    }
    return static_cast<uint32_t>(value);
}

ReturnCode DriverFt::open(int baud_rate) {
    ReturnCode return_code = DriverSerial::open(baud_rate);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (p_device_ == nullptr) {
        PI_ERROR("Device pointer is null in DriverFt::open()");
        return ReturnCode::NOT_INITIALIZED;
    }

    all_servo_ids_.clear();
    p_device_->get_servo_ids(all_servo_ids_);
    sync_read_ids_ = all_servo_ids_;
    dead_servo_ids_.clear();
    last_failed_servo_id_ = -1;

    servo_num_total_ = static_cast<int>(all_servo_ids_.size());
    PI_INFO("Driver", InfoLevel::ESSENTIAL_0, "Total number of FeeTech servos: %d", servo_num_total_);

    pres_pos_.assign(servo_num_total_, 0);
    pres_vel_.assign(servo_num_total_, 0);
    pres_load_.assign(servo_num_total_, 0);
    pres_temp_.assign(servo_num_total_, 0);
    pres_status_.assign(servo_num_total_, 0);

    pending_goal_position_.clear();
    pending_torque_limit_.clear();
    pending_goal_position_.reserve(servo_num_total_);
    pending_torque_limit_.reserve(servo_num_total_);

    if (servo_num_total_ == 0) {
        // Devices with no bus servos of their own (mirrors the DriverDxl empty
        // id-list tolerance); nothing to verify.
        return ReturnCode::SUCCESS;
    }

    // Verify SYNC READ support once (requires STS3215 firmware with the 0x82
    // instruction). Older firmware silently ignores it, which would otherwise
    // surface as a timeout on every control cycle -- fast-fail here instead.
    return_code = group_read_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR(
            "FeeTech SYNC READ verification failed on '%s'. Check servo power/cabling and that the "
            "servo firmware supports SYNC READ (0x82); there is no sequential-read fallback.",
            control_port_name_.c_str());
        return ReturnCode::FAIL;
    }

    PI_INFO("Driver", InfoLevel::ESSENTIAL_0, "DriverFt initialization complete: %d servos ready to operate",
            servo_num_total_);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFt::close() { return DriverSerial::close(); }

void DriverFt::flush_input() {
    if (port_handler_ >= 0) {
        ::tcflush(port_handler_, TCIFLUSH);
    }
}

size_t DriverFt::read_exact(uint8_t* p_buf, size_t length, int deadline_ms) {
    if (port_handler_ < 0 || p_buf == nullptr || length == 0) {
        return 0;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(deadline_ms);
    size_t received = 0;
    while (received < length) {
        const ssize_t n = ::read(port_handler_, p_buf + received, length - received);
        if (n > 0) {
            received += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            PI_ERROR("DriverFt::read_exact: read failed on '%s': %s", control_port_name_.c_str(),
                     std::strerror(errno));
            return received;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return received;
        }
        const int remain_ms =
            static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
        struct pollfd pfd {};
        pfd.fd = port_handler_;
        pfd.events = POLLIN;
        ::poll(&pfd, 1, remain_ms > 0 ? remain_ms : 1);
    }
    return received;
}

ReturnCode DriverFt::send_instruction(uint8_t id, uint8_t instruction, const uint8_t* p_params, size_t param_length) {
    if (param_length > kFtMaxInstructionParams) {
        PI_ERROR("DriverFt::send_instruction: param_length=%zu exceeds limit %zu (servo ID %d, instruction 0x%02X)",
                 param_length, kFtMaxInstructionParams, id, instruction);
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t packet[kFtTxBufCapacity];
    packet[0] = kFtHeaderByte;
    packet[1] = kFtHeaderByte;
    packet[2] = id;
    packet[3] = static_cast<uint8_t>(param_length + 2);
    packet[4] = instruction;
    if (param_length > 0) {
        std::memcpy(&packet[5], p_params, param_length);
    }
    // Checksum covers ID..params (bytes 2 .. 4+param_length).
    packet[5 + param_length] = ft_checksum(&packet[2], param_length + 3);

    return write_bytes(packet, kFtInstructionOverhead + param_length);
}

ReturnCode DriverFt::receive_status(uint8_t expected_id, uint8_t* p_params, size_t param_length, int timeout_ms,
                                    int* p_packet_id) {
    if (param_length > kFtMaxStatusParams) {
        PI_ERROR("DriverFt::receive_status: param_length=%zu exceeds capacity %zu", param_length, kFtMaxStatusParams);
        return ReturnCode::INVALID_PARAM;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    auto remaining_ms = [&deadline]() {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            return 0;
        }
        return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
    };

    // Resynchronize on the FF FF header, byte by byte. A servo ID is never
    // 0xFF (max 253), so the first non-FF byte after two or more FF bytes is
    // the packet ID.
    uint8_t packet_id = 0;
    int header_ff_count = 0;
    while (true) {
        uint8_t byte = 0;
        const int remain = remaining_ms();
        if (remain <= 0 || read_exact(&byte, 1, remain) != 1) {
            PI_ERROR("DriverFt: status packet timeout waiting for servo ID %d", expected_id);
            return ReturnCode::NO_RESPONSE;
        }
        if (byte == kFtHeaderByte) {
            header_ff_count++;
            continue;
        }
        if (header_ff_count >= 2) {
            packet_id = byte;
            break;
        }
        header_ff_count = 0;
    }

    uint8_t length_error[2] = {0, 0};
    if (read_exact(length_error, 2, remaining_ms()) != 2) {
        PI_ERROR("DriverFt: status packet truncated after header (servo ID %d)", packet_id);
        return ReturnCode::NO_RESPONSE;
    }
    const uint8_t packet_length = length_error[0];
    const uint8_t error_byte = length_error[1];

    if (packet_length != param_length + 2) {
        PI_ERROR("DriverFt: unexpected status length %d (expected %zu) from servo ID %d", packet_length,
                 param_length + 2, packet_id);
        flush_input();
        return ReturnCode::FAIL;
    }

    uint8_t body[kFtMaxStatusParams + 1];
    const size_t body_length = param_length + 1;  // params + checksum
    if (read_exact(body, body_length, remaining_ms()) != body_length) {
        PI_ERROR("DriverFt: status packet truncated in body (servo ID %d)", packet_id);
        return ReturnCode::NO_RESPONSE;
    }

    // Checksum covers ID, LEN, ERR and params.
    unsigned int sum = static_cast<unsigned int>(packet_id) + packet_length + error_byte;
    for (size_t i = 0; i < param_length; i++) {
        sum += body[i];
    }
    const uint8_t expected_checksum = static_cast<uint8_t>(~sum & 0xFF);
    if (body[param_length] != expected_checksum) {
        PI_ERROR("DriverFt: status checksum mismatch from servo ID %d (received 0x%02X, expected 0x%02X)", packet_id,
                 body[param_length], expected_checksum);
        flush_input();
        return ReturnCode::FAIL;
    }

    if (expected_id != kFtBroadcastId && packet_id != expected_id) {
        PI_ERROR("DriverFt: status packet from unexpected servo ID %d (expected %d)", packet_id, expected_id);
        flush_input();
        return ReturnCode::FAIL;
    }

    if (error_byte != 0) {
        // Servo answered but reports fault bits (voltage/angle/overheat/
        // overcurrent/overload). Surface them but treat the transaction as
        // completed; latched faults also show up in the Servo Status register.
        PI_WARN("DriverFt: servo ID %d status error bits 0x%02X", packet_id, error_byte);
    }

    if (param_length > 0 && p_params != nullptr) {
        std::memcpy(p_params, body, param_length);
    }
    if (p_packet_id != nullptr) {
        *p_packet_id = packet_id;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFt::txrx_instruction(uint8_t id, uint8_t instruction, const uint8_t* p_params, size_t param_length,
                                      uint8_t* p_rsp_params, size_t rsp_param_length) {
    ReturnCode return_code = ReturnCode::FAIL;
    for (int attempt = 1; attempt <= kFtSingleRegMaxAttempts; attempt++) {
        // Discard any stale bytes (late response of a previously timed-out
        // transaction) so they cannot be misread as this response.
        flush_input();

        return_code = send_instruction(id, instruction, p_params, param_length);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("DriverFt: failed to send instruction 0x%02X to servo ID %d", instruction, id);
            return return_code;
        }

        return_code = receive_status(id, p_rsp_params, rsp_param_length, kFtResponseTimeoutMs, nullptr);
        if (return_code == ReturnCode::SUCCESS) {
            return ReturnCode::SUCCESS;
        }

        if (attempt < kFtSingleRegMaxAttempts) {
            PI_WARN("DriverFt: transaction 0x%02X with servo ID %d failed (attempt %d/%d), retrying", instruction, id,
                    attempt, kFtSingleRegMaxAttempts);
        }
    }
    return return_code;
}

ReturnCode DriverFt::ping(int id) {
    return txrx_instruction(static_cast<uint8_t>(id), kFtInstPing, nullptr, 0, nullptr, 0);
}

ReturnCode DriverFt::read_register(int id, uint8_t address, uint8_t length, int32_t& value) {
    if (length != 1 && length != 2) {
        PI_ERROR("DriverFt::read_register: unsupported length %d (servo ID %d, address %d)", length, id, address);
        return ReturnCode::INVALID_PARAM;
    }

    const uint8_t params[2] = {address, length};
    uint8_t response[2] = {0, 0};
    ReturnCode return_code = txrx_instruction(static_cast<uint8_t>(id), kFtInstRead, params, 2, response, length);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("DriverFt: failed to read register %d (servo ID %d)", address, id);
        return return_code;
    }

    value = (length == 1) ? response[0] : static_cast<int32_t>(response[0] | (response[1] << 8));
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFt::write_register_raw(int id, uint8_t address, int32_t value, uint8_t length) {
    if (length != 1 && length != 2) {
        PI_ERROR("DriverFt::write_register: unsupported length %d (servo ID %d, address %d)", length, id, address);
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t params[3];
    params[0] = address;
    params[1] = static_cast<uint8_t>(value & 0xFF);
    size_t param_length = 2;
    if (length == 2) {
        params[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
        param_length = 3;
    }

    ReturnCode return_code = txrx_instruction(static_cast<uint8_t>(id), kFtInstWrite, params, param_length, nullptr, 0);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("DriverFt: failed to write register %d = %d (servo ID %d)", address, value, id);
    }
    return return_code;
}

ReturnCode DriverFt::write_register(int id, uint8_t address, int32_t value, uint8_t length) {
    if (address >= kFtSramStartAddr) {
        return write_register_raw(id, address, value, length);
    }

    // EEPROM register: the STS3215 ships write-locked (Lock Flag = 1) and
    // silently ignores EEPROM writes in that state, so wrap the write with an
    // unlock/re-lock pair.
    ReturnCode return_code = write_register_raw(id, kFtAddrLockFlag, kFtLockFlagUnlocked, 1);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("DriverFt: failed to unlock EEPROM before writing register %d (servo ID %d)", address, id);
        return return_code;
    }

    const ReturnCode write_result = write_register_raw(id, address, value, length);

    return_code = write_register_raw(id, kFtAddrLockFlag, kFtLockFlagLocked, 1);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("DriverFt: failed to re-lock EEPROM after writing register %d (servo ID %d)", address, id);
        return return_code;
    }

    return write_result;
}

ReturnCode DriverFt::sync_write(uint8_t address, uint8_t data_length,
                                const std::vector<std::pair<int, uint16_t>>& entries) {
    if (entries.empty()) {
        return ReturnCode::SUCCESS;
    }

    // SYNC WRITE params: ADDR, DATALEN, then (ID, DATA...) per servo.
    const size_t param_length = 2 + entries.size() * (1 + data_length);
    if (param_length > kFtMaxInstructionParams) {
        PI_ERROR("DriverFt::sync_write: packet too large for %zu servos (address %d)", entries.size(), address);
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t params[kFtMaxInstructionParams];
    params[0] = address;
    params[1] = data_length;
    size_t offset = 2;
    for (const auto& [id, value] : entries) {
        params[offset++] = static_cast<uint8_t>(id);
        params[offset++] = static_cast<uint8_t>(value & 0xFF);
        if (data_length == 2) {
            params[offset++] = static_cast<uint8_t>((value >> 8) & 0xFF);
        }
    }

    // SYNC WRITE is broadcast; servos do not answer.
    return send_instruction(kFtBroadcastId, kFtInstSyncWrite, params, param_length);
}

void DriverFt::classify_dead_servos() {
    bool need_probe = false;
    for (int id : all_servo_ids_) {
        if (dead_servo_ids_.count(id) == 0) {
            need_probe = true;
            break;
        }
    }
    if (need_probe) {
        for (int id : all_servo_ids_) {
            if (dead_servo_ids_.count(id) != 0) continue;
            if (ping(id) != ReturnCode::SUCCESS) {
                flush_input();
                dead_servo_ids_.insert(id);
                PI_ERROR("Identified dead FeeTech servo via ping: ID %d", id);
            }
        }
    }

    // Rebuild the SYNC READ id list so subsequent bulk reads only talk to the
    // alive servos (keeps their cached pres_* arrays fresh during recovery).
    sync_read_ids_.clear();
    for (int id : all_servo_ids_) {
        if (dead_servo_ids_.count(id) == 0) {
            sync_read_ids_.push_back(id);
        }
    }
    last_failed_servo_id_ = dead_servo_ids_.empty() ? -1 : *dead_servo_ids_.begin();
}

ReturnCode DriverFt::group_read_hardware_values() {
    // last_failed_servo_id_ / dead_servo_ids_ are sticky across cycles; see
    // DriverDxl::group_read_hardware_values() for the rationale.

    if (port_handler_ < 0) {
        PI_ERROR("Serial port is not open in group_read_hardware_values()");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (all_servo_ids_.empty()) {
        return ReturnCode::SUCCESS;
    }

    if (sync_read_ids_.empty()) {
        // Every servo is classified dead; nothing to read.
        return ReturnCode::FAIL;
    }

    // Build one SYNC READ over the feedback window for all alive servos.
    const size_t param_length = 2 + sync_read_ids_.size();
    if (param_length > kFtMaxInstructionParams) {
        PI_ERROR("DriverFt: too many servos (%zu) for one SYNC READ packet", sync_read_ids_.size());
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t params[kFtMaxInstructionParams];
    params[0] = kFtAddrPresentPosition;
    params[1] = kFtFeedbackLength;
    for (size_t i = 0; i < sync_read_ids_.size(); i++) {
        params[2 + i] = static_cast<uint8_t>(sync_read_ids_[i]);
    }

    flush_input();
    ReturnCode return_code = send_instruction(kFtBroadcastId, kFtInstSyncRead, params, param_length);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("DriverFt: failed to send SYNC READ packet");
        return return_code;
    }

    // Collect one status packet per servo. Responses arrive in the id-list
    // order, but a dropped response must not desynchronize the parse, so each
    // packet is matched by the ID it carries rather than by arrival order.
    std::set<int> pending_ids(sync_read_ids_.begin(), sync_read_ids_.end());
    uint8_t feedback[kFtFeedbackLength];
    for (size_t i = 0; i < sync_read_ids_.size(); i++) {
        int packet_id = -1;
        return_code = receive_status(kFtBroadcastId, feedback, kFtFeedbackLength, kFtResponseTimeoutMs, &packet_id);
        if (return_code != ReturnCode::SUCCESS) {
            break;
        }

        if (pending_ids.count(packet_id) == 0) {
            PI_ERROR("DriverFt: SYNC READ response from unexpected servo ID %d", packet_id);
            return_code = ReturnCode::FAIL;
            break;
        }
        pending_ids.erase(packet_id);

        const int data_index = find_data_index(packet_id);
        if (data_index < 0 || data_index >= servo_num_total_) {
            PI_ERROR("DriverFt: no data index registered for servo ID %d", packet_id);
            return_code = ReturnCode::FAIL;
            break;
        }

        pres_pos_[data_index] = static_cast<uint16_t>(feedback[0] | (feedback[1] << 8));
        pres_vel_[data_index] =
            decode_sign_magnitude(static_cast<uint32_t>(feedback[2] | (feedback[3] << 8)), kFtVelocitySignBit);
        pres_load_[data_index] =
            decode_sign_magnitude(static_cast<uint32_t>(feedback[4] | (feedback[5] << 8)), kFtLoadSignBit);
        // feedback[6] is Present Input Voltage (0.1 V); openpi has no consumer for it.
        pres_temp_[data_index] = feedback[7];
        pres_status_[data_index] = feedback[9];
    }

    if (return_code != ReturnCode::SUCCESS || !pending_ids.empty()) {
        for (int id : pending_ids) {
            PI_ERROR("DriverFt: no SYNC READ response from servo ID %d", id);
        }
        flush_input();
        classify_dead_servos();
        return ReturnCode::FAIL;
    }

    // Bulk read succeeded for every still-registered servo; refresh the
    // reported break point (dead_servo_ids_ stays sticky, see DriverDxl).
    last_failed_servo_id_ = dead_servo_ids_.empty() ? -1 : *dead_servo_ids_.begin();

    std::string info_pos, info_vel, info_load;
    for (int i = 0; i < servo_num_total_; i++) {
        info_pos += std::to_string(pres_pos_[i]) + ", ";
        info_vel += std::to_string(pres_vel_[i]) + ", ";
        info_load += std::to_string(pres_load_[i]) + ", ";
    }
    PI_INFO("Driver", InfoLevel::FREQUENT_3, "\nPositions (steps): %s\nVelocities (steps/s): %s\nLoads (0.1%%): %s",
            info_pos.c_str(), info_vel.c_str(), info_load.c_str());

    return ReturnCode::SUCCESS;
}

ReturnCode DriverFt::read_hardware_values(Servo* p_servo) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in read_hardware_values()");
        return ReturnCode::INVALID_PARAM;
    }

    if (servo_num_total_ <= 0) {
        PI_ERROR("Servo count is not properly initialized: %d", servo_num_total_);
        return ReturnCode::NOT_INITIALIZED;
    }

    if (p_servo->data_index_ < 0 || p_servo->data_index_ >= servo_num_total_) {
        PI_ERROR("Servo data index %d (servo ID %d) exceeds maximum index %d", p_servo->data_index_, p_servo->id_,
                 servo_num_total_ - 1);
        return ReturnCode::INVALID_PARAM;
    }

    ServoFt* p_servo_ft = (ServoFt*)p_servo;
    const int data_index = p_servo->data_index_;

    p_servo_ft->curr_pos_steps_ = pres_pos_[data_index];
    p_servo_ft->curr_pos_abs_ = p_servo_ft->steps_to_rad(pres_pos_[data_index]);
    p_servo_ft->curr_vel_ = p_servo_ft->get_vel_rad_sec(pres_vel_[data_index]);
    p_servo_ft->curr_tor_ = p_servo_ft->get_tor_nm(pres_load_[data_index]);
    p_servo_ft->temperature_ = static_cast<float>(pres_temp_[data_index]);
    // Servo Status error bits: bit0 voltage, bit1 angle/sensor, bit2 overheat,
    // bit3 overcurrent, bit5 overload (SMS/STS memory table). Raw value, no
    // decoding -- same convention as the other servo families.
    p_servo_ft->motor_error_code_ = pres_status_[data_index];

    return Driver::read_hardware_values(p_servo);
}

ReturnCode DriverFt::group_write_hardware_values() {
    if (port_handler_ < 0) {
        PI_ERROR("Serial port is not open in group_write_hardware_values()");
        return ReturnCode::NOT_INITIALIZED;
    }

    // Torque limit first, then goal position (mirrors DriverDxl's goal-current-
    // before-goal-position ordering so the limit is in place when motion starts).
    ReturnCode return_code = sync_write(kFtAddrTorqueLimit, 2, pending_torque_limit_);
    pending_torque_limit_.clear();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Group write for torque limit failed");
        pending_goal_position_.clear();
        return return_code;
    }

    return_code = sync_write(kFtAddrGoalPosition, 2, pending_goal_position_);
    pending_goal_position_.clear();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Group write for goal position failed");
        return return_code;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DriverFt::enable_torque(ServoFt* p_servo, bool enable) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in enable_torque()");
        return ReturnCode::INVALID_PARAM;
    }
    return write_register(p_servo->id_, kFtAddrTorqueEnable, enable ? 1 : 0, 1);
}

ReturnCode DriverFt::set_operating_mode(ServoFt* p_servo, uint8_t mode) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in set_operating_mode()");
        return ReturnCode::INVALID_PARAM;
    }
    return write_register(p_servo->id_, kFtAddrOperatingMode, mode, 1);
}

ReturnCode DriverFt::set_position_pid(ServoFt* p_servo, float pos_p, float pos_i, float pos_d) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in set_position_pid()");
        return ReturnCode::INVALID_PARAM;
    }

    auto to_gain_byte = [](float gain) {
        if (gain < 0.0f) gain = 0.0f;
        if (gain > 254.0f) gain = 254.0f;
        return static_cast<int32_t>(gain);
    };

    ReturnCode return_code = write_register(p_servo->id_, kFtAddrPositionPGain, to_gain_byte(pos_p), 1);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    return_code = write_register(p_servo->id_, kFtAddrPositionDGain, to_gain_byte(pos_d), 1);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    return write_register(p_servo->id_, kFtAddrPositionIGain, to_gain_byte(pos_i), 1);
}

ReturnCode DriverFt::set_acceleration(ServoFt* p_servo, uint8_t acceleration) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in set_acceleration()");
        return ReturnCode::INVALID_PARAM;
    }
    return write_register(p_servo->id_, kFtAddrAcceleration, acceleration, 1);
}

ReturnCode DriverFt::set_goal_position_direct(ServoFt* p_servo, int32_t goal_position) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in set_goal_position_direct()");
        return ReturnCode::INVALID_PARAM;
    }
    if (goal_position < kFtGoalPositionMin) goal_position = kFtGoalPositionMin;
    if (goal_position > kFtGoalPositionMax) goal_position = kFtGoalPositionMax;
    return write_register(p_servo->id_, kFtAddrGoalPosition, goal_position, 2);
}

ReturnCode DriverFt::set_goal_position_group(ServoFt* p_servo, int32_t goal_position) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in set_goal_position_group()");
        return ReturnCode::INVALID_PARAM;
    }
    if (dead_servo_ids_.count(p_servo->id_) != 0) {
        // Same policy as DriverDxl group writes: skip dead servos so the
        // control loop does not stall on their bus timeouts.
        return ReturnCode::SUCCESS;
    }
    if (goal_position < kFtGoalPositionMin) goal_position = kFtGoalPositionMin;
    if (goal_position > kFtGoalPositionMax) goal_position = kFtGoalPositionMax;
    pending_goal_position_.emplace_back(p_servo->id_, static_cast<uint16_t>(goal_position));
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFt::set_torque_limit_group(ServoFt* p_servo, int32_t torque_limit) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in set_torque_limit_group()");
        return ReturnCode::INVALID_PARAM;
    }
    if (dead_servo_ids_.count(p_servo->id_) != 0) {
        return ReturnCode::SUCCESS;
    }
    if (torque_limit < kFtTorqueLimitMin) torque_limit = kFtTorqueLimitMin;
    if (torque_limit > kFtTorqueLimitMax) torque_limit = kFtTorqueLimitMax;
    pending_torque_limit_.emplace_back(p_servo->id_, static_cast<uint16_t>(torque_limit));
    return ReturnCode::SUCCESS;
}

void DriverFt::discard_pending_group_writes(ServoFt* p_servo) {
    if (p_servo == nullptr) {
        return;
    }
    const int id = p_servo->id_;
    auto drop_id = [id](std::vector<std::pair<int, uint16_t>>& entries) {
        entries.erase(std::remove_if(entries.begin(), entries.end(),
                                     [id](const std::pair<int, uint16_t>& entry) { return entry.first == id; }),
                      entries.end());
    };
    drop_id(pending_goal_position_);
    drop_id(pending_torque_limit_);
}

ReturnCode DriverFt::get_torque_enabled(ServoFt* p_servo, int32_t& enabled) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in get_torque_enabled()");
        return ReturnCode::INVALID_PARAM;
    }
    return read_register(p_servo->id_, kFtAddrTorqueEnable, 1, enabled);
}

ReturnCode DriverFt::get_present_position(ServoFt* p_servo, int32_t& present_position) {
    if (p_servo == nullptr) {
        PI_ERROR("Servo pointer is null in get_present_position()");
        return ReturnCode::INVALID_PARAM;
    }
    return read_register(p_servo->id_, kFtAddrPresentPosition, 2, present_position);
}

ReturnCode DriverFt::sync_goal_position_to_present(ServoFt* p_servo) {
    int32_t present_position = 0;
    ReturnCode return_code = get_present_position(p_servo, present_position);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to read present position for goal sync (servo ID %d)", p_servo ? p_servo->id_ : -1);
        return return_code;
    }
    return set_goal_position_direct(p_servo, present_position);
}
