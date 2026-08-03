#include "pi_robotiq.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>

#include <modbus.h>

#include "pi_info.hpp"

struct RobotiqTransport::Impl {
    modbus_t* context = nullptr;
};

RobotiqTransport::RobotiqTransport(RobotiqConfig config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>()) {}

RobotiqTransport::~RobotiqTransport() { stop(); }

bool RobotiqTransport::start() {
    if (config_.endpoint.empty()) return false;
    if (config_.backend == RobotiqBackend::RTU) {
        impl_->context = modbus_new_rtu(config_.endpoint.c_str(), config_.baud_rate, 'N', 8, 1);
    } else {
        impl_->context = modbus_new_tcp(config_.endpoint.c_str(), config_.tcp_port);
    }
    if (!impl_->context) return false;
    const uint32_t timeout_seconds = static_cast<uint32_t>(config_.response_timeout_ms / 1000);
    const uint32_t timeout_microseconds = static_cast<uint32_t>(config_.response_timeout_ms % 1000) * 1000U;
    if (modbus_set_slave(impl_->context, config_.slave_id) != 0 ||
        modbus_set_response_timeout(impl_->context, timeout_seconds, timeout_microseconds) != 0 ||
        modbus_connect(impl_->context) != 0) {
        modbus_free(impl_->context);
        impl_->context = nullptr;
        return false;
    }
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_ = {};
    }
    running_ = true;
    thread_ = std::thread(&RobotiqTransport::run, this);
    return true;
}

void RobotiqTransport::stop() {
    if (running_) {
        hold();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    running_ = false;
    condition_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (impl_->context) {
        modbus_close(impl_->context);
        modbus_free(impl_->context);
        impl_->context = nullptr;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
}

bool RobotiqTransport::activate() {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!running_) return false;
    activation_requested_ = true;
    condition_.notify_all();
    return condition_.wait_for(lock, std::chrono::seconds(15), [this] {
        // Activation-required status codes are expected before gSTA reaches 3.
        // Keep polling so those transitional codes do not abort activation.
        return state_.activated || !running_;
    }) && state_.activated && state_.fault == 0;
}

void RobotiqTransport::set_target(float position, float speed, float force) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_position_ = std::clamp(position, 0.0f, 1.0f);
    target_speed_ = std::clamp(speed, 0.0f, 1.0f);
    target_force_ = std::clamp(force, 0.0f, 1.0f);
    hold_requested_ = false;
    ++command_generation_;
    condition_.notify_all();
}

void RobotiqTransport::hold() {
    std::lock_guard<std::mutex> lock(mutex_);
    hold_requested_ = true;
    condition_.notify_all();
}

RobotiqState RobotiqTransport::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void RobotiqTransport::run() {
    auto write_command = [this](uint8_t action, uint8_t position, uint8_t speed, uint8_t force) {
        uint16_t registers[3]{static_cast<uint16_t>(action << 8), position,
                              static_cast<uint16_t>((speed << 8) | force)};
        return modbus_write_registers(impl_->context, 0x03E8, 3, registers) == 3;
    };
    auto fail_transport = [this](const char* operation) {
        PI_ERROR("Robotiq Modbus %s failed: %s", operation, modbus_strerror(errno));
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.connected = false;
            state_.activated = false;
            state_.moving = false;
        }
        running_ = false;
        condition_.notify_all();
    };
    uint64_t applied_generation = UINT64_MAX;
    uint8_t previous_raw = config_.open_raw;
    auto previous_time = std::chrono::steady_clock::now();
    const auto period = std::chrono::milliseconds(1000 / std::max(1, config_.poll_frequency_hz));
    while (running_) {
        bool activate = false;
        bool hold = false;
        float position;
        float speed;
        float force;
        uint64_t generation;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            activate = activation_requested_;
            activation_requested_ = false;
            hold = hold_requested_;
            hold_requested_ = false;
            position = target_position_;
            speed = target_speed_;
            force = target_force_;
            generation = command_generation_;
        }
        if (activate) {
            if (!write_command(0x00, 0, 0, 0)) {
                fail_transport("activation reset write");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            if (!write_command(0x01, config_.open_raw, 0xff, 0x96)) {
                fail_transport("activation write");
                break;
            }
        }

        std::array<uint16_t, 3> registers{};
        if (modbus_read_registers(impl_->context, 0x07D0, 3, registers.data()) != 3) {
            fail_transport("read");
            break;
        }
        const uint8_t status = registers[0] >> 8;
        const uint8_t fault = registers[1] >> 8;
        const uint8_t raw = registers[2] >> 8;
        const uint8_t current = registers[2] & 0xff;
        const bool activated = (status & 0x01) && ((status >> 4) & 0x03) == 0x03;
        const bool moving = ((status >> 3) & 0x01) && ((status >> 6) & 0x03) == 0;
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - previous_time).count();
        const float normalized_position = raw_to_position(raw, config_.open_raw, config_.closed_raw);
        const float previous_position = raw_to_position(previous_raw, config_.open_raw, config_.closed_raw);
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.connected = true;
            state_.activated = activated;
            state_.ever_activated = state_.ever_activated || activated;
            state_.moving = moving;
            state_.position = normalized_position;
            state_.velocity = dt > 0 ? (normalized_position - previous_position) / dt : 0;
            state_.effort = 0.0f;
            state_.current = static_cast<float>(current) * 0.01f;
            state_.target = position;
            state_.fault = fault;
        }
        condition_.notify_all();
        previous_raw = raw;
        previous_time = now;

        bool write_ok = true;
        if (activated && hold) {
            write_ok = write_command(0x01, raw, normalized_to_raw(speed), normalized_to_raw(force, true));
            applied_generation = generation;
        } else if (activated && generation != applied_generation) {
            write_ok = write_command(0x09,
                position_to_raw(position, config_.open_raw, config_.closed_raw),
                normalized_to_raw(speed), normalized_to_raw(force, true));
            applied_generation = generation;
        } else if (activated && position < 0.05f && !moving) {
            write_ok = write_command(0x09, config_.closed_raw, normalized_to_raw(speed),
                                     normalized_to_raw(force, true));
        }
        if (!write_ok) {
            fail_transport("command write");
            break;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, period, [this, generation] {
            return !running_ || command_generation_ != generation || activation_requested_ || hold_requested_;
        });
    }
}
