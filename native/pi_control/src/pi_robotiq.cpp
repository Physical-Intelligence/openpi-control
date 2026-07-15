#include "pi_robotiq.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>

#include "pi_info.hpp"

#ifdef OPENPI_CONTROL_WITH_MODBUS
#include <modbus.h>
#endif

struct RobotiqTransport::Impl {
#ifdef OPENPI_CONTROL_WITH_MODBUS
    modbus_t* context = nullptr;
#endif
};

RobotiqTransport::RobotiqTransport(RobotiqConfig config)
    : config_(std::move(config)), impl_(std::make_unique<Impl>()) {}

RobotiqTransport::~RobotiqTransport() { stop(); }

uint8_t RobotiqTransport::position_to_raw(float normalized_open, uint8_t open_raw,
                                       uint8_t closed_raw) {
    const float value = closed_raw - std::clamp(normalized_open, 0.0f, 1.0f) *
                                         static_cast<float>(closed_raw - open_raw);
    return static_cast<uint8_t>(std::lround(value));
}

float RobotiqTransport::raw_to_position(uint8_t raw, uint8_t open_raw, uint8_t closed_raw) {
    if (closed_raw <= open_raw) return 1.0f;
    return std::clamp(static_cast<float>(closed_raw - raw) /
                          static_cast<float>(closed_raw - open_raw),
                      0.0f, 1.0f);
}

uint8_t RobotiqTransport::normalized_to_raw(float value, bool preserve_regrasp) {
    const long raw = std::lround(255.0f * std::clamp(value, 0.0f, 1.0f));
    return static_cast<uint8_t>(preserve_regrasp && raw == 0 && value > 0 ? 1 : raw);
}

bool RobotiqTransport::start() {
    if (config_.device.empty()) return true;
#ifdef OPENPI_CONTROL_WITH_MODBUS
    impl_->context = modbus_new_rtu(config_.device.c_str(), config_.baud_rate, 'N', 8, 1);
    if (!impl_->context) return false;
    modbus_set_slave(impl_->context, config_.slave_id);
    if (modbus_connect(impl_->context) != 0) {
        modbus_free(impl_->context);
        impl_->context = nullptr;
        return false;
    }
    running_ = true;
    thread_ = std::thread(&RobotiqTransport::run, this);
    return true;
#elif defined(OPENPI_CONTROL_MOCK_ROBOTIQ)
    running_ = true;
    thread_ = std::thread(&RobotiqTransport::run, this);
    return true;
#else
    PI_ERROR("Robotiq support was not built; enable OPENPI_CONTROL_WITH_MODBUS");
    return false;
#endif
}

void RobotiqTransport::stop() {
    if (running_) {
        hold();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    running_ = false;
    condition_.notify_all();
    if (thread_.joinable()) thread_.join();
#ifdef OPENPI_CONTROL_WITH_MODBUS
    if (impl_->context) {
        modbus_close(impl_->context);
        modbus_free(impl_->context);
        impl_->context = nullptr;
    }
#endif
    std::lock_guard<std::mutex> lock(mutex_);
    state_.connected = false;
}

bool RobotiqTransport::activate() {
    if (config_.device.empty()) return false;
    std::unique_lock<std::mutex> lock(mutex_);
    activation_requested_ = true;
    condition_.notify_all();
    return condition_.wait_for(lock, std::chrono::seconds(15), [this] {
        return state_.activated || state_.fault != 0 || !running_;
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
#ifdef OPENPI_CONTROL_WITH_MODBUS
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
        const uint8_t fault = registers[0] & 0xff;
        const uint8_t raw = registers[2] >> 8;
        const uint8_t current = registers[2] & 0xff;
        const bool activated = (status & 0x01) && ((status >> 4) & 0x03) == 0x03;
        const bool moving = ((status >> 3) & 0x01) && ((status >> 6) & 0x03) == 0;
        const auto now = std::chrono::steady_clock::now();
        const float dt = std::chrono::duration<float>(now - previous_time).count();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.connected = true;
            state_.activated = activated;
            state_.moving = moving;
            state_.position = raw_to_position(raw, config_.open_raw, config_.closed_raw);
            state_.velocity = dt > 0 ? (static_cast<float>(previous_raw) - raw) /
                                           (config_.closed_raw - config_.open_raw) / dt
                                     : 0;
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
            write_ok = write_command(0x01, raw, normalized_to_raw(speed),
                                     normalized_to_raw(force, true));
            applied_generation = generation;
        } else if (activated && generation != applied_generation) {
            write_ok = write_command(
                0x09, position_to_raw(position, config_.open_raw, config_.closed_raw),
                normalized_to_raw(speed), normalized_to_raw(force, true));
            applied_generation = generation;
        } else if (activated && position < 0.05f && !moving) {
            // Reassert close with rGTO and nonzero force after contact. This preserves
            // Robotiq automatic re-grasp instead of accepting contact as completion.
            write_ok = write_command(0x09, config_.closed_raw, normalized_to_raw(speed),
                                     normalized_to_raw(force, true));
        }
        if (!write_ok) {
            fail_transport("command write");
            break;
        }
        std::unique_lock<std::mutex> lock(mutex_);
        condition_.wait_for(lock, period, [this, generation] {
            return !running_ || command_generation_ != generation || activation_requested_ ||
                   hold_requested_;
        });
    }
#elif defined(OPENPI_CONTROL_MOCK_ROBOTIQ)
    const auto period = std::chrono::milliseconds(1000 / std::max(1, config_.poll_frequency_hz));
    {
        std::lock_guard<std::mutex> lock(mutex_);
        state_.connected = true;
    }
    condition_.notify_all();
    while (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (activation_requested_) {
            activation_requested_ = false;
            state_.activated = true;
            state_.connected = true;
            condition_.notify_all();
        }
        const float previous = state_.position;
        if (!hold_requested_ && state_.activated) {
            const float error = target_position_ - state_.position;
            state_.position += std::clamp(error, -0.02f, 0.02f);
            state_.moving = std::abs(error) > 0.001f;
        } else {
            hold_requested_ = false;
            state_.moving = false;
        }
        state_.target = target_position_;
        state_.velocity = (state_.position - previous) * config_.poll_frequency_hz;
        condition_.wait_for(lock, period);
    }
#endif
}
