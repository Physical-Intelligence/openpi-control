#include "pi_device_fr3.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <thread>

DeviceFR3::DeviceFR3(const CommandLineArgs& cla) : Device(cla) {
    dof_ = 7;
    dof_total_ = cla.robotiq_transport.empty() ? 7 : 8;
    servo_num_ = 7;
    servo_num_total_ = dof_total_;
    type_ = DeviceType::ARM;
}

DeviceFR3::~DeviceFR3() = default;

ReturnCode DeviceFR3::init(const CommandLineArgs& cla, int argc, char** argv,
                           std::shared_ptr<Topic> topic, std::shared_ptr<Driver> driver) {
    ReturnCode result = Device::init(cla, argc, argv, std::move(topic), std::move(driver));
    if (result != ReturnCode::SUCCESS) return result;
    driver_fr3_ = std::dynamic_pointer_cast<DriverFR3>(p_driver_);
    if (!driver_fr3_) return ReturnCode::NOT_INITIALIZED;
    if (!cla.robotiq_transport.empty()) {
        RobotiqConfig config;
        config.backend = cla.robotiq_transport == "tcp" ? RobotiqBackend::TCP : RobotiqBackend::RTU;
        config.endpoint = cla.robotiq_endpoint;
        config.tcp_port = cla.robotiq_port;
        config.baud_rate = cla.robotiq_baud_rate;
        config.slave_id = cla.robotiq_slave_id;
        config.poll_frequency_hz = cla.robotiq_poll_frequency;
        config.response_timeout_ms = cla.robotiq_timeout_ms;
        config.open_raw = static_cast<uint8_t>(cla.robotiq_min_position_raw);
        config.closed_raw = static_cast<uint8_t>(cla.robotiq_max_position_raw);
        robotiq_ = std::make_unique<RobotiqTransport>(std::move(config));
        robotiq_default_speed_ = cla.robotiq_default_speed;
        robotiq_default_force_ = cla.robotiq_default_force;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFR3::start(int baud_rate) {
    ReturnCode result = Device::start(baud_rate);
    if (result != ReturnCode::SUCCESS) return result;
    const auto arm_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!driver_fr3_->state().valid && std::chrono::steady_clock::now() < arm_deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!driver_fr3_->state().valid) return ReturnCode::NO_RESPONSE;
    if (robotiq_) {
        if (!robotiq_->start()) return ReturnCode::NO_RESPONSE;
        const auto gripper_deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!robotiq_->state().connected && std::chrono::steady_clock::now() < gripper_deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        if (!robotiq_->state().connected) return ReturnCode::NO_RESPONSE;
    }
    if (cla_.dont_go_to_home_pos) {
        driver_fr3_->hold();
        is_ready_ = true;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFR3::stop() {
    const ReturnCode result = Device::stop();
    if (robotiq_) robotiq_->stop();
    return result;
}

ReturnCode DeviceFR3::park_safely() {
    if (driver_fr3_) driver_fr3_->hold();
    if (robotiq_) robotiq_->hold();
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFR3::apply_action(const MsgJoints& msg) {
    if (msg.joints_.size() != static_cast<size_t>(dof_total_)) return ReturnCode::INVALID_PARAM;
    std::array<double, 7> target{};
    for (size_t i = 0; i < target.size(); ++i) {
        const float value = msg.joints_[i].curr_pos_;
        if (!std::isfinite(value)) return ReturnCode::INVALID_PARAM;
        target[i] = value;
    }
    if (robotiq_) {
        const float position = msg.joints_[7].curr_pos_;
        const auto gripper = robotiq_->state();
        if (!std::isfinite(position) || position < 0.0f || position > 1.0f) return ReturnCode::INVALID_PARAM;
        if (!gripper.connected) return ReturnCode::NO_RESPONSE;
        if (!gripper.activated || RobotiqTransport::has_operational_fault(gripper)) {
            return ReturnCode::HARDWARE_FAULT;
        }
    }
    ReturnCode result = driver_fr3_->set_target(target);
    if (result == ReturnCode::SUCCESS && robotiq_) {
        robotiq_->set_target(msg.joints_[7].curr_pos_, robotiq_default_speed_, robotiq_default_force_);
    }
    return result;
}

ReturnCode DeviceFR3::get_observation(MsgJoints& msg) {
    const auto state = driver_fr3_->state();
    if (!state.valid) return ReturnCode::NOT_INITIALIZED;
    const uint64_t now = std::chrono::duration_cast<std::chrono::nanoseconds>(
                             std::chrono::steady_clock::now().time_since_epoch()).count();
    const float age_ms = now >= state.monotonic_ns
                             ? static_cast<float>(now - state.monotonic_ns) / 1000000.0f
                             : -1.0f;
    for (size_t i = 0; i < 7; ++i) {
        msg.add_joint_info(static_cast<float>(state.q[i]), static_cast<float>(state.dq[i]),
                           static_cast<float>(state.torque[i]), 0.0f, 0.0f, age_ms);
    }
    if (robotiq_) {
        const auto gripper = robotiq_->state();
        msg.add_joint_info(gripper.position, gripper.velocity, gripper.effort, 0.0f,
                           gripper.current, -1.0f);
        if (!gripper.connected) return ReturnCode::NO_RESPONSE;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFR3::process_follower_msg(const MsgJoints& msg) { return apply_action(msg); }

ReturnCode DeviceFR3::read_hardware_values() {
    const auto state = driver_fr3_->state();
    if (state.faulted) {
        PI_ERROR("HARDWARE FAULT: %s", state.fault.c_str());
        return ReturnCode::HARDWARE_FAULT;
    }
    if (!state.valid) return ReturnCode::NOT_INITIALIZED;
    if (robotiq_) {
        const auto gripper = robotiq_->state();
        if (!gripper.connected) return ReturnCode::NO_RESPONSE;
        if (RobotiqTransport::has_operational_fault(gripper)) {
            PI_ERROR("HARDWARE FAULT: Robotiq reported fault 0x%02x", gripper.fault);
            return ReturnCode::HARDWARE_FAULT;
        }
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFR3::write_hardware_values() { return ReturnCode::SUCCESS; }

ReturnCode DeviceFR3::move_to_ready_position() {
    ReturnCode result = driver_fr3_->move_to_ready();
    if (result != ReturnCode::SUCCESS) return result;
    if (robotiq_) {
        if (!robotiq_->state().activated && !robotiq_->activate()) {
            driver_fr3_->hold();
            return ReturnCode::HARDWARE_FAULT;
        }
        robotiq_->set_target(1.0f, robotiq_default_speed_, robotiq_default_force_);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (std::chrono::steady_clock::now() < deadline) {
            const auto gripper = robotiq_->state();
            if (!gripper.connected) {
                driver_fr3_->hold();
                robotiq_->hold();
                return ReturnCode::NO_RESPONSE;
            }
            if (RobotiqTransport::has_operational_fault(gripper)) {
                driver_fr3_->hold();
                robotiq_->hold();
                return ReturnCode::HARDWARE_FAULT;
            }
            if (gripper.activated && gripper.position >= 0.98f) {
                is_ready_ = true;
                return ReturnCode::SUCCESS;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        PI_ERROR("FR3 move-to-ready timed out waiting for Robotiq to open");
        driver_fr3_->hold();
        robotiq_->hold();
        return ReturnCode::HARDWARE_FAULT;
    }
    is_ready_ = true;
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFR3::operate_as_leader() { return ReturnCode::NOT_SUPPORTED; }
ReturnCode DeviceFR3::operate_as_follower() { return ReturnCode::SUCCESS; }

ReturnCode DeviceFR3::get_servo_ids(std::vector<int>& servo_ids) {
    servo_ids.clear();
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFR3::set_control_mode(Role target_role, ControlModeIntent intent) {
    (void)intent;
    return target_role == Role::FOLLOWER ? ReturnCode::SUCCESS : ReturnCode::NOT_SUPPORTED;
}

ReturnCode DeviceFR3::runtime_hold() {
    if (rejects_direct_commands()) return ReturnCode::BUSY;
    clear_command_buffers_for_move_to_ready();
    return ReturnCode::SUCCESS;
}

void DeviceFR3::clear_command_buffers_for_move_to_ready() {
    if (driver_fr3_) driver_fr3_->hold();
    if (robotiq_) robotiq_->hold();
}
