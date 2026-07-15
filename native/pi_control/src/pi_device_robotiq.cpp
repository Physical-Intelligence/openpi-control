/*!
 * @file pi_device_robotiq.cpp
 * @brief Robotiq Modbus end-effector implementation.
 */

#include "pi_device_robotiq.hpp"

#include <chrono>
#include <thread>

DeviceRobotiq::DeviceRobotiq(const CommandLineArgs& cla) : DeviceEffector(cla) {
    RobotiqConfig config;
    config.device = cla.robotiq_device;
    config.baud_rate = cla.robotiq_baud_rate;
    config.slave_id = cla.robotiq_slave_id;
    config.poll_frequency_hz = cla.robotiq_poll_frequency;
    config.open_raw = static_cast<uint8_t>(cla.robotiq_min_position_raw);
    config.closed_raw = static_cast<uint8_t>(cla.robotiq_max_position_raw);
    default_speed_ = cla.robotiq_default_speed;
    default_force_ = cla.robotiq_default_force;
    transport_ = std::make_unique<RobotiqTransport>(config);
    dof_ = 1;
    dof_total_ = 1;
}

DeviceRobotiq::~DeviceRobotiq() { park_safely(); }

ReturnCode DeviceRobotiq::start(int baud_rate) {
    (void)baud_rate;
    if (!transport_->start()) return ReturnCode::NO_RESPONSE;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
    while (!state().connected && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!state().connected) {
        transport_->stop();
        return ReturnCode::NO_RESPONSE;
    }
    is_ready_ = true;
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceRobotiq::stop() {
    transport_->stop();
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceRobotiq::park_safely() {
    transport_->hold();
    transport_->stop();
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceRobotiq::apply_action(const MsgJoints& msg) {
    if (msg.joints_.empty()) return ReturnCode::INVALID_PARAM;
    return set_target(msg.joints_.back().curr_pos_, default_speed_, default_force_);
}

ReturnCode DeviceRobotiq::get_observation(MsgJoints& msg) {
    const auto snapshot = state();
    msg.add_joint_info(snapshot.position, snapshot.velocity, snapshot.effort, 0.0f,
                       snapshot.current);
    return snapshot.connected ? ReturnCode::SUCCESS : ReturnCode::NO_RESPONSE;
}

ReturnCode DeviceRobotiq::process_follower_msg(const MsgJoints& msg) {
    return apply_action(msg);
}

ReturnCode DeviceRobotiq::read_hardware_values() {
    const auto snapshot = state();
    if (!snapshot.connected) return ReturnCode::NO_RESPONSE;
    return snapshot.fault == 0 ? ReturnCode::SUCCESS : ReturnCode::HARDWARE_FAULT;
}

ReturnCode DeviceRobotiq::write_hardware_values() { return ReturnCode::SUCCESS; }

ReturnCode DeviceRobotiq::move_to_ready_position() {
    const ReturnCode result = runtime_hold();
    if (result == ReturnCode::SUCCESS) is_ready_ = true;
    return result;
}

ReturnCode DeviceRobotiq::operate_as_leader() { return ReturnCode::NOT_SUPPORTED; }

ReturnCode DeviceRobotiq::operate_as_follower() { return ReturnCode::SUCCESS; }

ReturnCode DeviceRobotiq::get_servo_ids(std::vector<int>& servo_ids) {
    (void)servo_ids;
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceRobotiq::set_control_mode(Role target_role, ControlModeIntent intent) {
    return target_role == Role::FOLLOWER || intent == ControlModeIntent::READY_MOVE_OVERRIDE
               ? ReturnCode::SUCCESS
               : ReturnCode::NOT_SUPPORTED;
}

ReturnCode DeviceRobotiq::runtime_hold() {
    transport_->hold();
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceRobotiq::activate_effector() {
    return transport_->activate() ? ReturnCode::SUCCESS : ReturnCode::HARDWARE_FAULT;
}

ReturnCode DeviceRobotiq::set_target(float position, float speed, float force) {
    const auto snapshot = state();
    if (!snapshot.connected) return ReturnCode::NO_RESPONSE;
    if (!snapshot.activated || snapshot.fault != 0) return ReturnCode::HARDWARE_FAULT;
    transport_->set_target(position, speed, force);
    return ReturnCode::SUCCESS;
}

RobotiqState DeviceRobotiq::state() const { return transport_->state(); }
