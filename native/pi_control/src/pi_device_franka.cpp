#include "pi_device_franka.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

DeviceFranka::DeviceFranka(const CommandLineArgs& cla) : DeviceArm(cla) {
    dof_ = 7;
    dof_total_ = cla.robotiq_device.empty() ? 7 : 8;
    servo_num_ = 7;
    servo_num_total_ = dof_total_;
}

DeviceFranka::~DeviceFranka() { park_safely(); }

ReturnCode DeviceFranka::init(const CommandLineArgs& cla, int argc, char** argv,
                              std::shared_ptr<Topic> topic, std::shared_ptr<Driver> driver) {
    ReturnCode result = Device::init(cla, argc, argv, topic, driver);
    if (result != ReturnCode::SUCCESS) return result;
    franka_driver_ = std::dynamic_pointer_cast<DriverFranka>(p_driver_);
    if (!franka_driver_) return ReturnCode::NOT_INITIALIZED;
    if (!cla.robotiq_device.empty()) {
        CommandLineArgs effector_cla = cla;
        effector_cla.device_config_type = DeviceConfigType::EFFECTOR;
        effector_cla.device_model = cla.effector_model.empty() ? "Robotiq" : cla.effector_model;
        effector_cla.device_id = cla.effector_id;
        p_effector_ = std::make_unique<DeviceRobotiq>(effector_cla);
        robotiq_ = static_cast<DeviceRobotiq*>(p_effector_.get());
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFranka::start(int baud_rate) {
    ReturnCode result = Device::start(baud_rate);
    if (result != ReturnCode::SUCCESS) return result;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!franka_driver_->state().valid && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!franka_driver_->state().valid) return ReturnCode::NO_RESPONSE;
    if (robotiq_) {
        result = robotiq_->start(baud_rate);
        if (result != ReturnCode::SUCCESS) return result;
    }
    if (cla_.dont_go_to_home_pos || cla_.franka_read_only) {
        franka_driver_->hold();
        is_ready_ = true;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFranka::park_safely() {
    if (franka_driver_) franka_driver_->hold();
    if (robotiq_) robotiq_->park_safely();
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFranka::apply_action(const MsgDroidCommand& msg) {
    if (cla_.franka_read_only) return ReturnCode::NOT_SUPPORTED;
    const uint64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count();
    if (msg.monotonic_ns == 0 || now_ns < msg.monotonic_ns ||
        now_ns - msg.monotonic_ns > 250000000ULL) {
        return ReturnCode::INVALID_PARAM;
    }
    if (!std::isfinite(msg.gripper_position) || !std::isfinite(msg.gripper_speed) ||
        !std::isfinite(msg.gripper_force) || msg.gripper_position < 0 ||
        msg.gripper_position > 1 || msg.gripper_speed < 0 || msg.gripper_speed > 1 ||
        msg.gripper_force < 0 || msg.gripper_force > 1) {
        return ReturnCode::INVALID_PARAM;
    }
    if (robotiq_) {
        const auto gripper = robotiq_->state();
        if (!gripper.connected) return ReturnCode::NO_RESPONSE;
        if (!gripper.activated || gripper.fault != 0) return ReturnCode::HARDWARE_FAULT;
    }
    ReturnCode result = franka_driver_->set_command(msg);
    if (result == ReturnCode::SUCCESS && robotiq_) {
        result = robotiq_->set_target(msg.gripper_position, msg.gripper_speed, msg.gripper_force);
        if (result != ReturnCode::SUCCESS) franka_driver_->hold();
    }
    return result;
}

ReturnCode DeviceFranka::get_observation(MsgDroidState& msg) {
    const auto state = franka_driver_->state();
    if (!state.valid) return ReturnCode::NOT_INITIALIZED;
    msg.sequence = state.sequence;
    msg.monotonic_ns = state.monotonic_ns;
    msg.hardware_timestamp_s = state.hardware_timestamp_s;
    for (size_t i = 0; i < 7; ++i) {
        msg.joint_position[i] = state.q[i];
        msg.joint_velocity[i] = state.dq[i];
        msg.joint_effort[i] = state.torque[i];
        msg.commanded_joint_position[i] = state.commanded_q[i];
        msg.external_joint_torque[i] = state.external_torque[i];
    }
    std::copy(state.external_wrench.begin(), state.external_wrench.end(), msg.cartesian_wrench.begin());
    msg.robot_mode = state.robot_mode;
    msg.joint_contact_bits = state.contact_bits;
    msg.joint_collision_bits = state.collision_bits;
    msg.control_command_success_rate = state.control_command_success_rate;
    if (robotiq_) {
        const auto gripper = robotiq_->state();
        msg.gripper = {gripper.position, gripper.velocity, gripper.effort, 0,
                       gripper.current, gripper.target};
        msg.flags |= gripper.connected ? 1 : 0;
        msg.flags |= gripper.activated ? 2 : 0;
        msg.flags |= gripper.fault != 0 ? 8 : 0;
    }
    msg.flags |= state.faulted ? 4 : 0;
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFranka::apply_action(const MsgJoints& msg) {
    (void)msg;
    return ReturnCode::NOT_SUPPORTED;
}

ReturnCode DeviceFranka::get_observation(MsgJoints& msg) {
    (void)msg;
    return ReturnCode::NOT_SUPPORTED;
}

ReturnCode DeviceFranka::process_follower_msg(const MsgJoints& msg) {
    (void)msg;
    return ReturnCode::NOT_SUPPORTED;
}

ReturnCode DeviceFranka::read_hardware_values() {
    const auto state = franka_driver_->state();
    if (state.faulted) {
        PI_ERROR("HARDWARE FAULT: %s", state.fault.c_str());
        return ReturnCode::HARDWARE_FAULT;
    }
    return state.valid ? ReturnCode::SUCCESS : ReturnCode::NOT_INITIALIZED;
}

ReturnCode DeviceFranka::write_hardware_values() { return ReturnCode::SUCCESS; }

ReturnCode DeviceFranka::move_to_ready_position() {
    const ReturnCode result = franka_driver_->move_to_ready();
    if (result == ReturnCode::SUCCESS) is_ready_ = true;
    return result;
}

ReturnCode DeviceFranka::operate_as_leader() { return ReturnCode::NOT_SUPPORTED; }
ReturnCode DeviceFranka::operate_as_follower() { return ReturnCode::SUCCESS; }

ReturnCode DeviceFranka::get_servo_ids(std::vector<int>& servo_ids) {
    servo_ids.clear();
    return ReturnCode::SUCCESS;
}

ReturnCode DeviceFranka::set_control_mode(Role target_role, ControlModeIntent intent) {
    (void)intent;
    return target_role == Role::FOLLOWER ? ReturnCode::SUCCESS : ReturnCode::NOT_SUPPORTED;
}

ReturnCode DeviceFranka::runtime_hold() {
    return Device::runtime_hold();
}

void DeviceFranka::clear_command_buffers_for_move_to_ready() {
    if (franka_driver_) franka_driver_->hold();
    if (robotiq_) robotiq_->runtime_hold();
}

ReturnCode DeviceFranka::activate_effector() {
    if (!robotiq_) return ReturnCode::NOT_SUPPORTED;
    return robotiq_->activate_effector();
}

ReturnCode DeviceFranka::recover() {
    const ReturnCode result = franka_driver_->recover();
    if (result == ReturnCode::SUCCESS) {
        {
            std::lock_guard<std::mutex> lock(emergency_mutex_);
            emergency_state_ = EmergencyRecoveryState::NONE;
            slow_move_active_ = false;
            failed_joint_ids_.clear();
        }
        is_ready_ = true;
        franka_driver_->hold();
    }
    return result;
}
