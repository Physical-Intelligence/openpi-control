/*!
 * @file pi_device_robotiq.hpp
 * @brief Robotiq Modbus end-effector device.
 */
#pragma once

#include <memory>

#include "pi_device_effector.hpp"
#include "pi_robotiq.hpp"

class DeviceRobotiq final : public DeviceEffector {
   public:
    explicit DeviceRobotiq(const CommandLineArgs& cla);
    ~DeviceRobotiq() override;

    ReturnCode start(int baud_rate) override;
    ReturnCode stop() override;
    ReturnCode park_safely() override;
    ReturnCode apply_action(const MsgJoints& msg) override;
    ReturnCode get_observation(MsgJoints& msg) override;
    ReturnCode process_follower_msg(const MsgJoints& msg) override;
    ReturnCode read_hardware_values() override;
    ReturnCode write_hardware_values() override;
    ReturnCode move_to_ready_position() override;
    ReturnCode operate_as_leader() override;
    ReturnCode operate_as_follower() override;
    ReturnCode get_servo_ids(std::vector<int>& servo_ids) override;
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;
    ReturnCode runtime_hold() override;
    ReturnCode activate_effector() override;

    ReturnCode set_target(float position, float speed, float force);
    RobotiqState state() const;

   private:
    std::unique_ptr<RobotiqTransport> transport_;
    float default_speed_ = 1.0f;
    float default_force_ = 1.0f;
};
