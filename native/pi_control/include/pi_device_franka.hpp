#pragma once

#include <memory>

#include "pi_device_arm.hpp"
#include "pi_device_robotiq.hpp"
#include "pi_driver_franka.hpp"

class DeviceFranka final : public DeviceArm {
   public:
    explicit DeviceFranka(const CommandLineArgs& cla);
    ~DeviceFranka() override;

    ReturnCode init(const CommandLineArgs& cla, int argc, char** argv,
                    std::shared_ptr<Topic> topic = nullptr,
                    std::shared_ptr<Driver> driver = nullptr) override;
    ReturnCode start(int baud_rate) override;
    ReturnCode park_safely() override;
    bool uses_droid_protocol() const override { return true; }
    ReturnCode apply_action(const MsgDroidCommand& msg) override;
    ReturnCode get_observation(MsgDroidState& msg) override;
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
    ReturnCode recover() override;

   protected:
    void reset_ready_state_for_move_to_ready() override { is_ready_ = false; }
    void clear_command_buffers_for_move_to_ready() override;

   private:
    std::shared_ptr<DriverFranka> franka_driver_;
    DeviceRobotiq* robotiq_ = nullptr;
};
