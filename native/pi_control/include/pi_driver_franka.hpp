#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "pi_driver.hpp"
#include "pi_franka_controller.hpp"

struct FrankaDriverState {
    uint64_t sequence = 0;
    uint64_t monotonic_ns = 0;
    double hardware_timestamp_s = 0;
    std::array<double, 7> q{};
    std::array<double, 7> dq{};
    std::array<double, 7> torque{};
    std::array<double, 7> commanded_q{};
    std::array<double, 7> external_torque{};
    std::array<double, 6> external_wrench{};
    uint16_t contact_bits = 0;
    uint16_t collision_bits = 0;
    int robot_mode = 0;
    double control_command_success_rate = 0;
    bool valid = false;
    bool faulted = false;
    std::string fault;
};

class DriverFranka final : public Driver {
   public:
    DriverFranka(Device* device, const CommandLineArgs& cla);
    ~DriverFranka() override;

    ReturnCode open(int baud_rate) override;
    ReturnCode close() override;
    FrankaDriverState state() const;
    ReturnCode set_command(const MsgDroidCommand& command);
    ReturnCode hold();
    ReturnCode move_to_ready();
    ReturnCode recover();

   private:
    struct Impl;
    void run_controller();
    void run_read_only();
    void run_mock();
    void apply_pending_command();

    CommandLineArgs cla_;
    std::unique_ptr<Impl> impl_;
    mutable std::mutex state_mutex_;
    mutable std::mutex pending_controller_mutex_;
    FrankaDriverState state_;
    FrankaControllerLimits limits_;
    FrankaController controller_;
    MsgDroidCommand pending_command_;
    bool command_pending_ = false;
    std::array<double, 7> reset_pose_{};
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    uint64_t last_command_sequence_ = 0;
};
