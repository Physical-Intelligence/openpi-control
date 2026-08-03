#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "pi_driver.hpp"
#include "pi_fr3_controller.hpp"

struct FR3DriverState {
    uint64_t sequence = 0;
    uint64_t monotonic_ns = 0;
    std::array<double, 7> q{};
    std::array<double, 7> dq{};
    std::array<double, 7> torque{};
    std::array<double, 7> commanded_q{};
    bool valid = false;
    bool faulted = false;
    std::string fault;
};

class DriverFR3 final : public Driver {
   public:
    DriverFR3(Device* device, const CommandLineArgs& cla);
    ~DriverFR3() override;

    ReturnCode open(int baud_rate) override;
    ReturnCode close() override;
    FR3DriverState state() const;
    ReturnCode set_target(const std::array<double, 7>& target);
    ReturnCode hold();
    ReturnCode move_to_ready();

   private:
    struct Impl;
    ReturnCode start_controller();
    void run_controller();
    void apply_pending_command();
    ReturnCode initialize_controller_state();

    CommandLineArgs cla_;
    std::unique_ptr<Impl> impl_;
    mutable std::mutex state_mutex_;
    mutable std::mutex pending_controller_mutex_;
    FR3DriverState state_;
    FR3ControllerLimits limits_;
    FR3Controller controller_;
    std::array<double, 7> pending_target_{};
    std::array<double, 7> reset_pose_{};
    bool target_pending_ = false;
    bool hold_pending_ = false;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::mutex controller_lifecycle_mutex_;
    std::condition_variable controller_lifecycle_cv_;
    bool controller_started_ = false;
    bool controller_exited_ = true;
};
