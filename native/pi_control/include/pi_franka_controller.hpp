#pragma once

#include <array>
#include <cstdint>

#include "pi_topic.hpp"

struct FrankaControllerGains {
    std::array<double, 6> cartesian_stiffness{400, 400, 400, 15, 15, 15};
    std::array<double, 6> cartesian_damping{37, 37, 37, 2, 2, 2};
    std::array<double, 7> joint_stiffness{40, 30, 50, 25, 35, 25, 10};
    std::array<double, 7> joint_damping{4, 6, 5, 5, 3, 2, 1};
};

struct FrankaControllerLimits {
    std::array<double, 3> cartesian_lower{-1, -1, -1};
    std::array<double, 3> cartesian_upper{1, 1, 1};
    std::array<double, 7> joint_lower{-2.65, -1.68, -2.80, -2.95, -2.70, 0.45, -2.90};
    std::array<double, 7> joint_upper{2.65, 1.68, 2.80, -0.16, 2.70, 4.40, 2.90};
    std::array<double, 7> velocity{2.075, 2.075, 2.075, 2.075, 2.51, 2.51, 2.51};
    std::array<double, 7> torque{86, 86, 86, 86, 11.5, 11.5, 11.5};
    double joint_margin = 0.2;
    double velocity_margin = 0.5;
    double cartesian_margin = 0.05;
    double joint_stiffness = 50;
    double velocity_stiffness = 20;
    double cartesian_stiffness = 200;
};

struct FrankaControllerInput {
    uint64_t monotonic_ns = 0;
    std::array<double, 7> q{};
    std::array<double, 7> dq{};
    std::array<double, 7> coriolis{};
    std::array<double, 42> flange_jacobian{};
    std::array<double, 42> end_effector_jacobian{};
    std::array<double, 3> end_effector_position{};
    double elbow_velocity = 0.0;
};

class FrankaController {
   public:
    explicit FrankaController(FrankaControllerGains gains = {}, FrankaControllerLimits limits = {});

    void set_command(const MsgDroidCommand& command);
    void hold(const std::array<double, 7>& measured_position);
    std::array<double, 7> compute(const FrankaControllerInput& input);
    const std::array<double, 7>& commanded_position() const { return target_position_; }

   private:
    static constexpr uint64_t kVelocityCommandTimeoutNs = 250000000;

    static void add_soft_limit(double value, double lower, double upper, double margin,
                               double stiffness, double& output);

    FrankaControllerGains gains_;
    FrankaControllerLimits limits_;
    MsgDroidCommand command_;
    std::array<double, 7> target_position_{};
    DroidControlMode active_mode_ = DroidControlMode::HOLD;
    bool initialized_ = false;
};
