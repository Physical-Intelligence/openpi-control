#include "pi_driver_fr3.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include <franka/control_types.h>
#include <franka/model.h>
#include <franka/robot.h>

#include "pi_info.hpp"

namespace {

template <size_t Size>
std::array<double, Size> parse_array(const std::string& value, const char* name) {
    std::array<double, Size> result{};
    std::stringstream stream(value);
    std::string item;
    size_t index = 0;
    while (std::getline(stream, item, ',') && index < result.size()) {
        size_t consumed = 0;
        result[index] = std::stod(item, &consumed);
        if (consumed != item.size() || !std::isfinite(result[index])) {
            throw std::invalid_argument(std::string(name) + " contains an invalid value");
        }
        ++index;
    }
    if (index != result.size() || std::getline(stream, item, ',')) {
        throw std::invalid_argument(std::string(name) + " has the wrong number of values");
    }
    return result;
}

uint64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class ResetMotionGenerator {
   public:
    explicit ResetMotionGenerator(double speed_factor, const std::array<double, 7>& goal)
        : goal_(goal) {
        for (size_t i = 0; i < 7; ++i) {
            max_velocity_[i] *= speed_factor;
            max_start_acceleration_[i] *= speed_factor;
            max_goal_acceleration_[i] *= speed_factor;
        }
    }

    franka::JointPositions operator()(const franka::RobotState& state, franka::Duration period) {
        if (!initialized_) {
            start_ = state.q_d;
            for (size_t i = 0; i < 7; ++i) delta_[i] = goal_[i] - start_[i];
            synchronize();
            initialized_ = true;
            return franka::JointPositions(start_);
        }
        time_ += period.toSec();
        std::array<double, 7> delta_desired{};
        const bool trajectory_finished = desired_values(time_, delta_desired);
        std::array<double, 7> positions{};
        for (size_t i = 0; i < 7; ++i) positions[i] = start_[i] + delta_desired[i];
        bool motion_finished = false;
        if (trajectory_finished) {
            positions = goal_;
            double max_error = 0;
            for (size_t i = 0; i < 7; ++i) max_error = std::max(max_error, std::abs(state.q[i] - goal_[i]));
            settle_time_ += period.toSec();
            motion_finished = max_error < 0.01 || settle_time_ > 4.0;
        }
        franka::JointPositions result(positions);
        result.motion_finished = motion_finished;
        return result;
    }

   private:
    bool desired_values(double time, std::array<double, 7>& desired) const {
        std::array<bool, 7> finished{};
        for (size_t i = 0; i < 7; ++i) {
            const int sign = delta_[i] < 0 ? -1 : (delta_[i] > 0 ? 1 : 0);
            const double constant_duration = second_sync_[i] - first_sync_[i];
            const double goal_ramp_duration = finish_sync_[i] - second_sync_[i];
            if (std::abs(delta_[i]) < 1e-6) {
                desired[i] = 0;
                finished[i] = true;
            } else if (time < first_sync_[i]) {
                desired[i] = -max_velocity_sync_[i] * sign / std::pow(first_sync_[i], 3) *
                             (0.5 * time - first_sync_[i]) * std::pow(time, 3);
            } else if (time < second_sync_[i]) {
                desired[i] = first_position_[i] + (time - first_sync_[i]) * max_velocity_sync_[i] * sign;
            } else if (time < finish_sync_[i]) {
                desired[i] = delta_[i] + 0.5 *
                    (1.0 / std::pow(goal_ramp_duration, 3) *
                         (time - first_sync_[i] - 2 * goal_ramp_duration - constant_duration) *
                         std::pow(time - first_sync_[i] - constant_duration, 3) +
                     (2 * time - 2 * first_sync_[i] - goal_ramp_duration - 2 * constant_duration)) *
                    max_velocity_sync_[i] * sign;
            } else {
                desired[i] = delta_[i];
                finished[i] = true;
            }
        }
        return std::all_of(finished.begin(), finished.end(), [](bool value) { return value; });
    }

    void synchronize() {
        std::array<double, 7> reachable_velocity = max_velocity_;
        std::array<double, 7> finish{};
        for (size_t i = 0; i < 7; ++i) {
            const int sign = delta_[i] < 0 ? -1 : (delta_[i] > 0 ? 1 : 0);
            if (std::abs(delta_[i]) <= 1e-6) continue;
            const double threshold = 0.75 * std::pow(max_velocity_[i], 2) / max_start_acceleration_[i] +
                                     0.75 * std::pow(max_velocity_[i], 2) / max_goal_acceleration_[i];
            if (std::abs(delta_[i]) < threshold) {
                reachable_velocity[i] = std::sqrt(4.0 / 3.0 * delta_[i] * sign *
                    max_start_acceleration_[i] * max_goal_acceleration_[i] /
                    (max_start_acceleration_[i] + max_goal_acceleration_[i]));
            }
            const double first = 1.5 * reachable_velocity[i] / max_start_acceleration_[i];
            const double goal_ramp = 1.5 * reachable_velocity[i] / max_goal_acceleration_[i];
            finish[i] = first / 2 + goal_ramp / 2 + std::abs(delta_[i]) / reachable_velocity[i];
        }
        const double synchronized_finish = *std::max_element(finish.begin(), finish.end());
        for (size_t i = 0; i < 7; ++i) {
            if (std::abs(delta_[i]) <= 1e-6) continue;
            const int sign = delta_[i] < 0 ? -1 : 1;
            const double a = 0.75 * (max_goal_acceleration_[i] + max_start_acceleration_[i]);
            const double b = -synchronized_finish * max_goal_acceleration_[i] * max_start_acceleration_[i];
            const double c = std::abs(delta_[i]) * max_goal_acceleration_[i] * max_start_acceleration_[i];
            max_velocity_sync_[i] = (-b - std::sqrt(std::max(0.0, b * b - 4 * a * c))) / (2 * a);
            first_sync_[i] = 1.5 * max_velocity_sync_[i] / max_start_acceleration_[i];
            const double goal_ramp = 1.5 * max_velocity_sync_[i] / max_goal_acceleration_[i];
            finish_sync_[i] = first_sync_[i] / 2 + goal_ramp / 2 + std::abs(delta_[i] / max_velocity_sync_[i]);
            second_sync_[i] = finish_sync_[i] - goal_ramp;
            first_position_[i] = max_velocity_sync_[i] * sign * 0.5 * first_sync_[i];
        }
    }

    const std::array<double, 7> goal_;
    std::array<double, 7> start_{};
    std::array<double, 7> delta_{};
    std::array<double, 7> max_velocity_sync_{};
    std::array<double, 7> first_sync_{};
    std::array<double, 7> second_sync_{};
    std::array<double, 7> finish_sync_{};
    std::array<double, 7> first_position_{};
    double time_ = 0;
    double settle_time_ = 0;
    bool initialized_ = false;
    std::array<double, 7> max_velocity_{2, 2, 2, 2, 2.5, 2.5, 2.5};
    std::array<double, 7> max_start_acceleration_{5, 5, 5, 5, 5, 5, 5};
    std::array<double, 7> max_goal_acceleration_{5, 5, 5, 5, 5, 5, 5};
};

void configure_collision_behavior(franka::Robot& robot) {
    robot.setCollisionBehavior(
        {{40, 40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40, 40}},
        {{40, 40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40, 40}},
        {{40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40}},
        {{40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40}});
}

}  // namespace

struct DriverFR3::Impl {
    std::unique_ptr<franka::Robot> robot;
};

DriverFR3::DriverFR3(Device* device, const CommandLineArgs& cla)
    : Driver(device, cla), cla_(cla), impl_(std::make_unique<Impl>()), controller_({}, limits_),
      reset_pose_(parse_array<7>(cla.fr3_reset_pose, "FR3 reset pose")) {}

DriverFR3::~DriverFR3() { close(); }

ReturnCode DriverFR3::initialize_controller_state() {
    const auto initial = impl_->robot->readOnce();
    {
        std::lock_guard<std::mutex> lock(pending_controller_mutex_);
        controller_.hold(initial.q);
        target_pending_ = false;
        hold_pending_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.q = initial.q;
        state_.dq = initial.dq;
        state_.torque = initial.tau_J;
        state_.commanded_q = initial.q;
        state_.valid = true;
        state_.faulted = false;
        state_.fault.clear();
    }
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFR3::open(int baud_rate) {
    (void)baud_rate;
    try {
        impl_->robot = std::make_unique<franka::Robot>(cla_.fr3_address, franka::RealtimeConfig::kIgnore);
        configure_collision_behavior(*impl_->robot);
        initialize_controller_state();
        const ReturnCode result = start_controller();
        if (result != ReturnCode::SUCCESS) {
            PI_ERROR("FR3 controller failed before its first control cycle");
            impl_->robot.reset();
        }
        return result;
    } catch (const std::exception& error) {
        PI_ERROR("Failed to start FR3: %s", error.what());
        return ReturnCode::FAIL;
    }
}

ReturnCode DriverFR3::start_controller() {
    {
        std::lock_guard<std::mutex> lock(controller_lifecycle_mutex_);
        controller_started_ = false;
        controller_exited_ = false;
    }
    stop_requested_ = false;
    running_ = true;
    try {
        thread_ = std::thread(&DriverFR3::run_controller, this);
    } catch (...) {
        running_ = false;
        {
            std::lock_guard<std::mutex> lock(controller_lifecycle_mutex_);
            controller_exited_ = true;
        }
        controller_lifecycle_cv_.notify_all();
        throw;
    }

    // Do not expose a controller as ready until libfranka has invoked its first
    // control callback. This guarantees that a subsequent close() cannot send
    // stop() before the motion it intends to stop has actually begun.
    std::unique_lock<std::mutex> lock(controller_lifecycle_mutex_);
    controller_lifecycle_cv_.wait(lock, [this] {
        return controller_started_ || controller_exited_;
    });
    const bool started = controller_started_;
    lock.unlock();

    if (!started && thread_.joinable()) {
        thread_.join();
    }
    return started ? ReturnCode::SUCCESS : ReturnCode::FAIL;
}

ReturnCode DriverFR3::close() {
    running_ = false;
    stop_requested_ = true;
    if (impl_->robot && thread_.joinable()) {
        try {
            impl_->robot->stop();
        } catch (const std::exception& error) {
            PI_WARN("FR3 stop failed: %s", error.what());
        }
    }
    if (thread_.joinable()) thread_.join();
    impl_->robot.reset();
    return ReturnCode::SUCCESS;
}

FR3DriverState DriverFR3::state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

ReturnCode DriverFR3::set_target(const std::array<double, 7>& target) {
    for (size_t i = 0; i < target.size(); ++i) {
        if (!std::isfinite(target[i]) || target[i] < limits_.joint_lower[i] || target[i] > limits_.joint_upper[i]) {
            return ReturnCode::INVALID_PARAM;
        }
    }
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);
    pending_target_ = target;
    target_pending_ = true;
    hold_pending_ = false;
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFR3::hold() {
    if (!state().valid) return ReturnCode::NOT_INITIALIZED;
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);
    hold_pending_ = true;
    target_pending_ = false;
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFR3::move_to_ready() {
    close();
    try {
        impl_->robot = std::make_unique<franka::Robot>(cla_.fr3_address, franka::RealtimeConfig::kIgnore);
        impl_->robot->automaticErrorRecovery();
        configure_collision_behavior(*impl_->robot);
        ResetMotionGenerator motion_generator(0.2, reset_pose_);
        impl_->robot->control([&](const franka::RobotState& state, franka::Duration period) {
            return motion_generator(state, period);
        });
        initialize_controller_state();
        const ReturnCode result = start_controller();
        if (result != ReturnCode::SUCCESS) {
            PI_ERROR("FR3 controller failed to restart after move-to-ready");
            impl_->robot.reset();
            return ReturnCode::HARDWARE_FAULT;
        }
        return result;
    } catch (const std::exception& error) {
        PI_ERROR("FR3 move-to-ready failed: %s", error.what());
        return ReturnCode::HARDWARE_FAULT;
    }
}

void DriverFR3::apply_pending_command() {
    std::unique_lock<std::mutex> lock(pending_controller_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    if (hold_pending_) {
        std::array<double, 7> measured{};
        {
            std::lock_guard<std::mutex> state_lock(state_mutex_);
            measured = state_.q;
        }
        controller_.hold(measured);
        hold_pending_ = false;
    } else if (target_pending_) {
        controller_.set_target(pending_target_);
        target_pending_ = false;
    }
}

void DriverFR3::run_controller() {
    try {
        auto model = impl_->robot->loadModel();
        if (stop_requested_) {
            running_ = false;
        } else {
            impl_->robot->control(
                [this, &model](const franka::RobotState& robot_state, franka::Duration) {
                    bool notify_started = false;
                    {
                        std::lock_guard<std::mutex> lock(controller_lifecycle_mutex_);
                        if (!controller_started_) {
                            controller_started_ = true;
                            notify_started = true;
                        }
                    }
                    if (notify_started) {
                        controller_lifecycle_cv_.notify_all();
                    }

                    // close() normally preempts an active loop with Robot::stop(). If
                    // stop was requested in the narrow interval before control() took
                    // ownership, finish the first cycle cooperatively instead.
                    if (stop_requested_) {
                        return franka::MotionFinished(franka::Torques(robot_state.tau_J_d));
                    }

                    FR3ControllerInput input;
                    input.q = robot_state.q;
                    input.dq = robot_state.dq;
                    input.coriolis = model.coriolis(robot_state);
                    input.flange_jacobian =
                        model.zeroJacobian(franka::Frame::kFlange, robot_state);
                    input.end_effector_jacobian =
                        model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                    input.end_effector_position = {robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                                                   robot_state.O_T_EE[14]};
                    input.elbow_velocity = robot_state.delbow_c[0];
                    apply_pending_command();
                    const auto torque = controller_.compute(input);
                    {
                        std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
                        if (lock.owns_lock()) {
                            state_.sequence++;
                            state_.monotonic_ns = monotonic_ns();
                            state_.q = robot_state.q;
                            state_.dq = robot_state.dq;
                            state_.torque = robot_state.tau_J;
                            state_.commanded_q = controller_.commanded_position();
                            state_.valid = true;
                            state_.faulted = false;
                        }
                    }
                    franka::Torques command(torque);
                    if (stop_requested_) {
                        return franka::MotionFinished(command);
                    }
                    return command;
                },
                true, 100.0);
        }
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!stop_requested_) {
            state_.faulted = true;
            state_.fault = error.what();
            PI_ERROR("HARDWARE FAULT: FR3 controller stopped: %s", error.what());
        }
    }
    running_ = false;
    {
        std::lock_guard<std::mutex> lock(controller_lifecycle_mutex_);
        controller_exited_ = true;
    }
    controller_lifecycle_cv_.notify_all();
}
