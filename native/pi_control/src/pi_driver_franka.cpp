#include "pi_driver_franka.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#include "pi_info.hpp"

#ifdef OPENPI_CONTROL_WITH_FRANKA
#include <franka/control_types.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#endif

namespace {

template <size_t Size>
std::array<double, Size> parse_array(const std::string& value, const char* name) {
    std::array<double, Size> result{};
    std::stringstream stream(value);
    std::string item;
    size_t index = 0;
    while (std::getline(stream, item, ',') && index < result.size()) {
        result[index++] = std::stod(item);
    }
    if (index != result.size() || std::getline(stream, item, ',')) {
        throw std::invalid_argument(std::string(name) + " has the wrong number of values");
    }
    return result;
}

FrankaControllerLimits parse_limits(const CommandLineArgs& cla) {
    FrankaControllerLimits limits;
    limits.joint_lower = parse_array<7>(cla.franka_joint_lower, "joint lower limits");
    limits.joint_upper = parse_array<7>(cla.franka_joint_upper, "joint upper limits");
    limits.velocity = parse_array<7>(cla.franka_joint_velocity, "joint velocity limits");
    limits.torque = parse_array<7>(cla.franka_torque_limit, "torque limits");
    limits.cartesian_lower = parse_array<3>(cla.franka_cartesian_lower, "Cartesian lower limits");
    limits.cartesian_upper = parse_array<3>(cla.franka_cartesian_upper, "Cartesian upper limits");
    const auto scalars = parse_array<6>(cla.franka_safety_scalars, "safety scalars");
    limits.joint_margin = scalars[0];
    limits.velocity_margin = scalars[1];
    limits.cartesian_margin = scalars[2];
    limits.joint_stiffness = scalars[3];
    limits.velocity_stiffness = scalars[4];
    limits.cartesian_stiffness = scalars[5];
    return limits;
}

uint64_t monotonic_ns() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

#ifdef OPENPI_CONTROL_WITH_FRANKA
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
            for (size_t i = 0; i < 7; ++i) {
                max_error = std::max(max_error, std::abs(state.q[i] - goal_[i]));
            }
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
                desired[i] = -max_velocity_sync_[i] * sign /
                             std::pow(first_sync_[i], 3) *
                             (0.5 * time - first_sync_[i]) * std::pow(time, 3);
            } else if (time < second_sync_[i]) {
                desired[i] = first_position_[i] +
                             (time - first_sync_[i]) * max_velocity_sync_[i] * sign;
            } else if (time < finish_sync_[i]) {
                desired[i] =
                    delta_[i] + 0.5 *
                                    (1.0 / std::pow(goal_ramp_duration, 3) *
                                         (time - first_sync_[i] - 2 * goal_ramp_duration -
                                          constant_duration) *
                                         std::pow(time - first_sync_[i] - constant_duration, 3) +
                                     (2 * time - 2 * first_sync_[i] - goal_ramp_duration -
                                      2 * constant_duration)) *
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
            const double threshold = 0.75 * std::pow(max_velocity_[i], 2) /
                                     max_start_acceleration_[i] +
                                     0.75 * std::pow(max_velocity_[i], 2) /
                                         max_goal_acceleration_[i];
            if (std::abs(delta_[i]) < threshold) {
                reachable_velocity[i] =
                    std::sqrt(4.0 / 3.0 * delta_[i] * sign *
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
            const double b = -synchronized_finish * max_goal_acceleration_[i] *
                             max_start_acceleration_[i];
            const double c = std::abs(delta_[i]) * max_goal_acceleration_[i] *
                             max_start_acceleration_[i];
            max_velocity_sync_[i] = (-b - std::sqrt(std::max(0.0, b * b - 4 * a * c))) /
                                    (2 * a);
            first_sync_[i] = 1.5 * max_velocity_sync_[i] / max_start_acceleration_[i];
            const double goal_ramp = 1.5 * max_velocity_sync_[i] / max_goal_acceleration_[i];
            finish_sync_[i] = first_sync_[i] / 2 + goal_ramp / 2 +
                              std::abs(delta_[i] / max_velocity_sync_[i]);
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
        {{40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40}},
        {{40, 40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40, 40}},
        {{40, 40, 40, 40, 40, 40}}, {{40, 40, 40, 40, 40, 40}});
}
#endif

}  // namespace

struct DriverFranka::Impl {
#ifdef OPENPI_CONTROL_WITH_FRANKA
    std::unique_ptr<franka::Robot> robot;
#endif
};

DriverFranka::DriverFranka(Device* device, const CommandLineArgs& cla)
    : Driver(device, cla),
      cla_(cla),
      impl_(std::make_unique<Impl>()),
      limits_(parse_limits(cla)),
      controller_({}, limits_),
      reset_pose_(parse_array<7>(cla.franka_reset_pose, "reset pose")) {}

DriverFranka::~DriverFranka() { close(); }

ReturnCode DriverFranka::open(int baud_rate) {
    (void)baud_rate;
#ifdef OPENPI_CONTROL_WITH_FRANKA
    try {
        const auto realtime = cla_.franka_realtime_config == "ignore"
                                  ? franka::RealtimeConfig::kIgnore
                                  : franka::RealtimeConfig::kEnforce;
        impl_->robot = std::make_unique<franka::Robot>(cla_.franka_address, realtime);
        configure_collision_behavior(*impl_->robot);
        const auto initial = impl_->robot->readOnce();
        {
            std::lock_guard<std::mutex> lock(pending_controller_mutex_);
            controller_.hold(initial.q);
            command_pending_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.q = initial.q;
            state_.dq = initial.dq;
            state_.commanded_q = initial.q;
            state_.valid = true;
            state_.faulted = false;
        }
        running_ = true;
        stop_requested_ = false;
        thread_ = std::thread(cla_.franka_read_only ? &DriverFranka::run_read_only
                                                    : &DriverFranka::run_controller,
                              this);
        return ReturnCode::SUCCESS;
    } catch (const std::exception& error) {
        PI_ERROR("Failed to start Franka: %s", error.what());
        return ReturnCode::FAIL;
    }
#elif defined(OPENPI_CONTROL_MOCK_FRANKA)
    running_ = true;
    thread_ = std::thread(&DriverFranka::run_mock, this);
    return ReturnCode::SUCCESS;
#else
    PI_ERROR("Franka support was not built; enable OPENPI_CONTROL_WITH_FRANKA");
    return ReturnCode::NOT_SUPPORTED;
#endif
}

ReturnCode DriverFranka::close() {
    running_ = false;
#ifdef OPENPI_CONTROL_WITH_FRANKA
    if (impl_->robot && thread_.joinable() && !cla_.franka_read_only) {
        stop_requested_ = true;
        try {
            impl_->robot->stop();
        } catch (const std::exception& error) {
            PI_WARN("Franka stop failed: %s", error.what());
        }
    }
#endif
    if (thread_.joinable()) thread_.join();
#ifdef OPENPI_CONTROL_WITH_FRANKA
    impl_->robot.reset();
#endif
    return ReturnCode::SUCCESS;
}

FrankaDriverState DriverFranka::state() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return state_;
}

ReturnCode DriverFranka::set_command(const MsgDroidCommand& command) {
    if (cla_.franka_read_only) return ReturnCode::NOT_SUPPORTED;
    if (!std::all_of(command.joint_position.begin(), command.joint_position.end(),
                     [](float value) { return std::isfinite(value); }) ||
        !std::all_of(command.joint_velocity.begin(), command.joint_velocity.end(),
                     [](float value) { return std::isfinite(value); })) {
        return ReturnCode::INVALID_PARAM;
    }
    if (command.mode != DroidControlMode::HOLD &&
        command.mode != DroidControlMode::JOINT_POSITION &&
        command.mode != DroidControlMode::JOINT_VELOCITY) {
        return ReturnCode::INVALID_PARAM;
    }
    if (command.mode == DroidControlMode::JOINT_VELOCITY) {
        for (size_t i = 0; i < command.joint_velocity.size(); ++i) {
            if (std::abs(command.joint_velocity[i]) > limits_.velocity[i]) {
                return ReturnCode::INVALID_PARAM;
            }
        }
    }
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);
    if (command.sequence <= last_command_sequence_) return ReturnCode::INVALID_PARAM;
    last_command_sequence_ = command.sequence;
    pending_command_ = command;
    command_pending_ = true;
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFranka::hold() {
    const auto snapshot = state();
    if (!snapshot.valid) return ReturnCode::NOT_INITIALIZED;
    std::lock_guard<std::mutex> lock(pending_controller_mutex_);
    pending_command_ = {};
    pending_command_.mode = DroidControlMode::HOLD;
    command_pending_ = true;
    return ReturnCode::SUCCESS;
}

ReturnCode DriverFranka::recover() {
#ifdef OPENPI_CONTROL_WITH_FRANKA
    if (!impl_->robot) return ReturnCode::NOT_INITIALIZED;
    close();
    try {
        const auto realtime = cla_.franka_realtime_config == "ignore"
                                  ? franka::RealtimeConfig::kIgnore
                                  : franka::RealtimeConfig::kEnforce;
        impl_->robot = std::make_unique<franka::Robot>(cla_.franka_address, realtime);
        impl_->robot->automaticErrorRecovery();
        configure_collision_behavior(*impl_->robot);
        const auto initial = impl_->robot->readOnce();
        {
            std::lock_guard<std::mutex> lock(pending_controller_mutex_);
            controller_.hold(initial.q);
            command_pending_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.q = initial.q;
            state_.dq = initial.dq;
            state_.commanded_q = initial.q;
            state_.valid = true;
            state_.faulted = false;
            state_.fault.clear();
        }
        running_ = true;
        stop_requested_ = false;
        thread_ = std::thread(&DriverFranka::run_controller, this);
        return ReturnCode::SUCCESS;
    } catch (const std::exception& error) {
        PI_ERROR("Franka recovery failed: %s", error.what());
        return ReturnCode::HARDWARE_FAULT;
    }
#elif defined(OPENPI_CONTROL_MOCK_FRANKA)
    return ReturnCode::SUCCESS;
#else
    return ReturnCode::NOT_SUPPORTED;
#endif
}

ReturnCode DriverFranka::move_to_ready() {
    if (cla_.franka_read_only) return ReturnCode::NOT_SUPPORTED;
#ifdef OPENPI_CONTROL_WITH_FRANKA
    close();
    try {
        const auto realtime = cla_.franka_realtime_config == "ignore"
                                  ? franka::RealtimeConfig::kIgnore
                                  : franka::RealtimeConfig::kEnforce;
        impl_->robot = std::make_unique<franka::Robot>(cla_.franka_address, realtime);
        impl_->robot->automaticErrorRecovery();
        configure_collision_behavior(*impl_->robot);
        ResetMotionGenerator motion_generator(0.2, reset_pose_);
        impl_->robot->control([&](const franka::RobotState& state, franka::Duration period) {
            return motion_generator(state, period);
        });
        const auto initial = impl_->robot->readOnce();
        {
            std::lock_guard<std::mutex> lock(pending_controller_mutex_);
            controller_.hold(initial.q);
            command_pending_ = false;
        }
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.q = initial.q;
            state_.dq = initial.dq;
            state_.commanded_q = initial.q;
            state_.valid = true;
            state_.faulted = false;
        }
        running_ = true;
        stop_requested_ = false;
        thread_ = std::thread(&DriverFranka::run_controller, this);
        return ReturnCode::SUCCESS;
    } catch (const std::exception& error) {
        PI_ERROR("Franka move-to-ready failed: %s", error.what());
        return ReturnCode::HARDWARE_FAULT;
    }
#elif defined(OPENPI_CONTROL_MOCK_FRANKA)
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_.q = reset_pose_;
    return ReturnCode::SUCCESS;
#else
    return ReturnCode::NOT_SUPPORTED;
#endif
}

void DriverFranka::apply_pending_command() {
    std::unique_lock<std::mutex> lock(pending_controller_mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    if (command_pending_) {
        controller_.set_command(pending_command_);
        command_pending_ = false;
    }
}

void DriverFranka::run_controller() {
#ifdef OPENPI_CONTROL_WITH_FRANKA
    try {
        auto model = impl_->robot->loadModel();
        impl_->robot->control([this, &model](const franka::RobotState& state, franka::Duration) {
            FrankaControllerInput input;
            input.monotonic_ns = monotonic_ns();
            input.q = state.q;
            input.dq = state.dq;
            input.coriolis = model.coriolis(state);
            input.flange_jacobian = model.zeroJacobian(franka::Frame::kFlange, state);
            input.end_effector_jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
            input.end_effector_position = {state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]};
            input.elbow_velocity = state.delbow_c[0];
            std::array<double, 7> torque;
            std::array<double, 7> commanded;
            apply_pending_command();
            torque = controller_.compute(input);
            commanded = controller_.commanded_position();
            {
                std::unique_lock<std::mutex> lock(state_mutex_, std::try_to_lock);
                if (lock.owns_lock()) {
                    state_.sequence++;
                    state_.monotonic_ns = monotonic_ns();
                    state_.hardware_timestamp_s = state.time.toSec();
                    state_.q = state.q;
                    state_.dq = state.dq;
                    state_.torque = state.tau_J;
                    state_.commanded_q = commanded;
                    state_.external_torque = state.tau_ext_hat_filtered;
                    state_.external_wrench = state.O_F_ext_hat_K;
                    state_.contact_bits = 0;
                    state_.collision_bits = 0;
                    for (size_t i = 0; i < 7; ++i) {
                        if (state.joint_contact[i] != 0) state_.contact_bits |= 1 << i;
                        if (state.joint_collision[i] != 0) state_.collision_bits |= 1 << i;
                    }
                    state_.robot_mode = static_cast<int>(state.robot_mode);
                    state_.control_command_success_rate = state.control_command_success_rate;
                    state_.valid = true;
                    state_.faulted = false;
                }
            }
            return franka::Torques(torque);
        }, true, 100.0);
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!stop_requested_) {
            state_.faulted = true;
            state_.fault = error.what();
            PI_ERROR("HARDWARE FAULT: Franka controller stopped: %s", error.what());
        }
    }
    running_ = false;
#endif
}

void DriverFranka::run_read_only() {
#ifdef OPENPI_CONTROL_WITH_FRANKA
    try {
        while (running_) {
            const auto robot_state = impl_->robot->readOnce();
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.sequence++;
            state_.monotonic_ns = monotonic_ns();
            state_.hardware_timestamp_s = robot_state.time.toSec();
            state_.q = robot_state.q;
            state_.dq = robot_state.dq;
            state_.torque = robot_state.tau_J;
            state_.commanded_q = robot_state.q_d;
            state_.external_torque = robot_state.tau_ext_hat_filtered;
            state_.external_wrench = robot_state.O_F_ext_hat_K;
            state_.robot_mode = static_cast<int>(robot_state.robot_mode);
            state_.control_command_success_rate = robot_state.control_command_success_rate;
            state_.valid = true;
        }
    } catch (const std::exception& error) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (running_) {
            state_.faulted = true;
            state_.fault = error.what();
        }
    }
    running_ = false;
#endif
}

void DriverFranka::run_mock() {
#ifdef OPENPI_CONTROL_MOCK_FRANKA
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_.q = reset_pose_;
        state_.valid = true;
    }
    while (running_) {
        FrankaControllerInput input;
        input.monotonic_ns = monotonic_ns();
        std::array<double, 7> commanded;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            input.q = state_.q;
            input.dq = state_.dq;
        }
        apply_pending_command();
        controller_.compute(input);
        commanded = controller_.commanded_position();
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_.sequence++;
            state_.monotonic_ns = monotonic_ns();
            state_.commanded_q = commanded;
            for (size_t i = 0; i < 7; ++i) state_.q[i] += 0.05 * (state_.commanded_q[i] - state_.q[i]);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
#endif
}
