#include "pi_franka_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

FrankaController::FrankaController(FrankaControllerGains gains, FrankaControllerLimits limits)
    : gains_(gains), limits_(limits) {}

void FrankaController::set_command(const MsgDroidCommand& command) { command_ = command; }

void FrankaController::hold(const std::array<double, 7>& measured_position) {
    command_.mode = DroidControlMode::HOLD;
    target_position_ = measured_position;
    initialized_ = true;
    active_mode_ = DroidControlMode::HOLD;
}

void FrankaController::add_soft_limit(double value, double lower, double upper, double margin,
                                      double stiffness, double& output) {
    const double upper_violation = value - upper;
    const double lower_violation = lower - value;
    if (upper_violation > 0 || lower_violation > 0) {
        throw std::runtime_error("DROID safety hard limit exceeded");
    }
    if (upper_violation > -margin) {
        output -= stiffness * (margin + upper_violation);
    } else if (lower_violation > -margin) {
        output += stiffness * (margin + lower_violation);
    }
}

std::array<double, 7> FrankaController::compute(const FrankaControllerInput& input) {
    if (!initialized_) {
        target_position_ = input.q;
        initialized_ = true;
    }
    DroidControlMode requested_mode = command_.mode;
    if (requested_mode == DroidControlMode::JOINT_VELOCITY && command_.monotonic_ns != 0 &&
        input.monotonic_ns > command_.monotonic_ns + kVelocityCommandTimeoutNs) {
        requested_mode = DroidControlMode::HOLD;
    }
    if (requested_mode != active_mode_) {
        // Polymetis initializes both velocity control and hold from measured q.
        if (requested_mode != DroidControlMode::JOINT_POSITION) target_position_ = input.q;
        active_mode_ = requested_mode;
    }

    std::array<double, 7> desired_velocity{};
    if (active_mode_ == DroidControlMode::JOINT_POSITION) {
        for (size_t i = 0; i < 7; ++i) target_position_[i] = command_.joint_position[i];
    } else if (active_mode_ == DroidControlMode::JOINT_VELOCITY) {
        // Exact Polymetis JointVelocityControl update at the fixed 1 kHz callback.
        for (size_t i = 0; i < 7; ++i) {
            desired_velocity[i] = command_.joint_velocity[i];
            target_position_[i] += desired_velocity[i] / 1000.0;
        }
    }

    std::array<double, 7> position_error{};
    std::array<double, 7> velocity_error{};
    std::array<double, 7> torque = input.coriolis;
    for (size_t i = 0; i < 7; ++i) {
        position_error[i] = target_position_[i] - input.q[i];
        velocity_error[i] = desired_velocity[i] - input.dq[i];
        torque[i] += gains_.joint_stiffness[i] * position_error[i] +
                     gains_.joint_damping[i] * velocity_error[i];
    }

    // Polymetis HybridJointSpacePD: J' Kx J e + J' Kxd J edot.
    if (active_mode_ == DroidControlMode::JOINT_POSITION) {
        for (size_t row = 0; row < 6; ++row) {
            double cartesian_position_error = 0;
            double cartesian_velocity_error = 0;
            for (size_t joint = 0; joint < 7; ++joint) {
                const double jacobian = input.flange_jacobian[row + 6 * joint];
                cartesian_position_error += jacobian * position_error[joint];
                cartesian_velocity_error += jacobian * velocity_error[joint];
            }
            const double wrench = gains_.cartesian_stiffness[row] * cartesian_position_error +
                                  gains_.cartesian_damping[row] * cartesian_velocity_error;
            for (size_t joint = 0; joint < 7; ++joint) {
                torque[joint] += input.flange_jacobian[row + 6 * joint] * wrench;
            }
        }
    }

    if (std::abs(input.elbow_velocity) > 2.075) {
        throw std::runtime_error("DROID safety elbow velocity hard limit exceeded");
    }
    std::array<double, 3> cartesian_force{};
    for (size_t axis = 0; axis < 3; ++axis) {
        add_soft_limit(input.end_effector_position[axis], limits_.cartesian_lower[axis],
                       limits_.cartesian_upper[axis], limits_.cartesian_margin,
                       limits_.cartesian_stiffness, cartesian_force[axis]);
    }
    for (size_t joint = 0; joint < 7; ++joint) {
        for (size_t axis = 0; axis < 3; ++axis) {
            torque[joint] += input.end_effector_jacobian[axis + 6 * joint] * cartesian_force[axis];
        }
        add_soft_limit(input.q[joint], limits_.joint_lower[joint], limits_.joint_upper[joint],
                       limits_.joint_margin, limits_.joint_stiffness, torque[joint]);
        add_soft_limit(input.dq[joint], -limits_.velocity[joint], limits_.velocity[joint],
                       limits_.velocity_margin, limits_.velocity_stiffness, torque[joint]);
        torque[joint] = std::clamp(torque[joint], -limits_.torque[joint], limits_.torque[joint]);
    }
    return torque;
}
