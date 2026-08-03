#include "pi_fr3_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

FR3Controller::FR3Controller(FR3ControllerGains gains, FR3ControllerLimits limits)
    : gains_(gains), limits_(limits) {}

void FR3Controller::set_target(const std::array<double, 7>& target_position) {
    target_position_ = target_position;
    initialized_ = true;
    position_command_active_ = true;
}

void FR3Controller::hold(const std::array<double, 7>& measured_position) {
    target_position_ = measured_position;
    initialized_ = true;
    position_command_active_ = false;
}

void FR3Controller::add_soft_limit(double value, double lower, double upper, double margin,
                                   double stiffness, double& output) {
    const double upper_violation = value - upper;
    const double lower_violation = lower - value;
    if (upper_violation > 0 || lower_violation > 0) {
        throw std::runtime_error("FR3 safety hard limit exceeded");
    }
    if (upper_violation > -margin) {
        output -= stiffness * (margin + upper_violation);
    } else if (lower_violation > -margin) {
        output += stiffness * (margin + lower_violation);
    }
}

std::array<double, 7> FR3Controller::compute(const FR3ControllerInput& input) {
    if (!initialized_) hold(input.q);

    std::array<double, 7> position_error{};
    std::array<double, 7> velocity_error{};
    std::array<double, 7> torque = input.coriolis;
    for (size_t i = 0; i < 7; ++i) {
        position_error[i] = target_position_[i] - input.q[i];
        velocity_error[i] = -input.dq[i];
        torque[i] += gains_.joint_stiffness[i] * position_error[i] +
                     gains_.joint_damping[i] * velocity_error[i];
    }

    // Tuned hybrid joint-space impedance term from the FR3 implementation.
    if (position_command_active_) {
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
        throw std::runtime_error("FR3 safety elbow velocity hard limit exceeded");
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
