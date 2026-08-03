#include <gtest/gtest.h>

#include "pi_fr3_controller.hpp"

TEST(FR3Controller, HoldsMeasuredPositionUntilFirstCommand) {
    FR3Controller controller;
    FR3ControllerInput input;
    input.q = {0, -0.6, 0, -2.5, 0, 1.8, 0};
    controller.compute(input);
    EXPECT_EQ(controller.commanded_position(), input.q);
}

TEST(FR3Controller, PositionCommandUsesTunedImpedanceGains) {
    FR3Controller controller;
    FR3ControllerInput input;
    input.q = {0, -0.6, 0, -2.5, 0, 1.8, 0};
    const std::array<double, 7> target{0.1, -0.6, 0, -2.5, 0, 1.8, 0};
    controller.set_target(target);
    const auto torque = controller.compute(input);
    EXPECT_NEAR(torque[0], 4.0, 1e-9);
    EXPECT_EQ(controller.commanded_position(), target);
}

TEST(FR3Controller, HoldDiscardsPreviousPositionTarget) {
    FR3Controller controller;
    FR3ControllerInput input;
    input.q = {0, -0.6, 0, -2.5, 0, 1.8, 0};
    controller.set_target({0.1, -0.6, 0, -2.5, 0, 1.8, 0});
    controller.hold(input.q);
    EXPECT_EQ(controller.commanded_position(), input.q);
}

TEST(FR3Controller, RejectsHardJointLimitViolations) {
    FR3Controller controller;
    FR3ControllerInput input;
    input.q = {3.0, -0.6, 0, -2.5, 0, 1.8, 0};
    EXPECT_THROW(controller.compute(input), std::runtime_error);
}
