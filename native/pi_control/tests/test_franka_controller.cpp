#include <gtest/gtest.h>

#include "pi_franka_controller.hpp"

TEST(FrankaController, HoldsMeasuredPositionUntilFirstCommand) {
    FrankaController controller;
    FrankaControllerInput input;
    input.q = {0, -0.6, 0, -2.5, 0, 1.8, 0};
    controller.compute(input);
    EXPECT_EQ(controller.commanded_position(), input.q);
    controller.compute(input);
    EXPECT_EQ(controller.commanded_position(), input.q);
}

TEST(FrankaController, HoldDiscardsThePreviousPositionTarget) {
    FrankaController controller;
    FrankaControllerInput input;
    input.q = {0, -0.6, 0, -2.5, 0, 1.8, 0};
    controller.compute(input);

    MsgDroidCommand position_command;
    position_command.mode = DroidControlMode::JOINT_POSITION;
    position_command.joint_position = {0.2f, -0.4f, 0.1f, -2.3f, 0.1f, 1.6f, 0.2f};
    controller.set_command(position_command);
    controller.compute(input);
    EXPECT_NE(controller.commanded_position(), input.q);

    MsgDroidCommand hold_command;
    hold_command.mode = DroidControlMode::HOLD;
    controller.set_command(hold_command);
    controller.compute(input);
    EXPECT_EQ(controller.commanded_position(), input.q);
    controller.compute(input);
    EXPECT_EQ(controller.commanded_position(), input.q);
}

TEST(FrankaController, IntegratesVelocityAtExactlyOneKilohertz) {
    FrankaController controller;
    FrankaControllerInput input;
    input.q = {0, -0.6, 0, -2.5, 0, 1.8, 0};
    controller.compute(input);
    MsgDroidCommand command;
    command.mode = DroidControlMode::JOINT_VELOCITY;
    command.joint_velocity[0] = 0.5f;
    controller.set_command(command);
    controller.compute(input);
    EXPECT_NEAR(controller.commanded_position()[0], 0.0005, 1e-9);
    controller.compute(input);
    EXPECT_NEAR(controller.commanded_position()[0], 0.001, 1e-9);
}

TEST(FrankaController, StaleVelocityCommandFallsBackToMeasuredHold) {
    FrankaController controller;
    FrankaControllerInput input;
    input.q = {0, -0.6, 0, -2.5, 0, 1.8, 0};
    input.monotonic_ns = 1000000000;
    controller.compute(input);

    MsgDroidCommand command;
    command.mode = DroidControlMode::JOINT_VELOCITY;
    command.monotonic_ns = input.monotonic_ns;
    command.joint_velocity[0] = 0.5f;
    controller.set_command(command);
    controller.compute(input);
    EXPECT_NEAR(controller.commanded_position()[0], 0.0005, 1e-9);

    input.q[0] = 0.0004;
    input.monotonic_ns += 250000001;
    controller.compute(input);
    EXPECT_NEAR(controller.commanded_position()[0], input.q[0], 1e-9);
}

TEST(FrankaController, RejectsHardJointLimitViolations) {
    FrankaController controller;
    FrankaControllerInput input;
    input.q = {3.0, -0.6, 0, -2.5, 0, 1.8, 0};
    EXPECT_THROW(controller.compute(input), std::runtime_error);
}
