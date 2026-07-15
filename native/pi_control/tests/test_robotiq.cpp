#include <gtest/gtest.h>

#include "pi_robotiq.hpp"

TEST(RobotiqCalibration, PublicPositionIsZeroClosedOneOpen) {
    EXPECT_EQ(RobotiqTransport::position_to_raw(1.0f, 3, 230), 3);
    EXPECT_EQ(RobotiqTransport::position_to_raw(0.0f, 3, 230), 230);
    EXPECT_FLOAT_EQ(RobotiqTransport::raw_to_position(3, 3, 230), 1.0f);
    EXPECT_FLOAT_EQ(RobotiqTransport::raw_to_position(230, 3, 230), 0.0f);
}

TEST(RobotiqCalibration, PositiveForceNeverDisablesRegrasp) {
    EXPECT_EQ(RobotiqTransport::normalized_to_raw(0.001f, true), 1);
    EXPECT_EQ(RobotiqTransport::normalized_to_raw(0.0f, true), 0);
}
