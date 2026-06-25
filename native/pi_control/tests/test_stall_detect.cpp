#include <gtest/gtest.h>

#include "pi_stall_detect.hpp"

// Implementation thresholds (pi_stall_detect.cpp): |vel| < 0.1, |dpos| < 0.1,
// |tor| > 0.1, sustained for 3 consecutive cycles.

TEST(StallDetect, RequiresThreeConsecutiveStallCycles) {
    StallDetect detector;
    EXPECT_FLOAT_EQ(detector.detect_stall(0.0f, 0.0f, 1.0f), 0.0f);
    EXPECT_FALSE(detector.is_stall_detected());
    EXPECT_FLOAT_EQ(detector.detect_stall(0.0f, 0.0f, 1.0f), 0.0f);
    EXPECT_FALSE(detector.is_stall_detected());
    EXPECT_FLOAT_EQ(detector.detect_stall(0.0f, 0.0f, 1.0f), 1.0f);
    EXPECT_TRUE(detector.is_stall_detected());
}

TEST(StallDetect, ReportsAverageTorqueOverStallWindow) {
    StallDetect detector;
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    detector.detect_stall(0.0f, 0.0f, 2.0f);
    EXPECT_FLOAT_EQ(detector.detect_stall(0.0f, 0.0f, 3.0f), 2.0f);
    EXPECT_FLOAT_EQ(detector.get_stall_tor(), 2.0f);
}

TEST(StallDetect, MovementResetsDetection) {
    StallDetect detector;
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    ASSERT_TRUE(detector.is_stall_detected());

    // High velocity breaks the stall condition and clears all state.
    EXPECT_FLOAT_EQ(detector.detect_stall(0.0f, 5.0f, 1.0f), 0.0f);
    EXPECT_FALSE(detector.is_stall_detected());
    EXPECT_FLOAT_EQ(detector.get_stall_tor(), 0.0f);
}

TEST(StallDetect, PositionChangeBreaksStallStreak) {
    StallDetect detector;
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    // Joint actually moved 0.5 rad: not a stall even though vel reads zero.
    detector.detect_stall(0.5f, 0.0f, 1.0f);
    EXPECT_FALSE(detector.is_stall_detected());
}

TEST(StallDetect, LowTorqueIsNeverAStall) {
    StallDetect detector;
    for (int i = 0; i < 10; ++i) {
        EXPECT_FLOAT_EQ(detector.detect_stall(0.0f, 0.0f, 0.05f), 0.0f);
    }
    EXPECT_FALSE(detector.is_stall_detected());
}

TEST(StallDetect, DetectionPersistsWhileConditionHolds) {
    StallDetect detector;
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    ASSERT_TRUE(detector.is_stall_detected());

    // The stalled state continues to be reported on subsequent stalled cycles.
    detector.detect_stall(0.0f, 0.0f, 1.0f);
    EXPECT_TRUE(detector.is_stall_detected());
    EXPECT_FLOAT_EQ(detector.get_stall_tor(), 1.0f);
}
