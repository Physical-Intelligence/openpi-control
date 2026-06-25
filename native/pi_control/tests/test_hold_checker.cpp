#include <gtest/gtest.h>

#include "pi_hold_checker.hpp"

TEST(HoldChecker, EntersHoldingAfterThresholdConsecutiveTrues) {
    HoldChecker checker(3);
    EXPECT_FALSE(checker.is_holding(true));
    EXPECT_FALSE(checker.is_holding(true));
    EXPECT_TRUE(checker.is_holding(true));
}

TEST(HoldChecker, FalseBeforeThresholdRestartsTheCount) {
    HoldChecker checker(3);
    checker.is_holding(true);
    checker.is_holding(true);
    EXPECT_FALSE(checker.is_holding(false));
    EXPECT_FALSE(checker.is_holding(true));
    EXPECT_FALSE(checker.is_holding(true));
    EXPECT_TRUE(checker.is_holding(true));
}

TEST(HoldChecker, ExitsOnlyAfterSustainedFalse) {
    // Exit hysteresis is 100 consecutive false conditions (EXIT_COUNT_THRESHOLD).
    HoldChecker checker(2);
    checker.is_holding(true);
    ASSERT_TRUE(checker.is_holding(true));

    for (int i = 0; i < 99; ++i) {
        EXPECT_TRUE(checker.is_holding(false)) << "premature exit at false #" << (i + 1);
    }
    EXPECT_FALSE(checker.is_holding(false));
}

TEST(HoldChecker, TrueDuringExitCountdownResetsHysteresis) {
    HoldChecker checker(2);
    checker.is_holding(true);
    ASSERT_TRUE(checker.is_holding(true));

    for (int i = 0; i < 50; ++i) {
        checker.is_holding(false);
    }
    // A single true resets the exit countdown; 99 more falses still hold.
    ASSERT_TRUE(checker.is_holding(true));
    for (int i = 0; i < 99; ++i) {
        EXPECT_TRUE(checker.is_holding(false));
    }
    EXPECT_FALSE(checker.is_holding(false));
}

TEST(HoldChecker, ResetClearsHoldingState) {
    HoldChecker checker(1);
    ASSERT_TRUE(checker.is_holding(true));
    checker.reset();
    // After reset the checker must re-accumulate the hold count.
    EXPECT_TRUE(checker.is_holding(true));  // threshold 1 re-enters immediately
    checker.reset();
    EXPECT_FALSE(checker.is_holding(false));
}

TEST(HoldChecker, SettingThresholdResetsState) {
    HoldChecker checker(1);
    ASSERT_TRUE(checker.is_holding(true));
    checker.set_hold_count_threshold(3);
    EXPECT_FALSE(checker.is_holding(true));
    EXPECT_FALSE(checker.is_holding(true));
    EXPECT_TRUE(checker.is_holding(true));
}
