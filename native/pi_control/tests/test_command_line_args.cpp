/*!
 * @file test_command_line_args.cpp
 * @brief Unit tests for CommandLineArgs helpers.
 */

#include <gtest/gtest.h>

#include "pi_command_line_args.hpp"

TEST(TorqRescaleCsv, ParsesCommaSeparatedFloats) {
    const auto values = CommandLineArgs::parse_torq_rescale_csv("0.8,0.8,0.8,1.5,1.5,1.5");
    ASSERT_EQ(values.size(), 6u);
    EXPECT_FLOAT_EQ(values[0], 0.8f);
    EXPECT_FLOAT_EQ(values[3], 1.5f);
}

TEST(TorqRescaleCsv, RejectsMalformedTokens) {
    EXPECT_TRUE(CommandLineArgs::parse_torq_rescale_csv("0.8,abc").empty());
    EXPECT_TRUE(CommandLineArgs::parse_torq_rescale_csv("0.8x,1.5").empty());
    EXPECT_TRUE(CommandLineArgs::parse_torq_rescale_csv("0.8,,1.5").empty());
}

TEST(TorqRescaleCsv, RejectsNonPhysicalValues) {
    EXPECT_TRUE(CommandLineArgs::parse_torq_rescale_csv("-0.1,0.8").empty());
    EXPECT_TRUE(CommandLineArgs::parse_torq_rescale_csv("inf,0.8").empty());
    EXPECT_TRUE(CommandLineArgs::parse_torq_rescale_csv("nan").empty());
}

TEST(TorqRescaleCsv, AllowsZeroForVirtualJoints) {
    const auto values = CommandLineArgs::parse_torq_rescale_csv("0,1.0");
    ASSERT_EQ(values.size(), 2u);
    EXPECT_FLOAT_EQ(values[0], 0.0f);
}
