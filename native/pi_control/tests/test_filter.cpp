#include <gtest/gtest.h>

#include <cmath>

#include "pi_filter.hpp"

TEST(FilterEma, SeedsWithFirstSampleThenBlends) {
    Filter f(FilterType::EMA, 0.5f);
    EXPECT_FLOAT_EQ(f.update(10.0f), 10.0f);
    EXPECT_FLOAT_EQ(f.update(0.0f), 5.0f);
    EXPECT_FLOAT_EQ(f.update(0.0f), 2.5f);
    EXPECT_FLOAT_EQ(f.get_latest_value(), 2.5f);
}

TEST(FilterEma, HighAlphaTracksInputClosely) {
    Filter f(FilterType::EMA, 1.0f);
    f.update(10.0f);
    EXPECT_FLOAT_EQ(f.update(-3.0f), -3.0f);
}

TEST(FilterMedian, RejectsImpulseOutliers) {
    Filter f(FilterType::MEDIAN, 0.2f, 3);
    EXPECT_FLOAT_EQ(f.update(1.0f), 1.0f);
    // Two samples => even count => mean of the two central values.
    EXPECT_FLOAT_EQ(f.update(3.0f), 2.0f);
    // Window {1, 3, 100}: the spike does not move the median.
    EXPECT_FLOAT_EQ(f.update(100.0f), 3.0f);
    // Window slides to {3, 100, 2}.
    EXPECT_FLOAT_EQ(f.update(2.0f), 3.0f);
}

TEST(FilterMovingAverage, AveragesPartialThenSlidingWindow) {
    Filter f(FilterType::MOVING_AVERAGE, 0.2f, 3);
    EXPECT_FLOAT_EQ(f.update(1.0f), 1.0f);
    EXPECT_FLOAT_EQ(f.update(2.0f), 1.5f);
    EXPECT_FLOAT_EQ(f.update(3.0f), 2.0f);
    // Window slides: {2, 3, 10}.
    EXPECT_FLOAT_EQ(f.update(10.0f), 5.0f);
}

TEST(FilterWindowed, ZeroWindowIsRejectedBeforeUpdate) {
    EXPECT_THROW((Filter(FilterType::MEDIAN, 0.2f, 0)), std::invalid_argument);
    EXPECT_THROW((Filter(FilterType::MOVING_AVERAGE, 0.2f, 0)), std::invalid_argument);
}

TEST(FilterKalman, SeedsWithFirstSampleAndConvergesToConstantSignal) {
    Filter f(FilterType::KALMAN, 0.2f, 5, 0.01f, 1.0f);
    EXPECT_FLOAT_EQ(f.update(5.0f), 5.0f);

    float estimate = 5.0f;
    for (int i = 0; i < 200; ++i) {
        estimate = f.update(10.0f);
    }
    EXPECT_NEAR(estimate, 10.0f, 0.1f);
}

TEST(FilterKalman, EstimateMovesMonotonicallyTowardStepInput) {
    Filter f(FilterType::KALMAN);
    f.update(0.0f);
    float prev = 0.0f;
    for (int i = 0; i < 50; ++i) {
        float next = f.update(10.0f);
        EXPECT_GE(next, prev - 1e-6f) << "step " << i;
        EXPECT_LE(next, 10.0f + 1e-6f) << "step " << i;
        prev = next;
    }
}

TEST(FilterAll, StableUnderConstantInput) {
    for (FilterType type :
         {FilterType::EMA, FilterType::MEDIAN, FilterType::MOVING_AVERAGE, FilterType::KALMAN}) {
        Filter f(type);
        float out = 0.0f;
        for (int i = 0; i < 100; ++i) {
            out = f.update(7.5f);
        }
        EXPECT_NEAR(out, 7.5f, 1e-3f) << "type=" << static_cast<int>(type);
        EXPECT_TRUE(std::isfinite(out));
    }
}
