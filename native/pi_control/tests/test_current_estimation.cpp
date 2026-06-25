#include <gtest/gtest.h>

#include "pi_current_estimation.hpp"

namespace {

MotorParams4CurrentEstimation make_motor_params() {
    MotorParams4CurrentEstimation params;
    params.kt0_ = 0.1f;   // Nm/A
    params.r0_ = 0.5f;    // Ohm
    params.t0_ = 25.0f;   // deg C
    params.eta_inv_ = 0.96f;
    params.eta_g_ = 1.0f;
    params.gear_ratio_ = 1.0f;
    return params;
}

}  // namespace

TEST(CurrentEstimation, IdleMotorDrawsOnlyBaselineCurrent) {
    CurrentEstimation estimator;
    // Default calibration: a_gain=0.256, b_offset=0, i_baseline=0.016.
    const float idle = estimator.estimate_idc_calibrated(make_motor_params(), 0.0f, 0.0f, 25.0f, 24.0f, 10.0f);
    EXPECT_NEAR(idle, 0.016f, 1e-5f);
}

TEST(CurrentEstimation, EstimateIsNeverNegative) {
    CurrentEstimation estimator;
    const auto params = make_motor_params();
    for (float torque : {-5.0f, -1.0f, 0.0f, 1.0f, 5.0f}) {
        for (float velocity : {-10.0f, 0.0f, 10.0f}) {
            const float current =
                estimator.estimate_idc_calibrated(params, torque, velocity, 40.0f, 24.0f, 10.0f);
            EXPECT_GE(current, 0.0f) << "torque=" << torque << " velocity=" << velocity;
        }
    }
}

TEST(CurrentEstimation, CurrentGrowsWithTorqueMagnitude) {
    CurrentEstimation estimator;
    const auto params = make_motor_params();
    const float i0 = estimator.estimate_idc_calibrated(params, 0.5f, 1.0f, 25.0f, 24.0f, 10.0f);
    const float i1 = estimator.estimate_idc_calibrated(params, 1.0f, 1.0f, 25.0f, 24.0f, 10.0f);
    const float i2 = estimator.estimate_idc_calibrated(params, 2.0f, 1.0f, 25.0f, 24.0f, 10.0f);
    EXPECT_LT(i0, i1);
    EXPECT_LT(i1, i2);
    // Sign of the torque must not matter for current draw.
    const float i2_neg = estimator.estimate_idc_calibrated(params, -2.0f, 1.0f, 25.0f, 24.0f, 10.0f);
    EXPECT_FLOAT_EQ(i2, i2_neg);
}

TEST(CurrentEstimation, StallConditionBoostsEstimate) {
    CurrentEstimation estimator;
    const auto params = make_motor_params();
    // Same operating point; only the max_torque threshold decides "stall".
    const float stalled =
        estimator.estimate_idc_calibrated(params, 11.0f, 0.01f, 25.0f, 24.0f, 10.0f);
    const float not_stalled =
        estimator.estimate_idc_calibrated(params, 11.0f, 0.01f, 25.0f, 24.0f, 20.0f);
    EXPECT_GT(stalled, not_stalled);
}

TEST(CurrentEstimation, MovingFastBlocksStallBoost) {
    CurrentEstimation estimator;
    const auto params = make_motor_params();
    // Above-threshold torque but the joint is clearly moving: no stall gain.
    // (Default stall_omega_thr is 0.05 rad/s.)
    const float fast = estimator.estimate_idc_calibrated(params, 11.0f, 5.0f, 25.0f, 24.0f, 10.0f);
    const float fast_high_threshold =
        estimator.estimate_idc_calibrated(params, 11.0f, 5.0f, 25.0f, 24.0f, 20.0f);
    EXPECT_FLOAT_EQ(fast, fast_high_threshold);
}
