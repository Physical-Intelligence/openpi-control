#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

#include "can/math_ops.h"

TEST(MathOps, Fmaxf3PicksLargestRegardlessOfPosition) {
    EXPECT_FLOAT_EQ(fmaxf3(3.0f, 1.0f, 2.0f), 3.0f);
    EXPECT_FLOAT_EQ(fmaxf3(1.0f, 3.0f, 2.0f), 3.0f);
    EXPECT_FLOAT_EQ(fmaxf3(1.0f, 2.0f, 3.0f), 3.0f);
    EXPECT_FLOAT_EQ(fmaxf3(-3.0f, -1.0f, -2.0f), -1.0f);
}

TEST(MathOps, Fminf3PicksSmallestRegardlessOfPosition) {
    EXPECT_FLOAT_EQ(fminf3(3.0f, 1.0f, 2.0f), 1.0f);
    EXPECT_FLOAT_EQ(fminf3(1.0f, 3.0f, 2.0f), 1.0f);
    EXPECT_FLOAT_EQ(fminf3(2.0f, 3.0f, 1.0f), 1.0f);
    EXPECT_FLOAT_EQ(fminf3(-3.0f, -1.0f, -2.0f), -3.0f);
}

TEST(MathOps, LimitNormScalesVectorDownPreservingDirection) {
    float x = 3.0f;
    float y = 4.0f;
    limit_norm(&x, &y, 2.5f);
    EXPECT_FLOAT_EQ(x, 1.5f);
    EXPECT_FLOAT_EQ(y, 2.0f);
    EXPECT_NEAR(std::sqrt(x * x + y * y), 2.5f, 1e-5f);
}

TEST(MathOps, LimitNormLeavesVectorWithinLimitUntouched) {
    float x = 3.0f;
    float y = 4.0f;
    limit_norm(&x, &y, 10.0f);
    EXPECT_FLOAT_EQ(x, 3.0f);
    EXPECT_FLOAT_EQ(y, 4.0f);

    // Norm exactly at the limit is not rescaled (strict comparison).
    limit_norm(&x, &y, 5.0f);
    EXPECT_FLOAT_EQ(x, 3.0f);
    EXPECT_FLOAT_EQ(y, 4.0f);
}

TEST(MathOps, LimitNormIsSafeOnZeroVector) {
    float x = 0.0f;
    float y = 0.0f;
    limit_norm(&x, &y, 1.0f);
    EXPECT_FLOAT_EQ(x, 0.0f);
    EXPECT_FLOAT_EQ(y, 0.0f);
}

TEST(MathOps, FloatToUintMapsRangeEndpoints) {
    EXPECT_EQ(float_to_uint(-1.0f, -1.0f, 1.0f, 12), 0);
    EXPECT_EQ(float_to_uint(1.0f, -1.0f, 1.0f, 12), (1 << 12) - 1);
    // Midpoint lands in the middle of the integer range (within truncation).
    EXPECT_NEAR(float_to_uint(0.0f, -1.0f, 1.0f, 12), (1 << 11), 1);
}

TEST(MathOps, FloatUintRoundTripStaysWithinQuantizationStep) {
    const float lo = -12.5f;
    const float hi = 12.5f;
    for (int bits : {8, 12, 16}) {
        const float step = (hi - lo) / static_cast<float>((1 << bits) - 1);
        for (float v = lo; v <= hi; v += (hi - lo) / 37.0f) {
            const int encoded = float_to_uint(v, lo, hi, bits);
            const float decoded = uint_to_float(encoded, lo, hi, bits);
            EXPECT_NEAR(decoded, v, step) << "bits=" << bits << " v=" << v;
        }
    }
}

// The declaration's parameter order is (val, max, min, bits): 0 maps to min and
// the full-scale integer maps to max. This is the inverse mapping convention the
// DM servo decoders rely on.
TEST(MathOps, IntToFloatMapsZeroToMinAndFullScaleToMax) {
    EXPECT_FLOAT_EQ(int_to_float(0, 10.0f, -10.0f, 8), -10.0f);
    EXPECT_FLOAT_EQ(int_to_float(255, 10.0f, -10.0f, 8), 10.0f);
    EXPECT_NEAR(int_to_float(127, 10.0f, -10.0f, 8), 0.0f, 0.1f);
}

TEST(MathOps, Float16RoundTripPreservesRepresentableValues) {
    // Powers of two and their sums are exactly representable in half precision.
    for (float v : {0.0f, 1.0f, -1.0f, 0.5f, -0.25f, 2.5f, 100.0f, -1024.0f}) {
        unsigned short h = 0;
        float back = -999.0f;
        float in = v;
        float32_to_float16(&in, &h);
        float16_to_float32(&h, &back);
        EXPECT_FLOAT_EQ(back, v) << "v=" << v;
    }
}

TEST(MathOps, Float16RoundTripApproximatesArbitraryValues) {
    // Half precision has a 10-bit mantissa => ~0.1% relative error.
    for (float v : {3.14159f, -2.71828f, 123.456f, -0.001234f}) {
        unsigned short h = 0;
        float back = 0.0f;
        float in = v;
        float32_to_float16(&in, &h);
        float16_to_float32(&h, &back);
        EXPECT_NEAR(back, v, std::fabs(v) * 2e-3f) << "v=" << v;
    }
}

TEST(MathOps, Float16HandlesHalfPrecisionMax) {
    float v = 65504.0f;  // Largest finite half-precision value.
    unsigned short h = 0;
    float back = 0.0f;
    float32_to_float16(&v, &h);
    float16_to_float32(&h, &back);
    EXPECT_FLOAT_EQ(back, 65504.0f);
}
