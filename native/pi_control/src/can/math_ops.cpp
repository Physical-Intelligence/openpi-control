/*!
 * @file math_ops.cpp
 * @brief Implementation of mathematical utility functions for CAN communication.
 */

#include <cmath>
#include <cstdint>
#include <cstring>

#include "can/math_ops.h"

float fmaxf3(float x, float y, float z) {
    float max_val = x;

    if (y > max_val) {
        max_val = y;
    }

    if (z > max_val) {
        max_val = z;
    }

    return max_val;
}

float fminf3(float x, float y, float z) {
    float min_val = x;

    if (y < min_val) {
        min_val = y;
    }

    if (z < min_val) {
        min_val = z;
    }

    return min_val;
}

int float_to_uint(float x, float x_min, float x_max, int bits) {
    int max_int = (1 << bits) - 1;

    return (int)((max_int * (x - x_min)) / (x_max - x_min));
}

void float32_to_float16(float* float32, unsigned short* float16) {
    uint32_t f32 = *((uint32_t*)float32);
    uint16_t sign = (f32 >> 16) & 0x8000;
    int16_t exp = ((f32 >> 23) & 0xFF) - 112;
    uint16_t mantissa = (f32 >> 13) & 0x3FF;

    if (exp <= 0) {
        if (exp < -10) {
            *float16 = sign;
            return;
        }
        mantissa = (mantissa | 0x400) >> (1 - exp);
        exp = 0;
    } else if (exp >= 31) {
        if (exp == 255 && mantissa != 0) {
            mantissa = 0x200;
        } else {
            mantissa = 0;
        }
        exp = 31;
    }

    *float16 = sign | (exp << 10) | mantissa;
}

void float16_to_float32(uint16_t* float16, float* float32) {
    uint16_t f16 = *float16;

    uint32_t sign = (f16 & 0x8000) << 16;
    uint32_t exp = (f16 & 0x7C00) >> 10;
    uint32_t mantissa = f16 & 0x03FF;

    if (exp == 0) {
        if (mantissa == 0) {
            uint32_t result = sign;
            memcpy(float32, &result, sizeof(float));

            return;
        } else {
            exp = 1;
            while ((mantissa & 0x0400) == 0) {
                mantissa <<= 1;
                exp -= 1;
            }
            mantissa &= 0x03FF;
            exp += 127 - 15 + 1;
        }
    } else if (exp == 0x1F) {
        exp = 255;
        if (mantissa != 0) {
            mantissa = (mantissa << 13) | 0x7FFFFF;
        } else {
            mantissa = 0;
        }
    } else {
        exp += 127 - 15;
        mantissa <<= 13;
    }

    uint32_t result = sign | (exp << 23) | mantissa;
    memcpy(float32, &result, sizeof(float));
}

void limit_norm(float* x, float* y, float limit) {
    float norm = std::sqrt((*x) * (*x) + (*y) * (*y));

    if (norm > limit) {
        *x = (*x * limit) / norm;
        *y = (*y * limit) / norm;
    }
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float max_int = static_cast<float>((1 << bits) - 1);

    return (static_cast<float>(x_int) * (x_max - x_min) / max_int) + x_min;
}

float int_to_float(int val, float min_, float max_, int bits) {
    float scaled = static_cast<float>(val) / ((1 << bits) - 1);

    return (min_ - max_) * scaled + max_;
}
