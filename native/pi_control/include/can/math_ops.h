/*!
 * @file math_ops.h
 * @brief Mathematical utility functions for CAN communication and data conversion.
 */

#pragma once
#include <math.h>

/*!
 * @brief Returns the maximum value among three floating-point numbers.
 * @param x First floating-point value.
 * @param y Second floating-point value.
 * @param z Third floating-point value.
 * @return The maximum value among x, y, and z.
 */
float fmaxf3(float x, float y, float z);

/*!
 * @brief Returns the minimum value among three floating-point numbers.
 * @param x First floating-point value.
 * @param y Second floating-point value.
 * @param z Third floating-point value.
 * @return The minimum value among x, y, and z.
 */
float fminf3(float x, float y, float z);

/*!
 * @brief Limits the Euclidean norm of a 2D vector to a specified maximum value.
 * @param x Pointer to the x-component of the vector (modified in-place).
 * @param y Pointer to the y-component of the vector (modified in-place).
 * @param limit Maximum allowed Euclidean norm of the vector.
 */
void limit_norm(float* x, float* y, float limit);

/*!
 * @brief Converts a floating-point value to an unsigned integer within a specified range.
 * @param x Floating-point value to convert (clamped to [x_min, x_max]).
 * @param x_min Minimum value of the input range.
 * @param x_max Maximum value of the input range.
 * @param bits Number of bits for the output integer (determines maximum value: 2^bits - 1).
 * @return Unsigned integer representation of x in the range [0, 2^bits - 1].
 */
int float_to_uint(float x, float x_min, float x_max, int bits);

/*!
 * @brief Converts an unsigned integer to a floating-point value within a specified range.
 * @param x_int Unsigned integer value to convert (in range [0, 2^bits - 1]).
 * @param x_min Minimum value of the output range.
 * @param x_max Maximum value of the output range.
 * @param bits Number of bits used for the input integer (determines maximum value: 2^bits - 1).
 * @return Floating-point representation of x_int in the range [x_min, x_max].
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits);

/*!
 * @brief Converts a signed integer to a floating-point value within a specified range.
 * @param val Signed integer value to convert.
 * @param max_ Maximum value of the output range.
 * @param min_ Minimum value of the output range.
 * @param bits Number of bits used for the input integer (determines integer range).
 * @return Floating-point representation of val in the range [min_, max_].
 */
float int_to_float(int val, float max_, float min_, int bits);

/*!
 * @brief Converts a 32-bit IEEE 754 floating-point value to 16-bit half-precision format.
 * @param float32 Pointer to the 32-bit floating-point value to convert (input).
 * @param float16 Pointer to the 16-bit half-precision value (output).
 */
void float32_to_float16(float* float32, unsigned short int* float16);

/*!
 * @brief Converts a 16-bit half-precision floating-point value to 32-bit IEEE 754 format.
 * @param float16 Pointer to the 16-bit half-precision value (input).
 * @param float32 Pointer to the 32-bit floating-point value (output).
 */
void float16_to_float32(unsigned short int* float16, float* float32);

