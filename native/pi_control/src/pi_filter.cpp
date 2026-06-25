/*!
 * @file pi_filter.cpp
 * @brief Implementation of various signal filtering algorithms for robot control applications.
 */

 #include <algorithm>
#include <boost/circular_buffer.hpp>
#include <vector>

#include "pi_filter.hpp"

/*!
 * @class FilterEMA
 * @brief Exponential Moving Average (EMA) filter implementation.
 *
 * Applies exponential smoothing: EMA = alpha * new_input + (1 - alpha) * previous_EMA.
 * Higher alpha values make the filter more responsive to recent changes.
 */
class FilterEMA : public FilterBase {
   public:
    explicit FilterEMA(float alpha) : alpha_(alpha), initialized_(false), ema_(0.0f) {}

    float update(float new_input) override {
        if (!initialized_) {
            ema_ = new_input;
            initialized_ = true;
        } else {
            ema_ = alpha_ * new_input + (1.0f - alpha_) * ema_;
        }
        return ema_;
    }

   private:
    float alpha_;       ///< Smoothing factor (0.0-1.0) determining filter responsiveness
    bool initialized_;  ///< Flag indicating whether the filter has been initialized
    float ema_;         ///< Current exponential moving average value
};

/*!
 * @class FilterMedian
 * @brief Median filter implementation for noise reduction.
 *
 * Computes median from a sliding window of recent inputs, removing outliers and impulse noise.
 * Less sensitive to extreme values than the mean.
 */
class FilterMedian : public FilterBase {
   public:
    explicit FilterMedian(size_t window_size) : buffer_(window_size) {}

    float update(float new_input) override {
        buffer_.push_back(new_input);

        std::vector<float> sorted_values(buffer_.begin(), buffer_.end());
        std::sort(sorted_values.begin(), sorted_values.end());

        size_t median_index = buffer_.size() / 2;

        if (sorted_values.size() % 2 == 0) {
            return (sorted_values[median_index - 1] + sorted_values[median_index]) / 2.0f;
        } else {
            return sorted_values[median_index];
        }
    }

   private:
    boost::circular_buffer<float> buffer_;  ///< Circular buffer storing recent input values
};

/*!
 * @class FilterMovingAverage
 * @brief Simple moving average filter implementation.
 *
 * Computes arithmetic mean of values in a sliding window. Maintains a running sum for efficient computation.
 */
class FilterMovingAverage : public FilterBase {
   public:
    explicit FilterMovingAverage(size_t window_size) : buffer_(window_size), sum_(0.0f) {}

    float update(float new_input) override {
        if (buffer_.full()) {
            sum_ -= buffer_.front();
        }

        buffer_.push_back(new_input);
        sum_ += new_input;

        return sum_ / buffer_.size();
    }

   private:
    boost::circular_buffer<float> buffer_;  ///< Circular buffer storing recent input values
    float sum_ = 0.0f;                      ///< Running sum of all values in buffer
};

/*!
 * @class FilterKalman
 * @brief Simplified one-dimensional Kalman filter implementation.
 *
 * Uses prediction-update cycle: prediction increases uncertainty, update combines prediction with measurement.
 * Higher q values make filter more responsive to measurements, higher r values trust model more.
 */
class FilterKalman : public FilterBase {
   public:
    explicit FilterKalman(float q = 0.01f, float r = 1.0f) : q_(q), r_(r), x_(0.0f), p_(1.0f), initialized_(false) {}

    float update(float new_input) override {
        if (!initialized_) {
            x_ = new_input;
            initialized_ = true;
        }

        p_ += q_;

        float k = p_ / (p_ + r_);

        x_ = x_ + k * (new_input - x_);
        p_ = (1 - k) * p_;

        return x_;
    }

   private:
    float q_;           ///< Process noise covariance (model uncertainty)
    float r_;           ///< Measurement noise covariance (sensor uncertainty)
    float x_;           ///< Current state estimate (filtered output)
    float p_;           ///< Error covariance (uncertainty in state estimate)
    bool initialized_;  ///< Flag indicating whether filter has been initialized
};

Filter::Filter(FilterType type, float ema_alpha, size_t window_size, float kalman_q, float kalman_r) : type_(type) {
    if ((type == FilterType::MEDIAN || type == FilterType::MOVING_AVERAGE) && window_size == 0) {
        throw std::invalid_argument("Window size must be greater than zero for median and moving-average filters");
    }

    switch (type) {
        case FilterType::EMA:
            p_filter_ = std::make_unique<FilterEMA>(ema_alpha);
            break;
        case FilterType::MEDIAN:
            p_filter_ = std::make_unique<FilterMedian>(window_size);
            break;
        case FilterType::MOVING_AVERAGE:
            p_filter_ = std::make_unique<FilterMovingAverage>(window_size);
            break;
        case FilterType::KALMAN:
            p_filter_ = std::make_unique<FilterKalman>(kalman_q, kalman_r);
            break;
        default:
            throw std::invalid_argument(
                "Invalid filter type specified (must be EMA, MEDIAN, MOVING_AVERAGE, or KALMAN)");
    }
}
