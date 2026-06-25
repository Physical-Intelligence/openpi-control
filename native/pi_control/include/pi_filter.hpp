/*!
 * @file pi_filter.hpp
 * @brief Filter class and filter types for signal filtering.
 */

#pragma once
#include <memory>

#include "pi_info.hpp"

/*!
 * @brief Types of signal filters.
 */
enum class FilterType {
    EMA,           ///< Exponential Moving Average filter.
    MEDIAN,        ///< Median filter.
    MOVING_AVERAGE, ///< Moving average filter.
    KALMAN         ///< Kalman filter.
};

/*!
 * @brief Abstract base class for all filter implementations.
 */
class FilterBase {
   public:
    /*!
     * @brief Destructor.
     */
    virtual ~FilterBase() = default;

    /*!
     * @brief Updates the filter with a new input value and returns the filtered output.
     * @param new_input New input value to filter.
     * @return Filtered output value.
     */
    virtual float update(float new_input) = 0;
};

/*!
 * @brief Wrapper class for implementing various signal filters with a unified interface.
 */
class Filter {
   public:
    /*!
     * @brief Constructor.
     * @param type Filter type to create.
     * @param ema_alpha Alpha parameter for EMA filter (0.0-1.0, default: 0.2).
     * @param window_size Window size for median and moving average filters (default: 5).
     * @param kalman_q Process noise covariance for Kalman filter (default: 0.01).
     * @param kalman_r Measurement noise covariance for Kalman filter (default: 1.0).
     */
    Filter(FilterType type, float ema_alpha = 0.2f, size_t window_size = 5, float kalman_q = 0.01f,
           float kalman_r = 1.0f);

    /*!
     * @brief Move constructor.
     * @param other Filter instance to move from.
     */
    Filter(Filter&& other) noexcept = default;

    /*!
     * @brief Updates the filter with a new input value and returns the filtered output.
     * @param new_input New input value to filter.
     * @return Filtered output value.
     */
    float update(float new_input) {
        if (p_filter_ == nullptr) {
            PI_ERROR("Filter: p_filter_ is nullptr in update()");
            return latest_value_;
        }
        latest_value_ = p_filter_->update(new_input);
        return latest_value_;
    }

    /*!
     * @brief Gets the latest filtered value without updating the filter.
     * @return The latest filtered value.
     */
    float get_latest_value() const { return latest_value_; }

    /*!
     * @brief Move assignment operator.
     * @param other Filter instance to move from.
     * @return Reference to this Filter instance.
     */
    Filter& operator=(Filter&& other) noexcept = default;

    // Copy constructor is deleted to prevent accidental filter duplication.
    Filter(const Filter&) = delete;

    // Copy assignment operator is deleted to prevent accidental filter duplication.
    Filter& operator=(const Filter&) = delete;

    /*!
     * @brief Destructor.
     */
    virtual ~Filter() = default;

   private:
    FilterType type_;  ///< Type of filter.
    std::unique_ptr<FilterBase> p_filter_;  ///< Pointer to the actual filter implementation.
    float latest_value_ = 0.0f;  ///< Cached latest filtered value.
};

