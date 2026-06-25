/*!
 * @file pi_stall_detect.cpp
 * @brief Implementation of the StallDetect class for servo motor stall condition detection.
 */

#include <cmath>
#include <numeric>

#include "pi_stall_detect.hpp"

#define POS_THRESHOLD 0.1        ///< Threshold for position change detection (radians)
#define VEL_THRESHOLD 0.1        ///< Threshold for velocity detection (rad/s)
#define TOR_THRESHOLD 0.1        ///< Threshold for torque detection (Nm)
#define STALL_COUNT_THRESHOLD 3  ///< Number of consecutive control cycles required to confirm stall

StallDetect::StallDetect() : stall_tor_buffer_(STALL_COUNT_THRESHOLD) {}

StallDetect::~StallDetect() {}

ReturnCode StallDetect::init(const CommandLineArgs& cla) {
    cla_ = cla;
    return ReturnCode::SUCCESS;
}

float StallDetect::detect_stall(float pos, float vel, float tor) {
    if (fabs(vel) < VEL_THRESHOLD && fabs(pos - pos_prev_) < POS_THRESHOLD && fabs(tor) > TOR_THRESHOLD) {
        stall_count_++;
        stall_tor_buffer_.push_back(tor);

        if (stall_count_ >= STALL_COUNT_THRESHOLD) {
            is_stall_detected_ = true;
            float sum = std::accumulate(stall_tor_buffer_.begin(), stall_tor_buffer_.end(), 0.0f);
            stall_tor_ = sum / stall_tor_buffer_.size();
            stall_count_ = 0;
        }
    } else {
        stall_count_ = 0;
        is_stall_detected_ = false;
        stall_tor_buffer_.clear();
        stall_tor_ = 0.0f;
    }

    pos_prev_ = pos;

    return stall_tor_;
}
