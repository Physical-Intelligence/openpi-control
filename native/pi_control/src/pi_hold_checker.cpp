/*!
 * @file pi_hold_checker.cpp
 * @brief Implementation of the HoldChecker class for hysteresis-based state detection.
 */

#include "pi_hold_checker.hpp"

#define EXIT_COUNT_THRESHOLD 100

HoldChecker::HoldChecker(int hold_count_threshold)
    : hold_count_(0), exit_count_(0), holding_status_(false), hold_count_threshold_(hold_count_threshold) {}

HoldChecker::~HoldChecker() {}

bool HoldChecker::is_holding(bool condition) {
    if (!holding_status_) {
        if (condition) {
            hold_count_++;
        } else {
            hold_count_ = 0;
        }

        if (hold_count_ >= hold_count_threshold_) {
            holding_status_ = true;
        }
    } else {
        if (condition) {
            exit_count_ = 0;
        } else {
            exit_count_++;
        }

        if (exit_count_ >= EXIT_COUNT_THRESHOLD) {
            holding_status_ = false;
        }
    }

    return holding_status_;
}

void HoldChecker::reset() {
    hold_count_ = 0;
    exit_count_ = 0;
    holding_status_ = false;
}

void HoldChecker::set_hold_count_threshold(int hold_count_threshold) {
    hold_count_threshold_ = hold_count_threshold;
    reset();
}