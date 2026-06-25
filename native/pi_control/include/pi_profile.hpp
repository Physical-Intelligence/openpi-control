/*!
 * @file pi_profile.hpp
 * @brief Monotonic timing helpers used by the control loop.
 */

#pragma once

#include <chrono>

using prof_clock_t = std::chrono::steady_clock;
using prof_time_t = prof_clock_t::time_point;
using prof_time_msec_t = long;

class Profile {
   public:
    static prof_time_t get_time_now() { return prof_clock_t::now(); }

    static prof_time_msec_t get_time_diff(const prof_time_t& start,
                                          const prof_time_t& end) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(end - start)
            .count();
    }

    static bool is_zero(const prof_time_t& value) {
        return value == prof_time_t{};
    }
};
