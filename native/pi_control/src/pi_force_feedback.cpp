/*!
 * @file pi_force_feedback.cpp
 * @brief Implementation of the ForceFeedback class for haptic force feedback in teleoperation systems.
 */

#include "pi_force_feedback.hpp"

#define FORCE_FEEDBACK_TORQUE_FILTER_BUFFER_SIZE 5

ForceFeedback::ForceFeedback()
    : follower_pos_filter_(FilterType::MOVING_AVERAGE, 0.2f, FORCE_FEEDBACK_TORQUE_FILTER_BUFFER_SIZE),
      follower_vel_filter_(FilterType::MOVING_AVERAGE, 0.2f, FORCE_FEEDBACK_TORQUE_FILTER_BUFFER_SIZE),
      follower_tor_filter_(FilterType::MOVING_AVERAGE, 0.2f, FORCE_FEEDBACK_TORQUE_FILTER_BUFFER_SIZE),
      stall_tor_filter_(FilterType::MOVING_AVERAGE, 0.2f, FORCE_FEEDBACK_TORQUE_FILTER_BUFFER_SIZE),
      force_feedback_torque_filter_(FilterType::MOVING_AVERAGE, 0.2f, FORCE_FEEDBACK_TORQUE_FILTER_BUFFER_SIZE) {}

ForceFeedback::~ForceFeedback() {}

ReturnCode ForceFeedback::init(const CommandLineArgs& cla) {
    cla_ = cla;

    return ReturnCode::SUCCESS;
}

#define SPRING_CONSTANT 1.0f       ///< Spring constant for virtual spring force calculation (Nm/unit)
#define STALL_TOR_GAIN 2.0f        ///< Gain factor for stall torque feedback component
#define FORCE_FEEDBACK_SCALE 0.3f  ///< Overall scaling factor for force feedback magnitude

float ForceFeedback::calc_force_feedback_torque(float leader_pos) {
    float force_feedback_torque = 0;

    if (leader_pos <= prev_leader_pos_ && stall_detect_.is_stall_detected() == true) {
        float follower_pos = follower_pos_filter_.get_latest_value();

        if (follower_pos >= leader_pos) {
            force_feedback_torque += -SPRING_CONSTANT * (leader_pos - follower_pos);

            force_feedback_torque += -STALL_TOR_GAIN * stall_tor_filter_.get_latest_value();

            force_feedback_torque *= FORCE_FEEDBACK_SCALE;
        }
    }

    prev_leader_pos_ = leader_pos;

    return force_feedback_torque;
}

ReturnCode ForceFeedback::update_follower_info(float leader_pos, float follower_pos, float follower_vel,
                                               float follower_tor, float follower_temperature,
                                               float follower_idc_current) {
    follower_pos_ = follower_pos;
    follower_vel_ = follower_vel;
    follower_tor_ = follower_tor;
    follower_temperature_ = follower_temperature;
    follower_idc_current_ = follower_idc_current;

    follower_pos_filter_.update(follower_pos_);
    follower_vel_filter_.update(follower_vel_);
    follower_tor_filter_.update(follower_tor_);

    float stall_tor = stall_detect_.detect_stall(follower_pos_, follower_vel_, follower_tor_);
    stall_tor_filter_.update(stall_tor);

    float force_feedback_torque = calc_force_feedback_torque(leader_pos);
    force_feedback_torque_filter_.update(force_feedback_torque);

    return ReturnCode::SUCCESS;
}