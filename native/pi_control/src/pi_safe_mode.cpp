/*!
 * @file pi_safe_mode.cpp
 * @brief Implementation of the SafeMode class for robot safety management and protective stops.
 */

#include "pi_safe_mode.hpp"
#include "pi_servo.hpp"

SafeMode::SafeMode(const CommandLineArgs& cla) {
    is_safe_mode_enabled_ = !cla.safety_feature_off;
    is_torque_mode_enabled_ = cla.safety_torque_mode;
}

SafeMode::~SafeMode() {}

ReturnCode SafeMode::graceful_management(Servo* p_servo, ReturnCode servo_return_code) {
    if (!is_safe_mode_enabled_) {
        return servo_return_code;
    }

    switch (servo_return_code) {
        case ReturnCode::SAFE_MODE_POS_EXCEED:
            return graceful_management_pos_exceed(p_servo);
        case ReturnCode::SAFE_MODE_POS_BEHIND:
            return graceful_management_pos_behind(p_servo);
        case ReturnCode::SAFE_MODE_VEL:
            return graceful_management_vel(p_servo);
        case ReturnCode::SAFE_MODE_TOR:
            return graceful_management_tor(p_servo);
        case ReturnCode::SAFE_MODE_SIG:
            return graceful_management_sig(p_servo);
        case ReturnCode::SAFE_MODE_TEMPERATURE:
            return graceful_management_temperature(p_servo);
        default:
            break;
    }
    return servo_return_code;
}

ReturnCode SafeMode::exit_safe_mode(Servo* p_servo, ReturnCode safe_mode_return_code) {
    (void)p_servo;

    if (!is_safe_mode_enabled_) {
        return ReturnCode::SUCCESS;
    }

    switch (safe_mode_return_code) {
        case ReturnCode::SAFE_MODE_POS_EXCEED:
            if (is_pos_exceed_) {
                PI_WARN("Safe mode exited: position limit exceeded");
                is_pos_exceed_ = false;
            }
            break;
        case ReturnCode::SAFE_MODE_POS_BEHIND:
            if (is_pos_behind_) {
                PI_WARN("Safe mode exited: position behind limit");
                is_pos_behind_ = false;
            }
            break;
        case ReturnCode::SAFE_MODE_VEL:
            if (is_vel_) {
                PI_WARN("Safe mode exited: abnormal velocity");
                is_vel_ = false;
            }
            break;
        case ReturnCode::SAFE_MODE_TOR:
            if (is_tor_) {
                PI_WARN("Safe mode exited: abnormal torque");
                is_tor_ = false;
            }
            break;
        case ReturnCode::SAFE_MODE_SIG:
            if (is_no_signal_) {
                PI_WARN("Safe mode exited: servo signal restored");
                is_no_signal_ = false;
            }
            break;
        case ReturnCode::SAFE_MODE_TEMPERATURE:
            if (is_temperature_) {
                PI_WARN("Safe mode exited: temperature limit restored");
                is_temperature_ = false;
            }
            break;
        default:
            break;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode SafeMode::update_servo_values(Servo* p_servo) {
    servo_pos_rad_ = p_servo->get_pos_rad_relative();
    servo_vel_ = p_servo->get_vel_rad_sec();
    servo_tor_ = p_servo->get_tor_nm();
    return ReturnCode::SUCCESS;
}

ReturnCode SafeMode::graceful_management_pos_exceed(Servo* p_servo) {
    is_pos_exceed_ = true;
    update_servo_values(p_servo);
    PI_WARN("Safe mode started: position limit exceeded (servo ID %d)", p_servo->id_);
    return ReturnCode::SUCCESS;
}

ReturnCode SafeMode::graceful_management_pos_behind(Servo* p_servo) {
    if (!is_pos_behind_) {
        // Latch transition: capture the hold anchor exactly once. While the
        // condition keeps holding, this handler runs every cycle -- updating
        // the anchor each time would make the hold track the live (sagging)
        // position, which is the drift this anchor exists to prevent.
        is_pos_behind_ = true;
        pos_behind_hold_pos_rad_ = p_servo->get_pos_rad_relative();
        PI_WARN("Safe mode started: position behind limit (servo ID %d), holding %.3f rad", p_servo->id_,
                pos_behind_hold_pos_rad_);
    }
    update_servo_values(p_servo);
    return ReturnCode::SUCCESS;
}

ReturnCode SafeMode::graceful_management_vel(Servo* p_servo) {
    is_vel_ = true;
    update_servo_values(p_servo);
    PI_WARN("Safe mode started: abnormal velocity detected (servo ID %d)", p_servo->id_);
    return ReturnCode::SAFE_MODE_VEL;
}

ReturnCode SafeMode::graceful_management_tor(Servo* p_servo) {
    is_tor_ = true;
    update_servo_values(p_servo);
    PI_WARN("Safe mode started: abnormal torque detected (servo ID %d)", p_servo->id_);

    if (p_servo->is_behind_more_than_tolerable_threshold()) {
        graceful_management(p_servo, ReturnCode::SAFE_MODE_POS_BEHIND);
    }
    return ReturnCode::SAFE_MODE_TOR;
}

ReturnCode SafeMode::graceful_management_sig(Servo* p_servo) {
    is_no_signal_ = true;
    update_servo_values(p_servo);
    PI_WARN("Safe mode started: servo signal loss detected (servo ID %d)", p_servo->id_);
    return ReturnCode::SAFE_MODE_SIG;
}

ReturnCode SafeMode::graceful_management_temperature(Servo* p_servo) {
    is_temperature_ = true;
    update_servo_values(p_servo);
    PI_WARN("Safe mode started: temperature limit exceeded (servo ID %d)", p_servo->id_);

    if (p_servo->is_behind_more_than_tolerable_threshold()) {
        graceful_management(p_servo, ReturnCode::SAFE_MODE_POS_BEHIND);
    }
    return ReturnCode::SAFE_MODE_TEMPERATURE;
}

float SafeMode::get_safe_tele_pos_rad(Servo* p_servo, float tele_pos) {
    if (!is_safe_mode_enabled_) {
        return tele_pos;
    }

    if (is_pos_exceed_) {
        if (tele_pos > p_servo->pos_max_rel_) {
            PI_WARN(
                "Safe mode: position limit exceeded (servo ID %d) - command=%.3f rad, current=%.3f rad, "
                "clamped to maximum=%.3f rad",
                p_servo->id_, tele_pos, p_servo->get_pos_rad_relative(), p_servo->pos_max_rel_);
            return p_servo->pos_max_rel_;
        }
        if (tele_pos < p_servo->pos_min_rel_) {
            PI_WARN(
                "Safe mode: position behind limit (servo ID %d) - command=%.3f rad, current=%.3f rad, "
                "clamped to minimum=%.3f rad",
                p_servo->id_, tele_pos, p_servo->get_pos_rad_relative(), p_servo->pos_min_rel_);
            return p_servo->pos_min_rel_;
        }
    }

    if (is_pos_behind_) {
        PI_WARN(
            "Safe mode: position behind limit (servo ID %d) - command=%.3f rad, current=%.3f rad, "
            "holding latch position %.3f rad",
            p_servo->id_, tele_pos, p_servo->get_pos_rad_relative(), pos_behind_hold_pos_rad_);
        return pos_behind_hold_pos_rad_;
    }

    return tele_pos;
}
