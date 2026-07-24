/*!
 * @file pi_servo_ft.cpp
 * @brief Implementation of the ServoFt class for FeeTech SMS/STS serial bus servos.
 */

#include <unistd.h>
#include <cmath>

#include "pi_driver_ft.hpp"
#include "pi_joint.hpp"
#include "pi_servo_ft.hpp"

// SMS/STS Operating Mode register values (addr 33).
#define FT_MODE_POSITION 0

// Torque Limit register full scale (0.1% of stall torque).
#define FT_TORQUE_LIMIT_FULL_SCALE 1000

// Acceleration register bounds (addr 41, unit 100 steps/s^2).
#define FT_ACCELERATION_MAX 254

// Verified torque disable: attempts and delay between attempts. The HX
// firmware on SO-ARM101 kits ACKs but ignores a torque-off that arrives right
// after a torque-on / goal write, so the disable is verified via readback and
// retried until the register actually reads 0.
#define FT_TORQUE_DISABLE_MAX_ATTEMPTS 5
#define FT_TORQUE_DISABLE_RETRY_DELAY_US 10000

const ServoFtParam g_servo_ft_param(DEFAULT_TOLERABLE_POS_DIFFERENCE_RAD, MAX_POS_DIFFERENCE_RAD,
                                    DEFAULT_VELOCITY_THRESHOLD_RAD_SEC);

ServoFt::ServoFt(Device* p_device, Joint* p_joint, Driver* p_driver) : Servo(p_device, p_joint, p_driver) {
    p_driver_ft_ = (DriverFt*)p_driver;
}

ServoFt::~ServoFt() {}

ReturnCode ServoFt::init_config_model(const json& servo_config, const DeviceConfig* p_config) {
    p_servo_param_ = &g_servo_ft_param;

    ReturnCode return_code = Servo::init_config_model(servo_config, p_config);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (servo_model_ == p_config->val_servo_model_ft_sts3215) {
        type_ = ServoType::FT_STS3215;
    } else {
        PI_ERROR("Unsupported servo model '%s' (servo ID %d)", servo_model_.c_str(), id_);
        return ReturnCode::NOT_SUPPORTED;
    }

    return_code = p_config->get_field_value(servo_config, p_config->fn_servo_resolution, servo_resolution_);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    PI_INFO("ServoFt", InfoLevel::HELPFUL_1, "Servo ID %d: servo_resolution=%d", id_, servo_resolution_);
    // STS3215 operates single-turn with the calibrated middle at
    // resolution / 2 (step 2048), which maps to 0 rad.
    center_offset_ = servo_resolution_ / 2;

    // Optional: Acceleration register value (unit 100 steps/s^2). Reuses the
    // generic prof_accel field; 0 (default) leaves the servo's own ramp off.
    return_code = p_config->get_field_value_optional(servo_config, p_config->fn_servo_prof_accel, prof_accel_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("ServoFt", InfoLevel::HELPFUL_1, "Servo ID %d: prof_accel=%d", id_, prof_accel_);
        if (prof_accel_ < 0 || prof_accel_ > FT_ACCELERATION_MAX) {
            PI_ERROR("Servo ID %d: prof_accel must be within 0..%d for FeeTech servos, but found %d", id_,
                     FT_ACCELERATION_MAX, prof_accel_);
            return ReturnCode::INVALID_PARAM;
        }
    }

    return ReturnCode::SUCCESS;
}

ReturnCode ServoFt::start_hardware() {
    if (p_driver_ft_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    if (p_device_ == nullptr) {
        PI_ERROR("Device pointer is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    if (p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: device is read only, skipping start hardware", id_);
        return ReturnCode::SUCCESS;
    }

    ReturnCode return_code = p_driver_ft_->enable_torque(this, false);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to disable torque (servo ID %d)", id_);
        return return_code;
    }
    usleep(200);

    return_code = p_driver_ft_->set_operating_mode(this, FT_MODE_POSITION);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to set position operating mode (servo ID %d)", id_);
        return return_code;
    }
    usleep(200);

    // Position PID gains are written only when a P gain is configured; the
    // factory defaults (P=32, D=32, I=0) are otherwise kept, matching the
    // LeRobot SO-ARM bring-up which does not touch these registers.
    if (pos_kp_ > 0) {
        return_code = p_driver_ft_->set_position_pid(this, pos_kp_, pos_ki_, pos_kd_);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to set position PID gain (servo ID %d)", id_);
            return return_code;
        }
        usleep(200);
    }

    if (prof_accel_ > 0) {
        return_code = p_driver_ft_->set_acceleration(this, static_cast<uint8_t>(prof_accel_));
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to set acceleration (servo ID %d)", id_);
            return return_code;
        }
        usleep(200);
    }

    // Align the goal with the present position BEFORE engaging torque so the
    // servo holds its pose instead of jumping to a stale goal register (same
    // rationale as ServoDxl::start_hardware()).
    return_code = p_driver_ft_->sync_goal_position_to_present(this);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to sync goal position to present position before enabling torque (servo ID %d)", id_);
        return return_code;
    }
    usleep(200);

    return_code = p_driver_ft_->enable_torque(this, true);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to enable torque (servo ID %d)", id_);
        return return_code;
    }
    usleep(200);

    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo %d started hardware", id_);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoFt::park_safely() {
    if (parked_ == true) {
        return ReturnCode::SUCCESS;
    }

    if (p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: device is read only, skipping park safely", id_);
        return ReturnCode::SUCCESS;
    }

    if (p_driver_ft_ == nullptr) {
        PI_ERROR("FeeTech driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    ReturnCode return_code = read_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to read current hardware values (servo ID %d)", id_);
        return return_code;
    }
    usleep(200);

    // Keep the goal register aligned with the resting pose so the next
    // torque-on (e.g. a restart without a full re-init) does not jump. This
    // must happen BEFORE the torque disable: a goal position write arriving
    // after the disable re-engages torque on SO-ARM101 kits (Hiwonder HX
    // firmware) and the arm stays locked after shutdown.
    return_code = p_driver_ft_->set_goal_position_direct(this, curr_pos_steps_);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to set goal position to current position (servo ID %d)", id_);
        return return_code;
    }
    usleep(200);

    return_code = p_driver_ft_->enable_torque(this, false);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to disable torque (servo ID %d)", id_);
        return return_code;
    }
    usleep(200);

    parked_ = true;

    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d parked safely", id_);

    return return_code;
}

ReturnCode ServoFt::move(float target_pos) {
    if (p_driver_ft_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    float clipped_pos = clipping(target_pos, pos_min_rel_, pos_max_rel_);
    float target_pos_absolute = get_pos_rad_absolute(clipped_pos);
    int32_t target_pos_steps = rad_to_steps(target_pos_absolute);

    ReturnCode return_code = p_driver_ft_->set_goal_position_group(this, target_pos_steps);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to set goal position (servo ID %d)", id_);
        return return_code;
    }

    PI_INFO("Servo", InfoLevel::FREQUENT_3, "Joint ID %d, Servo ID %d: move target_pos=%.3f rad, target_steps=%d",
            p_joint_->id_, id_, target_pos, target_pos_steps);

    return return_code;
}

ReturnCode ServoFt::move(float target_pos, float target_vel, float target_tor) {
    // target_vel is part of the Servo interface but the FeeTech position mode
    // does not consume an explicit goal velocity here.
    (void)target_vel;
    if (p_driver_ft_ == nullptr || p_joint_ == nullptr || p_device_ == nullptr) {
        PI_ERROR("Required pointers are not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    float clipped_pos = clipping(target_pos, pos_min_rel_, pos_max_rel_);
    float target_pos_absolute = get_pos_rad_absolute(clipped_pos);
    int32_t target_pos_steps = rad_to_steps(target_pos_absolute);

    float rescaled_torque = target_tor * p_joint_->torq_rescale_;
    float clipped_torque = clipping(rescaled_torque, p_joint_->torq_min_, p_joint_->torq_max_);
    int32_t torque_limit = torque_to_torque_limit(clipped_torque);

    PI_INFO("ServoFt", InfoLevel::FREQUENT_3,
            "Joint ID %d, Servo ID %d: move target_pos=%.3f rad, target_steps=%d, requested_torque=%.3f Nm, "
            "torque_limit=%d",
            p_joint_->id_, id_, target_pos, target_pos_steps, target_tor, torque_limit);

    ReturnCode return_code = p_driver_ft_->set_torque_limit_group(this, torque_limit);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to set torque limit (servo ID %d)", id_);
        return return_code;
    }

    return_code = p_driver_ft_->set_goal_position_group(this, target_pos_steps);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to set goal position (servo ID %d)", id_);
        return return_code;
    }

    return return_code;
}

ReturnCode ServoFt::apply_torque(float torque) {
    (void)torque;
    PI_ERROR(
        "Servo ID %d: apply_torque is not supported for %s (no current control loop); "
        "use position control with a torque limit instead",
        id_, servo_model_.c_str());
    return ReturnCode::NOT_SUPPORTED;
}

ReturnCode ServoFt::enable_torque(bool enable) {
    if (p_driver_ft_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    if (p_device_ != nullptr && p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: device is read only, skipping enable_torque", id_);
        return ReturnCode::SUCCESS;
    }
    return p_driver_ft_->enable_torque(this, enable);
}

int32_t ServoFt::torque_to_torque_limit(float torque) {
    if (kt_ <= 0) {
        PI_ERROR("Servo ID %d: kt is not configured; cannot convert torque to a torque limit", id_);
        return 0;
    }

    // The Torque Limit register is an unsigned magnitude (the servo applies it
    // in both directions), so only the requested torque magnitude matters.
    int32_t torque_limit = (int32_t)(fabs(torque) / kt_);
    if (torque_limit > FT_TORQUE_LIMIT_FULL_SCALE) {
        torque_limit = FT_TORQUE_LIMIT_FULL_SCALE;
    }
    return torque_limit;
}

ReturnCode ServoFt::change_control_mode_for_leader() {
    if (p_driver_ft_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    if (p_device_ != nullptr && p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                "Servo ID %d: device is read only, skipping change_control_mode_for_leader", id_);
        return ReturnCode::SUCCESS;
    }

    if (p_device_ != nullptr && p_device_->is_force_feedback_enabled()) {
        PI_ERROR(
            "Servo ID %d: force feedback is not supported for %s (no current control loop); "
            "run the leader without --force_feedback",
            id_, servo_model_.c_str());
        return ReturnCode::NOT_SUPPORTED;
    }

    // SO-ARM leader convention (same as LeRobot): fully passive, torque off so
    // the operator back-drives the arm. The lightweight leader hardware does
    // not need gravity compensation.
    //
    // Same-cycle goal entries queued by the final move-to-ready step must be
    // dropped BEFORE disabling torque: flushing a goal position after the
    // disable re-engages torque on SO-ARM101 kits (Hiwonder HX firmware) and
    // the leader ends up locked at home.
    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: leader mode, disabling torque (passive)", id_);
    p_driver_ft_->discard_pending_group_writes(this);

    // Disable-and-verify loop: the write is ACKed even when the firmware ignores
    // it (observed when the disable lands right after a torque-on / goal write on
    // the leader gripper), so read Torque Enable back and retry until it is 0.
    int32_t torque_enabled = -1;
    for (int attempt = 1; attempt <= FT_TORQUE_DISABLE_MAX_ATTEMPTS; attempt++) {
        ReturnCode return_code = p_driver_ft_->enable_torque(this, false);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to disable torque for passive leader (servo ID %d)", id_);
            return return_code;
        }
        usleep(200);

        return_code = p_driver_ft_->get_torque_enabled(this, torque_enabled);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to read back Torque Enable after passive-leader disable (servo ID %d)", id_);
            return return_code;
        }
        if (torque_enabled == 0) {
            return ReturnCode::SUCCESS;
        }

        PI_WARN("Servo ID %d: Torque Enable reads %d after passive-leader disable (attempt %d/%d), retrying", id_,
                torque_enabled, attempt, FT_TORQUE_DISABLE_MAX_ATTEMPTS);
        usleep(FT_TORQUE_DISABLE_RETRY_DELAY_US);
    }

    PI_ERROR("Servo ID %d: Torque Enable still reads %d after %d passive-leader disable attempts (expected 0)", id_,
             torque_enabled, FT_TORQUE_DISABLE_MAX_ATTEMPTS);
    return ReturnCode::FAIL;
}

ReturnCode ServoFt::change_control_mode_for_follower() {
    if (p_driver_ft_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    if (p_device_ != nullptr && p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                "Servo ID %d: device is read only, skipping change_control_mode_for_follower", id_);
        return ReturnCode::SUCCESS;
    }

    ReturnCode return_code = p_driver_ft_->enable_torque(this, false);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to disable torque (servo ID %d)", id_);
        return return_code;
    }
    usleep(10000);

    return_code = p_driver_ft_->set_operating_mode(this, FT_MODE_POSITION);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to set position operating mode (servo ID %d)", id_);
        return return_code;
    }

    // Align the goal with the present position before re-enabling torque to
    // avoid a stale-goal jump after the mode switch.
    return_code = p_driver_ft_->sync_goal_position_to_present(this);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to sync goal position to present (servo ID %d)", id_);
        return return_code;
    }

    return_code = p_driver_ft_->enable_torque(this, true);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to enable torque (servo ID %d)", id_);
        return return_code;
    }

    return return_code;
}
