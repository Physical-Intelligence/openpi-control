/*!
 * @file pi_servo.cpp
 * @brief Implementation of the Servo base class and factory methods for servo
 * creation.
 */

#include "pi_servo.hpp"

#include <cmath>

#include "pi_joint.hpp"
#include "pi_servo_can_encoder.hpp"
#include "pi_servo_dm.hpp"

#define MAX_CNT_POS_EXCEED \
    100  ///< Threshold count for position limit exceed detection
#define MAX_CNT_VEL_EXCEED \
    100  ///< Threshold count for velocity limit exceed detection
#define MAX_CNT_TOR_EXCEED \
    300  ///< Threshold count for torque limit exceed detection
#define MAX_CNT_POS_DIFFERENCE_EXCEED \
    1000  ///< Threshold count for position difference exceed detection
          ///< (leader-follower)
#define MAX_CNT_STALL_DETECTION \
    2000  ///< Threshold count for stall condition detection
#define MAX_CNT_TEMPERATURE_EXCEED \
    100  ///< Threshold count for temperature limit exceed detection
#define MAX_CNT_NORMAL_OPERATION \
    1000  ///< Threshold count for normal operation confirmation

Servo::Servo(Device* p_device, Joint* p_joint, Driver* p_driver)
    : checker_pos_exceed_(MAX_CNT_POS_EXCEED),
      checker_vel_exceed_(MAX_CNT_VEL_EXCEED),
      checker_tor_exceed_(MAX_CNT_TOR_EXCEED),
      checker_pos_difference_exceed_(MAX_CNT_POS_DIFFERENCE_EXCEED),
      checker_temperature_exceed_(MAX_CNT_TEMPERATURE_EXCEED),
      checker_stall_detection_(MAX_CNT_STALL_DETECTION),
      current_estimation_(),
      safe_mode_(p_device->get_cla()) {
    p_device_ = p_device;
    p_joint_ = p_joint;
    p_driver_ = p_driver;
}

Servo::~Servo() {
    Driver::unregister_servo(this);
}

ReturnCode Servo::init_config_model(const json& servo_config,
                                    const DeviceConfig* p_config) {
    ReturnCode return_code;

    return_code =
        p_config->get_field_value(servo_config, p_config->fn_servo_id, id_);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID: %d", id_);

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_data_index, data_index_);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: data_index=%d", id_,
            data_index_);

    p_driver_->register_servo_data_index(id_, data_index_, this);

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_model, servo_model_);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: servo_model=%s", id_,
            servo_model_.c_str());

    return_code = p_config->get_field_value(servo_config,
                                            p_config->fn_servo_pos_kp, pos_kp_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: pos_kp=%.3f", id_,
                pos_kp_);
    }

    return_code = p_config->get_field_value(servo_config,
                                            p_config->fn_servo_pos_ki, pos_ki_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: pos_ki=%.3f", id_,
                pos_ki_);
    }

    return_code = p_config->get_field_value(servo_config,
                                            p_config->fn_servo_pos_kd, pos_kd_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: pos_kd=%.3f", id_,
                pos_kd_);
    }

    return_code =
        p_config->get_field_value(servo_config, p_config->fn_servo_kt, kt_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: kt=%.3f", id_,
                kt_);
    }

    return_code =
        p_config->get_field_value(servo_config, p_config->fn_servo_kv, kv_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: kv=%.3f", id_,
                kv_);
    }

    return_code =
        p_config->get_field_value(servo_config, p_config->fn_servo_ka, ka_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: ka=%.3f", id_,
                ka_);
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_response_delay, response_delay_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1,
                "Servo ID %d: response_delay=%.3f seconds", id_,
                response_delay_);
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_dir_invert, dir_invert_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: dir_invert=%d (from model config)",
                id_, dir_invert_);
        if (dir_invert_ != 1 && dir_invert_ != -1) {
            PI_ERROR(
                "Servo ID %d: dir_invert must be 1 or -1, but found %d in "
                "model configuration file",
                id_, dir_invert_);
            return ReturnCode::INVALID_PARAM;
        }
    }

    // Config format 1.1.1 places the position limits in the model configuration
    // servo block; the individual configuration may still override them below.
    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_pos_min, pos_min_rel_);
    if (return_code == ReturnCode::SUCCESS) {
        pos_min_configured_ = true;
        PI_INFO("Servo", InfoLevel::HELPFUL_1,
                "Servo ID %d: pos_min=%.3f (from model config)", id_,
                pos_min_rel_);
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_pos_max, pos_max_rel_);
    if (return_code == ReturnCode::SUCCESS) {
        pos_max_configured_ = true;
        PI_INFO("Servo", InfoLevel::HELPFUL_1,
                "Servo ID %d: pos_max=%.3f (from model config)", id_,
                pos_max_rel_);

        if (pos_min_configured_ && pos_max_rel_ < pos_min_rel_) {
            PI_ERROR(
                "Servo ID %d: pos_max (%.3f) is smaller than pos_min (%.3f) "
                "in model configuration file",
                id_, pos_max_rel_, pos_min_rel_);
            return ReturnCode::INVALID_PARAM;
        }
    }

    return_code = init_current_estimation(servo_model_, p_config);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to initialize current estimation (servo ID %d)", id_);
        return return_code;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Servo::init_config_individual(const json& servo_config,
                                         const DeviceConfig* p_config) {
    ReturnCode return_code;
    position_wrap_period_ = 0;
    position_wrap_offset_rel_ = 0;
    position_wrap_initialized_ = false;

    int dir_invert_individual = 0;
    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_dir_invert, dir_invert_individual);
    if (return_code == ReturnCode::SUCCESS) {
        if (dir_invert_individual != 1 && dir_invert_individual != -1) {
            PI_ERROR(
                "Servo ID %d: dir_invert must be 1 or -1, but found %d in "
                "individual configuration file",
                id_, dir_invert_individual);
            return ReturnCode::INVALID_PARAM;
        }
        if (dir_invert_individual != dir_invert_) {
            PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                    "Servo ID %d: dir_invert overridden by individual config: %d -> %d",
                    id_, dir_invert_, dir_invert_individual);
        }
        dir_invert_ = dir_invert_individual;
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_pos_min, pos_min_rel_);
    if (return_code == ReturnCode::SUCCESS) {
        pos_min_configured_ = true;
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: pos_min=%.3f",
                id_, pos_min_rel_);
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_pos_max, pos_max_rel_);
    if (return_code == ReturnCode::SUCCESS) {
        pos_max_configured_ = true;
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: pos_max=%.3f",
                id_, pos_max_rel_);
    }

    if (pos_min_configured_ && pos_max_configured_ && pos_max_rel_ < pos_min_rel_) {
        PI_ERROR("Servo ID %d: pos_max (%.3f) is smaller than pos_min (%.3f)",
                 id_, pos_max_rel_, pos_min_rel_);
        return ReturnCode::INVALID_PARAM;
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_zero_pos, zero_pos_abs_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: zero_pos=%.3f",
                id_, zero_pos_abs_);
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_position_wrap_period,
        position_wrap_period_);
    if (return_code == ReturnCode::SUCCESS) {
        if (!pos_min_configured_ || !pos_max_configured_) {
            PI_ERROR(
                "Servo ID %d: position_wrap_period requires pos_min and "
                "pos_max in the model or individual configuration",
                id_);
            return ReturnCode::INVALID_PARAM;
        }
        const float raw_range = pos_max_rel_ - pos_min_rel_;
        if (!std::isfinite(pos_min_rel_) || !std::isfinite(pos_max_rel_) ||
            pos_min_rel_ >= pos_max_rel_ ||
            !std::isfinite(position_wrap_period_) ||
            position_wrap_period_ <= 0 || raw_range >= position_wrap_period_) {
            PI_ERROR(
                "Servo ID %d: position_wrap_period must be finite, positive, "
                "and greater than a finite ordered raw range width; "
                "raw_range=[%.3f, %.3f], period=%.3f",
                id_, pos_min_rel_, pos_max_rel_, position_wrap_period_);
            return ReturnCode::INVALID_PARAM;
        }
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                "Servo ID %d: position_wrap_period=%.3f", id_,
                position_wrap_period_);
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_spring_home_pos, spring_home_pos_rel_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                "Servo ID %d: spring_home_pos=%.3f", id_, spring_home_pos_rel_);
    }

    return_code = p_config->get_field_value(
        servo_config, p_config->fn_servo_home_pos, home_pos_rel_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Servo", InfoLevel::HELPFUL_1, "Servo ID %d: home_pos=%.3f",
                id_, home_pos_rel_);
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Servo::initialize_position_wrap() {
    if (position_wrap_period_ <= 0 || position_wrap_initialized_) {
        return ReturnCode::SUCCESS;
    }
    position_wrap_initialized_ = true;

    const float base_relative = (curr_pos_abs_ - zero_pos_abs_) * dir_invert_;
    const double min_turns =
        std::ceil((pos_min_rel_ - base_relative) / position_wrap_period_);
    const double max_turns =
        std::floor((pos_max_rel_ - base_relative) / position_wrap_period_);
    if (min_turns > max_turns) {
        PI_WARN(
            "Servo ID %d: no position-wrap candidate for measured_position=%.3f "
            "rad, period=%.3f rad, raw_range=[%.3f, %.3f] rad; leaving "
            "position unchanged for normal limit handling",
            id_, base_relative, position_wrap_period_, pos_min_rel_,
            pos_max_rel_);
        return ReturnCode::SUCCESS;
    }
    if (min_turns != max_turns) {
        PI_ERROR(
            "Servo ID %d: ambiguous position-wrap candidates for "
            "measured_position=%.3f rad, period=%.3f rad, raw_range=[%.3f, "
            "%.3f] rad",
            id_, base_relative, position_wrap_period_, pos_min_rel_,
            pos_max_rel_);
        return ReturnCode::INVALID_PARAM;
    }

    position_wrap_offset_rel_ =
        static_cast<float>(min_turns) * position_wrap_period_;
    if (position_wrap_offset_rel_ != 0) {
        PI_WARN(
            "Servo ID %d: corrected startup position wrap: measured=%.3f rad, "
            "offset=%+.3f rad, corrected=%.3f rad, raw_range=[%.3f, %.3f] rad",
            id_, base_relative, position_wrap_offset_rel_,
            get_pos_rad_relative(), pos_min_rel_, pos_max_rel_);
    }
    return ReturnCode::SUCCESS;
}

ReturnCode Servo::new_servos(const json& joint_config,
                             const DeviceConfig* p_config_model,
                             std::vector<std::unique_ptr<Servo>>& servos,
                             Device* p_device, Joint* p_joint,
                             Driver* p_driver) {
    ReturnCode return_code = ReturnCode::SUCCESS;
    std::vector<std::unique_ptr<Servo>> new_servos;

    if (p_config_model == nullptr) {
        PI_ERROR("Model configuration pointer is null");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (p_device == nullptr || p_joint == nullptr || p_driver == nullptr) {
        PI_ERROR("Servo factory dependencies are not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (!joint_config.contains(p_config_model->fn_servos)) {
        PI_ERROR("Servos information is not defined in configuration file");
        return ReturnCode::INVALID_PARAM;
    }

    auto servo_config_list = joint_config[p_config_model->fn_servos];
    if (servo_config_list.empty()) {
        PI_ERROR("At least one servo must be configured for each joint");
        return ReturnCode::INVALID_PARAM;
    }

    for (const auto& servo_config : servo_config_list) {
        std::string servo_model;

        return_code = p_config_model->get_field_value(
            servo_config, p_config_model->fn_servo_model, servo_model);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }

        std::unique_ptr<Servo> p_servo;
        if (servo_model == p_config_model->val_servo_model_dm_4340 ||
            servo_model == p_config_model->val_servo_model_dm_4310 ||
            servo_model == p_config_model->val_servo_model_encos_A4310) {
            p_servo = std::make_unique<ServoDm>(p_device, p_joint, p_driver);
            PI_INFO("Servo", InfoLevel::HELPFUL_1, "Created ServoDm instance");

        } else if (servo_model ==
                   p_config_model->val_servo_model_can_passive_encoder) {
            p_servo = std::make_unique<ServoCanPassiveEncoder>(p_device, p_joint,
                                                               p_driver);
            PI_INFO("Servo", InfoLevel::HELPFUL_1,
                    "Created ServoCanPassiveEncoder instance");
        } else {
            PI_ERROR("Unsupported servo model: %s", servo_model.c_str());
            return ReturnCode::NOT_SUPPORTED;
        }

        return_code = p_servo->init_config_model(servo_config, p_config_model);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to initialize servo with model configuration");
            return return_code;
        }

        new_servos.push_back(std::move(p_servo));

        PI_INFO("Servo", InfoLevel::HELPFUL_1,
                "Servo created: type=%s, id=%d, data_index=%d, "
                "total_servos=%d, joint_id=%d",
                servo_model.c_str(), new_servos.back()->id_,
                new_servos.back()->data_index_, (int)new_servos.size(),
                new_servos.back()->p_joint_->id_);
    }

    for (auto& p_servo : new_servos) {
        servos.push_back(std::move(p_servo));
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Servo::init_config_individual(
    const json& joint_config, const DeviceConfig* p_config_individual,
    std::vector<std::unique_ptr<Servo>>& servos) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (p_config_individual == nullptr) {
        PI_ERROR("Individual configuration pointer is null");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (!joint_config.contains(p_config_individual->fn_servos)) {
        PI_ERROR("Servos information is not defined in configuration file");
        return ReturnCode::INVALID_PARAM;
    }

    auto servo_config_list = joint_config[p_config_individual->fn_servos];
    if (servo_config_list.size() != servos.size()) {
        PI_ERROR(
            "Mismatch between model and individual configuration: servo "
            "count differs");
        return ReturnCode::INVALID_PARAM;
    }

    auto servo_it = servos.begin();

    for (const auto& servo_config : servo_config_list) {
        if (servo_it == servos.end()) {
            PI_ERROR(
                "Mismatch between model and individual configuration: servo "
                "count differs");
            return ReturnCode::INVALID_PARAM;
        }

        Servo* p_servo = servo_it->get();

        return_code =
            p_servo->init_config_individual(servo_config, p_config_individual);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR(
                "Failed to initialize servo with individual device "
                "configuration");
            return return_code;
        }
        servo_it++;
    }

    if (servo_it != servos.end()) {
        PI_ERROR(
            "Mismatch between model and individual configuration: servo "
            "count differs");
        return ReturnCode::INVALID_PARAM;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Servo::read_hardware_values() {
    prev_vel_ = curr_vel_;
    prev_tor_ = curr_tor_;
    pre_pos_abs_ = curr_pos_abs_;
    prev_pos_ = get_pos_rad_relative();

    if (p_driver_ == nullptr) {
        PI_ERROR("Driver handler is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    ReturnCode return_code = p_driver_->read_hardware_values(this);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (p_device_->is_read_only() == true) {
        // If the device is read only, skip the read hardware values
        return ReturnCode::SUCCESS;
    }

    idc_current_ = current_estimation_.estimate_idc_calibrated(
        motor_params_current_estimation_, curr_tor_, curr_vel_, temperature_,
        DEFAULT_VDC, get_tor_max());

    // Temperature force-stop must run BEFORE position/velocity/torque checks.
    // Rationale: once a servo crosses the FORCE_STOP threshold (93 C) and starts
    // losing holding torque, gravity drags the joint past its pos_min/pos_max
    // limit before we ever reach the temperature block at the bottom of this
    // function. The earlier-firing position check would then return first --
    // either swallowed by graceful_management() (safety_feature ON) or
    // propagated as SAFE_MODE_POS_EXCEED (safety_feature OFF) -- and the
    // temperature panic trip would never fire, allowing the servo to keep
    // heating until its own firmware thermal shutdown kicks in (~100 C).
    // Putting temperature first guarantees the slow ready-move + clean process
    // exit path runs at 93 C, well before the servo destroys itself.
    if (checker_temperature_exceed_.is_holding(
            temperature_ > TEMPERATURE_THRESHOLD_FORCE_STOP)) {
        PI_ERROR(
            "Servo ID %d temperature exceeds force-stop limit: threshold=%.1f C, "
            "current=%.1f C",
            id_, TEMPERATURE_THRESHOLD_FORCE_STOP, temperature_);
        return ReturnCode::SAFE_MODE_TEMPERATURE;
    } else {
        if (!is_behind_more_than_tolerable_threshold()) {
            safe_mode_.exit_safe_mode(this, ReturnCode::SAFE_MODE_TEMPERATURE);
        }
    }

    if (get_device_mode_belong_to() == DeviceMode::NORMAL) {
        if (checker_pos_exceed_.is_holding(
                get_pos_rad_relative() <
                    (pos_min_rel_ - get_pos_error_margin()) ||
                get_pos_rad_relative() >
                    (pos_max_rel_ + get_pos_error_margin()))) {
            PI_ERROR(
                "Servo ID %d position exceeds limits: min=%.3f, max=%.3f, "
                "current=%.3f",
                id_, pos_min_rel_, pos_max_rel_, get_pos_rad_relative());
            return safe_mode_.graceful_management(
                this, ReturnCode::SAFE_MODE_POS_EXCEED);
        } else {
            safe_mode_.exit_safe_mode(this, ReturnCode::SAFE_MODE_POS_EXCEED);
        }
    }

#if 0  // @todo Temporarily disabled for effector force feedback tests
    if (get_device_role_belong_to() == Role::FOLLOWER && get_device_mode_belong_to() == DeviceMode::NORMAL) {
        if (get_device_type_belong_to() == DeviceType::EFFECTOR ||
            (get_device_type_belong_to() == DeviceType::ARM &&
             get_device_message_type_belong_to() == MsgType::JOINT_INFO)) {
            float distance_to_target = fabs(get_tele_pos_rad() - get_pos_rad_relative());
            float distance_to_prev_pos = fabs(prev_pos_ - get_pos_rad_relative());

            if (checker_stall_detection_.is_holding(distance_to_target > get_pos_error_margin() &&
                                                    distance_to_prev_pos < get_pos_error_margin())) {
                PI_ERROR("Servo ID %d is in stall condition: tele_pos=%.3f, curr_pos=%.3f, prev_pos=%.3f", id_,
                         get_tele_pos_rad(), get_pos_rad_relative(), prev_pos_);
                return ReturnCode::STALL;
            }
            prev_pos_ = get_pos_rad_relative();
        }
    }
#endif

    if (get_device_type_belong_to() == DeviceType::ARM &&
        get_device_role_belong_to() == Role::FOLLOWER &&
        get_device_mode_belong_to() == DeviceMode::NORMAL &&
        get_device_message_type_belong_to() == MsgType::JOINT_INFO &&
        !is_device_in_move_to_ready_belong_to()) {
        // Skip the leader-pos based POS_BEHIND check while the device is doing
        // any move-to-ready: tele_pos_ is the stale leader cache (frozen by
        // clear_command_buffers_for_move_to_ready), and every step of the
        // internal ready trajectory necessarily widens |curr - tele_pos| past
        // the max threshold, which would otherwise spuriously trip safe mode
        // and mark the joint as failed. The move-to-ready state machine has
        // its own stuck/displacement tracking and is the authoritative source
        // of truth during that phase.
        if (checker_pos_difference_exceed_.is_holding(
                is_behind_more_than_max_threshold())) {
            PI_ERROR(
                "Servo ID %d position difference exceeds threshold: "
                "current_pos=%.3f, leader_pos=%.3f",
                id_, get_pos_rad_relative(), get_tele_pos_rad());
            return safe_mode_.graceful_management(
                this, ReturnCode::SAFE_MODE_POS_BEHIND);
        } else {
            safe_mode_.exit_safe_mode(this, ReturnCode::SAFE_MODE_POS_BEHIND);
        }
    }

    float max_velocity_threshold = MAX_VELOCITY_THRESHOLD;
    if (checker_vel_exceed_.is_holding(fabs(curr_vel_) >
                                       max_velocity_threshold)) {
        PI_ERROR(
            "Servo ID %d velocity exceeds limit: threshold=%.3f rad/s, "
            "current=%.3f rad/s",
            id_, max_velocity_threshold, curr_vel_);
        return safe_mode_.graceful_management(this, ReturnCode::SAFE_MODE_VEL);
    } else {
        safe_mode_.exit_safe_mode(this, ReturnCode::SAFE_MODE_VEL);
    }

    if (checker_tor_exceed_.is_holding(fabs(curr_tor_) > get_tor_max())) {
        if (!safe_mode_.is_torque_mode_enabled()) {
            if (!torque_mode_disabled_warning_active_) {
                torque_mode_disabled_warning_active_ = true;
                PI_WARN(
                    "Torque safe mode disabled: sustained measured torque exceeds limit; "
                    "continuing operation; launch with --safety_torque_mode to enable the "
                    "protective stop (servo ID %d, current=%.3f Nm, torq_max=%.3f Nm)",
                    id_, curr_tor_, get_tor_max());
            }
        } else {
            PI_ERROR(
                "Servo ID %d torque exceeds limit: max=%.3f Nm, current=%.3f Nm",
                id_, get_tor_max(), curr_tor_);
            return safe_mode_.graceful_management(this, ReturnCode::SAFE_MODE_TOR);
        }
    } else {
        torque_mode_disabled_warning_active_ = false;
        if (!is_behind_more_than_tolerable_threshold()) {
            safe_mode_.exit_safe_mode(this, ReturnCode::SAFE_MODE_TOR);
        }
    }

    // Temperature force-stop is now evaluated at the top of this function so it
    // cannot be masked by a position/velocity/torque check that fires first as
    // the joint sags under derating. See the early-return block above.

    return return_code;
}

float Servo::get_tele_pos_rad() {
    if (p_joint_ == nullptr) {
        PI_ERROR("Joint pointer is not initialized (servo ID %d)", id_);
        return 0;
    }
    return p_joint_->get_tele_pos_rad();
}

float Servo::get_tor_max() {
    if (p_joint_ == nullptr) {
        PI_ERROR("Joint pointer is null (servo ID %d)", id_);
        return 0;
    }
    return p_joint_->torq_max_;
}

float Servo::get_pos_error_margin() {
    if (p_joint_ == nullptr) {
        PI_ERROR("Joint pointer is null (servo ID %d)", id_);
        return 0;
    }
    return p_joint_->pos_error_margin_;
}
