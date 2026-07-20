/*!
 * @file pi_joint.cpp
 * @brief Implementation of the Joint class for robot joint control and management.
 */

#include "pi_joint.hpp"

#include <cmath>

Joint::Joint(Device* p_device, Driver* p_driver) {
    p_device_ = p_device;
    p_driver_ = p_driver;
}

ReturnCode Joint::init_config_model(const json& joint_config, const DeviceConfig* p_config) {
    // Initialize joint parameters from model configuration file
    ReturnCode return_code;

    // Load joint ID (required field)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_id, id_);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Joint ID field '%s' is not defined in model configuration", p_config->fn_joint_id.c_str());
        return return_code;
    }
    PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint ID: %d", id_);

    // Load spring constant (optional: used for spring force calculation)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_spring_constant, spring_constant_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: spring_constant=%.3f", id_, spring_constant_);
    }

    // Load spring preload (optional: initial spring force offset)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_spring_preload, spring_preload_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: spring_preload=%.3f", id_, spring_preload_);
    }

    // Load spring force configuration flag (optional: enables/disables spring effect)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_spring_force_config, spring_force_config_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: spring_force_config=%d", id_, spring_force_config_);
    }

    // Load spring type (optional: bidirectional, unidirectional, or reverse unidirectional)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_spring_type, spring_type_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: spring_type=%d", id_, spring_type_);
    }

    // Load threshold angle change (optional: minimum angle change for stability detection)
    return_code =
        p_config->get_field_value(joint_config, p_config->fn_joint_threshold_angle_change, threshold_angle_change_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: threshold_angle_change=%.3f", id_, threshold_angle_change_);
    }

    // Load threshold time (optional: minimum time since last significant change for stability)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_threshold_time_sec, threshold_time_sec_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: threshold_time_sec=%.3f", id_, threshold_time_sec_);
    }

    // Load torque rescale factor (optional: scaling factor for torque commands)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_torq_rescale, torq_rescale_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: torq_rescale=%.3f", id_, torq_rescale_);
    }

    // Load minimum torque limit (optional: lower bound for torque commands)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_torq_min, torq_min_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: torq_min=%.3f", id_, torq_min_);
    }

    // Load maximum torque limit (optional: upper bound for torque commands)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_torq_max, torq_max_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: torq_max=%.3f", id_, torq_max_);
    }

    // Load safe mode minimum torque limit (optional: lower bound for safe operation mode)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_safe_torq_min, safe_torq_min_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: safe_torq_min=%.3f", id_, safe_torq_min_);
    }

    // Load safe mode maximum torque limit (optional: upper bound for safe operation mode)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_safe_torq_max, safe_torq_max_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: safe_torq_max=%.3f", id_, safe_torq_max_);
    }

    // Load maximum velocity limit (optional: upper bound for velocity commands)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_vel_max, vel_max_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: vel_max=%.3f", id_, vel_max_);
    }

    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_follow_vel_max, follow_vel_max_);
    if (return_code == ReturnCode::SUCCESS) {
        follow_vel_max_configured_ = true;
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: follow_vel_max=%.3f", id_, follow_vel_max_);
    } else {
        follow_vel_max_ = vel_max_;
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: follow_vel_max=%.3f (from vel_max)", id_, follow_vel_max_);
    }
    if (!std::isfinite(follow_vel_max_) || follow_vel_max_ <= 0.0f) {
        PI_ERROR("Joint %d: effective follow_vel_max must be positive and finite, but found %.3f", id_,
                 follow_vel_max_);
        return ReturnCode::INVALID_PARAM;
    }

    return_code =
        p_config->get_field_value(joint_config, p_config->fn_joint_follow_viscous_damping, follow_viscous_damping_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: follow_viscous_damping=%.6f", id_,
                follow_viscous_damping_);
        if (!std::isfinite(follow_viscous_damping_) || follow_viscous_damping_ < 0.0f) {
            PI_ERROR("Joint %d: follow_viscous_damping must be nonnegative and finite, but found %.6f", id_,
                     follow_viscous_damping_);
            return ReturnCode::INVALID_PARAM;
        }
    }

    // Load gravity feedforward scale (optional: empirical fit on top of the model
    // torques; i2rt shipped the YAM with per-joint factors and these mirror that)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_gravity_comp_factor, gravity_comp_factor_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: gravity_comp_factor=%.3f", id_, gravity_comp_factor_);
    }

    // Load grip torque limit (optional: direct torque-mode bound and symmetric
    // position-mode target-error bound).
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_grip_torque_limit, grip_torque_limit_nm_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: grip_torque_limit=%.3f Nm", id_, grip_torque_limit_nm_);
        if (grip_torque_limit_nm_ < 0) {
            PI_ERROR("Joint %d: grip_torque_limit must be >= 0, but found %.3f", id_, grip_torque_limit_nm_);
            return ReturnCode::INVALID_PARAM;
        }
    }

    // Load position rescale factor (optional: scaling factor for position commands)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_pos_rescale, pos_rescale_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: pos_rescale=%.3f", id_, pos_rescale_);
    }

    // Load position error margin (optional: tolerance for position reached detection)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_pos_error_margin, pos_error_margin_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: pos_error_margin=%.3f", id_, pos_error_margin_);
    }

    // Load asymmetric upper-bound safety margin for normalized [0, 1] command interpretation
    // (optional, default 0.0 keeps legacy behavior). See Joint::pos_max_safety_margin_ doc
    // for the full rationale -- this exists to keep spring-loaded leader handles (which idle
    // at normalized=1.0) from driving the follower into pos_max + pos_error_margin and
    // tripping the per-servo position-limit guard at startup.
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_pos_max_safety_margin, pos_max_safety_margin_);
    if (return_code == ReturnCode::SUCCESS) {
        pos_max_safety_margin_configured_ = true;
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: pos_max_safety_margin=%.3f", id_, pos_max_safety_margin_);
    }

    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_normalized_pos_min, normalized_pos_min_);
    if (return_code == ReturnCode::SUCCESS) {
        normalized_pos_min_configured_ = true;
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: normalized_pos_min=%.3f rad", id_, normalized_pos_min_);
    }

    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_normalized_pos_max, normalized_pos_max_);
    if (return_code == ReturnCode::SUCCESS) {
        normalized_pos_max_configured_ = true;
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: normalized_pos_max=%.3f rad", id_, normalized_pos_max_);
    }

    // Load maximum acceleration limit (optional: upper bound for acceleration commands)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_accel_max, accel_max_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: accel_max=%.3f", id_, accel_max_);
    }

    // Load safe mode derating factor (optional: velocity/acceleration reduction factor for safe operation)
    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_safe_mode_derating, safe_mode_derating_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: safe_mode_derating=%.3f", id_, safe_mode_derating_);
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Joint::init_config_individual(const json& joint_config, const DeviceConfig* p_config) {
    ReturnCode return_code;

    return_code = p_config->get_field_value(joint_config, p_config->fn_joint_spring_invert, spring_invert_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: spring_invert=%d", id_, spring_invert_);
    }

    return_code =
        p_config->get_field_value(joint_config, p_config->fn_joint_reference_servo_index, reference_servo_index_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Joint %d: reference_servo_index=%d", id_, reference_servo_index_);
    }
    if (reference_servo_index_ < 0 || reference_servo_index_ >= (int)servos_.size()) {
        PI_ERROR("Joint %d reference_servo_index=%d is out of range for %d servos", id_, reference_servo_index_,
                 (int)servos_.size());
        return ReturnCode::INVALID_PARAM;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Joint::validate_normalized_position_range() {
    if (normalized_pos_min_configured_ != normalized_pos_max_configured_) {
        PI_ERROR("Joint %d: normalized_pos_min and normalized_pos_max must be configured together", id_);
        return ReturnCode::INVALID_PARAM;
    }

    if (!has_normalized_position_range()) {
        return ReturnCode::SUCCESS;
    }

    const float raw_pos_min = get_pos_min_relative();
    const float raw_pos_max = get_pos_max_relative();
    if (pos_max_safety_margin_configured_) {
        PI_ERROR("Joint %d: normalized_pos_min/max cannot be combined with pos_max_safety_margin", id_);
        return ReturnCode::INVALID_PARAM;
    }
    if (!std::isfinite(raw_pos_min) || !std::isfinite(raw_pos_max) || raw_pos_min >= raw_pos_max) {
        PI_ERROR("Joint %d: raw position range must be finite and ordered, found [%.3f, %.3f]",
                 id_, raw_pos_min, raw_pos_max);
        return ReturnCode::INVALID_PARAM;
    }
    if (!std::isfinite(normalized_pos_min_) || !std::isfinite(normalized_pos_max_) ||
        normalized_pos_min_ >= normalized_pos_max_) {
        PI_ERROR("Joint %d: normalized position range must be finite and ordered, found [%.3f, %.3f]",
                 id_, normalized_pos_min_, normalized_pos_max_);
        return ReturnCode::INVALID_PARAM;
    }
    if (normalized_pos_min_ < raw_pos_min || normalized_pos_max_ > raw_pos_max) {
        PI_ERROR("Joint %d: normalized position range [%.3f, %.3f] must be contained in raw range [%.3f, %.3f]",
                 id_, normalized_pos_min_, normalized_pos_max_, raw_pos_min, raw_pos_max);
        return ReturnCode::INVALID_PARAM;
    }

    PI_INFO("Joint", InfoLevel::ESSENTIAL_0,
            "Joint %d: raw position range=[%.3f, %.3f] rad; normalized position range=[%.3f, %.3f] rad",
            id_, raw_pos_min, raw_pos_max, normalized_pos_min_, normalized_pos_max_);
    return ReturnCode::SUCCESS;
}

ReturnCode Joint::new_joints(const DeviceConfig* p_config_model, const DeviceConfig* p_config_individual,
                             std::vector<std::unique_ptr<Joint>>& joints, Device* p_device, Driver* p_driver) {
    ReturnCode return_code;
    std::vector<std::unique_ptr<Joint>> new_joints;

    if (p_config_model == nullptr) {
        PI_ERROR("Model configuration pointer is null");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (!p_config_model->values_.contains(p_config_model->fn_joints)) {
        PI_ERROR("Joints information is not defined in model configuration file");
        return ReturnCode::INVALID_PARAM;
    }

    auto joint_config_list = p_config_model->values_[p_config_model->fn_joints];
    if (joint_config_list.empty()) {
        PI_ERROR("At least one joint must be configured for each device");
        return ReturnCode::INVALID_PARAM;
    }

    for (const auto& joint_config : joint_config_list) {
        auto p_joint = std::make_unique<Joint>(p_device, p_driver);

        return_code = p_joint->init_config_model(joint_config, p_config_model);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to initialize joint %d from model configuration", p_joint->id_);
            return return_code;
        }

        return_code = Servo::new_servos(joint_config, p_config_model, p_joint->servos_, p_device, p_joint.get(), p_driver);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to create servos for joint %d", p_joint->id_);
            return return_code;
        }

        PI_INFO("Joint", InfoLevel::HELPFUL_1, "Created joint ID %d with %d servo(s)", p_joint->id_,
                (int)p_joint->servos_.size());
        new_joints.push_back(std::move(p_joint));
    }

    if (p_config_individual == nullptr) {
        PI_ERROR("Individual configuration pointer is null");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (!p_config_individual->values_.contains(p_config_individual->fn_joints)) {
        PI_ERROR("Joints information is not defined in individual configuration file");
        return ReturnCode::INVALID_PARAM;
    }

    auto joint_config_individual_list = p_config_individual->values_[p_config_individual->fn_joints];

    auto joint_it = new_joints.begin();
    for (const auto& joint_config_individual : joint_config_individual_list) {
        if (joint_it == new_joints.end()) {
            PI_ERROR("Joint count mismatch: model config has %d joints, but individual config has more",
                     (int)new_joints.size());
            return ReturnCode::INVALID_PARAM;
        }

        Joint* p_joint = joint_it->get();

        return_code = p_joint->init_config_individual(joint_config_individual, p_config_individual);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to initialize joint %d from individual configuration", p_joint->id_);
            return return_code;
        }

        return_code = Servo::init_config_individual(joint_config_individual, p_config_individual, p_joint->servos_);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to initialize servos for joint %d from individual configuration", p_joint->id_);
            return return_code;
        }

        return_code = p_joint->validate_normalized_position_range();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to validate normalized position range for joint %d", p_joint->id_);
            return return_code;
        }

        joint_it++;
    }

    if (joint_it != new_joints.end()) {
        PI_ERROR("Joint count mismatch: individual config has fewer joints than model config");
        return ReturnCode::INVALID_PARAM;
    }

    for (auto& p_joint : new_joints) {
        joints.push_back(std::move(p_joint));
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Joint::read_hardware_values() {
    if (p_driver_ == nullptr) {
        PI_ERROR("Driver handler is not initialized for joint %d", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    ReturnCode first_error = ReturnCode::SUCCESS;
    bool stalled = false;
    for (auto& p_servo : servos_) {
        ReturnCode return_code = p_servo->read_hardware_values();
        if (return_code == ReturnCode::STALL) {
            stalled = true;
        } else if (return_code != ReturnCode::SUCCESS &&
                   first_error == ReturnCode::SUCCESS) {
            first_error = return_code;
        }
    }

    if (first_error != ReturnCode::SUCCESS) {
        stalled_position_ = 0;
        return first_error;
    }

    if (stalled) {
        stalled_position_ = get_pos_rad_relative();
        tele_pos_ = stalled_position_;
        PI_INFO("Joint", InfoLevel::DETAIL_2, "Stall detected for joint %d: locked position=%.3f", id_,
                stalled_position_);
    } else {
        stalled_position_ = 0;
    }

    return ReturnCode::SUCCESS;
}

float Joint::calc_spring_force() {
    float spring_home_abs = get_spring_home_pos_absolute();
    float displacement = get_pos_rad_absolute() - spring_home_abs;

    float displacement_preload = spring_preload_ / spring_constant_;

    float adjusted_displacement = displacement + displacement_preload;

    float spring_torque = 0;

    int adjusted_spring_type = (spring_invert_ == true) ? -spring_type_ : spring_type_;

    if (spring_type_ == SPRING_TYPE_BIDIRECTIONAL) {
        spring_torque = -spring_constant_ * adjusted_displacement;
    } else {
        if (adjusted_spring_type == SPRING_TYPE_UNIDIRECTIONAL) {
            if (adjusted_displacement > 0) {
                spring_torque = -spring_constant_ * adjusted_displacement;
            } else {
                spring_torque = 0;
            }
        } else if (adjusted_spring_type == SPRING_TYPE_REVERSE_UNIDIR) {
            if (adjusted_displacement < 0) {
                spring_torque = -spring_constant_ * adjusted_displacement;
            } else {
                spring_torque = 0;
            }
        }
    }

    spring_torque = clipping(spring_torque, torq_min_, torq_max_);

    PI_INFO("Joint", InfoLevel::FREQUENT_3,
            "Joint %d: constant=%.3f, preload=%.3f, invert=%d, spring_type=%d, adjusted_spring_type=%d, "
            "displacement=%.3f, adjusted_displacement=%.3f, pos_abs=%.3f, spring_home_abs=%.3f, torque=%.3f, "
            "torq_min=%.3f, torq_max=%.3f",
            id_, spring_constant_, spring_preload_, spring_invert_, spring_type_, adjusted_spring_type, displacement,
            adjusted_displacement, get_pos_rad_absolute(), spring_home_abs, spring_torque, torq_min_, torq_max_);

    return spring_torque;
}

bool Joint::update_stability(const prof_time_t& step_started_time) {
    float delta_pos = fabs(get_pos_rad_absolute() - prev_pos_);

    bool is_prev_time_never_used = Profile::is_zero(last_significant_change_time_);

    if (is_prev_time_never_used || (delta_pos > threshold_angle_change_)) {
        last_significant_change_time_ = step_started_time;
        prev_pos_ = get_pos_rad_absolute();
    }

    prof_time_msec_t time_since_last_significant_change =
        Profile::get_time_diff(last_significant_change_time_, step_started_time);

    bool is_stable = (time_since_last_significant_change / 1000.0) > threshold_time_sec_;

    PI_INFO("Joint", InfoLevel::FREQUENT_3,
            "Joint %d: curr_pos_abs=%.3f, prev_pos=%.3f, delta_pos=%.3f, threshold_angle_change=%.3f, "
            "time_since_last_sig_change=%ld msec, threshold_time_sec=%.3f, is_stable=%d, "
            "is_prev_time_never_used=%d",
            id_, get_pos_rad_absolute(), prev_pos_, delta_pos, threshold_angle_change_,
            time_since_last_significant_change, threshold_time_sec_, is_stable, is_prev_time_never_used);

    return is_stable;
}

ReturnCode Joint::apply_spring_force(float base_torque, bool commandline_flag) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    float spring_force = 0;

    if (commandline_flag == true) {
        if (spring_force_config_ == true) {
            if (spring_enabled_ == true) {
                spring_force = calc_spring_force();
            }
        }
    }

    float total_torque = spring_force + base_torque;

    return_code = apply_torque(total_torque);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to apply torque for joint %d", id_);
        return return_code;
    }

    PI_INFO("Joint", InfoLevel::FREQUENT_3, "Joint %d: spring_torque=%.3f, gravity_torque=%.3f, total_torque=%.3f", id_,
            spring_force, base_torque, total_torque);

    return return_code;
}

ReturnCode Joint::change_control_mode_for_spring() {
    if (spring_force_config_ == true) {
        spring_enabled_ = true;
    } else {
        spring_enabled_ = false;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Joint::change_control_mode_for_leader(const prof_time_t& current_time) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    return_code = change_control_mode_for_spring();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to change spring control mode for joint %d", id_);
        return return_code;
    }

    prev_pos_ = get_pos_rad_relative();
    last_significant_change_time_ = current_time;

    for (auto& p_servo : servos_) {
        return_code = p_servo->change_control_mode_for_leader();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to change control mode to leader for joint %d, servo %d", id_, p_servo->id_);
            return return_code;
        }
    }

    return return_code;
}

ReturnCode Joint::change_control_mode_for_follower() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    for (auto& p_servo : servos_) {
        return_code = p_servo->change_control_mode_for_follower();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to change control mode to follower for joint %d, servo %d", id_, p_servo->id_);
            return return_code;
        }
    }

    return return_code;
}

ReturnCode Joint::park_safely() {
    ReturnCode first_error = ReturnCode::SUCCESS;

    for (auto& p_servo : servos_) {
        ReturnCode return_code = p_servo->park_safely();
        if (return_code != ReturnCode::SUCCESS && first_error == ReturnCode::SUCCESS) {
            first_error = return_code;
        }
    }

    return first_error;
}

ReturnCode Joint::start_hardware() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    for (auto& p_servo : servos_) {
        return_code = p_servo->start_hardware();
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    return return_code;
}

ReturnCode Joint::verify_position_fresh() {
    if (servos_.empty() || reference_servo_index_ < 0 ||
        reference_servo_index_ >= (int)servos_.size()) {
        PI_ERROR("Reference servo not initialised in verify_position_fresh(): Joint%d", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    return servos_[reference_servo_index_]->verify_position_fresh();
}

ReturnCode Joint::verify_operational() {
    for (auto& p_servo : servos_) {
        ReturnCode return_code = p_servo->verify_operational();
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }
    return ReturnCode::SUCCESS;
}

ReturnCode Joint::hold_at_current_position() {
    const float curr = get_pos_rad_relative();
    // Initialise the ready-move integration state so move_to_ready_position()'s init block
    // (which sets prev_target_pos_ = start_pos) and this hold-at-current command agree.
    prev_target_pos_ = curr;
    target_pos_ = curr;
    return move(curr);
}

void Joint::reset_stuck_counter() {
    stuck_last_pos_rad_ = get_pos_rad_relative();
    stuck_iter_count_ = 0;
    stuck_latched_ = false;
}

bool Joint::update_and_check_stuck() {
    const float curr_pos = get_pos_rad_relative();
    const float move_delta = fabs(curr_pos - stuck_last_pos_rad_);
    stuck_last_pos_rad_ = curr_pos;

    const float displacement = fabs(target_pos_ - curr_pos);
    if (displacement > pos_error_margin_) {
        if (move_delta < READY_MOVE_STUCK_POS_DELTA_RAD) {
            stuck_iter_count_++;
        } else {
            stuck_iter_count_ = 0;
        }
    } else {
        // Joint is inside the tolerance window: not stuck regardless of motion floor.
        stuck_iter_count_ = 0;
    }

    if (stuck_iter_count_ > READY_MOVE_STUCK_ITER_THRESHOLD) {
        stuck_latched_ = true;
    }
    // Latched for the rest of this ready move (cleared by reset_stuck_counter()): a joint that
    // already failed to make progress must not reset the device-level arrival confirmation by
    // twitching past the motion floor. It still receives commands, so if it physically frees up
    // it lands within tolerance and the latch becomes irrelevant for the exit criterion.
    return stuck_latched_;
}

float Joint::clamp_target_to_grip_torque_limit(float target_pos) {
    last_grip_requested_target_pos_ = target_pos;
    last_grip_applied_target_pos_ = target_pos;
    grip_limiter_active_ = false;

    if (grip_torque_limit_nm_ <= 0.0f || servos_.empty() ||
        reference_servo_index_ >= (int)servos_.size()) {
        return target_pos;
    }

    const Servo* p_reference = servos_[reference_servo_index_].get();
    // Effective kp, not the configured one: ServoDm substitutes a kp-10
    // default when pos_kp is configured 0.
    const float kp = p_reference->get_effective_pos_kp();
    if (kp <= 0.0f) {
        return target_pos;
    }

    float limit = grip_torque_limit_nm_;
    const float temperature = p_reference->temperature_;
    if (temperature >= TEMPERATURE_THRESHOLD_CAUTIOUS) {
        const float capped =
            (temperature < TEMPERATURE_THRESHOLD_CRITICAL) ? temperature : TEMPERATURE_THRESHOLD_CRITICAL;
        limit *=
            1.0f - ((capped - TEMPERATURE_THRESHOLD_CAUTIOUS) / TEMPERATURE_THRESHOLD_RANGE);
    }

    const float window = limit / kp;
    const float current = get_pos_rad_relative();
    const float clamped = clipping(target_pos, current - window, current + window);
    last_grip_applied_target_pos_ = clamped;
    grip_limiter_active_ = (clamped != target_pos);
    if (grip_limiter_active_) {
        PI_INFO("Joint", InfoLevel::FREQUENT_3,
                "Joint %d: symmetric grip torque limit active: target=%.3f clamped to %.3f "
                "(current=%.3f, window=%.3f rad, limit=%.3f Nm)",
                id_, target_pos, clamped, current, window, limit);
    }
    return clamped;
}

ReturnCode Joint::move(float target_pos) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    for (auto& p_servo : servos_) {
        return_code = p_servo->move(target_pos);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    return return_code;
}

ReturnCode Joint::apply_torque(float torque) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    for (auto& p_servo : servos_) {
        return_code = p_servo->apply_torque(torque);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    return return_code;
}

ReturnCode Joint::apply_torque_with_damping(float torque) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    for (auto& p_servo : servos_) {
        return_code = p_servo->apply_torque_with_damping(torque);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    return return_code;
}

ReturnCode Joint::move(float target_pos, float target_vel, float target_tor) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    for (auto& p_servo : servos_) {
        return_code = p_servo->move(target_pos, target_vel, target_tor);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    }

    return return_code;
}

void Joint::set_tele_pos_rad(float tele_pos) {
    float adjusted_tele_pos = tele_pos;

    if (stalled_position_ != 0) {
        if (tele_pos_moving_dir_ > 0 && tele_pos < stalled_position_) {
            stalled_position_ = 0;
            PI_INFO("Joint", InfoLevel::DETAIL_2,
                    "Stall cleared for joint %d: movement direction reversed from positive to negative", id_);
        } else if (tele_pos_moving_dir_ < 0 && tele_pos > stalled_position_) {
            stalled_position_ = 0;
            PI_INFO("Joint", InfoLevel::DETAIL_2,
                    "Stall cleared for joint %d: movement direction reversed from negative to positive", id_);
        } else {
            adjusted_tele_pos = stalled_position_;
            PI_INFO("Joint", InfoLevel::DETAIL_2, "Joint %d remains stalled: tele_pos=%.3f, adjusted_tele_pos=%.3f",
                    id_, tele_pos, adjusted_tele_pos);
        }
    } else {
        tele_pos_moving_dir_ = tele_pos_ - prev_tele_pos_;
        prev_tele_pos_ = tele_pos_;
    }

    tele_pos_ = tele_pos;

}
