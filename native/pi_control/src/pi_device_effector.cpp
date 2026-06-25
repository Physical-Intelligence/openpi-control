/*!
 * @file pi_device_effector.cpp
 * @brief Implementation of the DeviceEffector class for robot end-effector
 * device control and management.
 */

#include <cmath>

#include "pi_device_effector.hpp"

#define FORCE_FEEDBACK_TORQUE_FILTER_BUFFER_SIZE 5

DeviceEffector::DeviceEffector(const CommandLineArgs& cla) : Device(cla) {
    type_ = DeviceType::EFFECTOR;
}

DeviceEffector::~DeviceEffector() {
    ReturnCode return_code = DeviceEffector::park_safely();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to park effector safely before destruction");
    }
    // Joints are automatically destroyed via unique_ptr
}

ReturnCode DeviceEffector::set_control_mode(Role target_role,
                                            ControlModeIntent intent) {
    // Base implementation:
    // - Apply/clear READY_MOVE_OVERRIDE via the override flag (does not mutate
    // persistent control_mode_).
    // - Use Joint::change_control_mode_for_{leader,follower}() to map to
    // servo/driver operation modes.
    //
    // Concrete effectors may override when hardware-specific steps are required.
    if (intent == ControlModeIntent::READY_MOVE_OVERRIDE) {
        set_ready_move_force_position_mode(true);
    } else {
        set_ready_move_force_position_mode(false);
    }

    const bool use_follower_like =
        (intent == ControlModeIntent::READY_MOVE_OVERRIDE) ||
        (target_role == Role::FOLLOWER);
    ReturnCode rc = ReturnCode::SUCCESS;
    if (use_follower_like) {
        for (auto& p_joint : joints_) {
            rc = p_joint->change_control_mode_for_follower();
            if (rc != ReturnCode::SUCCESS) return rc;
        }
        return ReturnCode::SUCCESS;
    }

    prof_time_t current_time = Profile::get_time_now();
    for (auto& p_joint : joints_) {
        rc = p_joint->change_control_mode_for_leader(current_time);
        if (rc != ReturnCode::SUCCESS) return rc;
    }
    return ReturnCode::SUCCESS;
}

void DeviceEffector::clear_command_buffers_for_move_to_ready() {
    // Overwrite buffered leader targets so the follower holds position after
    // returning from move-to-ready.
    int i = 0;
    for (auto& p_joint : joints_) {
        const float current = p_joint->get_pos_rad_relative();
        if (i < (int)tele_pos_.size()) tele_pos_[i] = current;
        if (i < (int)tele_vel_.size()) tele_vel_[i] = 0.0f;
        if (i < (int)tele_tor_.size()) tele_tor_[i] = 0.0f;

        p_joint->set_tele_pos_rad(current);
        p_joint->set_tele_vel_rad_sec(0.0f);
        p_joint->set_tele_tor_nm(0.0f);
        i++;
    }
}

ReturnCode DeviceEffector::park_safely() {
    ReturnCode first_error = ReturnCode::SUCCESS;

    for (auto& p_joint : joints_) {
        ReturnCode return_code = p_joint->park_safely();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to park joint %d safely", p_joint->id_);
            if (first_error == ReturnCode::SUCCESS) {
                first_error = return_code;
            }
        }
    }
    if (first_error == ReturnCode::SUCCESS) {
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "Effector %s_%s completed safe parking process", model_.c_str(),
                id_.c_str());
    }
    return first_error;
}

ReturnCode DeviceEffector::start(int baud_rate) {
    ReturnCode return_code = Device::start(baud_rate);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    auto fail_after_partial_start = [this](ReturnCode start_error) {
        ReturnCode cleanup_return_code = stop();
        if (cleanup_return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to clean up effector %s_%s after start error: cleanup error code=%d", model_.c_str(),
                     id_.c_str(), static_cast<int>(cleanup_return_code));
        }
        return start_error;
    };

    for (auto& p_joint : joints_) {
        return_code = p_joint->start_hardware();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to start hardware for joint %d in %s_%s",
                     p_joint->id_, model_.c_str(), id_.c_str());
            return fail_after_partial_start(return_code);
        }
    }

    return_code = read_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to read hardware values during start");
        return fail_after_partial_start(return_code);
    }

    // Mirror DeviceArm::start(): send a one-shot hold-at-current command per joint so the
    // first ready-move iteration steps incrementally from the true current pose. Read-only
    // read-only teaching handles skip this block entirely; their write_hardware_values() and
    // move_to_ready_position() already short-circuit on is_read_only().
    if (is_read_only() == false) {
        const auto failed_before_hold = failed_joint_ids_snapshot();
        bool any_hold_sent = false;
        for (auto& p_joint : joints_) {
            const int16_t sid = p_joint->reference_servo_id();
            if (sid >= 0 && failed_before_hold.count(sid) != 0) {
                continue;
            }
            ReturnCode verify_rc = p_joint->verify_position_fresh();
            if (verify_rc != ReturnCode::SUCCESS) {
                PI_WARN("%s_%s: joint id=%d cache stale after enable; marking failed and "
                        "skipping startup hold-at-current",
                        model_.c_str(), id_.c_str(), p_joint->id_);
                if (sid >= 0) {
                    mark_joint_failed_during_recovery(sid);
                    set_last_failed_joint_id(sid);
                }
                is_normal_status_ = false;
                continue;
            }
            ReturnCode hold_rc = p_joint->hold_at_current_position();
            if (hold_rc != ReturnCode::SUCCESS) {
                PI_WARN("%s_%s: joint id=%d hold-at-current failed (rc=%d); marking failed",
                        model_.c_str(), id_.c_str(), p_joint->id_, static_cast<int>(hold_rc));
                if (sid >= 0) {
                    mark_joint_failed_during_recovery(sid);
                    set_last_failed_joint_id(sid);
                }
                is_normal_status_ = false;
                continue;
            }
            p_joint->reset_stuck_counter();
            any_hold_sent = true;
        }

        if (any_hold_sent) {
            ReturnCode write_rc = write_hardware_values();
            if (write_rc != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to write hardware values for startup hold-at-current in %s_%s",
                         model_.c_str(), id_.c_str());
                return fail_after_partial_start(write_rc);
            }
        }
    }

    return_code = move_to_ready_position();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to move effector to ready position");
        return fail_after_partial_start(return_code);
    }

    return_code = write_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to write hardware values during start");
        return fail_after_partial_start(return_code);
    }

    return return_code;
}

int DeviceEffector::probe_failed_servo_id() {
    for (auto& p_joint : joints_) {
        if (p_joint == nullptr) continue;
        if (p_joint->read_hardware_values() != ReturnCode::SUCCESS) {
            return static_cast<int>(p_joint->reference_servo_id());
        }
    }
    return -1;
}

ReturnCode DeviceEffector::read_hardware_values() {
    ReturnCode return_code = ReturnCode::SUCCESS;
    ReturnCode group_return_code = ReturnCode::SUCCESS;
    const bool in_recovery = is_in_emergency_recovery();
    // Mirror DeviceArm: tolerate partial per-joint failures both during recovery and during
    // startup (is_ready_ == false) so a single dead joint cannot block the entire arm from
    // reaching ready.
    const bool tolerate_partial_failure = in_recovery || (is_ready_ == false);

    // Only call group_read_hardware_values() if no Arm is attached (Effector
    // operates alone) If Arm is attached, Arm will call it to avoid duplicate
    // calls
    if (is_driver_created_by_this_) {
        if (p_driver_ == nullptr) {
            PI_ERROR("Driver handler is not initialized");
            return ReturnCode::FAIL;
        }

        group_return_code = p_driver_->group_read_hardware_values();
        if (group_return_code != ReturnCode::SUCCESS) {
            // Fall through to per-joint reads so we can identify which joint
            // actually failed (via set_last_failed_joint_id) instead of leaving
            // the placeholder -1 for enter_emergency_recovery().
            PI_ERROR("Group read hardware values failed for %s_%s; falling back to per-joint reads to identify failing joint",
                     model_.c_str(), id_.c_str());
        }
    }

    const auto failed_ids = failed_joint_ids_snapshot();
    bool any_joint_success = false;
    ReturnCode soft_escalation_code = ReturnCode::SUCCESS;
    for (auto& p_joint : joints_) {
        const int16_t sid = p_joint->reference_servo_id();
        if (sid >= 0 && failed_ids.count(sid) != 0) {
            continue;
        }
        return_code = p_joint->read_hardware_values();
        if (return_code == ReturnCode::SUCCESS) {
            any_joint_success = true;
            continue;
        }
        // SIG (CAN disconnect / motor no-response): genuinely unreachable.
        // Mark failed during recovery/startup so subsequent iterations skip
        // the bus probe; otherwise escalate so the main loop arms recovery.
        if (return_code == ReturnCode::SAFE_MODE_SIG) {
            is_normal_status_ = false;
            PI_ERROR("Failed to read hardware values for joint %d (servo %d) in %s_%s (rc=%d, SIG)",
                     p_joint->id_, sid, model_.c_str(), id_.c_str(), static_cast<int>(return_code));
            if (sid >= 0) set_last_failed_joint_id(static_cast<int>(sid));
            if (tolerate_partial_failure) {
                if (sid >= 0) mark_joint_failed_during_recovery(sid);
                continue;
            }
            return return_code;
        }

        // TEMPERATURE (panic trip at 93 C): triggers recovery, but the servo is
        // still alive and accepts commands -- only its internal thermal
        // protection gates produced torque. Disabling it from the ready move
        // shifts the unsupported load onto neighbouring joints and cascades the
        // overheat. Outside recovery, escalate so the main loop arms recovery;
        // inside recovery, keep the joint active so move_to_ready_position
        // continues commanding it slowly toward home (which is what unloads it).
        if (return_code == ReturnCode::SAFE_MODE_TEMPERATURE) {
            is_normal_status_ = false;
            PI_ERROR("Failed to read hardware values for joint %d (servo %d) in %s_%s (rc=%d, TEMP)",
                     p_joint->id_, sid, model_.c_str(), id_.c_str(), static_cast<int>(return_code));
            if (sid >= 0) set_last_failed_joint_id(static_cast<int>(sid));
            if (!tolerate_partial_failure) {
                return return_code;
            }
            any_joint_success = true;
            continue;
        }

        if (return_code == ReturnCode::HARDWARE_FAULT) {
            is_normal_status_ = false;
            PI_ERROR("Failed to read hardware values for joint %d (servo %d) in %s_%s "
                     "(rc=%d, HARDWARE_FAULT); see preceding HARDWARE FAULT message",
                     p_joint->id_, sid, model_.c_str(), id_.c_str(), static_cast<int>(return_code));
            if (sid >= 0) set_last_failed_joint_id(static_cast<int>(sid));
            if (tolerate_partial_failure) {
                if (sid >= 0) mark_joint_failed_during_recovery(sid);
                continue;
            }
            return return_code;
        }

        // Soft safety codes (POS_BEHIND, POS_EXCEED, VEL, TOR): SafeMode already
        // logged the limit violation and is clamping commands. Do NOT mark the
        // joint as failed -- it is still alive and must keep receiving
        // move-to-ready commands. Outside recovery/startup we still propagate.
        if (!tolerate_partial_failure) {
            is_normal_status_ = false;
            PI_ERROR("Failed to read hardware values for joint %d (servo %d) in %s_%s (rc=%d)",
                     p_joint->id_, sid, model_.c_str(), id_.c_str(), static_cast<int>(return_code));
            if (sid >= 0) set_last_failed_joint_id(static_cast<int>(sid));
            if (soft_escalation_code == ReturnCode::SUCCESS) {
                soft_escalation_code = return_code;
            }
        }
        if (tolerate_partial_failure) any_joint_success = true;
    }

    // During startup or recovery, swallow the bulk-read failure as long as at least one
    // joint is still alive -- the ready movement will proceed with the alive subset.
    if (tolerate_partial_failure) {
        if (any_joint_success) return ReturnCode::SUCCESS;
        return return_code;
    }

    if (group_return_code != ReturnCode::SUCCESS) {
        return group_return_code;
    }
    if (soft_escalation_code != ReturnCode::SUCCESS) {
        return soft_escalation_code;
    }
    return return_code;
}

ReturnCode DeviceEffector::write_hardware_values() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (is_read_only() == true) {
        // If the device is read only, skip the write hardware values
        return ReturnCode::SUCCESS;
    }

    // Only call group_write_hardware_values() if no Arm is attached (Effector
    // operates alone) If Arm is attached, Arm will call it to avoid duplicate
    // calls
    if (is_driver_created_by_this_) {
        if (p_driver_ == nullptr) {
            PI_ERROR("Driver handler is not initialized");
            return ReturnCode::FAIL;
        }
        return_code = p_driver_->group_write_hardware_values();
    }

    return return_code;
}

ReturnCode DeviceEffector::init(const CommandLineArgs& cla, int argc,
                                char** argv, std::shared_ptr<Topic> p_topic,
                                std::shared_ptr<Driver> p_driver) {
    ReturnCode return_code = Device::init(cla, argc, argv, p_topic, p_driver);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Base device initialization failed");
        return return_code;
    }

    // `open_at_min` is an installation-specific characteristic; prefer the individual config.
    // Fall back to model config for backward compatibility.
    return_code = p_config_individual_->get_field_value(
        p_config_individual_->values_, p_config_individual_->fn_effector_open_at_min,
        open_at_min_);
    if (return_code != ReturnCode::SUCCESS) {
        return_code = p_config_model_->get_field_value(
            p_config_model_->values_, p_config_model_->fn_effector_open_at_min,
            open_at_min_);
        if (return_code != ReturnCode::SUCCESS) {
            open_at_min_ = false;
        }
    }
    PI_INFO("DeviceEffector", InfoLevel::HELPFUL_1, "%s_%s: open_at_min=%d",
            model_.c_str(), id_.c_str(), open_at_min_);

    std::string control_mode_str;
    return_code = p_config_individual_->get_field_value(
        p_config_individual_->values_,
        p_config_individual_->fn_effector_control_mode, control_mode_str);
    if (return_code == ReturnCode::SUCCESS) {
        if (control_mode_str ==
            p_config_individual_->val_effector_control_mode_torque) {
            control_mode_ = EffectorControlMode::TORQUE;
        } else if (control_mode_str ==
                   p_config_individual_->val_effector_control_mode_position) {
            control_mode_ = EffectorControlMode::POSITION;
        } else {
            PI_ERROR("Invalid effector control mode: %s",
                     control_mode_str.c_str());
            return ReturnCode::INVALID_PARAM;
        }
        PI_INFO("DeviceEffector", InfoLevel::HELPFUL_1,
                "%s_%s: control_mode=%s", model_.c_str(), id_.c_str(),
                control_mode_str.c_str());
    } else {
        control_mode_ = EffectorControlMode::POSITION;
        PI_WARN(
            "Control mode not set in configuration, defaulting to position "
            "control mode");
    }

    // Torque-mode spring offset (rad): monopi ControlFollowGripper "offset",
    // subtracted from the position error. Installation-specific zero trim;
    // optional, default 0 (prefer adjusting the servo zero instead).
    return_code = p_config_individual_->get_field_value(
        p_config_individual_->values_,
        p_config_individual_->fn_effector_grip_spring_offset, grip_spring_offset_);
    if (return_code == ReturnCode::SUCCESS) {
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: grip_spring_offset from individual config=%.3f rad",
                model_.c_str(), id_.c_str(), grip_spring_offset_);
    }

    float dist_to_torque;
    return_code = p_config_individual_->get_field_value(
        p_config_individual_->values_,
        p_config_individual_->fn_effector_dist_to_torque_const, dist_to_torque);
    if (return_code == ReturnCode::SUCCESS) {
        distance_to_torque_ = dist_to_torque;
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: distance_to_torque from individual config=%.3f",
                model_.c_str(), id_.c_str(), distance_to_torque_);
    } else {
        distance_to_torque_ = OPT_DISTANCE_TO_TORQUE_DEFAULT;
        PI_WARN(
            "Distance-to-torque constant not set in configuration, using "
            "default value: %.3f",
            distance_to_torque_);
    }

    bool spring_effect;
    return_code = p_config_individual_->get_field_value(
        p_config_individual_->values_, p_config_individual_->fn_spring_effect,
        spring_effect);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0, "%s_%s: spring_effect=%d",
            model_.c_str(), id_.c_str(), spring_effect);

    return_code = enable_spring_effect(spring_effect);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to enable spring effect for %s_%s", model_.c_str(),
                 id_.c_str());
        return return_code;
    }

    return_code = Joint::new_joints(p_config_model_, p_config_individual_,
                                    joints_, this, p_driver_.get());
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to create effector joints");
        return return_code;
    }

    if (p_arm_ == nullptr) {
        PI_WARN("No arm attached to effector %s_%s", model_.c_str(),
                id_.c_str());
    }

    dof_total_ = dof_ = (int)joints_.size();
    PI_INFO("DeviceEffector", InfoLevel::HELPFUL_1, "%s_%s: DOF=%d",
            model_.c_str(), id_.c_str(), dof_);

    servo_num_ = 0;
    for (auto& p_joint : joints_) {
        servo_num_ += p_joint->get_servo_num();
    }
    PI_INFO("DeviceEffector", InfoLevel::HELPFUL_1, "%s_%s: servo_num=%d",
            model_.c_str(), id_.c_str(), servo_num_);

    if (p_arm_ != nullptr) {
        start_index_joint_ = p_arm_->get_dof();
        start_index_servo_ = p_arm_->get_servo_num();
    } else {
        start_index_joint_ = 0;
        start_index_servo_ = 0;
    }

    PI_INFO("DeviceEffector", InfoLevel::HELPFUL_1,
            "%s_%s: start_index_joint_=%d", model_.c_str(), id_.c_str(),
            start_index_joint_);
    PI_INFO("DeviceEffector", InfoLevel::HELPFUL_1,
            "%s_%s: start_index_servo_=%d", model_.c_str(), id_.c_str(),
            start_index_servo_);

    tele_pos_.assign(dof_, 0.0f);
    tele_vel_.assign(dof_, 0.0f);
    tele_tor_.assign(dof_, 0.0f);

    force_feedbacks_.clear();
    for (int i = 0; i < dof_; i++) {
        force_feedbacks_.push_back(std::make_unique<ForceFeedback>());
    }

    PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
            "%s_%s: Created %d force feedback objects (DOF=%d)", model_.c_str(),
            id_.c_str(), (int)force_feedbacks_.size(), dof_);

    if (cla_.effector_max_pos != OPT_EFFECTOR_MAX_POS_DEFAULT) {
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: effector_max_pos=%.3f", model_.c_str(), id_.c_str(),
                cla_.effector_max_pos);
        joints_.front()->set_pos_max_relative(cla_.effector_max_pos);
    }

    if (cla_.effector_min_pos != OPT_EFFECTOR_MIN_POS_DEFAULT) {
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: effector_min_pos=%.3f", model_.c_str(), id_.c_str(),
                cla_.effector_min_pos);
        joints_.front()->set_pos_min_relative(cla_.effector_min_pos);
    }

    const float EPSILON = 1e-6;
    if (fabs(cla_.distance_to_torque - OPT_DISTANCE_TO_TORQUE_DEFAULT) >
        EPSILON) {
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: distance_to_torque from command line=%.3f",
                model_.c_str(), id_.c_str(), cla_.distance_to_torque);
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: distance_to_torque from individual config=%.3f",
                model_.c_str(), id_.c_str(), distance_to_torque_);
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: default distance_to_torque=%.3f", model_.c_str(),
                id_.c_str(), OPT_DISTANCE_TO_TORQUE_DEFAULT);
        distance_to_torque_ = cla_.distance_to_torque;
    }

    if (cla_.effector_control_mode != OPT_EFFECTOR_CONTROL_MODE_UNDEFINED) {
        if (cla_.effector_control_mode == OPT_EFFECTOR_CONTROL_MODE_TORQUE) {
            control_mode_ = EffectorControlMode::TORQUE;
        } else if (cla_.effector_control_mode ==
                   OPT_EFFECTOR_CONTROL_MODE_POSITION) {
            control_mode_ = EffectorControlMode::POSITION;
        } else {
            PI_ERROR("Invalid effector control mode from command line: %s",
                     cla_.effector_control_mode.c_str());
            return ReturnCode::INVALID_PARAM;
        }
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: control_mode overridden by command line argument: %s",
                model_.c_str(), id_.c_str(),
                cla_.effector_control_mode.c_str());
    }

    for (int i = 0; i < dof_; i++) {
        return_code = force_feedbacks_[i]->init(cla);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to initialize force feedback object %d", i);
            return return_code;
        }
    }

    return ReturnCode::SUCCESS;
}
float DeviceEffector::get_normalized_gripper_position(Joint* p_joint) {
    if (p_joint == nullptr) {
        PI_ERROR("Invalid joint pointer");
        return 0;
    }

    if (p_joint->get_reference_servo() == nullptr) {
        PI_ERROR("Joint %d has no reference servo", p_joint->id_);
        return 0;
    }

    ReturnCode return_code = p_joint->read_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to read joint %d before normalizing gripper position",
                 p_joint->id_);
        return 0;
    }

    float pos_curr = p_joint->get_pos_rad_relative();
    float raw_pos_max = p_joint->get_pos_max_relative();
    float raw_pos_min = p_joint->get_pos_min_relative();
    float pos_max = p_joint->get_normalized_pos_max_relative();
    float pos_min = p_joint->get_normalized_pos_min_relative();
    int dir_invert = p_joint->get_dir_invert();

    PI_INFO("DeviceEffector", InfoLevel::FREQUENT_3,
            "Current position: %.3f, raw range: [%.3f, %.3f], normalized range: [%.3f, %.3f]",
            pos_curr, raw_pos_min, raw_pos_max, pos_min, pos_max);

    float max_range = pos_max - pos_min;
    if (max_range <= 0) {
        PI_ERROR(
            "Joint %d configuration error: maximum position (%.3f) must be "
            "greater than minimum position (%.3f)",
            p_joint->id_, pos_max, pos_min);
        return 0;
    }

    float clipped_current_pos = p_joint->clipping(pos_curr, pos_min, pos_max);

    float curr_grip_pos = clipped_current_pos - pos_min;
    float normalized_pos = curr_grip_pos / max_range;

    // open_at_min_ is the one explicit "which end of travel is open" knob: it flips
    // the published value so normalized 1.0 always means open. The servo-level
    // dir_invert must NOT feed this decision -- it is a joint-frame sign convention
    // (relative = (absolute - zero) * dir_invert), and the [pos_min, pos_max] range
    // this function normalizes over is already expressed in that inverted frame.
    // Treating dir_invert == -1 as "open at min" published the YAM gripper (whose
    // open end sits at relative pos_max) inverted: physically closed read 1.0.
    if (open_at_min_ == true) {
        normalized_pos = 1.0 - normalized_pos;
    }

    PI_INFO("DeviceEffector", InfoLevel::FREQUENT_3,
            "Normalized position: %.3f, current: %.3f, max: %.3f, min: %.3f, "
            "dir_invert: %d",
            normalized_pos, pos_curr, pos_max, pos_min, dir_invert);

    return normalized_pos;
}

float DeviceEffector::get_gripper_pos_rad_relative_from_normalized(
    float normalized_pos) {
    if (joints_.empty() || joints_.front() == nullptr) {
        PI_ERROR("No joints available");
        return 0;
    }
    Joint* p_joint = joints_.front().get();
    if (p_joint->get_reference_servo() == nullptr) {
        PI_ERROR("Joint %d has no reference servo", p_joint->id_);
        return 0;
    }

    // Callers feed the return value straight into a position command, so the
    // failure value matters: for E_Yam (open at pos_max) returning 0 rad on a
    // bad input meant FULL CLOSE -- a producer sending 1.0001 from float
    // rounding slammed the gripper to the exact opposite of its intent, every
    // cycle. Non-finite values (NaN passes both range comparisons) would
    // propagate into the CAN float->uint encode, which is UB. So: hold the
    // current position on non-finite input, clamp finite out-of-range input.
    if (!std::isfinite(normalized_pos)) {
        PI_ERROR("Normalized position is not finite; holding current position");
        return p_joint->get_pos_rad_relative();
    }
    if (normalized_pos < 0 || normalized_pos > 1) {
        PI_WARN(
            "Normalized position must be in range [0.0, 1.0], but received "
            "value: %.3f; clamping",
            normalized_pos);
        normalized_pos = p_joint->clipping(normalized_pos, 0.0f, 1.0f);
    }

    int dir_invert = p_joint->get_dir_invert();
    float raw_pos_max = p_joint->get_pos_max_relative();
    float raw_pos_min = p_joint->get_pos_min_relative();
    float pos_max = p_joint->get_normalized_pos_max_relative();
    float pos_min = p_joint->get_normalized_pos_min_relative();

    // Legacy effectors may still use an asymmetric upper-bound safety margin.
    // Explicit normalized ranges already encode both semantic endpoints and
    // are validated as mutually exclusive with this margin.
    //
    // We pull pos_max IN, but leave pos_min alone, so:
    //   * normalized=0.0 still maps to the raw pos_min (full close, needed
    //     to grip objects),
    //   * normalized=1.0 maps to (pos_max - safety_margin) instead of
    //     pos_max so PID overshoot / encoder noise at the steady state
    //     cannot push the actual reading past pos_max + pos_error_margin
    //     and trip the per-servo "position exceeds limits" guard in
    //     Servo::read_hardware_values. This was hitting bimanual data
    //     collection because the leader's spring-loaded gripper handle
    //     idles at normalized=1.0 -- the follower would charge to pos_max
    //     and immediately fault.
    //
    // The remap stays linear so AI training data remains a clean linear
    // function of normalized command (just over a slightly tighter range).
    float safety_margin =
        p_joint->has_normalized_position_range() ? 0.0f : p_joint->pos_max_safety_margin_;
    if (safety_margin < 0) {
        // Negative margins would expand the range past pos_max -- nonsensical
        // and unsafe. Clamp to 0 (legacy behavior) and warn.
        PI_WARN(
            "Joint %d: ignoring negative pos_max_safety_margin (%.3f); "
            "treating as 0",
            p_joint->id_, safety_margin);
        safety_margin = 0.0f;
    }
    float effective_pos_max = pos_max - safety_margin;
    if (effective_pos_max <= pos_min) {
        // Margin large enough to invert / collapse the range. Fall back to
        // legacy mapping rather than silently jamming the gripper at
        // pos_min, which would also disable the gripper entirely.
        PI_WARN(
            "Joint %d: pos_max_safety_margin (%.3f) collapses the gripper "
            "range [%.3f, %.3f]; reverting to legacy mapping for this call",
            p_joint->id_, safety_margin, pos_min, pos_max);
        effective_pos_max = raw_pos_max;
        safety_margin = 0.0f;
    }

    float effective_range = effective_pos_max - pos_min;
    if (effective_range <= 0) {
        PI_ERROR(
            "Joint %d has invalid configuration: maximum position (%.3f) must "
            "be greater than minimum position (%.3f)",
            p_joint->id_, pos_max, pos_min);
        return 0;
    }

    float invert_applied_normalized = normalized_pos;
    if (open_at_min_ == true) {
        // Mirror get_normalized_gripper_position(): state publishes ``1 - x`` for
        // open-at-min grippers, so incoming normalized commands must be flipped
        // back before mapping onto the joint range. Without this, the round trip
        // command(read()) mirrors the gripper across mid-travel. As on the read
        // side, the servo-level dir_invert frame sign must not feed this flip.
        invert_applied_normalized = 1.0f - normalized_pos;
    }

    float relative_pos = invert_applied_normalized * effective_range;
    float converted_rad_relative = relative_pos + pos_min;

    PI_INFO("DeviceEffector", InfoLevel::FREQUENT_3,
            "Converted position: %.3f, normalized_pos: %.3f, dir_invert: %d, "
            "raw_range: [%.3f, %.3f], normalized_range: [%.3f, %.3f], "
            "safety_margin: %.3f, effective_max: %.3f",
            converted_rad_relative, normalized_pos, dir_invert, raw_pos_min, raw_pos_max,
            pos_min, pos_max, safety_margin, effective_pos_max);

    return converted_rad_relative;
}

ReturnCode DeviceEffector::get_observation(MsgJoints& msg) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (joints_.size() == 0) {
        PI_ERROR("No joints found in effector");
        return ReturnCode::NOT_INITIALIZED;
    }

    for (auto& p_joint : joints_) {
        msg.add_joint_info(get_normalized_gripper_position(p_joint.get()),
                           p_joint->get_vel_rad_sec(), p_joint->get_tor_nm(),
                           p_joint->get_temperature(),
                           p_joint->get_idc_current());
    }

    return return_code;
}

ReturnCode DeviceEffector::operate_as_leader() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (mode_ == DeviceMode::NORMAL) {
        if (joints_.size() == 0) {
            PI_ERROR("No joints found in effector");
            return ReturnCode::FAIL;
        }

        if (enabled_force_feedback_ &&
            force_feedbacks_.size() != joints_.size()) {
            PI_ERROR("Force feedback state is not initialized for %s_%s: joints=%d, feedback objects=%d",
                     model_.c_str(), id_.c_str(), (int)joints_.size(), (int)force_feedbacks_.size());
            return ReturnCode::NOT_INITIALIZED;
        }

        if (!p_algo_) {
            PI_ERROR("Algorithm handler is not initialized");
            return ReturnCode::NOT_INITIALIZED;
        }

        prof_time_t step_start_time = Profile::get_time_now();

        float base_torque = 0;
        if (enabled_force_feedback_ == true) {
            base_torque += force_feedbacks_[0]->get_force_feedback_torque();
        }

        if (enabled_spring_effect_ == true) {
            std::vector<Joint*> joint_ptrs;
            for (auto& p_joint : joints_) {
                joint_ptrs.push_back(p_joint.get());
            }
            return_code =
                p_algo_->check_stability_control(joint_ptrs, step_start_time);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to check stability control for %s_%s",
                         model_.c_str(), id_.c_str());
                return return_code;
            }
        }

        for (auto& p_joint : joints_) {
            if (enabled_spring_effect_ == true) {
                return_code = p_joint->apply_spring_force(
                    base_torque, enabled_spring_effect_);
                if (return_code != ReturnCode::SUCCESS) {
                    PI_ERROR(
                        "Failed to apply spring force to joint %d in %s_%s",
                        p_joint->id_, model_.c_str(), id_.c_str());
                    return return_code;
                }
            } else {
                return_code = p_joint->apply_torque(base_torque);
                if (return_code != ReturnCode::SUCCESS) {
                    PI_ERROR("Failed to apply torque to joint %d in %s_%s",
                             p_joint->id_, model_.c_str(), id_.c_str());
                    return return_code;
                }
            }
        }
    }

    return return_code;
}

ReturnCode DeviceEffector::process_follower_msg(const MsgJoints& msg_joints) {
    if (enabled_force_feedback_ == false) {
        return ReturnCode::SUCCESS;
    }

    if (joints_.empty() || force_feedbacks_.size() != joints_.size()) {
        PI_ERROR("Force feedback state is not initialized for %s_%s: joints=%d, feedback objects=%d",
                 model_.c_str(), id_.c_str(), (int)joints_.size(), (int)force_feedbacks_.size());
        return ReturnCode::NOT_INITIALIZED;
    }

    const int expected_joint_count =
        start_index_joint_ + static_cast<int>(joints_.size());
    if (expected_joint_count < 0 ||
        msg_joints.joints_.size() != static_cast<size_t>(expected_joint_count)) {
        PI_ERROR(
            "Joint count mismatch in %s_%s: expected joints=%d, message "
            "joints=%d",
            model_.c_str(), id_.c_str(), expected_joint_count,
            (int)msg_joints.joints_.size());
        return ReturnCode::INVALID_PARAM;
    }

    if (p_topic_ == nullptr) {
        PI_ERROR("Topic handler is not initialized for %s_%s",
                 model_.c_str(), id_.c_str());
        return ReturnCode::NOT_INITIALIZED;
    }

    int i = 0;
    for (auto& p_joint : joints_) {
        int effector_index = start_index_joint_ + i;

        float pos = msg_joints.joints_[effector_index].curr_pos_;
        float vel = msg_joints.joints_[effector_index].curr_vel_;
        float tor = msg_joints.joints_[effector_index].curr_tor_;
        float temperature = msg_joints.joints_[effector_index].temperature_;
        float idc_current = msg_joints.joints_[effector_index].idc_current_;

        float leader_pos = get_normalized_gripper_position(p_joint.get());

        p_joint->follower_pos_ = pos;
        p_joint->follower_vel_ = vel;
        p_joint->follower_tor_ = tor;
        p_joint->follower_temperature_ = temperature;
        p_joint->follower_idc_current_ = idc_current;

        force_feedbacks_[i]->update_follower_info(leader_pos, pos, vel, tor,
                                                  temperature, idc_current);

        i++;
    }

    std::vector<float> effector_data;
    effector_data.push_back(force_feedbacks_[0]->get_follower_pos());
    effector_data.push_back(force_feedbacks_[0]->get_follower_vel());
    effector_data.push_back(force_feedbacks_[0]->get_follower_tor());
    effector_data.push_back(force_feedbacks_[0]->get_stall_tor());
    effector_data.push_back(force_feedbacks_[0]->get_force_feedback_torque());

    MsgEffectorInfo msg_effector_info(&effector_data);
    return p_topic_->publish(msg_effector_info);
}

ReturnCode DeviceEffector::apply_action(const MsgJoints& msg) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    const int expected_joint_count =
        start_index_joint_ + static_cast<int>(joints_.size());
    if (expected_joint_count < 0 ||
        msg.joints_.size() != static_cast<size_t>(expected_joint_count)) {
        PI_ERROR(
            "Joint count mismatch in %s_%s: expected joints=%d, message "
            "joints=%d",
            model_.c_str(), id_.c_str(), expected_joint_count,
            (int)msg.joints_.size());
        return ReturnCode::INVALID_PARAM;
    }

    int i = 0;
    for (auto& p_joint : joints_) {
        int effector_index = start_index_joint_ + i;

        float normalized_pos = msg.joints_[effector_index].curr_pos_;
        tele_pos_[i] =
            get_gripper_pos_rad_relative_from_normalized(normalized_pos);

        p_joint->set_tele_pos_rad(tele_pos_[i]);
        tele_vel_[i] = msg.joints_[effector_index].curr_vel_;
        p_joint->set_tele_vel_rad_sec(tele_vel_[i]);
        tele_tor_[i] = msg.joints_[effector_index].curr_tor_;
        p_joint->set_tele_tor_nm(tele_tor_[i]);

        PI_INFO("DeviceEffector", InfoLevel::DETAIL_2,
                "Effector joint %d apply_action(): tele_pos_rel=%.3f",
                p_joint->id_, tele_pos_[i]);

        i++;
    }

    return return_code;
}

ReturnCode DeviceEffector::move_to_ready_position() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (is_read_only() == true) {
        // If the device is read only, skip the move to ready position
        is_ready_ = true;
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s is read only, skipping move to ready position",
                model_.c_str(), id_.c_str());
        return ReturnCode::SUCCESS;
    }

    // Scoped to startup so explicit COMMAND_MOVE_TO_READY_POS and
    // emergency-recovery homing still execute with the flag set. Seed targets
    // at the current pose so the post-ready loop holds in place instead of
    // driving toward the home default.
    if (cla_.dont_go_to_home_pos == true && is_startup_ready_phase()) {
        for (auto& p_joint : joints_) {
            const float current = p_joint->get_pos_rad_relative();
            p_joint->target_pos_ = current;
            p_joint->prev_target_pos_ = current;
        }
        // Rebase the buffered command source onto the current pose so the
        // post-ready loop holds in place (the buffers default to home).
        clear_command_buffers_for_move_to_ready();
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: Skip startup move to home position (dont_go_to_home_pos "
                "flag is set); holding current pose",
                model_.c_str(), id_.c_str());
        is_ready_ = true;
        return ReturnCode::SUCCESS;
    }

    // Unified velocity-bounded effector ready move. Step size is identical for every joint
    // (NORMAL speed for startup/command, ERROR speed during emergency recovery). Time scales
    // with distance from home; the heartbeat watchdog catches genuine control-loop hangs.
    // Always honour the failed-joint set so communication-failure joints are skipped from the
    // first iteration onward, and treat stuck joints as "within tolerance" for the exit
    // criterion so the ready movement cannot loop forever.
    const std::unordered_set<int16_t> failed_ids = failed_joint_ids_snapshot();
    const float step = ready_move_step_rad();

    if (init_count_ < 1) {
        reached_cnt_ = 0;
        float max_displacement = 0.0f;
        for (auto& p_joint : joints_) {
            p_joint->target_pos_ = p_joint->get_home_pos_relative();
            const float start_pos = p_joint->get_pos_rad_relative();
            p_joint->prev_target_pos_ = start_pos;
            p_joint->reset_stuck_counter();
            max_displacement = std::max(
                max_displacement, static_cast<float>(fabs(p_joint->get_home_pos_relative() - start_pos)));
        }
        ready_move_budget_init(max_displacement, step);
    }

    bool all_within_tolerance_or_stuck = true;
    int responsive_joint_count = 0;
    int failed_joint_count = 0;
    int stuck_joint_count = 0;
    int16_t first_stuck_joint_id = -1;
    float worst_displacement = 0.0f;
    int16_t worst_joint_sid = -1;

    for (auto& p_joint : joints_) {
        const int16_t sid = p_joint->reference_servo_id();
        // `failed_ids` uses hardware servo ids, not abstract joint ids (see
        // read_hardware_values/mark_joint_failed_during_recovery). Look up by sid;
        // joint ids and servo ids are independent namespaces in the config.
        if (sid >= 0 && failed_ids.count(sid) != 0) {
            ++failed_joint_count;
            continue;
        }
        ++responsive_joint_count;

        const bool is_stuck = p_joint->update_and_check_stuck();

        const float current_pos = p_joint->get_pos_rad_relative();
        float displacement = fabs(p_joint->target_pos_ - current_pos);
        const bool outside_tolerance =
            displacement > p_joint->pos_error_margin_ + READY_MOVE_TOLERANCE_HYSTERESIS_RAD;
        if (is_stuck && outside_tolerance) {
            // Only joints that are stuck AND short of the target count toward the completion
            // report; a latched joint that later freed up and arrived is not a failure.
            ++stuck_joint_count;
            if (first_stuck_joint_id < 0 && sid >= 0) first_stuck_joint_id = sid;
        }
        if (outside_tolerance) {
            if (displacement > worst_displacement) {
                worst_displacement = displacement;
                worst_joint_sid = sid;
            }
            if (!is_stuck) {
                all_within_tolerance_or_stuck = false;
                reached_cnt_ = 0;
            }
        }

        // Command integration (same concept as arm): generate next command from
        // last commanded value.
        float cmd_pos = p_joint->prev_target_pos_;

        // Use measured current to detect overshoot.
        const float delta_meas = p_joint->target_pos_ - current_pos;

        float delta_cmd = p_joint->target_pos_ - cmd_pos;
        if (fabs(delta_meas) > p_joint->pos_error_margin_) {
            if ((delta_meas > 0.0f && delta_cmd < 0.0f) ||
                (delta_meas < 0.0f && delta_cmd > 0.0f)) {
                // Overshoot correction: reset integration state to measured
                // current and continue toward target.
                cmd_pos = current_pos;
                p_joint->prev_target_pos_ = cmd_pos;
                delta_cmd = p_joint->target_pos_ - cmd_pos;
            }
        }

        float new_pos = p_joint->target_pos_;
        if (delta_cmd > step) {
            new_pos = cmd_pos + step;
        } else if (delta_cmd < -step) {
            new_pos = cmd_pos - step;
        }

        p_joint->prev_target_pos_ = new_pos;

        PI_INFO("DeviceEffector", InfoLevel::DETAIL_2,
                "Effector joint %d: Step to ready target %.3f, cmd=%.3f, "
                "current=%.3f, step=%.6f, safe_mode_derating=%.3f",
                p_joint->id_, p_joint->target_pos_, new_pos, current_pos, step,
                p_joint->safe_mode_derating_);

        return_code = p_joint->move(new_pos);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR(
                "Failed to move effector joint %d (servo %d) in move_to_ready_position()",
                p_joint->id_, sid);
            // Mark failed using the servo id namespace -- matches what the UI
            // dialog and emergency_recovery payload speak in.
            if (sid >= 0) {
                mark_joint_failed_during_recovery(sid);
                set_last_failed_joint_id(sid);
            }
            is_normal_status_ = false;
            continue;
        }
    }

    const bool budget_exhausted = ready_move_budget_spend();
    if ((ready_move_budget_used_ % READY_MOVE_PROGRESS_LOG_PERIOD_ITERS) == 0 && worst_joint_sid >= 0) {
        // Periodic laggard visibility while the ready move runs (issue #5).
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s: ready move in progress (iter %ld/%ld): worst joint servo id=%d, "
                "displacement=%.3f rad, %d joint(s) stuck",
                model_.c_str(), id_.c_str(), ready_move_budget_used_, ready_move_budget_total_,
                static_cast<int>(worst_joint_sid), worst_displacement, stuck_joint_count);
    }

    if (responsive_joint_count == 0) {
        PI_WARN("%s_%s: all %d effector joints unresponsive during ready movement; completing",
                model_.c_str(), id_.c_str(), failed_joint_count);
        is_ready_ = true;
        init_count_ = 0;
    } else if (all_within_tolerance_or_stuck) {
        reached_cnt_++;
        if (reached_cnt_ > DEVICE_ARRIVED_CONFIRM_CNT) {
            is_ready_ = true;
            init_count_ = 0;
        }
    } else if (budget_exhausted) {
        // Hard backstop against a joint oscillating past the stuck-motion floor and resetting
        // the arrival confirmation forever (issue #5): complete best-effort, name the laggard.
        PI_ERROR("%s_%s: effector ready movement exceeded its iteration budget (%ld iters); "
                 "completing best-effort (worst joint servo id=%d, displacement=%.3f rad)",
                 model_.c_str(), id_.c_str(), ready_move_budget_total_,
                 static_cast<int>(worst_joint_sid), worst_displacement);
        is_normal_status_ = false;
        if (worst_joint_sid >= 0) set_last_failed_joint_id(worst_joint_sid);
        is_ready_ = true;
        init_count_ = 0;
    } else {
        is_ready_ = false;
    }

    if (is_ready_ == true) {
        if (stuck_joint_count > 0) {
            is_normal_status_ = false;
            if (first_stuck_joint_id >= 0) {
                set_last_failed_joint_id(static_cast<int>(first_stuck_joint_id));
            }
            PI_ERROR("%s_%s: effector ready movement completed with %d stuck joint(s) "
                     "(first stuck id=%d); reporting ready failure",
                     model_.c_str(), id_.c_str(), stuck_joint_count,
                     static_cast<int>(first_stuck_joint_id));
        }
        PI_INFO("DeviceEffector", InfoLevel::ESSENTIAL_0,
                "%s_%s is ready to operate", model_.c_str(), id_.c_str());
    }

    init_count_++;

    return return_code;
}

ReturnCode DeviceEffector::operate_as_follower() {
    if (is_read_only() == true) {
        // If the device is read only, skip the operate as follower
        return ReturnCode::SUCCESS;
    }

    if (joints_.size() == 0) {
        PI_ERROR("No joints found in effector");
        return ReturnCode::FAIL;
    }

    ReturnCode return_code = ReturnCode::SUCCESS;

    int i = 0;
    for (auto& p_joint : joints_) {
        if (control_mode_ == EffectorControlMode::TORQUE) {
            // Torque mode is already force-bounded: move_joint_with_torque
            // clips distance_to_torque_ * error to the joint torque limits.
            return_code = move_joint_with_torque(p_joint.get(), tele_pos_[i]);
        } else {
            // Position mode: bound the motor-side PD demand in both directions.
            // A blocked opening stop can overheat the gripper just as a blocked
            // closing command can.
            return_code =
                move(p_joint.get(), p_joint->clamp_target_to_grip_torque_limit(tele_pos_[i]), 0, 1.0);
        }
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to move effector joint %d in operate_as_follower() for %s_%s",
                     p_joint->id_, model_.c_str(), id_.c_str());
            return return_code;
        }
        i++;
    }

    return return_code;
}

ReturnCode DeviceEffector::set_effector_kd(float effector_kd) {
    ReturnCode first_error = ReturnCode::SUCCESS;
    for (auto& p_joint : joints_) {
        ReturnCode return_code = p_joint->set_pos_kd(effector_kd);
        if (return_code != ReturnCode::SUCCESS &&
            first_error == ReturnCode::SUCCESS) {
            first_error = return_code;
        }
    }

    return first_error;
}
