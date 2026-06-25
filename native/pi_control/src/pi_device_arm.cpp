/*!
 * @file pi_device_arm.cpp
 * @brief Implementation of the DeviceArm class for robotic arm device control and management.
 */

 #include <unistd.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include "pi_device_arm.hpp"
#include "pi_device_effector.hpp"
#include "pi_joint.hpp"

DeviceArm::DeviceArm(const CommandLineArgs& cla) : Device(cla) { type_ = DeviceType::ARM; }

DeviceArm::~DeviceArm() {
    ReturnCode return_code = DeviceArm::park_safely();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to park arm safely before destruction");
    }
    // Joints are automatically destroyed via unique_ptr
    p_effector_.reset();
}

void DeviceArm::request_move_to_ready_position(int request_id) {
    // Forward to base and attached effector (if any).
    Device::request_move_to_ready_position(request_id);
    if (p_effector_) {
        p_effector_->request_move_to_ready_position(request_id);
    }
}

void DeviceArm::request_move_to_ready_and_stop(int request_id) {
    Device::request_move_to_ready_and_stop(request_id);
    if (p_effector_) {
        p_effector_->request_move_to_ready_position(request_id);
    }
}

ReturnCode DeviceArm::set_control_mode(Role target_role, ControlModeIntent intent) {
    // Base arm behavior:
    // - READY_MOVE_OVERRIDE: force follower-like (position-based) control to execute move_to_ready_position()
    //   safely from the current pose.
    // - NORMAL_OPERATION: follow target_role (leader vs follower).
    //
    // The ARX device subclass may override hardware-specific behavior.
    const bool use_follower_like = (intent == ControlModeIntent::READY_MOVE_OVERRIDE) || (target_role == Role::FOLLOWER);
    ReturnCode rc = ReturnCode::SUCCESS;

    if (use_follower_like) {
        for (auto& p_joint : joints_) {
            rc = p_joint->change_control_mode_for_follower();
            if (rc != ReturnCode::SUCCESS) return rc;
        }
    } else {
        prof_time_t current_time = Profile::get_time_now();
        for (auto& p_joint : joints_) {
            rc = p_joint->change_control_mode_for_leader(current_time);
            if (rc != ReturnCode::SUCCESS) return rc;
        }
    }

    // Apply to attached effector too (if any). During emergency recovery we skip the
    // effector: its servos almost certainly share the same dead bus that triggered recovery,
    // and per-joint transactions could exceed the emergency-shutdown budget.
    if (p_effector_ && !is_in_emergency_recovery()) {
        const Role eff_role = p_effector_->get_device_role();
        // For READY_MOVE_OVERRIDE we always force follower-like, independent of effector role.
        const Role eff_target = (intent == ControlModeIntent::READY_MOVE_OVERRIDE) ? Role::FOLLOWER : eff_role;
        rc = p_effector_->set_control_mode(eff_target, intent);
        if (rc != ReturnCode::SUCCESS) return rc;
    }

    if (use_follower_like) {
        reset_slew_targets_to_current();
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DeviceArm::set_runtime_force_feedback(bool enabled, float gain) {
    ReturnCode rc = Device::set_runtime_force_feedback(enabled, gain);
    if (rc != ReturnCode::SUCCESS) {
        return rc;
    }
    if (p_effector_) {
        rc = p_effector_->set_runtime_force_feedback(enabled, gain);
    }
    return rc;
}

ReturnCode DeviceArm::set_runtime_force_feedback_gain(float gain) {
    ReturnCode rc = Device::set_runtime_force_feedback_gain(gain);
    if (rc != ReturnCode::SUCCESS) {
        return rc;
    }
    if (p_effector_) {
        rc = p_effector_->set_runtime_force_feedback_gain(gain);
    }
    return rc;
}

void DeviceArm::reset_ready_state_for_move_to_ready() {
    // Reset base device state
    Device::reset_ready_state_for_move_to_ready();

    // Reset arm-specific ready state. The per-joint displacement-to-home cache is also
    // cleared so the next move-to-ready captures a fresh baseline for progress reporting.
    is_ready_arm_ = false;
    moving_joint_index_ = 0;
    ready_move_initial_disp_to_home_.clear();
}

void DeviceArm::clear_command_buffers_for_move_to_ready() {
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
    reset_slew_targets_to_current();
}

void DeviceArm::reset_slew_targets_to_current() {
    if (planning_type_ != TrajectoryPlanningType::SLEW_POS_GRAVITY) {
        return;
    }

    for (auto& p_joint : joints_) {
        const float current = p_joint->get_pos_rad_relative();
        p_joint->target_pos_ = current;
        p_joint->adjusted_target_pos_ = current;
        p_joint->prev_target_pos_ = current;
    }
}

ReturnCode DeviceArm::park_safely() {
    ReturnCode first_error = ReturnCode::SUCCESS;

    // Park the effector before sequentially parking the arm joints so its
    // firmware communication watchdog cannot expire before it is disabled.
    if (!p_effector_) {
        PI_WARN("No effector attached when park_safely() is called for %s_%s", model_.c_str(), id_.c_str());
    } else {
        ReturnCode return_code = p_effector_->park_safely();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to park effector safely: %s_%s", model_.c_str(), id_.c_str());
            first_error = return_code;
        }
    }

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
        PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0, "Arm %s_%s completed safe parking process", model_.c_str(),
                id_.c_str());
    }
    return first_error;
}

ReturnCode DeviceArm::read_hardware_values() {
    ReturnCode return_code = ReturnCode::SUCCESS;
    ReturnCode group_return_code = ReturnCode::SUCCESS;
    const bool in_recovery = is_in_emergency_recovery();
    // Treat the pre-ready startup phase as a "partial failure tolerated" mode too: if a
    // single joint loses CAN during the very first read, we want to mark it failed and let
    // the rest of the arm reach ready instead of aborting the whole start path.
    const bool tolerate_partial_failure = in_recovery || (is_ready_ == false);

    if (is_driver_created_by_this_) {
        if (p_driver_ == nullptr) {
            PI_ERROR("Driver handler is not initialized in read_hardware_values()");
            return ReturnCode::FAIL;
        }
        group_return_code = p_driver_->group_read_hardware_values();
        if (group_return_code != ReturnCode::SUCCESS) {
            // Pull the driver-level dead-set and mark every affected joint as failed.
            const auto driver_dead = p_driver_->dead_servo_ids();
            const int driver_failed_servo_id = p_driver_->last_failed_servo_id();
            if (driver_failed_servo_id >= 0) {
                set_last_failed_joint_id(driver_failed_servo_id);
            }
            if (tolerate_partial_failure) {
                for (int sid : driver_dead) {
                    mark_joint_failed_during_recovery(static_cast<int16_t>(sid));
                }
            }
            if (in_recovery) {
                PI_WARN("Group read failed during recovery for %s_%s (dead_servo_ids=%zu, first=%d); continuing with cached data for responsive joints",
                        model_.c_str(), id_.c_str(), driver_dead.size(), driver_failed_servo_id);
            } else if (is_ready_ == false) {
                is_normal_status_ = false;
                PI_WARN("Group read failed during startup for %s_%s (dead_servo_ids=%zu, first=%d); marking dead servos failed and continuing",
                        model_.c_str(), id_.c_str(), driver_dead.size(), driver_failed_servo_id);
            } else {
                PI_ERROR("Group read hardware values failed for %s_%s (dead_servo_ids=%zu, first=%d)",
                         model_.c_str(), id_.c_str(), driver_dead.size(), driver_failed_servo_id);
            }
            return_code = ReturnCode::SUCCESS;
        }
    }

    const auto failed_ids = failed_joint_ids_snapshot();
    bool any_joint_success = false;
    // First soft escalation (VEL/TOR) seen this cycle. These codes are meant to
    // reach the main loop so it arms emergency recovery, but the per-joint loop
    // must still finish reading every joint -- so latch the code here instead of
    // letting the next (healthy) joint's SUCCESS overwrite it. Without the
    // latch, only a fault on the *last* joint in iteration order ever
    // escalated; joints 1..n-1 were silently swallowed.
    ReturnCode soft_escalation_code = ReturnCode::SUCCESS;
    for (auto& p_joint : joints_) {
        // Track joints by their reference servo id (hardware-level). This is
        // the same id space driver_failed_servo_id, enter_emergency_recovery()'s
        // failed_joint_id payload, and the UI dialog all speak in -- keeping
        // everything consistent.
        const int16_t sid = p_joint->reference_servo_id();
        if (sid >= 0 && failed_ids.count(sid) != 0) {
            // Already marked failed (recovery or startup partial-fail): skip per-joint read
            // so we don't spam the bus with retries.
            continue;
        }
        return_code = p_joint->read_hardware_values();
        if (return_code == ReturnCode::SUCCESS) {
            any_joint_success = true;
            continue;
        }
        // SIG (CAN disconnect / motor no-response) means we genuinely cannot
        // talk to the servo. Disable the joint so subsequent iterations skip
        // the bus probe; if we are not yet in recovery, escalate to the main
        // loop so it can enter emergency recovery.
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

        // TEMPERATURE (panic trip at 93 C) is the *trigger* for recovery, but
        // unlike SIG the servo is still electrically alive and accepts CAN
        // commands -- its own thermal protection just gates how much torque it
        // produces. Disabling it from the ready move is exactly the wrong
        // thing: ready is the pose that *unloads* this joint (relieving gravity
        // torque is what lets it cool), and skipping it shifts the unsupported
        // load onto neighbouring joints and cascades the overheat. So: outside
        // recovery, escalate so the main loop arms recovery; inside recovery,
        // keep the joint in the active set and let move_to_ready_position keep
        // commanding it slowly toward home.
        if (return_code == ReturnCode::SAFE_MODE_TEMPERATURE) {
            is_normal_status_ = false;
            PI_ERROR("Failed to read hardware values for joint %d (servo %d) in %s_%s (rc=%d, TEMP)",
                     p_joint->id_, sid, model_.c_str(), id_.c_str(), static_cast<int>(return_code));
            if (sid >= 0) set_last_failed_joint_id(static_cast<int>(sid));
            if (!tolerate_partial_failure) {
                return return_code;
            }
            // Count this joint toward any_joint_success so a single overheated
            // joint cannot collapse the whole arm's read into "all dead".
            any_joint_success = true;
            continue;
        }

        // A servo-provided hardware fault is actionable and must not be
        // collapsed into a generic signal timeout. Outside recovery, escalate
        // immediately; during recovery/startup, skip the faulted joint so the
        // remaining joints can still move to ready.
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

        // Soft safety codes reaching this point are VEL/TOR (POS_BEHIND and
        // POS_EXCEED self-manage inside SafeMode and return SUCCESS; SIG and
        // TEMPERATURE have dedicated branches above). Do NOT mark the joint as
        // failed -- it is still alive and must keep receiving move-to-ready
        // commands. Outside recovery/startup, latch the code so it propagates
        // after the loop and the main loop can enter emergency recovery.
        if (!tolerate_partial_failure) {
            is_normal_status_ = false;
            PI_ERROR("Failed to read hardware values for joint %d (servo %d) in %s_%s (rc=%d)",
                     p_joint->id_, sid, model_.c_str(), id_.c_str(), static_cast<int>(return_code));
            if (sid >= 0) set_last_failed_joint_id(static_cast<int>(sid));
            if (soft_escalation_code == ReturnCode::SUCCESS) {
                soft_escalation_code = return_code;
            }
        }
        // Treat soft errors as success-equivalent for the per-iteration
        // any_joint_success bookkeeping during partial-failure-tolerant phases:
        // SafeMode kept the joint usable, we just couldn't follow the leader's
        // exact target. Without this, every joint hitting POS_BEHIND would
        // incorrectly mark the whole arm as "all joints failed" and the read
        // would propagate an error to the main loop.
        if (tolerate_partial_failure) any_joint_success = true;
    }

    // Suppress per-iteration error propagation as long as SOME joint is still alive while
    // partial failure is tolerated (recovery or startup). If every joint failed, return the
    // last error so the main loop sees it and can react.
    if (tolerate_partial_failure && any_joint_success) {
        if (!logged_per_joint_read_fallback_) {
            logged_per_joint_read_fallback_ = true;
            const size_t alive = joints_.size() - failed_ids.size();
            const char* phase = in_recovery ? "[RECOVERY]" : "[STARTUP]";
            PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                    "%s %s_%s: per-joint read fallback active (%zu/%zu joints alive, %zu failed)",
                    phase, model_.c_str(), id_.c_str(), alive, joints_.size(), failed_ids.size());
        }
        return ReturnCode::SUCCESS;
    }
    if (tolerate_partial_failure && !any_joint_success) {
        const char* phase = in_recovery ? "[RECOVERY]" : "[STARTUP]";
        PI_ERROR("%s %s_%s: all joints failed to read; propagating error",
                 phase, model_.c_str(), id_.c_str());
        return return_code;
    }

    // Outside recovery: if the bulk read failed but every arm joint responded,
    // the offending servo must live on the attached effector (it shares the same
    // bus). Probe the effector joints so set_last_failed_joint_id() can name a
    // real id instead of leaving the placeholder -1.
    if (group_return_code != ReturnCode::SUCCESS && any_joint_success
        && last_failed_joint_id() < 0 && p_effector_ != nullptr) {
        auto* p_eff = dynamic_cast<DeviceEffector*>(p_effector_.get());
        if (p_eff != nullptr) {
            const int eff_failed_servo_id = p_eff->probe_failed_servo_id();
            if (eff_failed_servo_id >= 0) {
                set_last_failed_joint_id(eff_failed_servo_id);
            }
        }
    }

    // Outside recovery & after first ready: if the bulk read failed, propagate that failure
    // so the main loop triggers emergency recovery. At this point set_last_failed_joint_id()
    // has been called for whichever joint(s) failed per-joint reads, so the recovery
    // notification carries the real failed_joint_id. During startup (is_ready_ == false) we
    // already marked dead servos failed above and let the rest continue toward ready, so we
    // intentionally swallow the bulk-read error here.
    if (group_return_code != ReturnCode::SUCCESS && is_ready_ == true) {
        return group_return_code;
    }

    // Propagate the latched soft escalation (VEL/TOR) from whichever joint
    // tripped it first, now that every joint has been read this cycle.
    if (soft_escalation_code != ReturnCode::SUCCESS) {
        return soft_escalation_code;
    }

    return return_code;
}

float DeviceArm::get_ready_move_completion_ratio() const {
    // Already at ready: trivially complete.
    if (is_ready_arm_) return 1.0f;
    if (joints_.empty()) return 0.0f;

    const bool in_recovery = is_in_emergency_recovery();
    const auto failed_ids = in_recovery ? failed_joint_ids_snapshot()
                                        : std::unordered_set<int16_t>{};

    // If every joint is unresponsive there is no physical motion to track. Report
    // 100% so the UI matches the imminent SHUTDOWN_AFTER_ERROR (which also publishes
    // progress = 1.0) instead of jumping from 0% straight to 100%.
    if (in_recovery && failed_ids.size() >= joints_.size()) return 1.0f;

    // Lazy init: snapshot the initial |home - current| once at the first call so that
    // subsequent calls measure progress relative to that baseline. The cache is cleared
    // by reset_ready_state_for_move_to_ready(), so each new move-to-ready (startup,
    // command, emergency) gets a fresh baseline. Skip joints that have already been
    // marked failed during recovery -- they cannot move, so including them would peg
    // the average at "no progress" forever.
    if (ready_move_initial_disp_to_home_.empty()) {
        for (const auto& p_joint : joints_) {
            const int16_t sid = p_joint->reference_servo_id();
            if (sid < 0) continue;
            if (in_recovery && failed_ids.count(sid) != 0) continue;
            const float home = p_joint->get_home_pos_relative();
            const float curr = p_joint->get_pos_rad_relative();
            ready_move_initial_disp_to_home_[sid] = std::fabs(home - curr);
        }
    }

    float sum_ratio = 0.0f;
    int count = 0;
    for (const auto& p_joint : joints_) {
        const int16_t sid = p_joint->reference_servo_id();
        if (sid < 0) continue;
        if (in_recovery && failed_ids.count(sid) != 0) continue;
        auto it = ready_move_initial_disp_to_home_.find(sid);
        if (it == ready_move_initial_disp_to_home_.end()) {
            // Joint became responsive after the lazy-init snapshot (e.g. the bus
            // recovered briefly). Seed its baseline now so future iterations can
            // track its progress.
            const float home = p_joint->get_home_pos_relative();
            const float curr = p_joint->get_pos_rad_relative();
            ready_move_initial_disp_to_home_[sid] = std::fabs(home - curr);
            it = ready_move_initial_disp_to_home_.find(sid);
        }
        const float initial = it->second;
        if (initial <= 1e-6f) {
            // Joint was already at home when the move started -- count as 100% done.
            sum_ratio += 1.0f;
        } else {
            const float home = p_joint->get_home_pos_relative();
            const float curr = p_joint->get_pos_rad_relative();
            const float remaining = std::fabs(home - curr);
            float r = 1.0f - (remaining / initial);
            if (r < 0.0f) r = 0.0f;
            if (r > 1.0f) r = 1.0f;
            sum_ratio += r;
        }
        ++count;
    }
    if (count == 0) return 1.0f;
    return sum_ratio / static_cast<float>(count);
}

ReturnCode DeviceArm::write_hardware_values() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (is_driver_created_by_this_) {
        if (p_driver_ == nullptr) {
            PI_ERROR("Driver handler is not initialized in write_hardware_values()");
            return ReturnCode::FAIL;
        }
        return_code = p_driver_->group_write_hardware_values();
        // During emergency recovery a dead servo on the shared bus causes the group write
        // to fail for everyone, even though the per-joint move() calls inside
        // ``move_to_ready_position`` already issued their individual writes. Treat it as
        // best-effort so the recovery state machine keeps stepping toward COMPLETED instead
        // of bailing out on every iteration.
        if (return_code != ReturnCode::SUCCESS && is_in_emergency_recovery()) {
            PI_WARN("Group write failed during recovery for %s_%s; continuing best-effort",
                    model_.c_str(), id_.c_str());
            return_code = ReturnCode::SUCCESS;
        }
    }

    return return_code;
}

ReturnCode DeviceArm::init(const CommandLineArgs& cla, int argc, char** argv, std::shared_ptr<Topic> p_topic,
                           std::shared_ptr<Driver> p_driver) {
    ReturnCode return_code;

    return_code = Device::init(cla, argc, argv, p_topic, p_driver);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Base device initialization failed");
        return return_code;
    }

    if (!p_config_model_->values_.contains(p_config_model_->fn_joint_init_sequence)) {
        PI_ERROR("Joint initialization sequence array not found in model configuration file");
        return ReturnCode::INVALID_PARAM;
    }

    std::string sequence_str;
    for (int16_t id : p_config_model_->values_[p_config_model_->fn_joint_init_sequence]) {
        joint_init_sequence_.push_back(id);
        sequence_str += std::to_string(id) + ", ";
    }
    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "Joint initialization sequence: %s", sequence_str.c_str());

    bool spring_effect;
    return_code = p_config_individual_->get_field_value(p_config_individual_->values_,
                                                        p_config_individual_->fn_spring_effect, spring_effect);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "%s_%s: spring_effect=%d", model_.c_str(), id_.c_str(), spring_effect);

    bool gravity_compensation;
    return_code = p_config_individual_->get_field_value(
        p_config_individual_->values_, p_config_individual_->fn_gravity_compensation, gravity_compensation);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    if (gravity_compensation) {
        PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "%s_%s: gravity_compensation=1", model_.c_str(), id_.c_str());
    } else {
        PI_WARN("%s_%s: gravity_compensation=0 - leader mode will apply zero torque (check individual config)",
                model_.c_str(), id_.c_str());
    }

    return_code = enable_spring_effect(spring_effect);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to enable spring effect for %s_%s", model_.c_str(), id_.c_str());
        return return_code;
    }

    return_code = enable_gravity_compensation(gravity_compensation);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to enable gravity compensation for %s_%s", model_.c_str(), id_.c_str());
        return return_code;
    }

    return_code = Joint::new_joints(p_config_model_, p_config_individual_, joints_, this, p_driver_.get());
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to create joints");
        return return_code;
    }

    for (auto& p_joint : joints_) {
        joint_id_to_pointer_[p_joint->id_] = p_joint.get();
        max_vel_.push_back(p_joint->vel_max_);
    }

    dof_ = (int)joints_.size();
    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "DeviceArm %s_%s: DOF=%d", model_.c_str(), id_.c_str(), dof_);

    for (int i = 0; i < dof_; i++) {
        tele_pos_.push_back(0);
        tele_vel_.push_back(0);
        tele_tor_.push_back(0);

        follower_pos_.push_back(0);
        follower_vel_.push_back(0);
        follower_tor_.push_back(0);
        follower_temperature_.push_back(0);
        follower_idc_current_.push_back(0);

        follower_pos_filter_.push_back(Filter(FilterType::EMA));
        follower_vel_filter_.push_back(Filter(FilterType::EMA));
        follower_tor_filter_.push_back(Filter(FilterType::EMA));
    }

    servo_num_ = 0;
    for (auto& p_joint : joints_) {
        servo_num_ += p_joint->get_servo_num();
    }
    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "DeviceArm %s_%s: servo_num=%d", model_.c_str(), id_.c_str(),
            servo_num_);

    if (cla.effector_model != "" && cla.effector_model != p_config_model_->val_effector_type_none) {
        cla_effector_ = cla;
        cla_effector_.device_config_type = DeviceConfigType::EFFECTOR;
        cla_effector_.device_model = cla.effector_model;
        cla_effector_.device_id = cla.effector_id;

        return_code = config_effector_model_.init_config_model(cla_effector_);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to read effector model configuration");
            return return_code;
        }

        return_code = config_effector_individual_.init_config_individual(cla_effector_);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to read effector individual configuration");
            return return_code;
        }

        p_effector_.reset(new_device(config_effector_model_, config_effector_individual_, cla_effector_));
        if (!p_effector_) {
            PI_ERROR("Failed to create effector device");
            return ReturnCode::FAIL;
        }

        p_effector_->set_mode(mode_);
        static_cast<DeviceEffector*>(p_effector_.get())->set_arm(this);

        return_code = p_effector_->init(cla_effector_, argc, argv, p_topic_, p_driver_);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Effector initialization failed");
            return return_code;
        }

        dof_effector_ = p_effector_->get_dof();
        servo_num_effector_ = p_effector_->get_servo_num();
    } else {
        p_effector_.reset();
        dof_effector_ = 0;
        servo_num_effector_ = 0;
    }

    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "DeviceArm %s_%s: DOF_EFFECTOR=%d", model_.c_str(), id_.c_str(),
            dof_effector_);
    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "DeviceArm %s_%s: SERVO_NUM_EFFECTOR=%d", model_.c_str(), id_.c_str(),
            servo_num_effector_);

    dof_total_ = dof_ + dof_effector_;
    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "DeviceArm %s_%s: DOF_TOTAL=%d", model_.c_str(), id_.c_str(),
            dof_total_);

    servo_num_total_ = servo_num_ + servo_num_effector_;

    // Pre-allocate vectors to avoid allocations in control loop
    target_tor_.resize(dof_total_, 0.0f);
    current_motor_positions_.resize(dof_total_, 0.0f);
    slew_goal_positions_.resize(dof_, 0.0f);
    PI_INFO("DeviceArm", InfoLevel::HELPFUL_1, "DeviceArm %s_%s: SERVO_NUM_TOTAL=%d", model_.c_str(), id_.c_str(),
            servo_num_total_);

    // Leaders float on model gravity torque (i2rt behavior): without this the
    // gravity-compensation mode applies zero torque and the arm hangs limp,
    // drooping the moment the operator lets go. Arms whose algo lacks a
    // gravity model (base Algo no-op) still get zero torques.
    if (role_ == Role::LEADER && cla_.leader_gravity_compensation) {
        enabled_gravity_compensation_ = true;
        PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                "%s_%s: leader gravity compensation enabled (algo_type=%s)",
                model_.c_str(), id_.c_str(),
                p_algo_ ? "present" : "absent");
    } else if (role_ == Role::LEADER) {
        enabled_gravity_compensation_ = false;
        PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                "%s_%s: leader gravity compensation disabled",
                model_.c_str(), id_.c_str());
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DeviceArm::start(int baud_rate) {
    ReturnCode return_code = Device::start(baud_rate);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    auto fail_after_partial_start = [this](ReturnCode start_error) {
        ReturnCode cleanup_return_code = stop();
        if (cleanup_return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to clean up arm %s_%s after start error: cleanup error code=%d", model_.c_str(),
                     id_.c_str(), static_cast<int>(cleanup_return_code));
        }
        return start_error;
    };

    for (auto& p_joint : joints_) {
        return_code = p_joint->start_hardware();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to start hardware for joint in %s_%s", model_.c_str(), id_.c_str());
            return fail_after_partial_start(return_code);
        }
    }

    return_code = read_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to read hardware values during start");
        return fail_after_partial_start(return_code);
    }

    // Before any ready-move stepping, send a one-shot "hold at current position" command
    // for every joint whose cache is fresh. Without this, the first move command computed
    // by move_to_ready_position() would propagate the stale prev_target_pos_/current_pos
    // mismatch, causing an abrupt jump on the very first iteration. Joints whose verify
    // step fails or that are already on the failed list get marked failed and skipped --
    // the rest of startup continues so the arm can still reach ready on partial hardware.
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
        PI_ERROR("Failed to move arm to ready position");
        return fail_after_partial_start(return_code);
    }

    if (p_effector_) {
        return_code = p_effector_->start(baud_rate);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to move effector to ready position: %s_%s", model_.c_str(), id_.c_str());
            return fail_after_partial_start(return_code);
        }
    }

    return_code = write_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to write hardware values during start");
        return fail_after_partial_start(return_code);
    }

    return return_code;
}

ReturnCode DeviceArm::stop() {
    ReturnCode return_code = Device::stop();

    for (auto& p_joint : joints_) {
        ReturnCode joint_return_code = p_joint->stop_hardware();
        if (return_code == ReturnCode::SUCCESS && joint_return_code != ReturnCode::SUCCESS) {
            return_code = joint_return_code;
        }
    }

    if (p_effector_) {
        ReturnCode effector_return_code = p_effector_->stop();
        if (return_code == ReturnCode::SUCCESS) {
            return_code = effector_return_code;
        }
    }

    return return_code;
}

ReturnCode DeviceArm::sleep() {
    ReturnCode return_code = Device::sleep();
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (p_effector_) {
        return_code = p_effector_->sleep();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Effector sleep() failed for %s_%s", model_.c_str(), id_.c_str());
            return return_code;
        }
    }

    return return_code;
}

ReturnCode DeviceArm::step() {
    ReturnCode return_code = ReturnCode::SUCCESS;
    const bool in_recovery = is_in_emergency_recovery();

    // Step the arm before its effector so arm-side bus failures are reported
    // against the correct joint.
    return_code = Device::step();
    if (return_code != ReturnCode::SUCCESS) {
        // The arm's read path captured the failed joint id; let the main loop
        // propagate it through enter_emergency_recovery().
        return return_code;
    }

    // During emergency recovery we skip the effector entirely. Reads on the shared bus are
    // almost certainly failing (a dead servo on the bus is usually what triggered recovery),
    // and the effector's own Device::step() would early-return with
    // PI_ERROR("Failed to read hardware values") each iteration -- both spamming the UI log
    // and preventing the arm's own recovery state machine from ever advancing to MOVING_SLOW.
    // The arm's slow ready move is the only motion we want to attempt at this point.
    if (p_effector_ && !in_recovery) {
        return_code = p_effector_->step();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Effector step() failed for %s_%s", model_.c_str(), id_.c_str());
            // Propagate the effector's failed servo id into this arm so the
            // main loop's enter_emergency_recovery(rc, arm.last_failed_joint_id())
            // can name the real servo instead of the placeholder -1. The arm's
            // read succeeded above, so any
            // failure here is a genuine effector-side fault.
            if (last_failed_joint_id() < 0) {
                const int eff_failed = p_effector_->last_failed_joint_id();
                if (eff_failed >= 0) {
                    set_last_failed_joint_id(eff_failed);
                } else {
                    auto* p_eff = dynamic_cast<DeviceEffector*>(p_effector_.get());
                    if (p_eff != nullptr) {
                        const int probed = p_eff->probe_failed_servo_id();
                        if (probed >= 0) set_last_failed_joint_id(probed);
                    }
                }
            }
            return return_code;
        }
    } else if (p_effector_ && in_recovery && !logged_effector_skip_in_recovery_) {
        logged_effector_skip_in_recovery_ = true;
        PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                "[RECOVERY] %s_%s: skipping effector step (dead bus); arm will recover alone",
                model_.c_str(), id_.c_str());
    }

    return return_code;
}

ReturnCode DeviceArm::process_follower_msg(const MsgJoints& msg_joints) {
    if (msg_joints.joints_.size() != static_cast<size_t>(dof_total_)) {
        PI_ERROR("Joint count mismatch in %s_%s: arm joints=%d, message joints=%d",
                 model_.c_str(), id_.c_str(), dof_total_, (int)msg_joints.joints_.size());
        return ReturnCode::INVALID_PARAM;
    }

    int i = 0;
    for (auto& p_joint : joints_) {
        follower_pos_[i] = msg_joints.joints_[i].curr_pos_;
        p_joint->follower_pos_ = follower_pos_[i];
        p_joint->follower_pos_valid_ = true;
        follower_pos_filter_[i].update(follower_pos_[i]);

        follower_vel_[i] = msg_joints.joints_[i].curr_vel_;
        p_joint->follower_vel_ = follower_vel_[i];
        follower_vel_filter_[i].update(follower_vel_[i]);

        follower_tor_[i] = msg_joints.joints_[i].curr_tor_;
        p_joint->follower_tor_ = follower_tor_[i];
        follower_tor_filter_[i].update(follower_tor_[i]);

        follower_temperature_[i] = msg_joints.joints_[i].temperature_;
        p_joint->follower_temperature_ = follower_temperature_[i];

        follower_idc_current_[i] = msg_joints.joints_[i].idc_current_;
        p_joint->follower_idc_current_ = follower_idc_current_[i];
        i++;
    }

    if (p_effector_ != nullptr) {
        return p_effector_->process_follower_msg(msg_joints);
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DeviceArm::apply_action(const MsgJoints& msg) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (msg.joints_.size() != static_cast<size_t>(dof_total_)) {
        PI_ERROR("Joint count mismatch in %s_%s: arm joints=%d, message joints=%d",
                 model_.c_str(), id_.c_str(), dof_total_, (int)msg.joints_.size());
        return ReturnCode::INVALID_PARAM;
    }

    int i = 0;
    for (auto& p_joint : joints_) {
        tele_pos_[i] = msg.joints_[i].curr_pos_;
        p_joint->set_tele_pos_rad(tele_pos_[i]);

        tele_vel_[i] = msg.joints_[i].curr_vel_;
        p_joint->set_tele_vel_rad_sec(tele_vel_[i]);

        tele_tor_[i] = msg.joints_[i].curr_tor_;
        p_joint->set_tele_tor_nm(tele_tor_[i]);

        i++;
    }

    ///< @todo Assign a dedicated topic channel to effector for independent control
    if (p_effector_) {
        return_code = p_effector_->apply_action(msg);
    }

    return return_code;
}

ReturnCode DeviceArm::get_observation(MsgJoints& msg) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    for (auto& p_joint : joints_) {
        msg.add_joint_info(p_joint->get_pos_rad_relative(), p_joint->get_vel_rad_sec(), p_joint->get_tor_nm(),
                           p_joint->get_temperature(), p_joint->get_idc_current());
    }

    ///< @todo Assign a dedicated topic channel to effector for independent observation
    if (p_effector_) {
        return_code = p_effector_->get_observation(msg);
    }

    return return_code;
}

ReturnCode DeviceArm::move_to_ready_position() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    // Scoped to startup so explicit COMMAND_MOVE_TO_READY_POS and
    // emergency-recovery homing still execute with the flag set. Seed every
    // joint's target at its current pose: targets default to home, so without
    // this the first normal-operation step would command the very move this
    // flag exists to skip.
    if (cla_.dont_go_to_home_pos == true && is_startup_ready_phase()) {
        for (auto& p_joint : joints_) {
            const float current = p_joint->get_pos_rad_relative();
            p_joint->target_pos_ = current;
            p_joint->prev_target_pos_ = current;
        }
        // The follower's steady-state command source is the buffered leader
        // targets (tele_pos_), which default to home; rebase them onto the
        // current pose or the first post-ready step undoes the skip.
        clear_command_buffers_for_move_to_ready();
        PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                "%s_%s: Skip startup move to home position (dont_go_to_home_pos flag is set); holding current pose",
                model_.c_str(), id_.c_str());
        is_ready_ = true;
        is_ready_arm_ = true;
        return ReturnCode::SUCCESS;
    }

    // Unified velocity-bounded move-to-ready. Step size is identical for every joint and
    // every iteration: step = max_vel * loop_dt (see Device::ready_move_step_rad()). NORMAL
    // speed is used for startup / command-driven / MOVE_TO_READY_AND_STOP; ERROR speed is
    // used while emergency recovery is active. Time scales with distance from home -- there
    // is NO try_max-based premature ready timeout. The heartbeat watchdog (refreshed at the
    // end of every successful step()) catches genuine control-loop hangs.
    //
    // Always honour the failed-joint set regardless of emergency state. Communication-failure
    // joints marked during startup (DeviceArm::start() hold-at-current, read_hardware_values
    // per-joint failure) must be skipped from the very first ready iteration so the rest of
    // the arm can still reach home. Stuck joints that keep receiving commands but never move
    // are detected per-iter and treated as "within tolerance" for the exit criterion so the
    // ready movement cannot loop forever.
    const bool in_recovery = is_in_emergency_recovery();
    const std::unordered_set<int16_t> failed_ids = failed_joint_ids_snapshot();
    const float step = ready_move_step_rad();

    if (is_ready_arm_ == false) {
        if (init_count_ < 1) {
            reached_cnt_ = 0;
            float max_displacement = 0.0f;
            for (auto& p_joint : joints_) {
                const float start_pos = p_joint->get_pos_rad_relative();
                p_joint->target_pos_ = p_joint->get_home_pos_relative();
                p_joint->prev_target_pos_ = start_pos;
                p_joint->reset_stuck_counter();
                max_displacement = std::max(
                    max_displacement, static_cast<float>(fabs(p_joint->get_home_pos_relative() - start_pos)));
                PI_INFO("DeviceArm", InfoLevel::DETAIL_2,
                        "Joint %d initialized to move to ready position: home=%.3f, current=%.3f, "
                        "step=%.6f rad/loop", p_joint->id_,
                        p_joint->get_home_pos_relative(), start_pos, step);
            }
            ready_move_budget_init(max_displacement, step);
        }

        if (moving_mode_ == MovingMode::PARALLEL) {
            bool all_within_tolerance_or_stuck = true;
            int responsive_joint_count = 0;
            int failed_joint_count = 0;
            int stuck_joint_count = 0;
            int16_t first_stuck_joint_id = -1;
            float worst_displacement = 0.0f;
            int16_t worst_joint_sid = -1;

            for (auto& p_joint : joints_) {
                const int16_t sid = p_joint->reference_servo_id();

                // Best-effort: skip joints that have already failed. They cannot accept commands,
                // so the safest thing is to leave them at their last commanded value.
                // `failed_ids` is populated with hardware-level servo ids (see
                // read_hardware_values/mark_joint_failed_during_recovery), so look it up by
                // servo id, not joint id -- those id spaces are NOT the same for arms whose
                // joint ids are remapped from servo ids (e.g. ARX_L5 joint 3 = servo 4,
                // joint 4 = servo 5).
                if (sid >= 0 && failed_ids.count(sid) != 0) {
                    ++failed_joint_count;
                    continue;
                }
                ++responsive_joint_count;

                // Drive the per-joint stuck counter once per iteration; it updates the
                // measured-motion baseline and reports whether the joint has crossed the
                // stuck threshold while still outside the tolerance window (latched for
                // the rest of this ready move once crossed).
                const bool is_stuck = p_joint->update_and_check_stuck();

                float displacement = fabs(p_joint->target_pos_ - p_joint->get_pos_rad_relative());
                const bool outside_tolerance =
                    displacement > p_joint->pos_error_margin_ + READY_MOVE_TOLERANCE_HYSTERESIS_RAD;
                if (is_stuck && outside_tolerance) {
                    // Only joints that are stuck AND short of the target count toward the
                    // completion report; a latched joint that later freed up and arrived is
                    // not a failure. Report in the hardware (servo) id space the UI dialog
                    // and emergency_recovery payload use.
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

                // Command integration: generate next command from last commanded value (not from measured current).
                float cmd_pos = p_joint->prev_target_pos_;

                // Use measured current to detect overshoot.
                const float current_pos = p_joint->get_pos_rad_relative();
                const float delta_meas = p_joint->target_pos_ - current_pos;

                // If we overshot the target (measured position crossed over), flip direction by resetting
                // integration state to the measured position and continue toward target.
                float delta_cmd = p_joint->target_pos_ - cmd_pos;
                if (fabs(delta_meas) > p_joint->pos_error_margin_) {
                    if ((delta_meas > 0.0f && delta_cmd < 0.0f) || (delta_meas < 0.0f && delta_cmd > 0.0f)) {
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

                // Update last commanded target (integration state).
                p_joint->prev_target_pos_ = new_pos;

                PI_INFO(
                    "DeviceArm",
                    InfoLevel::DETAIL_2,
                    "Joint %d: Step to ready target %.3f, cmd=%.3f, current=%.3f, step=%.6f, safe_mode_derating=%.3f",
                    p_joint->id_,
                    p_joint->target_pos_,
                    new_pos,
                    current_pos,
                    step,
                    p_joint->safe_mode_derating_
                );


                return_code = p_joint->move(new_pos);
                if (return_code != ReturnCode::SUCCESS) {
                    PI_ERROR("Failed to move joint %d (servo %d) in move_to_ready_position()",
                             p_joint->id_, sid);
                    // Mark this joint failed so future iterations skip it. Keep working on
                    // the remaining joints regardless of recovery state -- partial progress
                    // is preferable to refusing to move any joint at all. mark / last id use
                    // the hardware servo id namespace (see comment above the failed_ids
                    // lookup) for consistency with the UI dialog.
                    if (sid >= 0) {
                        mark_joint_failed_during_recovery(sid);
                        set_last_failed_joint_id(sid);
                    }
                    is_normal_status_ = false;
                    continue;
                }
            }

            const bool budget_exhausted = ready_move_budget_spend();
            if ((ready_move_budget_used_ % READY_MOVE_PROGRESS_LOG_PERIOD_ITERS) == 0
                && worst_joint_sid >= 0) {
                // Periodic laggard visibility: names the joint holding up the ready move while
                // it is still running, so a slow/oscillating joint is identifiable from the log
                // instead of requiring bus-counter forensics (issue #5).
                PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                        "%s_%s: ready move in progress (iter %ld/%ld): worst joint servo id=%d, "
                        "displacement=%.3f rad, %d joint(s) stuck",
                        model_.c_str(), id_.c_str(), ready_move_budget_used_, ready_move_budget_total_,
                        static_cast<int>(worst_joint_sid), worst_displacement, stuck_joint_count);
            }

            if (responsive_joint_count == 0) {
                // No joints can be moved anymore; declare ready so the main loop can exit.
                PI_WARN("%s_%s: all %d joints unresponsive during ready movement; completing",
                        model_.c_str(), id_.c_str(), failed_joint_count);
                is_ready_arm_ = true;
                init_count_ = 0;
            } else if (all_within_tolerance_or_stuck) {
                reached_cnt_++;
                if (reached_cnt_ > DEVICE_ARRIVED_CONFIRM_CNT) {
                    is_ready_arm_ = true;
                    init_count_ = 0;
                }
            } else if (budget_exhausted) {
                // A joint oscillating past the stuck-motion floor (e.g. gravity sag fighting the
                // position hold) can reset both the stuck counter and the arrival confirmation
                // forever. The distance-derived budget is the hard backstop: complete
                // best-effort and name the laggard instead of looping at full control rate.
                PI_ERROR("%s_%s: ready movement exceeded its iteration budget (%ld iters); "
                         "completing best-effort (worst joint servo id=%d, displacement=%.3f rad)",
                         model_.c_str(), id_.c_str(), ready_move_budget_total_,
                         static_cast<int>(worst_joint_sid), worst_displacement);
                is_normal_status_ = false;
                if (worst_joint_sid >= 0) set_last_failed_joint_id(worst_joint_sid);
                is_ready_arm_ = true;
                init_count_ = 0;
            } else {
                is_ready_arm_ = false;
            }

            if (is_ready_arm_ == true) {
                if (stuck_joint_count > 0) {
                    is_normal_status_ = false;
                    if (first_stuck_joint_id >= 0) {
                        set_last_failed_joint_id(static_cast<int>(first_stuck_joint_id));
                    }
                    PI_ERROR("%s_%s: ready movement completed with %d stuck joint(s) "
                             "(first stuck id=%d); reporting ready failure",
                             model_.c_str(), id_.c_str(), stuck_joint_count,
                             static_cast<int>(first_stuck_joint_id));
                }
                if (in_recovery) {
                    PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                            "[RECOVERY] %s_%s: arm joints reached ready position; finalizing recovery",
                            model_.c_str(), id_.c_str());
                    // Tell the base Device to publish SHUTDOWN_AFTER_ERROR and stop the topic so
                    // the main loop exits cleanly.
                    mark_emergency_recovery_completed();
                } else {
                    PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0, "%s_%s completed moving arm joints to home position",
                            model_.c_str(), id_.c_str());
                }
            }

        } else if (moving_mode_ == MovingMode::SEQUENTIAL) {
            if (moving_joint_index_ >= (int)joints_.size()) {
                PI_ERROR("Invalid joint index for sequential movement: %d (max: %d)", moving_joint_index_,
                         (int)joints_.size());
                return ReturnCode::INVALID_PARAM;
            }

            if (joint_init_sequence_.size() != joints_.size()) {
                PI_ERROR("Joint initialization sequence size mismatch: sequence=%d, joints=%d",
                         (int)joint_init_sequence_.size(), (int)joints_.size());
                return ReturnCode::NOT_INITIALIZED;
            }

            uint16_t joint_id_to_move = joint_init_sequence_[moving_joint_index_];
            Joint* p_joint = joint_id_to_pointer_[joint_id_to_move];

            if (p_joint == nullptr) {
                PI_ERROR("Invalid joint ID in initialization sequence: %d", joint_id_to_move);
                return ReturnCode::INVALID_PARAM;
            }

            const int16_t sid = p_joint->reference_servo_id();

            // Skip ahead past failed joints so SEQUENTIAL doesn't block forever on one.
            // Lookup must use the servo id (see PARALLEL branch above for the namespace
            // explanation: joint ids and servo ids do not match for arms like ARX_L5).
            if (sid >= 0 && failed_ids.count(sid) != 0) {
                reached_cnt_ = 0;
                init_count_ = 0;
                moving_joint_index_++;
                if (moving_joint_index_ >= (int)joint_init_sequence_.size()) {
                    moving_joint_index_ = 0;
                    is_ready_arm_ = true;
                    PI_WARN("%s_%s: completed sequential ready movement with one or more failed joints",
                            model_.c_str(), id_.c_str());
                }
            } else {
                const bool is_stuck = p_joint->update_and_check_stuck();

                // Same distance-derived backstop as PARALLEL (budget re-initialises per joint
                // because init_count_ resets on every advance): an oscillating joint advances
                // with an error instead of blocking the sequence forever.
                if (ready_move_budget_spend()) {
                    is_normal_status_ = false;
                    if (sid >= 0) set_last_failed_joint_id(sid);
                    PI_ERROR("%s_%s: joint id=%d (servo %d) exceeded the sequential ready-move "
                             "budget (%ld iters, displacement=%.3f rad); advancing",
                             model_.c_str(), id_.c_str(), p_joint->id_, sid, ready_move_budget_total_,
                             fabs(p_joint->target_pos_ - p_joint->get_pos_rad_relative()));
                    reached_cnt_ = 0;
                    init_count_ = 0;
                    moving_joint_index_++;
                    if (moving_joint_index_ >= (int)joint_init_sequence_.size()) {
                        moving_joint_index_ = 0;
                        is_ready_arm_ = true;
                        PI_WARN("%s_%s: completed sequential ready movement after budget exhaustion on joint id=%d",
                                model_.c_str(), id_.c_str(), p_joint->id_);
                    }
                    return ReturnCode::SUCCESS;
                }

                return_code = move(p_joint, p_joint->target_pos_, 0, p_joint->safe_mode_derating_);
                if (return_code != ReturnCode::SUCCESS) {
                    PI_ERROR("Failed to move joint %d (servo %d) to target position in move_to_ready_position()",
                             joint_id_to_move, sid);
                    // Mirror PARALLEL: mark failed (by servo id, see PARALLEL branch) and
                    // advance to the next joint instead of aborting the whole start path.
                    if (sid >= 0) {
                        mark_joint_failed_during_recovery(sid);
                        set_last_failed_joint_id(sid);
                    }
                    is_normal_status_ = false;
                    reached_cnt_ = 0;
                    init_count_ = 0;
                    moving_joint_index_++;
                    if (moving_joint_index_ >= (int)joint_init_sequence_.size()) {
                        moving_joint_index_ = 0;
                        is_ready_arm_ = true;
                        PI_WARN("%s_%s: completed sequential ready movement after move failure on joint id=%d",
                                model_.c_str(), id_.c_str(), p_joint->id_);
                    }
                    return ReturnCode::SUCCESS;
                }

                float displacement = fabs(p_joint->target_pos_ - p_joint->get_pos_rad_relative());
                const bool reached_or_stuck =
                    (displacement <= p_joint->pos_error_margin_ + READY_MOVE_TOLERANCE_HYSTERESIS_RAD) || is_stuck;
                if (reached_or_stuck) {
                    reached_cnt_++;
                    if (reached_cnt_ > DEVICE_ARRIVED_CONFIRM_CNT) {
                        if (is_stuck && displacement >= p_joint->pos_error_margin_) {
                            // Report ready partial-fail caused by this stuck joint. The UI
                            // dialog reads the servo id, not the abstract joint id.
                            is_normal_status_ = false;
                            if (sid >= 0) set_last_failed_joint_id(sid);
                            PI_ERROR("%s_%s: joint id=%d (servo %d) stuck during sequential ready movement; advancing",
                                     model_.c_str(), id_.c_str(), p_joint->id_, sid);
                        }
                        reached_cnt_ = 0;
                        init_count_ = 0;
                        moving_joint_index_++;
                        if (moving_joint_index_ >= (int)joint_init_sequence_.size()) {
                            moving_joint_index_ = 0;
                            is_ready_arm_ = true;
                            PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0,
                                    "%s_%s completed moving arm joints to home position", model_.c_str(), id_.c_str());
                        }
                    }
                }
            }
        } else {
            PI_ERROR("Unsupported moving mode: %d", (int)moving_mode_);
            return ReturnCode::INVALID_PARAM;
        }
    } else {
        for (auto& p_joint : joints_) {
            return_code = move(p_joint.get(), p_joint->get_pos_rad_relative(), 0, p_joint->safe_mode_derating_);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to maintain joint position in move_to_ready_position()");
                return return_code;
            }
        }
    }

    if (is_ready_arm_ == true) {
        if (p_effector_ == nullptr) {
            is_ready_ = true;
            PI_WARN("No effector attached to arm %s_%s", model_.c_str(), id_.c_str());
        } else if (in_recovery) {
            // The effector is not being stepped during recovery (see DeviceArm::step), so its
            // own is_ready_ flag will never flip. Mark it ready unconditionally so the recovery
            // state machine can complete -- we've already done everything we can for the arm.
            p_effector_->force_to_be_ready();
            is_ready_ = true;
        } else {
            if (p_effector_->is_ready() == true) {
                is_ready_ = true;
                PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0, "%s_%s: Effector is also ready", model_.c_str(),
                        id_.c_str());
            }
        }

        if (is_ready_ == true) {
            PI_INFO("DeviceArm", InfoLevel::ESSENTIAL_0, "%s_%s: All arm and effector joints moved to home position",
                    model_.c_str(), id_.c_str());
        }
    }

    init_count_++;

    return return_code;
}

ReturnCode DeviceArm::operate_as_leader() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (mode_ == DeviceMode::NORMAL) {
        prof_time_t step_start_time = Profile::get_time_now();

        // Gravity feedforward is the leader's float: it runs in BOTH leader
        // modes. Plain gravity-compensation mode applies it alone (the arm
        // stays where the operator leaves it); bilateral force-feedback mode
        // layers the follower-tracking position spring ON TOP of it -- without
        // the feedforward the operator carries the arm's full weight the
        // moment teleop engages, and a released leader drags the follower down
        // with it. Arms whose algo has no gravity model (base Algo no-op)
        // keep zero torques here.
        std::fill(target_tor_.begin(), target_tor_.end(), 0.0f);

        if (enabled_gravity_compensation_ == true) {
            if (!p_algo_) {
                PI_ERROR("Algorithm handler is not initialized in operate_as_leader()");
                return ReturnCode::NOT_INITIALIZED;
            }

            // Reuse pre-allocated vector
            int i = 0;
            for (auto& p_joint : joints_) {
                current_motor_positions_[i++] = p_joint->get_pos_rad_relative() * p_joint->get_dir_invert();
            }

            return_code = p_algo_->gravity_compensation(current_motor_positions_, target_tor_);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Gravity compensation algorithm execution failed for %s_%s", model_.c_str(), id_.c_str());
                return return_code;
            }

            // Same empirical per-joint fit the follower path applies.
            i = 0;
            for (auto& p_joint : joints_) {
                target_tor_[i] *= p_joint->gravity_comp_factor_;
                i++;
            }

        }

        if (is_force_feedback_enabled()) {
            // Bilateral: weak position spring toward the paired follower plus
            // the gravity feedforward computed above. Joint::move(pos, 0, tor)
            // is called directly because Device::move() drops the torque
            // argument for the leader's NONE planning type. The feedforward is
            // multiplied by torq_rescale_ to match the float path exactly:
            // disengaged float goes through Servo::apply_torque, which rescales
            // internally, while the 3-arg move does not -- without this the
            // leader's gravity support drops 10-20% per joint the moment teleop
            // engages (the arm gets heavier in the operator's hand and a
            // released leader sags, dragging the follower down with it).
            int i = 0;
            for (auto& p_joint : joints_) {
                if (!p_joint->follower_pos_valid_) {
                    // No follower state has arrived yet (slow joiner): gravity
                    // float only. Springing toward the zero-initialized
                    // follower_pos_ would pull the leader toward the home pose.
                    // apply_torque matches the disengaged float path exactly
                    // (it rescales by torq_rescale_ internally).
                    return_code = p_joint->apply_torque(target_tor_[i]);
                } else {
                    const float follower_pos = p_joint->get_follower_pos_rad();
                    p_joint->adjusted_target_pos_ = follower_pos;
                    return_code = p_joint->move(follower_pos, 0, target_tor_[i] * p_joint->torq_rescale_);
                }
                if (return_code != ReturnCode::SUCCESS) {
                    PI_ERROR("Failed to move joint %d to follower's position in operate_as_leader() for %s_%s",
                             p_joint->id_, model_.c_str(), id_.c_str());
                    return return_code;
                }
                i++;
            }
        } else {
            if (enabled_spring_effect_ == true) {
                std::vector<Joint*> joint_ptrs;
                for (auto& p_joint : joints_) {
                    joint_ptrs.push_back(p_joint.get());
                }
                return_code = p_algo_->check_stability_control(joint_ptrs, step_start_time);
                if (return_code != ReturnCode::SUCCESS) {
                    PI_ERROR("Failed to check stability control in operate_as_leader() for %s_%s", model_.c_str(),
                             id_.c_str());
                    return return_code;
                }

                int i = 0;
                for (auto& p_joint : joints_) {
                    return_code = p_joint->apply_spring_force(target_tor_[i], enabled_spring_effect_);
                    if (return_code != ReturnCode::SUCCESS) {
                        PI_ERROR("Failed to apply spring force to joint %d in %s_%s", p_joint->id_, model_.c_str(),
                                 id_.c_str());
                        return return_code;
                    }
                    i++;
                }
            } else {
                int i = 0;
                for (auto& p_joint : joints_) {
                    return_code = p_joint->apply_torque(target_tor_[i]);
                    if (return_code != ReturnCode::SUCCESS) {
                        PI_ERROR("Failed to apply torque to joint %d in %s_%s", p_joint->id_, model_.c_str(),
                                 id_.c_str());
                        return return_code;
                    }
                    i++;
                }
            }
        }
    }

    return return_code;
}

ReturnCode DeviceArm::operate_as_follower() {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (!p_algo_ && planning_type_ == TrajectoryPlanningType::SLEW_POS_GRAVITY) {
        PI_ERROR("Algorithm handler is not initialized in operate_as_follower()");
        return ReturnCode::NOT_INITIALIZED;
    }

    // Reuse pre-allocated vector (reset to zero)
    std::fill(target_tor_.begin(), target_tor_.end(), 0.0f);

    if (planning_type_ == TrajectoryPlanningType::SLEW_POS_GRAVITY) {
        // Reuse pre-allocated vector
        int i = 0;
        for (auto& p_joint : joints_) {
            current_motor_positions_[i++] = p_joint->get_pos_rad_relative() * p_joint->get_dir_invert();
        }

        return_code = p_algo_->gravity_compensation(current_motor_positions_, target_tor_);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Gravity compensation algorithm execution failed for %s_%s", model_.c_str(), id_.c_str());
            return return_code;
        }

        // Empirical per-joint fit on top of the model torques (matches the factors
        // i2rt shipped for the same hardware). Defaults to 1.0 when unconfigured.
        i = 0;
        for (auto& p_joint : joints_) {
            target_tor_[i] *= p_joint->gravity_comp_factor_;
            if (planning_type_ == TrajectoryPlanningType::SLEW_POS_GRAVITY) {
                target_tor_[i] -= p_joint->follow_viscous_damping_ * p_joint->get_vel_rad_sec();
            }
            i++;
        }
    }

    float slew_scale = 1.0f;
    if (planning_type_ == TrajectoryPlanningType::SLEW_POS_GRAVITY) {
        if (slew_goal_positions_.size() != joints_.size() || control_frequency_ <= 0) {
            PI_ERROR("Invalid synchronized slew state in operate_as_follower() for %s_%s", model_.c_str(),
                     id_.c_str());
            return ReturnCode::NOT_INITIALIZED;
        }

        const float dt = 1.0f / static_cast<float>(control_frequency_);
        int slew_i = 0;
        for (auto& p_joint : joints_) {
            const float goal = p_joint->get_safe_tele_pos_rad();
            slew_goal_positions_[slew_i] = goal;
            const float delta = std::fabs(goal - p_joint->prev_target_pos_);
            if (delta > 0.0f) {
                slew_scale = std::min(slew_scale, p_joint->follow_vel_max_ * dt / delta);
            }
            slew_i++;
        }
    }

    int i = 0;
    for (auto& p_joint : joints_) {
        if (planning_type_ == TrajectoryPlanningType::SLEW_POS_GRAVITY) {
            const float previous = p_joint->prev_target_pos_;
            const float cmd = previous + slew_scale * (slew_goal_positions_[i] - previous);
            return_code = move(p_joint.get(), cmd, target_tor_[i], 1.0);
        } else {
            return_code =
                move(p_joint.get(), p_joint->get_safe_tele_pos_rad(), target_tor_[i], 1.0);
        }
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to move joint %d in operate_as_follower() for %s_%s",
                     p_joint->id_, model_.c_str(), id_.c_str());
            return return_code;
        }
        i++;
    }

    return return_code;
}
