/*!
 * @file pi_device.cpp
 * @brief Implementation of the Device base class and factory method for robot device management.
 */

#include <vector>
#include <mutex>
#include <cmath>

#include "pi_device.hpp"
#include "pi_device_arm_arx.hpp"
#include "pi_device_config.hpp"
#include "pi_device_effector_arx.hpp"
#include "pi_info.hpp"

Device::Device(const CommandLineArgs& cla)
    : model_(cla.device_model), id_(cla.device_id), moving_mode_(cla.moving_mode), msg_type_(cla.msg_type), cla_(cla) {
    enabled_force_feedback_ = cla.force_feedback >= 0.0f;
}

Device::~Device() = default;

ReturnCode Device::set_runtime_force_feedback(bool enabled, float gain) {
    if (role_ != Role::LEADER) {
        return ReturnCode::NOT_SUPPORTED;
    }
    if (enabled && gain < 0.0f) {
        return ReturnCode::INVALID_PARAM;
    }
    enabled_force_feedback_ = enabled;
    cla_.force_feedback = enabled ? gain : -1.0f;
    return set_control_mode(Role::LEADER, ControlModeIntent::NORMAL_OPERATION);
}

ReturnCode Device::set_runtime_force_feedback_gain(float gain) {
    if (role_ != Role::LEADER) {
        return ReturnCode::NOT_SUPPORTED;
    }
    if (gain < 0.0f) {
        return ReturnCode::INVALID_PARAM;
    }
    cla_.force_feedback = gain;
    return ReturnCode::SUCCESS;
}

ReturnCode Device::runtime_hold() {
    if (rejects_direct_commands()) {
        return ReturnCode::BUSY;
    }
    clear_command_buffers_for_move_to_ready();
    return ReturnCode::SUCCESS;
}

bool Device::is_running() {
    if (p_topic_) {
        return p_topic_->is_running();
    }
    return false;
}

ReturnCode Device::start(int baud_rate) {
    if (is_driver_created_by_this_) {
        if (p_driver_ == nullptr) {
            PI_ERROR("Device driver is not initialized in start()");
            return ReturnCode::NOT_INITIALIZED;
        }

        ReturnCode return_code = p_driver_->open(baud_rate);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to open device driver");
            return return_code;
        }
    }
    return ReturnCode::SUCCESS;
}

ReturnCode Device::arm_comm_loss_protection() {
    if (p_driver_ == nullptr) {
        return ReturnCode::SUCCESS;
    }
    return p_driver_->arm_comm_loss_protection();
}

ReturnCode Device::step() {
    ReturnCode return_code = read_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        // During emergency recovery a read failure is expected for the joints that originally
        // dropped out. Concrete devices (e.g. DeviceArm) fall back to per-joint reads internally,
        // so we still want to keep stepping. For NON-recovery paths, propagate the error so the
        // main loop can trigger emergency recovery.
        if (!is_in_emergency_recovery()) {
            PI_ERROR("Failed to read hardware values");
            return return_code;
        }
        PI_WARN("Failed to read hardware values during emergency recovery; continuing best-effort");
    }

    // Handle COMMAND_MOVE_TO_READY_POS state machine.
    //
    // We intentionally implement this here (device-level) rather than mutating each device's persistent
    // control configuration, so restoration is deterministic and resilient to errors/timeouts:
    // - READY_MOVE_OVERRIDE forces a safe, position-based behavior to execute move_to_ready_position()
    //   from the *current* pose.
    // - When done, we restore NORMAL_OPERATION based on the device's actual role + configured settings.
    if (move_to_ready_cmd_state_ == MoveToReadyCmdState::REQUESTED) {
        // Clear any buffered leader commands / interpolation segments so we don't \"jump\" after returning
        // to normal operation.
        clear_command_buffers_for_move_to_ready();

        // Switch to safe mode for ready move and restart ready state machines.
        return_code = set_control_mode(Role::FOLLOWER, ControlModeIntent::READY_MOVE_OVERRIDE);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to set control mode for READY_MOVE_OVERRIDE");
            // Still proceed to reset ready flags so the system can recover via normal ready logic.
        }
        reset_ready_state_for_move_to_ready();
        move_to_ready_cmd_state_ = MoveToReadyCmdState::MOVING;
    }

    // Handle emergency recovery state machine. The first call (REQUESTED) switches the device into
    // a follower-like position-based mode (so leader devices in torque mode can also move to ready),
    // clears stale command buffers, and reuses the existing move-to-ready state machine but at the
    // ERROR velocity (cla.move_to_ready_vel_rad_s_error). After this transition the rest of step()
    // runs normally; concrete devices skip failed joints via `failed_joint_ids_snapshot()`.
    bool just_entered_recovery = false;
    {
        std::unique_lock<std::mutex> lock(emergency_mutex_);
        if (emergency_state_ == EmergencyRecoveryState::REQUESTED) {
            PI_WARN("Device %s_%s: entering emergency recovery (cause=%d, joint=%d, vel_error=%.3f rad/s, timeout=%d ms)",
                    model_.c_str(), id_.c_str(), emergency_cause_, emergency_failed_joint_id_,
                    cla_.move_to_ready_vel_rad_s_error, cla_.emergency_shutdown_timeout_ms);
            just_entered_recovery = true;
        }
    }

    if (just_entered_recovery) {
        // Call out to virtual methods + topic publish without holding emergency_mutex_.
        clear_command_buffers_for_move_to_ready();
        ReturnCode mode_rc = set_control_mode(Role::FOLLOWER, ControlModeIntent::READY_MOVE_OVERRIDE);
        if (mode_rc != ReturnCode::SUCCESS) {
            PI_ERROR("[RECOVERY] %s_%s: set_control_mode(READY_MOVE_OVERRIDE) failed (rc=%d); continuing best-effort",
                     model_.c_str(), id_.c_str(), static_cast<int>(mode_rc));
        }
        reset_ready_state_for_move_to_ready();
        {
            std::lock_guard<std::mutex> lock(emergency_mutex_);
            emergency_start_time_ = std::chrono::steady_clock::now();
            last_step_heartbeat_ = emergency_start_time_;  // First heartbeat: now.
            emergency_state_ = EmergencyRecoveryState::MOVING_SLOW;
            // slow_move_active_ remains true until COMPLETED / TIMED_OUT.
        }
        PI_INFO("Device", InfoLevel::ESSENTIAL_0,
                "[RECOVERY] %s_%s: MOVING_SLOW (slow ready move begins)", model_.c_str(), id_.c_str());
    }

    EmergencyRecoveryState state_now = EmergencyRecoveryState::NONE;
    float recovery_progress = 0.0f;
    {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        if (emergency_state_ == EmergencyRecoveryState::MOVING_SLOW) {
            check_emergency_timeout_locked();
        }
        state_now = emergency_state_;
    }
    if (state_now == EmergencyRecoveryState::MOVING_SLOW) {
        // Progress estimate is driven by the concrete device's actual ready-move progress
        // (e.g. DeviceArm averages per-joint remaining displacement). It's UI-only and
        // intentionally decoupled from the heartbeat watchdog -- a slow but progressing
        // recovery should NOT be forced to terminate just because we're "out of time".
        recovery_progress = get_ready_move_completion_ratio();
        if (recovery_progress < 0.0f) recovery_progress = 0.0f;
        if (recovery_progress > 1.0f) recovery_progress = 1.0f;
    }

    // If recovery timed out, publish SHUTDOWN_AFTER_ERROR and stop the topic so the loop exits.
    // This must happen outside emergency_mutex_ since mark_emergency_recovery_completed re-locks.
    if (state_now == EmergencyRecoveryState::TIMED_OUT) {
        PI_ERROR("[RECOVERY] %s_%s: heartbeat-staleness watchdog tripped (limit=%d ms); force-parking",
                 model_.c_str(), id_.c_str(), cla_.emergency_shutdown_timeout_ms);
        mark_emergency_recovery_completed();
        return ReturnCode::SUCCESS;
    }

    // Throttled publish of recovery progress so UI dialogs can render a progress bar without
    // overwhelming the ZMQ topic at full control-loop rate. RECOVERY_IN_PROGRESS is published
    // only during emergency recovery for back-compat with the existing DeviceErrorDialog.
    if (state_now == EmergencyRecoveryState::MOVING_SLOW) {
        const auto now_progress = std::chrono::steady_clock::now();
        const auto since_last_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       now_progress - last_recovery_progress_publish_)
                                       .count();
        if (since_last_ms >= EMERGENCY_PROGRESS_PUBLISH_INTERVAL_MS) {
            last_recovery_progress_publish_ = now_progress;
            publish_recovery_progress(recovery_progress);
        }
    }

    // Unified READY_MOVE_IN_PROGRESS heartbeat: published every
    // EMERGENCY_PROGRESS_PUBLISH_INTERVAL_MS while *any* move-to-ready is active (startup,
    // command-driven, or emergency recovery). Python uses this to wait unbounded for
    // DEVICE_INFO_READY_NOW without timing out on a slow-but-progressing move.
    const bool startup_ready_in_progress = (is_ready_ == false &&
                                            move_to_ready_cmd_state_ == MoveToReadyCmdState::IDLE &&
                                            state_now == EmergencyRecoveryState::NONE);
    const bool command_ready_in_progress = (move_to_ready_cmd_state_ == MoveToReadyCmdState::MOVING &&
                                            state_now == EmergencyRecoveryState::NONE);
    const bool emergency_ready_in_progress = (state_now == EmergencyRecoveryState::MOVING_SLOW);
    if (startup_ready_in_progress || command_ready_in_progress || emergency_ready_in_progress) {
        const auto now_progress = std::chrono::steady_clock::now();
        const auto since_last_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       now_progress - last_ready_move_progress_publish_)
                                       .count();
        if (since_last_ms >= EMERGENCY_PROGRESS_PUBLISH_INTERVAL_MS) {
            last_ready_move_progress_publish_ = now_progress;
            int source = READY_MOVE_SOURCE_STARTUP;
            bool is_error = false;
            if (emergency_ready_in_progress) {
                source = READY_MOVE_SOURCE_EMERGENCY;
                is_error = true;
            } else if (command_ready_in_progress) {
                source = READY_MOVE_SOURCE_COMMAND;
            }
            float progress_for_publish = emergency_ready_in_progress ? recovery_progress
                                                                     : get_ready_move_completion_ratio();
            publish_ready_move_progress(source, is_error, progress_for_publish);
        }
    }

    if (is_ready_ == false) {
        const bool was_ready = is_ready_;
        return_code = move_to_ready_position();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to move device to ready position");
            return return_code;
        }

        // On initial startup, the device reaches ready via move_to_ready_position() but may still be
        // left in a follower-like (position-based) mode. Gravity compensation
        // requires the correct torque-capable mode. Enter NORMAL_OPERATION once
        // when we first become ready (do not interfere with commanded move-to-ready state machine).
        if (!was_ready && is_ready_) {
            if (move_to_ready_cmd_state_ == MoveToReadyCmdState::IDLE) {
                ReturnCode rc_mode = set_control_mode(role_, ControlModeIntent::NORMAL_OPERATION);
                if (rc_mode != ReturnCode::SUCCESS) {
                    PI_ERROR("Failed to set control mode for NORMAL_OPERATION after initial ready");
                }
            } else if (move_to_ready_cmd_state_ == MoveToReadyCmdState::MOVING) {
                // The main loop publishes READY_NOW immediately after this step. Attach the
                // lifecycle request id to that first completion instead of a later re-announcement.
                completed_move_to_ready_request_id_ = move_to_ready_request_id_;
                move_to_ready_request_id_ = 0;
            }
        }
    } else {
        if (move_to_ready_cmd_state_ == MoveToReadyCmdState::MOVING) {
            // We have just reached ready after a commanded re-entry; restore normal operation modes.
            move_to_ready_cmd_state_ = MoveToReadyCmdState::RESTORE;
            ReturnCode rc_restore = set_control_mode(role_, ControlModeIntent::NORMAL_OPERATION);
            if (rc_restore != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to restore control mode after move-to-ready");
                // Keep running; device is ready and normal operation can still proceed.
            }
            // Clear again after restoring modes so follower holds at home until a new leader command arrives.
            clear_command_buffers_for_move_to_ready();
            move_to_ready_cmd_state_ = MoveToReadyCmdState::IDLE;

            if (stop_after_ready_ && p_topic_) {
                PI_INFO("Device", InfoLevel::ESSENTIAL_0, "Move-to-ready complete; stopping (%s_%s)",
                        get_model().c_str(), get_id().c_str());
                p_topic_->stop();
            }
        }

        if (role_ == Role::LEADER) {
            return_code = operate_as_leader();
        } else if (role_ == Role::FOLLOWER) {
            return_code = operate_as_follower();
        } else {
            PI_ERROR("Unsupported device role: %d", static_cast<int>(role_));
            return ReturnCode::NOT_SUPPORTED;
        }
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to operate device for role %d", static_cast<int>(role_));
            return return_code;
        }
    }

    return_code = write_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to write hardware values");
        return return_code;
    }

    if (is_topic_created_by_this_) {
        if (p_topic_ == nullptr) {
            PI_ERROR("Device topic is not initialized in step()");
            return ReturnCode::NOT_INITIALIZED;
        }

        if (role_ == Role::LEADER) {
            // A leader inside emergency recovery (or force-parked after it) is
            // driving ITSELF -- its measured positions are the recovery
            // trajectory, not operator intent. The leader's state topic doubles
            // as the paired follower's live-command stream, so publishing here
            // makes the follower faithfully replay the leader's self-homing
            // sweep across the workspace with whatever it is gripping. Going
            // silent instead freezes the follower at its last target (the
            // interpolator holds) and surfaces as a stale leader state to the
            // operator's client -- both the safe outcome.
            if (is_in_emergency_recovery()) {
                if (!logged_recovery_publish_gate_) {
                    logged_recovery_publish_gate_ = true;
                    PI_INFO("Device", InfoLevel::ESSENTIAL_0,
                            "[RECOVERY] %s_%s: leader observation publishing gated for the rest of "
                            "this session (followers must not mirror the recovery move)",
                            model_.c_str(), id_.c_str());
                }
            } else {
                if (msg_type_ == MsgType::JOINT_INFO) {
                    MsgJoints msg;
                    return_code = get_observation(msg);
                    if (return_code != ReturnCode::SUCCESS) {
                        PI_ERROR("Failed to get leader joint observation");
                        return return_code;
                    }
                    msg.measured_idc_current_ = 0.0f;
                    return_code = p_topic_->publish(msg);
                    if (return_code != ReturnCode::SUCCESS) {
                        PI_ERROR("Failed to publish leader joint observation");
                        return return_code;
                    }
                } else {
                    PI_ERROR("Invalid message type for teleoperation in %s_%s: %d", model_.c_str(), id_.c_str(),
                             (int)msg_type_);
                    return ReturnCode::NOT_SUPPORTED;
                }
            }
        } else if (role_ == Role::FOLLOWER) {
            MsgJoints msg;
            return_code = get_observation(msg);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to get follower joint observation");
                return return_code;
            }
            msg.measured_idc_current_ = 0.0f;
            return_code = p_topic_->publish(msg);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to publish follower joint observation");
                return return_code;
            }
        }

        p_topic_->step();

        // Lifecycle commands (including heartbeats) were just dispatched by
        // p_topic_->step(), so the staleness check sees the freshest possible
        // timestamp.
        check_client_heartbeat_watchdog();
    }

    // Heartbeat refresh: any time step() reaches this point, the control loop is still making
    // forward progress. We always refresh last_step_heartbeat_ so the heartbeat-staleness
    // check (used by the unified READY_MOVE / emergency-recovery watchdog) only fires when the
    // loop has actually hung. Concrete devices also rely on this for the non-emergency ready-
    // move path: a heavy arm doing a slow startup ready move must not be killed by a stale
    // heartbeat just because we used to refresh only in MOVING_SLOW.
    {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        last_step_heartbeat_ = std::chrono::steady_clock::now();
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Device::sleep() {
    if (is_topic_created_by_this_ == true) {
        if (p_topic_ == nullptr) {
            PI_ERROR("Device topic is not initialized in sleep()");
            return ReturnCode::NOT_INITIALIZED;
        }

        ReturnCode return_code = p_topic_->sleep();
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Topic sleep() failed");
            return return_code;
        }
    }
    return ReturnCode::SUCCESS;
}

ReturnCode Device::stop() {
    PI_INFO("Device", InfoLevel::ESSENTIAL_0, "Stopping device %s_%s", model_.c_str(), id_.c_str());

    ReturnCode park_return_code = park_safely();
    if (park_return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to park device %s_%s safely", model_.c_str(), id_.c_str());
    } else {
        PI_INFO("Device", InfoLevel::ESSENTIAL_0, "Device %s_%s parked safely", model_.c_str(), id_.c_str());
    }

    ReturnCode first_error = park_return_code;

    if (is_topic_created_by_this_) {
        if (p_topic_ == nullptr) {
            PI_ERROR("Device topic is not initialized");
            if (first_error == ReturnCode::SUCCESS) {
                first_error = ReturnCode::NOT_INITIALIZED;
            }
        } else {
            ReturnCode topic_return_code = p_topic_->stop();
            if (topic_return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Topic stop() failed");
                if (first_error == ReturnCode::SUCCESS) {
                    first_error = topic_return_code;
                }
            }
        }
    }

    if (is_driver_created_by_this_) {
        if (p_driver_ == nullptr) {
            PI_ERROR("Device driver is not initialized");
            if (first_error == ReturnCode::SUCCESS) {
                first_error = ReturnCode::NOT_INITIALIZED;
            }
        } else {
            ReturnCode driver_return_code = p_driver_->close();
            if (driver_return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Driver close() failed");
                if (first_error == ReturnCode::SUCCESS) {
                    first_error = driver_return_code;
                }
            }
        }
    }

    if (first_error == ReturnCode::SUCCESS) {
        PI_INFO("Device", InfoLevel::ESSENTIAL_0, "Device %s_%s completed stop process", model_.c_str(), id_.c_str());
    }

    return first_error;
}

ReturnCode Device::init(const CommandLineArgs& cla, int argc, char** argv, std::shared_ptr<Topic> p_topic,
                        std::shared_ptr<Driver> p_driver) {

    if (p_config_model_ == nullptr) {
        PI_ERROR("Model configuration pointer is null");
        return ReturnCode::NOT_INITIALIZED;
    }

    control_frequency_ = cla.control_frequency;

    std::string planning_type;
    if (cla.planning_type == OPT_PLANING_TYPE_DEFAULT) {
        ReturnCode return_code = p_config_individual_->get_field_value(
            p_config_model_->values_, p_config_model_->fn_planning_type, planning_type);
        if (return_code != ReturnCode::SUCCESS) {
            return return_code;
        }
    } else {
        planning_type = cla.planning_type;
    }

    // Arm-scoped override: lets a client enable e.g. gravity-compensated follower
    // planning on the arm without dragging the attached effector onto the same
    // planner (a trapezoidal gripper would crawl through its travel) and without
    // editing the shared model config that leaders also load.
    if (type_ == DeviceType::ARM && !cla.arm_planning_type.empty()
        && cla.arm_planning_type != OPT_DEFAULT_NONE) {
        planning_type = cla.arm_planning_type;
    }

    PI_INFO("Device", InfoLevel::HELPFUL_1, "%s_%s: planning_type=%s", model_.c_str(), id_.c_str(),
            planning_type.c_str());

    if (planning_type == p_config_model_->val_planning_type_none) {
        planning_type_ = TrajectoryPlanningType::NONE;
    } else if (planning_type == p_config_model_->val_planning_type_slew_pos_gravity) {
        planning_type_ = TrajectoryPlanningType::SLEW_POS_GRAVITY;
    } else {
        PI_ERROR("Unsupported planning type '%s' for %s_%s", planning_type.c_str(), model_.c_str(), id_.c_str());
        return ReturnCode::NOT_SUPPORTED;
    }

    // Check if the device is read only
    ReturnCode return_code = p_config_model_->get_field_value(p_config_model_->values_, p_config_model_->fn_read_only, is_read_only_);
    const bool read_only_defined_in_model_config = (return_code == ReturnCode::SUCCESS);
    if (!read_only_defined_in_model_config) {
        // If the field is not defined, set it to false
        is_read_only_ = false;
    }

    PI_INFO("Device", InfoLevel::ESSENTIAL_0, "%s_%s: read_only=%d (defined_in_model_config=%d)",
            model_.c_str(), id_.c_str(), (int)is_read_only_, (int)read_only_defined_in_model_config);

    PI_INFO("Device", InfoLevel::ESSENTIAL_0, "Initializing topic...");
    if (p_topic != nullptr) {
        p_topic_ = p_topic;
        is_topic_created_by_this_ = false;
    } else {
        p_topic_ = Topic::new_topic(this, p_config_model_, cla, argc, argv);
        if (p_topic_ == nullptr) {
            PI_ERROR("Failed to create topic");
            return ReturnCode::FAIL;
        }
        is_topic_created_by_this_ = true;
    }

    PI_INFO("Device", InfoLevel::ESSENTIAL_0, "Initializing driver...");
    if (p_driver != nullptr) {
        p_driver_ = p_driver;
        is_driver_created_by_this_ = false;
    } else {
        p_driver_ = Driver::new_driver(this, p_config_model_, cla);
        if (p_driver_ == nullptr) {
            PI_ERROR("Failed to create driver");
            return ReturnCode::FAIL;
        }
        is_driver_created_by_this_ = true;
    }

    PI_INFO("Device", InfoLevel::ESSENTIAL_0, "Initializing algorithm manager...");
    p_algo_.reset(Algo::new_algo(this, p_config_model_, cla));
    if (p_algo_) {
        ReturnCode return_code = p_algo_->init(p_config_model_, p_config_individual_, cla);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Algorithm initialization failed");
            return ReturnCode::FAIL;
        }
    } else {
        PI_WARN("Algorithm not set for device %s_%s (device may operate without kinematics)", model_.c_str(),
                id_.c_str());
    }

    role_ = cla.role;

    return ReturnCode::SUCCESS;
}

Device* Device::new_device(const DeviceConfig& cfg_model, const DeviceConfig& cfg_individual,
                           const CommandLineArgs& cla) {
    Device* p_device = nullptr;
    ReturnCode return_code;

    std::string device_model_in_config;
    return_code = cfg_model.get_field_value(cfg_model.values_, cfg_model.fn_device_model, device_model_in_config);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Device model name is not defined in configuration file");
        return nullptr;
    } else {
        if (cla.device_model != device_model_in_config) {
            PI_ERROR("Device model name mismatch: configuration file='%s', command line='%s'",
                     device_model_in_config.c_str(), cla.device_model.c_str());
            return nullptr;
        }
    }

    std::string device_type;
    return_code = cfg_model.get_field_value(cfg_model.values_, cfg_model.fn_device_type, device_type);
    if (return_code == ReturnCode::SUCCESS) {
        if (device_type == cfg_model.val_device_type_arm) {
            std::string arm_type;
            return_code = cfg_model.get_field_value(cfg_model.values_, cfg_model.fn_arm_type, arm_type);
            if (return_code == ReturnCode::SUCCESS) {
                if (arm_type == cfg_model.val_arm_type_arx) {
                    p_device = new DeviceArmArx(cla);
                    PI_INFO("Device", InfoLevel::DETAIL_2, "Created DeviceArmArx for %s_%s", cla.device_model.c_str(),
                            cla.device_id.c_str());
                } else {
                    PI_ERROR("Invalid arm type: %s", arm_type.c_str());
                    return nullptr;
                }
            } else {
                PI_ERROR("Arm type is not defined in configuration file");
                return nullptr;
            }
        } else if (device_type == cfg_model.val_device_type_effector) {
            std::string effector_type;
            return_code = cfg_model.get_field_value(cfg_model.values_, cfg_model.fn_effector_type, effector_type);
            if (return_code == ReturnCode::SUCCESS) {
                if (effector_type == cfg_model.val_effector_type_arx) {
                    p_device = new DeviceEffectorArx(cla);
                    PI_INFO("Device", InfoLevel::DETAIL_2, "Created DeviceEffectorArx for %s_%s",
                            cla.device_model.c_str(), cla.device_id.c_str());
                } else {
                    PI_ERROR("Invalid effector type: %s", effector_type.c_str());
                    return nullptr;
                }
            } else {
                PI_ERROR("Effector type is not defined in configuration file");
                return nullptr;
            }
        } else {
            PI_ERROR("Invalid device type: %s", device_type.c_str());
            return nullptr;
        }
    } else {
        PI_ERROR("Device type is not defined in configuration file");
        return nullptr;
    }

    p_device->p_config_model_ = &cfg_model;
    p_device->p_config_individual_ = &cfg_individual;

    return p_device;
}

ReturnCode Device::move(Joint* p_joint, float target_pos, float target_tor,
                        float safe_mode_derating) {
    (void)safe_mode_derating;
    p_joint->adjusted_target_pos_ = target_pos;

    ReturnCode return_code;
    if (planning_type_ == TrajectoryPlanningType::SLEW_POS_GRAVITY) {
        p_joint->prev_target_pos_ = target_pos;
        p_joint->target_tor_ = target_tor;
        return_code = p_joint->move(target_pos, 0, target_tor);
        p_joint->prev_target_tor_ = target_tor;
    } else if (planning_type_ == TrajectoryPlanningType::NONE) {
        // The synchronized follower slew integrates from the last commanded
        // target for every planning type, so keep it current here too.
        p_joint->prev_target_pos_ = target_pos;
        return_code = p_joint->move(target_pos);
    } else {
        PI_ERROR("Invalid planning type %d in %s_%s", (int)planning_type_,
                 model_.c_str(), id_.c_str());
        return ReturnCode::INVALID_PARAM;
    }

    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Joint movement failed in %s_%s", model_.c_str(), id_.c_str());
        return ReturnCode::FAIL;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode Device::apply_action(const MsgJoystick& msg_joystick) {
    // The base Device class never implements joystick handling — joystick
    // dispatch lives on device subclasses that actually consume it. Devices
    // that fall through to this default get one warning
    // at startup to surface a misconfiguration; without the static guard
    // every joystick frame (~50 Hz from a leader effector) re-emitted the
    // same warning, which floods the per-arm log buffer the GUI drains
    // and can starve the Qt main thread on Stop / final flush.
    static std::once_flag s_joystick_not_implemented_warning;
    std::call_once(s_joystick_not_implemented_warning, [] {
        PI_WARN("Joystick control not implemented: only logging received joystick message");
    });

    PI_INFO("Joystick", InfoLevel::FREQUENT_3, "Joystick side: %d", msg_joystick.side_);

    for (size_t i = 0; i < msg_joystick.button_.size(); i++) {
        PI_INFO("Joystick", InfoLevel::FREQUENT_3, "Joystick button[%zu]=%d", i, msg_joystick.button_[i]);
    }

    for (size_t i = 0; i < msg_joystick.channel_.size(); i++) {
        PI_INFO("Joystick", InfoLevel::FREQUENT_3, "Joystick channel[%zu]=%.3f", i, msg_joystick.channel_[i]);
    }
    return ReturnCode::SUCCESS;
}

ReturnCode Device::publish_device_info(int info_key, std::vector<float>* p_float_data, std::vector<int>* p_int_data) {
    if (p_topic_ == nullptr) {
        PI_ERROR("Topic is not initialized in publish_device_info()");
        return ReturnCode::NOT_INITIALIZED;
    }

    MsgDeviceInfo msg_device_info(info_key, p_float_data, p_int_data);
    return p_topic_->publish(msg_device_info);
}

ReturnCode Device::enter_emergency_recovery(ReturnCode cause, int failed_joint_id) {
    int cause_int = static_cast<int>(cause);
    bool was_idle = false;
    // Only SIG (CAN disconnect / motor no-response) means we genuinely cannot
    // talk to the servo, so it must be removed from the move-to-ready set.
    // TEMPERATURE and all soft safety codes leave the servo electrically alive
    // and command-receptive -- it is the *trigger* for recovery but it must
    // continue to receive the slow ready-move commands so the arm can unload
    // gravity torque and the joint can cool. Adding it to failed_joint_ids_
    // would make move_to_ready_position skip it forever, freezing the arm in
    // place and cascading the overheat onto its neighbours.
    const bool disable_failed_joint_for_move = (cause == ReturnCode::SAFE_MODE_SIG);
    {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        if (emergency_state_ == EmergencyRecoveryState::NONE) {
            emergency_state_ = EmergencyRecoveryState::REQUESTED;
            emergency_cause_ = cause_int;
            emergency_failed_joint_id_ = failed_joint_id;
            slow_move_active_ = true;
            failed_joint_ids_.clear();
            if (disable_failed_joint_for_move && failed_joint_id >= 0) {
                failed_joint_ids_.insert(static_cast<int16_t>(failed_joint_id));
            }
            emergency_completed_published_ = false;
            was_idle = true;
        } else {
            // Already recovering. Record additional unreachable joints (SIG
            // only); never auto-disable a TEMPERATURE / soft-fault joint.
            if (disable_failed_joint_for_move && failed_joint_id >= 0) {
                failed_joint_ids_.insert(static_cast<int16_t>(failed_joint_id));
            }
        }
    }

    if (was_idle) {
        PI_INFO("Device", InfoLevel::ESSENTIAL_0,
                "[RECOVERY] %s_%s: armed (cause=%d, failed_joint=%d, vel_error=%.3f rad/s, heartbeat_staleness=%d ms)",
                model_.c_str(), id_.c_str(), cause_int, failed_joint_id,
                cla_.move_to_ready_vel_rad_s_error, cla_.emergency_shutdown_timeout_ms);

        // Stop accepting incoming leader joint commands immediately so external
        // teleoperation cannot keep pushing target positions on top of the slow
        // recovery move. This is the C++-side equivalent of the Python backend
        // halting movements execution on device error.
        if (p_topic_) {
            p_topic_->set_pause_leader_command_listening(true);
        }

        std::vector<int> int_payload = {cause_int, failed_joint_id, 0};
        std::vector<float> float_payload = {0.0f};
        ReturnCode rc = publish_device_info(DEVICE_INFO_ERROR_DETECTED, &float_payload, &int_payload);
        if (rc != ReturnCode::SUCCESS) {
            PI_ERROR("[RECOVERY] %s_%s: failed to publish DEVICE_INFO_ERROR_DETECTED (rc=%d); continuing recovery",
                     model_.c_str(), id_.c_str(), static_cast<int>(rc));
        }
    }
    return ReturnCode::SUCCESS;
}

void Device::check_client_heartbeat_watchdog() {
    if (!client_heartbeat_seen_ || client_watchdog_tripped_) {
        return;
    }
    // Recovery and move-to-ready own the arm; their own watchdogs apply.
    if (is_in_emergency_recovery() || is_in_any_move_to_ready_state() || is_ready_ == false) {
        return;
    }
    const auto silence_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                std::chrono::steady_clock::now() - last_client_heartbeat_)
                                .count();
    if (silence_ms < kClientHeartbeatTimeoutMs) {
        return;
    }
    client_watchdog_tripped_ = true;
    PI_ERROR("%s_%s: client heartbeat lost (%lld ms of silence); dropping to safe idle",
             model_.c_str(), id_.c_str(), static_cast<long long>(silence_ms));
    if (role_ == Role::LEADER) {
        // An unsupervised leader must stop driving the follower's live stream
        // with force feedback engaged; plain gravity float is its safe idle.
        if (is_force_feedback_enabled()) {
            (void)set_runtime_force_feedback(false, -1.0f);
        }
    } else if (role_ == Role::FOLLOWER) {
        // Stop consuming the (still-publishing) leader stream and hold.
        if (p_topic_) {
            p_topic_->set_pause_leader_command_listening(true);
        }
        (void)runtime_hold();
    }
}

void Device::mark_emergency_recovery_completed() {
    int cause_int = 0;
    int failed_joint_id = -1;
    bool should_publish = false;
    {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        if (emergency_state_ == EmergencyRecoveryState::NONE) {
            return;
        }
        if (!emergency_completed_published_) {
            emergency_state_ = EmergencyRecoveryState::COMPLETED;
            emergency_completed_published_ = true;
            should_publish = true;
            cause_int = emergency_cause_;
            failed_joint_id = emergency_failed_joint_id_;
        } else {
            emergency_state_ = EmergencyRecoveryState::COMPLETED;
        }
    }

    if (should_publish) {
        PI_INFO("Device", InfoLevel::ESSENTIAL_0,
                "[RECOVERY] %s_%s: COMPLETED -> publishing SHUTDOWN_AFTER_ERROR (cause=%d, joint=%d)",
                model_.c_str(), id_.c_str(), cause_int, failed_joint_id);
        std::vector<int> int_payload = {cause_int, failed_joint_id, 0};
        std::vector<float> float_payload = {1.0f};
        ReturnCode rc = publish_device_info(DEVICE_INFO_SHUTDOWN_AFTER_ERROR, &float_payload, &int_payload);
        if (rc != ReturnCode::SUCCESS) {
            PI_ERROR("[RECOVERY] %s_%s: failed to publish DEVICE_INFO_SHUTDOWN_AFTER_ERROR (rc=%d)",
                     model_.c_str(), id_.c_str(), static_cast<int>(rc));
        }
        // Stop the topic so the main loop exits cleanly via is_running() == false.
        if (p_topic_) {
            ReturnCode stop_rc = p_topic_->stop();
            if (stop_rc != ReturnCode::SUCCESS) {
                PI_ERROR("[RECOVERY] %s_%s: Topic->stop() failed (rc=%d)",
                         model_.c_str(), id_.c_str(), static_cast<int>(stop_rc));
            }
        }
    }
}

void Device::publish_recovery_progress(float progress) {
    int cause_int = 0;
    int failed_joint_id = -1;
    {
        std::lock_guard<std::mutex> lock(emergency_mutex_);
        if (emergency_state_ != EmergencyRecoveryState::MOVING_SLOW) {
            return;
        }
        cause_int = emergency_cause_;
        failed_joint_id = emergency_failed_joint_id_;
    }
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    std::vector<int> int_payload = {cause_int, failed_joint_id, 0};
    std::vector<float> float_payload = {progress};
    publish_device_info(DEVICE_INFO_RECOVERY_IN_PROGRESS, &float_payload, &int_payload);
}

void Device::publish_ready_move_progress(int source, bool is_error, float progress) {
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    std::vector<int> int_payload = {source, is_error ? 1 : 0};
    std::vector<float> float_payload = {progress};
    publish_device_info(DEVICE_INFO_READY_MOVE_IN_PROGRESS, &float_payload, &int_payload);
}

bool Device::check_emergency_timeout_locked() {
    // Caller MUST hold emergency_mutex_.
    //
    // Heartbeat-based watchdog: we no longer enforce an absolute wall-clock cap on the
    // entire recovery sequence. As long as ``step()`` keeps refreshing
    // ``last_step_heartbeat_`` we keep waiting (a heavy arm doing a slow ready move can
    // legitimately need tens of seconds). The state only flips to TIMED_OUT when the gap
    // between heartbeats exceeds ``cla_.emergency_shutdown_timeout_ms`` -- which means the
    // control loop has actually hung (deadlock, blocked device I/O, ...) and the safest
    // thing left is to force-park.
    if (emergency_state_ != EmergencyRecoveryState::MOVING_SLOW) {
        return false;
    }
    if (cla_.emergency_shutdown_timeout_ms <= 0) {
        // Watchdog disabled: caller opted in to wait forever.
        return false;
    }
    const auto now = std::chrono::steady_clock::now();
    const auto since_heartbeat_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_step_heartbeat_).count();
    if (since_heartbeat_ms >= cla_.emergency_shutdown_timeout_ms) {
        const auto total_elapsed_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(now - emergency_start_time_).count();
        PI_ERROR("Emergency recovery heartbeat stale: no step() progress for %lld ms (limit=%d ms, total=%lld ms); "
                 "treating as hung",
                 static_cast<long long>(since_heartbeat_ms), cla_.emergency_shutdown_timeout_ms,
                 static_cast<long long>(total_elapsed_ms));
        emergency_state_ = EmergencyRecoveryState::TIMED_OUT;
        return true;
    }
    return false;
}
