/*!
 * @file pi_topic.cpp
 * @brief Implementation of the Topic base class and factory method for creating topic instances.
 */

#include "pi_topic.hpp"
#include "pi_device.hpp"
#include "pi_device_config.hpp"
#include "pi_topic_zmq.hpp"
#include "pi_device_effector.hpp"

Topic::Topic(Device* p_device, const CommandLineArgs& cla)
    : role_(cla.role), topic_joint_(cla.topic_joint), p_device_(p_device) {}

Topic::~Topic() {}

std::string Topic::generate_unique_topic_name(const std::string& base_topic_name, const std::string& device_model,
                                              const std::string& device_id) {
    std::string topic_name = base_topic_name + "_" + device_model + "_" + device_id;

    std::replace(topic_name.begin(), topic_name.end(), '.', '_');
    std::replace(topic_name.begin(), topic_name.end(), '-', '_');

    return topic_name;
}

std::shared_ptr<Topic> Topic::new_topic(Device* p_device, const DeviceConfig* p_config, const CommandLineArgs& cla,
                                        int argc, char** argv) {
    if (p_config == nullptr) {
        PI_ERROR("Configuration pointer is null");
        return nullptr;
    }

    std::string topic_type;
    ReturnCode return_code = ReturnCode::SUCCESS;

    PI_INFO("Topic", InfoLevel::DETAIL_2, "Initializing topic with configuration values...");

    if (cla.topic_type == OPT_TOPIC_TYPE_UNDEFINED) {
        return_code = p_config->get_field_value(p_config->values_, p_config->fn_topic_type, topic_type);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Topic type is not defined in configuration file");
            return nullptr;
        }

        PI_INFO("Topic", InfoLevel::DETAIL_2, "Topic type set from configuration file: %s", topic_type.c_str());

    } else {
        topic_type = cla.topic_type;
        PI_INFO("Topic", InfoLevel::DETAIL_2, "Topic type set from command line: %s", topic_type.c_str());
    }

    std::shared_ptr<Topic> p_topic = nullptr;

    if (topic_type == p_config->val_topic_type_zmq) {
        p_topic = std::make_shared<TopicZmq>(p_device, cla, argc, argv);
        PI_INFO("Topic", InfoLevel::DETAIL_2, "Created ZMQ topic");

    } else {
        PI_ERROR("Unsupported topic type: %s", topic_type.c_str());
        return nullptr;
    }

    PI_INFO("Topic", InfoLevel::DETAIL_2, "Topic initialization completed");

    return p_topic;
}

ReturnCode Topic::publish(const MsgJoints& msg) {
    (void)msg;
    msg_id_generation_++;

    return ReturnCode::SUCCESS;
}

ReturnCode Topic::process_leader_msg(const MsgCommand& msg) {

    // Heartbeats are pure liveness signals: handle them before every other
    // gate (including the emergency-recovery command drop below, which would
    // otherwise WARN once per second for the duration of a recovery).
    if (msg.command_ == DEVICE_COMMAND_HEARTBEAT) {
        if (p_device_ != nullptr) {
            p_device_->note_client_heartbeat();
        }
        return ReturnCode::SUCCESS;
    }

    // During emergency recovery the device drives its own slow ready move. Reject every
    // incoming control command EXCEPT the operator-override escape hatches so external
    // teleop / scripts cannot interfere with the recovery trajectory:
    //   - DEVICE_COMMAND_STOP                  : explicit hard stop, always honored.
    //   - DEVICE_COMMAND_MOVE_TO_READY_AND_STOP: "Stop Run Now" path -- the operator
    //                                            wants to abandon slow recovery and
    //                                            switch to a normal-speed ready move
    //                                            followed by topic shutdown.
    if (p_device_ != nullptr && p_device_->is_in_emergency_recovery() &&
        msg.command_ != DEVICE_COMMAND_STOP &&
        msg.command_ != DEVICE_COMMAND_MOVE_TO_READY_AND_STOP) {
        PI_WARN("Topic: dropping command %d while %s_%s is in emergency recovery",
                msg.command_, p_device_->get_model().c_str(), p_device_->get_id().c_str());
        return ReturnCode::BUSY;
    }

    if (msg.command_ != DEVICE_COMMAND_STOP &&
        (msg.num_param_float_ < 0 ||
         static_cast<size_t>(msg.num_param_float_) != msg.param_float_.size() ||
         msg.num_param_int_ < 0 ||
         static_cast<size_t>(msg.num_param_int_) != msg.param_int_.size())) {
        PI_ERROR(
            "Command parameter counts do not match payloads: float count=%d, "
            "float payload=%zu, int count=%d, int payload=%zu",
            msg.num_param_float_, msg.param_float_.size(), msg.num_param_int_,
            msg.param_int_.size());
        return ReturnCode::INVALID_PARAM;
    }

    if (msg.command_ == DEVICE_COMMAND_STOP) {
        is_running_ = false;

    } else if (msg.command_ == DEVICE_COMMAND_PAUSE_LEADER_COMMAND_LISTENING) {
        // Follower-only: ignore leader joint commands.
        if (p_device_ != nullptr && p_device_->get_device_role() == Role::FOLLOWER) {
            pause_leader_command_listening_ = true;
            PI_INFO("Topic", InfoLevel::ESSENTIAL_0, "Paused leader command listening for %s_%s",
                    p_device_->get_model().c_str(), p_device_->get_id().c_str());
        }

    } else if (msg.command_ == DEVICE_COMMAND_RESUME_LEADER_COMMAND_LISTENING) {
        // Follower-only: resume leader joint commands.
        if (p_device_ != nullptr && p_device_->get_device_role() == Role::FOLLOWER) {
            pause_leader_command_listening_ = false;
            PI_INFO("Topic", InfoLevel::ESSENTIAL_0, "Resumed leader command listening for %s_%s",
                    p_device_->get_model().c_str(), p_device_->get_id().c_str());
        }

    } else if (msg.command_ == DEVICE_COMMAND_MOVE_TO_READY_POS_AND_PAUSE_LEADER_COMMAND_LISTENING) {
        // Combined command:
        // - Leader and follower BOTH start move-to-ready.
        // - Follower additionally stops reacting to incoming leader joint commands.
        if (p_device_ == nullptr) {
            PI_ERROR("Device pointer is not initialized");
            return ReturnCode::NOT_INITIALIZED;
        }

        if (p_device_->get_device_role() == Role::FOLLOWER) {
            pause_leader_command_listening_ = true;
        }
        const int request_id = msg.num_param_int_ > 0 ? msg.param_int_[0] : 0;
        p_device_->request_move_to_ready_position(request_id);
        PI_INFO("Topic", InfoLevel::ESSENTIAL_0,
                "Requested move-to-ready%s for %s_%s",
                (p_device_->get_device_role() == Role::FOLLOWER) ? " and paused leader command listening" : "",
                p_device_->get_model().c_str(), p_device_->get_id().c_str());

    } else if (msg.command_ == DEVICE_COMMAND_MOVE_TO_READY_AND_STOP) {
        if (p_device_ == nullptr) {
            PI_ERROR("Device pointer is not initialized");
            return ReturnCode::NOT_INITIALIZED;
        }

        if (p_device_->get_device_role() == Role::FOLLOWER) {
            pause_leader_command_listening_ = true;
        }
        const int request_id = msg.num_param_int_ > 0 ? msg.param_int_[0] : 0;
        p_device_->request_move_to_ready_and_stop(request_id);
        PI_INFO("Topic", InfoLevel::ESSENTIAL_0,
                "Requested move-to-ready-and-stop%s for %s_%s",
                (p_device_->get_device_role() == Role::FOLLOWER) ? " (paused leader cmds)" : "",
                p_device_->get_model().c_str(), p_device_->get_id().c_str());

    } else if (msg.command_ == DEVICE_COMMAND_MOVE_TO_READY_POS) {
        if (p_device_ == nullptr) {
            PI_ERROR("Device pointer is not initialized");
            return ReturnCode::NOT_INITIALIZED;
        }
        const int request_id = msg.num_param_int_ > 0 ? msg.param_int_[0] : 0;
        p_device_->request_move_to_ready_position(request_id);
        PI_INFO("Topic", InfoLevel::ESSENTIAL_0, "Requested move-to-ready for %s_%s",
                p_device_->get_model().c_str(), p_device_->get_id().c_str());

    } else if (msg.command_ == DEVICE_COMMAND_ENTER_GRAVITY_COMPENSATION) {
        if (p_device_ == nullptr) return ReturnCode::NOT_INITIALIZED;
        return p_device_->set_runtime_force_feedback(false, -1.0f);

    } else if (msg.command_ == DEVICE_COMMAND_ENABLE_FORCE_FEEDBACK) {
        if (p_device_ == nullptr) return ReturnCode::NOT_INITIALIZED;
        const float gain = msg.num_param_float_ > 0 ? msg.param_float_[0] : p_device_->get_cla().force_feedback;
        return p_device_->set_runtime_force_feedback(true, gain);

    } else if (msg.command_ == DEVICE_COMMAND_SET_FORCE_FEEDBACK_GAIN) {
        if (p_device_ == nullptr) return ReturnCode::NOT_INITIALIZED;
        if (msg.num_param_float_ != 1) return ReturnCode::INVALID_PARAM;
        return p_device_->set_runtime_force_feedback_gain(msg.param_float_[0]);

    } else if (msg.command_ == DEVICE_COMMAND_HOLD) {
        if (p_device_ == nullptr) return ReturnCode::NOT_INITIALIZED;
        return p_device_->runtime_hold();

    } else if (msg.command_ == DEVICE_COMMAND_SET_EFFECTOR_MIN_MAX_POS) {
        int num_param_float = msg.num_param_float_;
        if (num_param_float != 2) {
            PI_ERROR(
                "Invalid parameter count for set effector min/max position command: received=%d (expected: 2)",
                num_param_float);
            return ReturnCode::INVALID_PARAM;
        }
        float min_pos = msg.param_float_[0];
        float max_pos = msg.param_float_[1];
        DeviceEffector* p_effector = dynamic_cast<DeviceEffector*>(p_device_);
        if (p_effector == nullptr) {
            PI_ERROR("Set effector position limits command received for a non-effector device");
            return p_device_ == nullptr ? ReturnCode::NOT_INITIALIZED : ReturnCode::NOT_SUPPORTED;
        }
        PI_INFO("Topic", InfoLevel::DETAIL_2, "Set effector position limits: min_pos=%.3f, max_pos=%.3f",
                min_pos, max_pos);
        return p_effector->set_effector_min_max_pos(min_pos, max_pos);

    } else if (msg.command_ == DEVICE_COMMAND_SET_DISTANCE_TO_TORQUE) {
        int num_param_float = msg.num_param_float_;
        if (num_param_float != 1) {
            PI_ERROR("Invalid parameter count for set distance-to-torque command: received=%d (expected: 1)",
                     num_param_float);
            return ReturnCode::INVALID_PARAM;
        }
        float distance_to_torque = msg.param_float_[0];
        DeviceEffector* p_effector = dynamic_cast<DeviceEffector*>(p_device_);
        if (p_effector == nullptr) {
            PI_ERROR("Set distance-to-torque command received for a non-effector device");
            return p_device_ == nullptr ? ReturnCode::NOT_INITIALIZED : ReturnCode::NOT_SUPPORTED;
        }
        PI_INFO("Topic", InfoLevel::DETAIL_2, "Set distance-to-torque conversion factor: %.3f",
                distance_to_torque);
        return p_effector->set_distance_to_torque(distance_to_torque);

    } else if (msg.command_ == DEVICE_COMMAND_SET_EFFECTOR_KD) {
        int num_param_float = msg.num_param_float_;
        if (num_param_float != 1) {
            PI_ERROR("Invalid parameter count for set effector Kd command: received=%d (expected: 1)",
                     num_param_float);
            return ReturnCode::INVALID_PARAM;
        }
        float effector_kd = msg.param_float_[0];
        DeviceEffector* p_effector = dynamic_cast<DeviceEffector*>(p_device_);
        if (p_effector == nullptr) {
            PI_ERROR("Set effector Kd command received for a non-effector device");
            return p_device_ == nullptr ? ReturnCode::NOT_INITIALIZED : ReturnCode::NOT_SUPPORTED;
        }
        PI_INFO("Topic", InfoLevel::DETAIL_2, "Set effector derivative gain (Kd): %.3f", effector_kd);
        return p_effector->set_effector_kd(effector_kd);
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Topic::process_leader_msg(const MsgJoints& msg) {

    if (p_device_ == nullptr) {
        PI_ERROR("Device pointer is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }

    // During emergency recovery the device drives its own slow ready move; drop ALL
    // incoming leader joint commands (regardless of role) so external teleoperation
    // cannot push fresh target positions on top of the recovery trajectory.
    if (p_device_->is_in_emergency_recovery()) {
        return ReturnCode::SUCCESS;
    }

    // Follower-only pause: drop leader joint commands.
    if (p_device_->get_device_role() == Role::FOLLOWER && pause_leader_command_listening_) {
        return ReturnCode::SUCCESS;
    }

    return p_device_->apply_action(msg);
}

ReturnCode Topic::process_direct_msg(const MsgJoints& msg) {
    if (p_device_ == nullptr) {
        return ReturnCode::NOT_INITIALIZED;
    }
    if (p_device_->get_device_role() != Role::FOLLOWER) {
        return ReturnCode::NOT_SUPPORTED;
    }
    if (p_device_->rejects_direct_commands()) {
        return ReturnCode::BUSY;
    }
    return p_device_->apply_action(msg);
}

ReturnCode Topic::process_leader_msg(const MsgJoystick& msg) {

    if (p_device_ == nullptr) {
        PI_ERROR("Device pointer is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }

    // Drop joystick input while the device is in emergency recovery -- operator
    // sticks must not nudge the slow ready move.
    if (p_device_->is_in_emergency_recovery()) {
        return ReturnCode::SUCCESS;
    }

    const ReturnCode return_code = p_device_->apply_action(msg);

    PI_INFO("Topic", InfoLevel::FREQUENT_3, "Left joystick message received");

    return return_code;
}

ReturnCode Topic::process_follower_msg(const MsgJoints& msg) {
    if (p_device_ == nullptr) {
        PI_ERROR("Device pointer is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }
    return p_device_->process_follower_msg(msg);
}
