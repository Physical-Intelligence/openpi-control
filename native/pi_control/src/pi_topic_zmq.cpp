/*!
 * @file pi_topic_zmq.cpp
 * @brief Implementation of the TopicZmq class for managing ZeroMQ topics with specific roles.
 */

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

#include "pi_device.hpp"
#include "pi_device_effector.hpp"
#include "pi_info.hpp"
#include "pi_profile.hpp"
#include "pi_topic_zmq.hpp"

#define BASE_PORT_JOINT_DYNAMIC 8500   ///< Base port for dynamic joint topics (changed from 6000 to avoid conflicts with common web servers and Zoom)
#define PORT_RANGE_JOINT_DYNAMIC 1000  ///< Port range for dynamic allocation (8500-9499, avoiding Zoom's 8801-8810)

namespace {

int control_period_from_frequency(int control_frequency) {
    if (control_frequency <= 0) {
        throw std::invalid_argument("ZMQ control frequency must be positive");
    }
    return 1000 / control_frequency;
}

}  // namespace

TopicZmq::TopicZmq(Device* p_device, const CommandLineArgs& cla, int argc, char** argv)
    : Topic(p_device, cla),
      context_(1),
      pub_joint_(context_, ZMQ_PUB),
      pub_follower_joint_(context_, ZMQ_PUB),
      pub_device_info_(context_, ZMQ_PUB),
      pub_effector_info_(context_, ZMQ_PUB),
      pub_joystick_(context_, ZMQ_PUB),
      sub_joint_(context_, ZMQ_SUB),
      sub_direct_joint_(context_, ZMQ_SUB),
      sub_follower_joint_(context_, ZMQ_SUB),
      sub_command_(context_, ZMQ_SUB),
      sub_joystick_left_(context_, ZMQ_SUB),
      sub_joystick_right_(context_, ZMQ_SUB),
      control_frequency_(cla.control_frequency),
      control_period_(control_period_from_frequency(control_frequency_)),
      topic_joint_(cla.topic_joint),
      topic_state_(cla.topic_state),
      topic_direct_command_(cla.topic_direct_command),
      topic_lifecycle_(cla.topic_lifecycle),
      topic_status_(cla.topic_status) {
    (void)argc;
    (void)argv;

    PI_INFO("Topic", InfoLevel::DETAIL_2, "Initializing ZeroMQ...");
    pause_leader_command_listening_ = (cla.role == Role::FOLLOWER);

    if (topic_state_.empty()) {
        topic_state_ = generate_unique_topic_name(TOPIC_FOLLOWER_JOINT_INFO, p_device->get_model(), p_device->get_id());
    }
    if (topic_lifecycle_.empty()) {
        topic_lifecycle_ = TOPIC_COMMAND_INFO;
    }
    if (topic_status_.empty()) {
        topic_status_ = generate_unique_topic_name(TOPIC_DEVICE_INFO, p_device->get_model(), p_device->get_id());
    }
    init_port(pub_device_info_, ZmqPortType::PUB, topic_status_);
    topic_follower_joint_ = topic_state_;
    init_port(pub_follower_joint_, ZmqPortType::PUB, topic_state_);

    if (cla.role == Role::LEADER) {
        init_port(pub_joint_, ZmqPortType::PUB, topic_joint_);
        sub_direct_joint_.close();

        std::string topic_effector_info =
            generate_unique_topic_name(TOPIC_EFFECTOR_INFO, p_device->get_model(), p_device->get_id());
        init_port(pub_effector_info_, ZmqPortType::PUB, topic_effector_info);

        // Joystick publisher (leader-only). Bound when the user explicitly
        // selects a joystick topic via --topic_joystick (default: NONE).
        if (cla.topic_joystick != OPT_DEFAULT_NONE && !cla.topic_joystick.empty()) {
            topic_joystick_ = cla.topic_joystick;
            init_port(pub_joystick_, ZmqPortType::PUB, topic_joystick_);
        } else {
            pub_joystick_.close();
        }

        if (!cla.paired_follower_state_topic.empty()) {
            topic_follower_joint_ = cla.paired_follower_state_topic;
            init_port(sub_follower_joint_, ZmqPortType::SUB, topic_follower_joint_);
        } else {
            sub_follower_joint_.close();
        }

    } else if (cla.role == Role::FOLLOWER) {
        init_port(sub_joint_, ZmqPortType::SUB, topic_joint_);
        if (!topic_direct_command_.empty()) {
            init_port(sub_direct_joint_, ZmqPortType::SUB, topic_direct_command_);
        } else {
            sub_direct_joint_.close();
        }
        sub_joystick_left_.close();
        sub_joystick_right_.close();
    }

    init_port(sub_command_, ZmqPortType::SUB, topic_lifecycle_);

    prev_time_ = Profile::get_time_now();

    PI_INFO("Topic", InfoLevel::DETAIL_2, "ZeroMQ initialization completed");
}

TopicZmq::~TopicZmq() {
    pub_device_info_.close();
    pub_joint_.close();
    pub_follower_joint_.close();
    pub_effector_info_.close();
    if (pub_joystick_) {
        pub_joystick_.close();
    }
    sub_command_.close();
    sub_joint_.close();
    sub_direct_joint_.close();
    sub_joystick_left_.close();
    sub_joystick_right_.close();
    if (sub_follower_joint_) {
        sub_follower_joint_.close();
    }

    context_.close();
}

// Drains every queued (topic, payload) pair on a SUB socket and keeps only the
// newest payload of the expected size. State-like streams (leader joint
// targets, paired-follower state, joystick) must consume to the latest sample
// each control step: processing one message per step means any transient
// slowdown of this loop leaves a queue that is then replayed one stale target
// per cycle -- an unbounded and permanent teleoperation lag (the SUB high
// water mark holds ~10 s of 100 Hz traffic). Returns whether a payload was
// captured into p_latest and the first malformed-frame error encountered while
// draining.
struct DrainResult {
    bool have_payload = false;
    ReturnCode first_error = ReturnCode::SUCCESS;

    void record_error(ReturnCode error) {
        if (first_error == ReturnCode::SUCCESS) {
            first_error = error;
        }
    }
};

static DrainResult drain_sub_to_latest(zmq::socket_t& sub, const std::string& expected_topic, void* p_latest,
                                       size_t expected_size) {
    DrainResult result;
    while (true) {
        zmq::message_t topic_msg;
        zmq::recv_result_t topic_result = sub.recv(topic_msg, zmq::recv_flags::dontwait);
        if (!topic_result) {
            break;
        }
        if (*topic_result == 0) {
            PI_ERROR("Received empty topic frame");
            if (sub.get(zmq::sockopt::rcvmore)) {
                zmq::message_t discard;
                (void)sub.recv(discard, zmq::recv_flags::dontwait);
            }
            result.record_error(ReturnCode::INVALID_PARAM);
            continue;
        }

        std::string topic(static_cast<char*>(topic_msg.data()), topic_msg.size());
        if (topic != expected_topic) {
            PI_ERROR("Received topic name does not match expected topic %s: received=%s", expected_topic.c_str(),
                     topic.c_str());
            // Drop the data part (if any) and keep draining; a foreign topic
            // sharing the hashed port must not wedge the stream.
            if (sub.get(zmq::sockopt::rcvmore)) {
                zmq::message_t discard;
                (void)sub.recv(discard, zmq::recv_flags::dontwait);
            }
            result.record_error(ReturnCode::INVALID_PARAM);
            continue;
        }

        if (!sub.get(zmq::sockopt::rcvmore)) {
            PI_ERROR("Received message without data part");
            result.record_error(ReturnCode::INVALID_PARAM);
            continue;
        }

        zmq::message_t message;
        zmq::recv_result_t message_result = sub.recv(message, zmq::recv_flags::dontwait);
        if (!message_result) {
            PI_ERROR("Failed to receive message data");
            result.record_error(ReturnCode::FAIL);
            continue;
        }
        if (*message_result == 0) {
            PI_ERROR("Received empty message data");
            result.record_error(ReturnCode::INVALID_PARAM);
            continue;
        }
        if (message.size() != expected_size) {
            PI_ERROR("Received message size (%zu) does not match expected size (%zu)", message.size(),
                     expected_size);
            result.record_error(ReturnCode::INVALID_PARAM);
            continue;
        }

        memcpy(p_latest, message.data(), expected_size);
        result.have_payload = true;
    }
    return result;
}

static bool joint_info_count_is_valid(const ZmqJointInfo& zmq_msg, int device_dof) {
    constexpr int kMaxWireJoints = sizeof(zmq_msg.joint_pos) / sizeof(zmq_msg.joint_pos[0]);
    if (device_dof < 0 || device_dof > kMaxWireJoints || zmq_msg.joint_num != device_dof) {
        PI_ERROR("Invalid joint count: received=%d, device DOF=%d, wire max=%d", zmq_msg.joint_num, device_dof,
                 kMaxWireJoints);
        return false;
    }
    return true;
}

ReturnCode TopicZmq::step() {
    ReturnCode first_error = ReturnCode::SUCCESS;
    auto record_result = [&first_error](ReturnCode result) {
        if (result != ReturnCode::SUCCESS &&
            first_error == ReturnCode::SUCCESS) {
            first_error = result;
        }
        return result;
    };

    if (sub_command_) {
        zmq::message_t topic_msg;
        zmq::recv_result_t topic_result = sub_command_.recv(topic_msg, zmq::recv_flags::dontwait);

        if (topic_result) {
            if (*topic_result == 0) {
                PI_ERROR("Received empty topic frame");
                if (sub_command_.get(zmq::sockopt::rcvmore)) {
                    zmq::message_t discard;
                    (void)sub_command_.recv(discard, zmq::recv_flags::dontwait);
                }
                record_result(ReturnCode::INVALID_PARAM);
            } else {
                std::string topic(static_cast<char*>(topic_msg.data()), topic_msg.size());
                if (topic != topic_lifecycle_) {
                    PI_ERROR("Received topic name does not match expected topic %s: received=%s",
                             topic_lifecycle_.c_str(), topic.c_str());
                    // Consume the orphaned data frame so the next step() does not
                    // misread it as a topic frame, then fall through to the joint
                    // drains -- a foreign topic sharing the hashed port must not
                    // starve live command intake (same contract as
                    // drain_sub_to_latest). The old `return FAIL` here desynced the
                    // multipart stream AND skipped every drain below for the step.
                    if (sub_command_.get(zmq::sockopt::rcvmore)) {
                        zmq::message_t discard;
                        (void)sub_command_.recv(discard, zmq::recv_flags::dontwait);
                    }
                    record_result(ReturnCode::INVALID_PARAM);
                } else if (sub_command_.get(zmq::sockopt::rcvmore)) {
                    zmq::message_t message;
                    zmq::recv_result_t message_result = sub_command_.recv(message, zmq::recv_flags::dontwait);

                    if (!message_result) {
                        PI_ERROR("Failed to receive message data");
                        record_result(ReturnCode::FAIL);
                    } else if (*message_result == 0) {
                        PI_ERROR("Received empty message data");
                        record_result(ReturnCode::INVALID_PARAM);
                    } else {
                        if (message.size() == sizeof(ZmqCommand)) {
                            // Directly interpret the message buffer as ZmqCommand
                            // Safe because: same compiler/platform, POD type, size matches
                            const ZmqCommand* zmq_msg = reinterpret_cast<const ZmqCommand*>(message.data());
                            if (zmq_msg->num_param_float < 0 || zmq_msg->num_param_float > 10 ||
                                zmq_msg->num_param_int < 0 || zmq_msg->num_param_int > 10) {
                                // Skip this command but keep stepping: the message
                                // was fully consumed, and an early return here
                                // would starve the joint drains below.
                                PI_ERROR("Received lifecycle command with invalid parameter counts");
                                record_result(ReturnCode::INVALID_PARAM);
                            } else {
                                MsgCommand msg;
                                msg.command_ = zmq_msg->command;
                                msg.num_param_float_ = zmq_msg->num_param_float;
                                msg.num_param_int_ = zmq_msg->num_param_int;
                                for (int i = 0; i < zmq_msg->num_param_float; i++) {
                                    msg.param_float_.push_back(zmq_msg->param_float[i]);
                                }
                                for (int i = 0; i < zmq_msg->num_param_int; i++) {
                                    msg.param_int_.push_back(zmq_msg->param_int[i]);
                                }
                                const ReturnCode command_result =
                                    record_result(Topic::process_leader_msg(msg));
                                const int request_id = msg.num_param_int_ > 0 ? msg.param_int_[0] : 0;
                                std::vector<int> ack_data{request_id, static_cast<int>(command_result), msg.command_};
                                p_device_->publish_device_info(DEVICE_INFO_COMMAND_ACK, nullptr, &ack_data);
                            }
                        } else {
                            PI_ERROR("Received message size (%zu) does not match expected size (%zu)", message.size(),
                                     sizeof(ZmqCommand));
                            record_result(ReturnCode::INVALID_PARAM);
                        }
                    }
                } else {
                    PI_ERROR("Received message without data part");
                    record_result(ReturnCode::INVALID_PARAM);
                }
            }
        }
    }

    if (!is_running_) {
        return first_error;
    }

    if (sub_joint_) {
        ZmqJointInfo latest_joint{};
        const DrainResult drain_result =
            drain_sub_to_latest(sub_joint_, topic_joint_, &latest_joint, sizeof(latest_joint));
        record_result(drain_result.first_error);
        if (drain_result.have_payload) {
            record_result(process_leader_msg_joint(&latest_joint));
        }
    }

    if (sub_direct_joint_) {
        ZmqJointInfo latest_direct{};
        const DrainResult drain_result =
            drain_sub_to_latest(sub_direct_joint_, topic_direct_command_, &latest_direct, sizeof(latest_direct));
        record_result(drain_result.first_error);
        if (drain_result.have_payload) {
            record_result(process_leader_msg_joint(&latest_direct, true));
        }
    }

    if (sub_follower_joint_) {
        ZmqJointInfo latest_follower{};
        const DrainResult drain_result =
            drain_sub_to_latest(sub_follower_joint_, topic_follower_joint_, &latest_follower,
                                sizeof(latest_follower));
        record_result(drain_result.first_error);
        if (drain_result.have_payload) {
            record_result(process_follower_msg_joint(&latest_follower));
        }
    }

    if (sub_joystick_left_) {
        ZmqJoystickInfo latest_left{};
        const DrainResult drain_result =
            drain_sub_to_latest(sub_joystick_left_, TOPIC_JOYSTICK_INFO_LEFT, &latest_left, sizeof(latest_left));
        record_result(drain_result.first_error);
        if (drain_result.have_payload) {
            record_result(process_leader_msg_joystick(&latest_left, MsgJoystick::Side::LEFT));
        }
    }

    if (sub_joystick_right_) {
        ZmqJoystickInfo latest_right{};
        const DrainResult drain_result =
            drain_sub_to_latest(sub_joystick_right_, TOPIC_JOYSTICK_INFO_RIGHT, &latest_right,
                                sizeof(latest_right));
        record_result(drain_result.first_error);
        if (drain_result.have_payload) {
            record_result(process_leader_msg_joystick(&latest_right, MsgJoystick::Side::RIGHT));
        }
    }


    return first_error;
}

ReturnCode TopicZmq::stop() {
    is_running_ = false;
    return ReturnCode::SUCCESS;
}

ReturnCode TopicZmq::sleep() {
    prof_time_t curr_time = Profile::get_time_now();

    prof_time_msec_t time_spent = Profile::get_time_diff(prev_time_, curr_time);

    prof_time_msec_t time_to_sleep = control_period_ - time_spent;

    if (time_to_sleep > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(time_to_sleep));
    }
    prev_time_ = Profile::get_time_now();

    return ReturnCode::SUCCESS;
}

ReturnCode TopicZmq::publish(const MsgJoints& msg) {
    // Value-initialize: only the populated slots get written below, but the whole
    // sizeof(zmq_msg) goes over the wire — without {} the unused array tail and
    // struct padding are uninitialized stack bytes (flagged by SIL valgrind).
    ZmqJointInfo zmq_msg{};
    unsigned int msg_id = msg_id_generation_;
    const int dof_total = p_device_->get_dof_total();
    const int wire_capacity = sizeof(zmq_msg.joint_pos) / sizeof(zmq_msg.joint_pos[0]);

    zmq_msg.msg_id = msg_id;
    zmq_msg.msg_type = (int)MsgType::JOINT_INFO;

    if (dof_total < 0 || dof_total > wire_capacity) {
        PI_ERROR("Device joint count exceeds ZMQ wire capacity: device DOF=%d (max: %d)", dof_total, wire_capacity);
        return ReturnCode::INVALID_PARAM;
    }

    if ((int)msg.joints_.size() != dof_total) {
        PI_ERROR("Joint count mismatch in TopicZmq::publish: msg.joints_.size()=%d, p_device_->get_dof_total()=%d",
                 (int)msg.joints_.size(), dof_total);
        return ReturnCode::INVALID_PARAM;
    }

    zmq_msg.joint_num = dof_total;
    for (int i = 0; i < dof_total; i++) {
        zmq_msg.joint_pos[i] = msg.joints_[i].curr_pos_;
        zmq_msg.joint_vel[i] = msg.joints_[i].curr_vel_;
        zmq_msg.joint_tor[i] = msg.joints_[i].curr_tor_;
        zmq_msg.temperature[i] = msg.joints_[i].temperature_;
        zmq_msg.idc_current[i] = msg.joints_[i].idc_current_;
        zmq_msg.joint_age_ms[i] = msg.joints_[i].frame_age_ms_;
    }
    zmq_msg.measured_idc_current = msg.measured_idc_current_;

    if (p_device_ == nullptr) {
        PI_ERROR("Device pointer is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (!pub_follower_joint_) {
        PI_ERROR("ZMQ publisher for follower joint information is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }
    zmq::message_t follower_topic_msg(topic_state_.c_str(), topic_state_.length());
    zmq::message_t follower_message(&zmq_msg, sizeof(zmq_msg));
    pub_follower_joint_.send(follower_topic_msg, zmq::send_flags::sndmore);
    pub_follower_joint_.send(follower_message, zmq::send_flags::dontwait);

    if (p_device_->get_device_role() == Role::LEADER) {
        if (pub_joint_) {
            zmq::message_t topic_msg(topic_joint_.c_str(), topic_joint_.length());
            zmq::message_t message(&zmq_msg, sizeof(zmq_msg));

            pub_joint_.send(topic_msg, zmq::send_flags::sndmore);
            pub_joint_.send(message, zmq::send_flags::dontwait);

        } else {
            PI_ERROR("ZMQ publisher for joint information is not initialized");
            return ReturnCode::NOT_INITIALIZED;
        }
    } else if (p_device_->get_device_role() != Role::FOLLOWER) {
        PI_ERROR("Device role has invalid value: %d", (int)p_device_->get_device_role());
        return ReturnCode::NOT_INITIALIZED;
    }

    return Topic::publish(msg);
}

ReturnCode TopicZmq::publish(const MsgDeviceInfo& msg) {
    ZmqDeviceInfo zmq_msg{};

    zmq_msg.info_key = msg.info_key_;
    zmq_msg.num_param_float = (int)msg.float_data_.size();
    zmq_msg.num_param_int = (int)msg.int_data_.size();

    int int_array_size = sizeof(zmq_msg.param_int) / sizeof(zmq_msg.param_int[0]);
    int float_array_size = sizeof(zmq_msg.param_float) / sizeof(zmq_msg.param_float[0]);

    if (zmq_msg.num_param_float > float_array_size || zmq_msg.num_param_int > int_array_size) {
        PI_ERROR("Parameter count exceeds array size: num_param_float=%d (max: %d), num_param_int=%d (max: %d)",
                 zmq_msg.num_param_float, float_array_size, zmq_msg.num_param_int, int_array_size);
        return ReturnCode::INVALID_PARAM;
    }

    for (int i = 0; i < zmq_msg.num_param_float; i++) {
        zmq_msg.param_float[i] = msg.float_data_[i];
    }

    for (int i = 0; i < zmq_msg.num_param_int; i++) {
        zmq_msg.param_int[i] = msg.int_data_[i];
    }

    if (pub_device_info_) {
        zmq::message_t topic_msg(topic_status_.c_str(), topic_status_.length());
        zmq::message_t message(&zmq_msg, sizeof(zmq_msg));

        pub_device_info_.send(topic_msg, zmq::send_flags::sndmore);
        pub_device_info_.send(message, zmq::send_flags::dontwait);

    } else {
        PI_ERROR("ZMQ publisher for device info is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode TopicZmq::publish(const MsgEffectorInfo& msg) {
    ZmqEffectorInfo zmq_msg{};
    const size_t wire_capacity = sizeof(zmq_msg.param_float) / sizeof(zmq_msg.param_float[0]);

    if (msg.param_float_.size() > wire_capacity) {
        PI_ERROR("Effector parameter count exceeds ZMQ wire capacity: count=%zu (max: %zu)", msg.param_float_.size(),
                 wire_capacity);
        return ReturnCode::INVALID_PARAM;
    }

    zmq_msg.num_param_float = (int)msg.param_float_.size();
    for (int i = 0; i < zmq_msg.num_param_float; i++) {
        zmq_msg.param_float[i] = msg.param_float_[i];
    }

    if (pub_effector_info_) {
        zmq::message_t topic_msg(TOPIC_EFFECTOR_INFO, sizeof(TOPIC_EFFECTOR_INFO));
        zmq::message_t message(&zmq_msg, sizeof(zmq_msg));

        pub_effector_info_.send(topic_msg, zmq::send_flags::sndmore);
        pub_effector_info_.send(message, zmq::send_flags::dontwait);
    } else {
        PI_ERROR("ZMQ publisher for effector info is not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode TopicZmq::process_leader_msg_joint(ZmqJointInfo* p_zmq_msg,
                                               bool direct) {
    if (p_zmq_msg->msg_type != static_cast<int>(MsgType::JOINT_INFO)) {
        PI_ERROR("Received message with unsupported message type: %d",
                 p_zmq_msg->msg_type);
        return ReturnCode::INVALID_PARAM;
    }

    const int dof_total = p_device_->get_dof_total();
    if (!joint_info_count_is_valid(*p_zmq_msg, dof_total)) {
        return ReturnCode::INVALID_PARAM;
    }

    MsgJoints msg;
    msg.msg_id_ = p_zmq_msg->msg_id;
    for (int i = 0; i < dof_total; ++i) {
        // Command ingest: the sender's joint_age_ms slots carry no meaning
        // for a target command, so record "unknown" explicitly.
        msg.add_joint_info(
            p_zmq_msg->joint_pos[i], p_zmq_msg->joint_vel[i],
            p_zmq_msg->joint_tor[i], p_zmq_msg->temperature[i],
            p_zmq_msg->idc_current[i], -1.0f);
    }
    msg.measured_idc_current_ = p_zmq_msg->measured_idc_current;
    return direct ? Topic::process_direct_msg(msg)
                  : Topic::process_leader_msg(msg);
}

ReturnCode TopicZmq::publish(const MsgJoystick& msg) {
    if (!pub_joystick_) {
        PI_ERROR("ZMQ publisher for joystick info is not initialized "
                 "(was --topic_joystick set, and is the device in leader role?)");
        return ReturnCode::NOT_INITIALIZED;
    }

    ZmqJoystickInfo zmq_msg{};

    constexpr int kMaxChannels = sizeof(zmq_msg.channel) / sizeof(zmq_msg.channel[0]);
    constexpr int kMaxButtons  = sizeof(zmq_msg.button)  / sizeof(zmq_msg.button[0]);

    int channel_num = std::min<int>(static_cast<int>(msg.channel_.size()), kMaxChannels);
    int button_num  = std::min<int>(static_cast<int>(msg.button_.size()),  kMaxButtons);

    zmq_msg.mode = msg.mode_;
    // Carry the physical side in-payload regardless of the selected topic.
    zmq_msg.side = static_cast<int8_t>(msg.side_);
    zmq_msg.channel_num = static_cast<int8_t>(channel_num);
    zmq_msg.button_num  = static_cast<int8_t>(button_num);
    for (int i = 0; i < channel_num; i++) {
        zmq_msg.channel[i] = msg.channel_[i];
    }
    for (int i = 0; i < button_num; i++) {
        zmq_msg.button[i] = msg.button_[i];
    }
    // raw_channel reuses channel_num for the populated count (same index layout
    // as channel). Unpopulated trailing entries stay at the value-initialised 0.
    int raw_channel_num = std::min<int>(static_cast<int>(msg.raw_channel_.size()), channel_num);
    for (int i = 0; i < raw_channel_num; i++) {
        zmq_msg.raw_channel[i] = msg.raw_channel_[i];
    }

    zmq::message_t topic_msg(topic_joystick_.c_str(), topic_joystick_.length());
    zmq::message_t message(&zmq_msg, sizeof(zmq_msg));

    pub_joystick_.send(topic_msg, zmq::send_flags::sndmore);
    pub_joystick_.send(message, zmq::send_flags::dontwait);

    // Topic::publish(const MsgJoystick&) is pure virtual, so no base call is needed.
    return ReturnCode::SUCCESS;
}

ReturnCode TopicZmq::process_leader_msg_joystick(ZmqJoystickInfo* p_zmq_msg, MsgJoystick::Side topic_side) {

    MsgJoystick msg_joystick;

    // The counts come straight off the wire and bound reads of the fixed
    // payload arrays. Reject malformed packets rather than silently executing
    // a truncated operator command.
    constexpr int kMaxChannels =
        static_cast<int>(sizeof(p_zmq_msg->channel) / sizeof(p_zmq_msg->channel[0]));
    constexpr int kMaxButtons =
        static_cast<int>(sizeof(p_zmq_msg->button) / sizeof(p_zmq_msg->button[0]));
    const int channel_num = static_cast<unsigned char>(p_zmq_msg->channel_num);
    const int button_num = static_cast<unsigned char>(p_zmq_msg->button_num);
    if (channel_num > kMaxChannels || button_num > kMaxButtons) {
        PI_ERROR("Invalid joystick counts: channels=%d (max: %d), buttons=%d (max: %d)", channel_num,
                 kMaxChannels, button_num, kMaxButtons);
        return ReturnCode::INVALID_PARAM;
    }

    for (int i = 0; i < channel_num; i++) {
        msg_joystick.channel_.push_back(p_zmq_msg->channel[i]);
        msg_joystick.raw_channel_.push_back(p_zmq_msg->raw_channel[i]);
    }
    for (int i = 0; i < button_num; i++) {
        msg_joystick.button_.push_back(p_zmq_msg->button[i]);
    }
    msg_joystick.mode_ = p_zmq_msg->mode;

    // Prefer the publisher-supplied (reconciled) side from the in-payload
    // ``side`` byte; only fall back to the topic-name-derived side when the
    // byte is out of range (legacy publishers without the field leave the
    // leading-pad byte at zero, which is also LEFT — same behavior as before).
    const int8_t payload_side = p_zmq_msg->side;
    if (payload_side == static_cast<int8_t>(MsgJoystick::Side::LEFT) ||
        payload_side == static_cast<int8_t>(MsgJoystick::Side::RIGHT)) {
        msg_joystick.side_ = static_cast<MsgJoystick::Side>(payload_side);
    } else {
        msg_joystick.side_ = topic_side;
    }

    return Topic::process_leader_msg(msg_joystick);
}

ReturnCode TopicZmq::process_follower_msg_joint(ZmqJointInfo* p_zmq_msg) {

    if (p_zmq_msg->msg_type != (int)MsgType::JOINT_INFO) {
        PI_ERROR("Received message is not a joint info message: %d", p_zmq_msg->msg_type);
        return ReturnCode::INVALID_PARAM;
    }

    const int dof_total = p_device_->get_dof_total();
    if (!joint_info_count_is_valid(*p_zmq_msg, dof_total)) {
        return ReturnCode::INVALID_PARAM;
    }

    MsgJoints msg;
    for (int i = 0; i < dof_total; i++) {
        // Follower state ingest: forward the publisher's frame age as-is.
        msg.add_joint_info(p_zmq_msg->joint_pos[i], p_zmq_msg->joint_vel[i], p_zmq_msg->joint_tor[i],
                           p_zmq_msg->temperature[i], p_zmq_msg->idc_current[i], p_zmq_msg->joint_age_ms[i]);
    }
    msg.measured_idc_current_ = p_zmq_msg->measured_idc_current;

    return Topic::process_follower_msg(msg);
}

int TopicZmq::generate_port_from_topic(const std::string& topic_name) {
    // Backward-compatible helper: return the first candidate.
    const std::vector<int> ports = generate_port_candidates_from_topic(topic_name, /*max_candidates=*/1);
    return ports.empty() ? BASE_PORT_JOINT_DYNAMIC : ports[0];
}

static uint32_t fnv1a_32(const std::string& s) {
    uint32_t h = 2166136261u;
    for (unsigned char c : s) {
        h ^= static_cast<uint32_t>(c);
        h *= 16777619u;
    }
    return h;
}

static int probe_step(uint32_t h, int range) {
    if (range <= 1) {
        return 1;
    }
    int step = static_cast<int>(h % static_cast<uint32_t>(range));
    if (step == 0) {
        step = 1;
    }
    // Make it odd.
    if (step % 2 == 0) {
        step += 1;
    }
    // Avoid multiples of 5 for typical ranges (e.g. 1000 = 2^3 * 5^3).
    while (step % 5 == 0) {
        step += 2;
    }
    step %= range;
    return step == 0 ? 1 : step;
}

std::vector<int> TopicZmq::generate_port_candidates_from_topic(const std::string& topic_name, int max_candidates) {
    const int range = PORT_RANGE_JOINT_DYNAMIC;
    const int base = BASE_PORT_JOINT_DYNAMIC;
    if (range <= 0 || max_candidates <= 0) {
        return {};
    }
    const int k = std::min(max_candidates, range);

    const uint32_t h1 = fnv1a_32(topic_name);
    const uint32_t h2 = fnv1a_32(topic_name + std::string("\x1fprobe"));
    const int step = probe_step(h2, range);

    std::vector<int> out;
    out.reserve(static_cast<size_t>(k));
    for (int i = 0; i < k; i++) {
        const int port = base + static_cast<int>((h1 + static_cast<uint32_t>(i * step)) % static_cast<uint32_t>(range));
        out.push_back(port);
    }
    return out;
}

ReturnCode TopicZmq::init_port(zmq::socket_t& port, const ZmqPortType& port_type, const std::string& topic_name) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    const std::vector<int> candidates = generate_port_candidates_from_topic(
        topic_name, (port_type == ZmqPortType::PUB) ? kZmqPortProbeCandidatesPub : kZmqPortProbeCandidatesSub);

    // Required reliability settings (avoid unbounded memory on slow consumers).
    port.set(zmq::sockopt::linger, 0);
    port.set(zmq::sockopt::sndhwm, kZmqSndHwm);
    port.set(zmq::sockopt::rcvhwm, kZmqRcvHwm);

    if (port_type == ZmqPortType::PUB) {
        std::runtime_error last_error("ZMQ bind failed");
        for (int joint_port_address : candidates) {
            std::string joint_address = "tcp://127.0.0.1:" + std::to_string(joint_port_address);
            try {
                port.bind(joint_address);
                PI_INFO("Topic", InfoLevel::ESSENTIAL_0, "%s_%s will publish to topic %s on port %d",
                        p_device_->get_model().c_str(), p_device_->get_id().c_str(), topic_name.c_str(), joint_port_address);
                return return_code;
            } catch (const zmq::error_t& e) {
                // If the port is already in use, try the next candidate.
                if (e.num() == EADDRINUSE) {
                    continue;
                }

                // Log detailed error information including the address and port
                PI_ERROR("Failed to bind ZMQ publisher socket");
                PI_ERROR("  Address: %s", joint_address.c_str());
                PI_ERROR("  Topic: %s", topic_name.c_str());
                PI_ERROR("  Port: %d", joint_port_address);
                PI_ERROR("  Device: %s_%s", p_device_->get_model().c_str(), p_device_->get_id().c_str());
                PI_ERROR("  Error message: %s", e.what());
                PI_ERROR("  Error code: %d", e.num());

                std::string detailed_error = "ZMQ bind failed: Address=" + joint_address +
                                             ", Topic=" + topic_name +
                                             ", Port=" + std::to_string(joint_port_address) +
                                             ", Error=" + std::string(e.what());
                last_error = std::runtime_error(detailed_error);
                break;
            }
        }
        // If we got here, we exhausted candidates (or hit a non-EADDRINUSE error).
        throw last_error;
    } else if (port_type == ZmqPortType::SUB) {
        const std::string host = "127.0.0.1";
        try {
            port.set(zmq::sockopt::subscribe, topic_name.c_str());
            for (int joint_port_address : candidates) {
                std::string joint_address = "tcp://" + host + ":" + std::to_string(joint_port_address);
                port.connect(joint_address);
            }
            // Log only the first candidate to avoid noisy logs.
            if (!candidates.empty()) {
                PI_INFO("Topic", InfoLevel::ESSENTIAL_0, "%s_%s subscribed to topic %s on port %d",
                        p_device_->get_model().c_str(), p_device_->get_id().c_str(), topic_name.c_str(), candidates[0]);
            }
        } catch (const zmq::error_t& e) {
            // Use the first candidate for error reporting context.
            const int joint_port_address = candidates.empty() ? generate_port_from_topic(topic_name) : candidates[0];
            std::string joint_address = "tcp://" + host + ":" + std::to_string(joint_port_address);
            // Log detailed error information including the address and port
            PI_ERROR("Failed to connect ZMQ subscriber socket");
            PI_ERROR("  Address: %s", joint_address.c_str());
            PI_ERROR("  Topic: %s", topic_name.c_str());
            PI_ERROR("  Port: %d", joint_port_address);
            PI_ERROR("  Device: %s_%s", p_device_->get_model().c_str(), p_device_->get_id().c_str());
            PI_ERROR("  Error message: %s", e.what());
            PI_ERROR("  Error code: %d", e.num());

            // Re-throw as std::runtime_error with detailed message for exception handler
            std::string detailed_error = "ZMQ connect failed: Address=" + joint_address +
                                        ", Topic=" + topic_name +
                                        ", Port=" + std::to_string(joint_port_address) +
                                        ", Error=" + std::string(e.what());
            throw std::runtime_error(detailed_error);
        }
    }

    return return_code;
}
