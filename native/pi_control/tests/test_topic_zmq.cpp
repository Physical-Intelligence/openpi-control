#include <atomic>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <unistd.h>
#include <zmq.hpp>

#include "pi_device.hpp"
#include "pi_profile.hpp"

#define private public
#include "pi_topic_zmq.hpp"
#undef private

namespace {

class TestDevice : public Device {
   public:
    TestDevice(const CommandLineArgs& cla, int dof_total) : Device(cla) {
        role_ = cla.role;
        dof_total_ = dof_total;
    }

    ReturnCode apply_action(const MsgJoints&) override {
        ++joint_action_count;
        return ReturnCode::SUCCESS;
    }
    ReturnCode apply_action(const MsgJoystick&) override {
        ++joystick_action_count;
        return ReturnCode::SUCCESS;
    }
    ReturnCode get_observation(MsgJoints&) override { return ReturnCode::SUCCESS; }
    ReturnCode process_follower_msg(const MsgJoints&) override {
        ++follower_action_count;
        return ReturnCode::SUCCESS;
    }
    ReturnCode read_hardware_values() override { return ReturnCode::SUCCESS; }
    ReturnCode write_hardware_values() override { return ReturnCode::SUCCESS; }
    ReturnCode move_to_ready_position() override { return ReturnCode::SUCCESS; }
    ReturnCode operate_as_leader() override { return ReturnCode::SUCCESS; }
    ReturnCode operate_as_follower() override { return ReturnCode::SUCCESS; }
    ReturnCode get_servo_ids(std::vector<int>&) override { return ReturnCode::SUCCESS; }
    ReturnCode set_control_mode(Role, ControlModeIntent) override { return ReturnCode::SUCCESS; }

    int joint_action_count = 0;
    int joystick_action_count = 0;
    int follower_action_count = 0;
};

CommandLineArgs make_cla() {
    static std::atomic<unsigned int> next_id{0};
    const std::string suffix = std::to_string(getpid()) + "_" + std::to_string(next_id.fetch_add(1));

    CommandLineArgs cla{};
    cla.device_model = "topic_zmq_test";
    cla.device_id = suffix;
    cla.role = Role::FOLLOWER;
    cla.control_frequency = 100;
    cla.moving_mode = MovingMode::PARALLEL;
    cla.msg_type = MsgType::JOINT_INFO;
    cla.force_feedback = -1.0f;
    cla.topic_joint = "topic_zmq_test_joint_" + suffix;
    cla.topic_state = "topic_zmq_test_state_" + suffix;
    cla.topic_lifecycle = "topic_zmq_test_lifecycle_" + suffix;
    cla.topic_status = "topic_zmq_test_status_" + suffix;
    cla.topic_joystick = OPT_DEFAULT_NONE;
    return cla;
}

class TopicZmqPublishBoundsTest : public testing::Test {
   protected:
    CommandLineArgs cla_ = make_cla();
    TestDevice device_{cla_, 11};
    TopicZmq topic_{&device_, cla_, 0, nullptr};
};

TEST_F(TopicZmqPublishBoundsTest, RejectsJointPayloadLargerThanWireArray) {
    MsgJoints msg;
    for (int i = 0; i < 11; ++i) {
        msg.add_joint_info(i, i, i, i, i, -1.0f);
    }

    EXPECT_EQ(topic_.publish(msg), ReturnCode::INVALID_PARAM);
}

TEST_F(TopicZmqPublishBoundsTest, RejectsEffectorPayloadLargerThanWireArray) {
    std::vector<float> params(31, 1.0f);
    MsgEffectorInfo msg(&params);

    EXPECT_EQ(topic_.publish(msg), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqPublishAvailability, RejectsMissingFollowerStatePublisher) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.pub_follower_joint_.close();
    MsgJoints msg;
    msg.add_joint_info(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, -1.0f);

    EXPECT_EQ(topic.publish(msg), ReturnCode::NOT_INITIALIZED);
}

TEST(TopicZmqPublishAvailability, RejectsMissingEffectorInfoPublisher) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.pub_effector_info_.close();
    std::vector<float> params{0.0f};
    MsgEffectorInfo msg(&params);

    EXPECT_EQ(topic.publish(msg), ReturnCode::NOT_INITIALIZED);
}

TEST_F(TopicZmqPublishBoundsTest, RejectsLeaderJointPayloadLargerThanWireArray) {
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 11;

    EXPECT_EQ(topic_.process_leader_msg_joint(&msg), ReturnCode::INVALID_PARAM);
}

TEST_F(TopicZmqPublishBoundsTest, RejectsFollowerJointPayloadLargerThanWireArray) {
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 11;

    EXPECT_EQ(topic_.process_follower_msg_joint(&msg), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqJointCount, RejectsLeaderPayloadForDifferentDof) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.set_pause_leader_command_listening(false);
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 2;

    EXPECT_EQ(topic.process_leader_msg_joint(&msg), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.joint_action_count, 0);
}

TEST(TopicZmqJointCount, RejectsFollowerPayloadForDifferentDof) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    ZmqJointInfo msg{};
    msg.msg_type = (int)MsgType::JOINT_INFO;
    msg.joint_num = 2;

    EXPECT_EQ(topic.process_follower_msg_joint(&msg), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqStep, PreservesEarlierDispatchFailureAcrossStreams) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string leader_endpoint =
        "inproc://topic_zmq_step_leader_" + cla.device_id;
    topic.sub_joint_.bind(leader_endpoint);
    zmq::socket_t leader_sender(topic.context_, ZMQ_PAIR);
    leader_sender.connect(leader_endpoint);

    topic.sub_follower_joint_.close();
    topic.sub_follower_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string follower_endpoint =
        "inproc://topic_zmq_step_follower_" + cla.device_id;
    topic.sub_follower_joint_.bind(follower_endpoint);
    zmq::socket_t follower_sender(topic.context_, ZMQ_PAIR);
    follower_sender.connect(follower_endpoint);

    ZmqJointInfo invalid_leader{};
    invalid_leader.msg_type = -1;
    invalid_leader.joint_num = 1;
    leader_sender.send(
        zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    leader_sender.send(zmq::buffer(&invalid_leader, sizeof(invalid_leader)),
                       zmq::send_flags::none);

    ZmqJointInfo valid_follower{};
    valid_follower.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    valid_follower.joint_num = 1;
    follower_sender.send(
        zmq::buffer(topic.topic_follower_joint_), zmq::send_flags::sndmore);
    follower_sender.send(zmq::buffer(&valid_follower, sizeof(valid_follower)),
                         zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.follower_action_count, 1);
}

TEST(TopicZmqStep, ReportsMalformedLifecyclePayload) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_command_.close();
    topic.sub_command_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_lifecycle_" + cla.device_id;
    topic.sub_command_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    const uint8_t malformed_payload = 0;
    sender.send(zmq::buffer(topic.topic_lifecycle_), zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&malformed_payload, sizeof(malformed_payload)),
                zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqStep, StopDoesNotDispatchQueuedJointCommand) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.set_pause_leader_command_listening(false);

    topic.sub_command_.close();
    topic.sub_command_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string lifecycle_endpoint =
        "inproc://topic_zmq_step_stop_lifecycle_" + cla.device_id;
    topic.sub_command_.bind(lifecycle_endpoint);
    zmq::socket_t lifecycle_sender(topic.context_, ZMQ_PAIR);
    lifecycle_sender.connect(lifecycle_endpoint);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string joint_endpoint =
        "inproc://topic_zmq_step_stop_joint_" + cla.device_id;
    topic.sub_joint_.bind(joint_endpoint);
    zmq::socket_t joint_sender(topic.context_, ZMQ_PAIR);
    joint_sender.connect(joint_endpoint);

    ZmqCommand stop{};
    stop.command = DEVICE_COMMAND_STOP;
    lifecycle_sender.send(
        zmq::buffer(topic.topic_lifecycle_), zmq::send_flags::sndmore);
    lifecycle_sender.send(zmq::buffer(&stop, sizeof(stop)), zmq::send_flags::none);

    ZmqJointInfo joint{};
    joint.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    joint.joint_num = 1;
    joint_sender.send(zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    joint_sender.send(zmq::buffer(&joint, sizeof(joint)), zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::SUCCESS);
    EXPECT_FALSE(topic.is_running());
    EXPECT_EQ(device.joint_action_count, 0);
}

TEST(TopicZmqStep, ReportsMalformedJointPayload) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_malformed_joint_" + cla.device_id;
    topic.sub_joint_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    const uint8_t malformed_payload = 0;
    sender.send(zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&malformed_payload, sizeof(malformed_payload)),
                zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.joint_action_count, 0);
}

TEST(TopicZmqStep, EmptyJointTopicDoesNotStarveLaterValidFrame) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);
    topic.set_pause_leader_command_listening(false);

    topic.sub_joint_.close();
    topic.sub_joint_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_empty_joint_topic_" + cla.device_id;
    topic.sub_joint_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    zmq::message_t empty_topic;
    ZmqJointInfo discarded_payload{};
    sender.send(empty_topic, zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&discarded_payload, sizeof(discarded_payload)),
                zmq::send_flags::none);

    ZmqJointInfo valid_payload{};
    valid_payload.msg_type = static_cast<int>(MsgType::JOINT_INFO);
    valid_payload.joint_num = 1;
    sender.send(zmq::buffer(topic.topic_joint_), zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&valid_payload, sizeof(valid_payload)),
                zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(device.joint_action_count, 1);
}

TEST(TopicZmqStep, ReportsEmptyLifecycleTopic) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 1);
    TopicZmq topic(&device, cla, 0, nullptr);

    topic.sub_command_.close();
    topic.sub_command_ = zmq::socket_t(topic.context_, ZMQ_PAIR);
    const std::string endpoint =
        "inproc://topic_zmq_step_empty_lifecycle_topic_" + cla.device_id;
    topic.sub_command_.bind(endpoint);
    zmq::socket_t sender(topic.context_, ZMQ_PAIR);
    sender.connect(endpoint);

    zmq::message_t empty_topic;
    ZmqCommand payload{};
    sender.send(empty_topic, zmq::send_flags::sndmore);
    sender.send(zmq::buffer(&payload, sizeof(payload)), zmq::send_flags::none);

    EXPECT_EQ(topic.step(), ReturnCode::INVALID_PARAM);
}

TEST(TopicZmqJoystickCount, RejectsInvalidCountsBeforeDispatch) {
    CommandLineArgs cla = make_cla();
    TestDevice device(cla, 0);
    TopicZmq topic(&device, cla, 0, nullptr);
    ZmqJoystickInfo msg{};

    msg.channel_num = static_cast<int>(sizeof(msg.channel) / sizeof(msg.channel[0])) + 1;
    EXPECT_EQ(
        topic.process_leader_msg_joystick(&msg, MsgJoystick::Side::LEFT),
        ReturnCode::INVALID_PARAM);

    msg.channel_num = 0;
    msg.button_num = -1;
    EXPECT_EQ(
        topic.process_leader_msg_joystick(&msg, MsgJoystick::Side::LEFT),
        ReturnCode::INVALID_PARAM);

    EXPECT_EQ(device.joystick_action_count, 0);
}

TEST(TopicZmqInitializationDeathTest, RejectsZeroControlFrequency) {
    CommandLineArgs cla = make_cla();
    cla.control_frequency = 0;
    TestDevice device(cla, 1);

    EXPECT_EXIT(
        {
            try {
                TopicZmq topic(&device, cla, 0, nullptr);
            } catch (const std::invalid_argument&) {
                _exit(0);
            } catch (...) {
                _exit(2);
            }
            _exit(1);
        },
        testing::ExitedWithCode(0), "");
}

TEST(TopicZmqInitializationDeathTest, RejectsNegativeControlFrequency) {
    CommandLineArgs cla = make_cla();
    cla.control_frequency = -1;
    TestDevice device(cla, 1);

    EXPECT_EXIT(
        {
            try {
                TopicZmq topic(&device, cla, 0, nullptr);
            } catch (const std::invalid_argument&) {
                _exit(0);
            } catch (...) {
                _exit(2);
            }
            _exit(1);
        },
        testing::ExitedWithCode(0), "");
}

}  // namespace
