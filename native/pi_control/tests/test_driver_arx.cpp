#include <cstddef>

#include <gtest/gtest.h>

#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

#include "pi_device.hpp"

#define private public
#include "pi_driver_arx.hpp"
#undef private

#include "pi_driver_arx_encoder.hpp"
#include "pi_servo_can_encoder.hpp"
#include "pi_servo_dm.hpp"

namespace {

class DriverArxTestDevice : public Device {
   public:
    explicit DriverArxTestDevice(const CommandLineArgs& cla) : Device(cla) {}

    ReturnCode apply_action(const MsgJoints&) override { return ReturnCode::SUCCESS; }
    ReturnCode get_observation(MsgJoints&) override { return ReturnCode::SUCCESS; }
    ReturnCode process_follower_msg(const MsgJoints&) override { return ReturnCode::SUCCESS; }
    ReturnCode read_hardware_values() override { return ReturnCode::SUCCESS; }
    ReturnCode write_hardware_values() override { return ReturnCode::SUCCESS; }
    ReturnCode move_to_ready_position() override { return ReturnCode::SUCCESS; }
    ReturnCode operate_as_leader() override { return ReturnCode::SUCCESS; }
    ReturnCode operate_as_follower() override { return ReturnCode::SUCCESS; }
    ReturnCode get_servo_ids(std::vector<int>&) override { return ReturnCode::SUCCESS; }
    ReturnCode set_control_mode(Role, ControlModeIntent) override { return ReturnCode::SUCCESS; }
};

class DriverArxTestOtherDriver : public Driver {
   public:
    DriverArxTestOtherDriver(Device* p_device, const CommandLineArgs& cla) : Driver(p_device, cla) {}

    ReturnCode open(int) override { return ReturnCode::SUCCESS; }
    ReturnCode close() override { return ReturnCode::SUCCESS; }
};

class DriverArxTestServoDm : public ServoDm {
   public:
    DriverArxTestServoDm(Device* p_device, Driver* p_driver, const ServoDmParam* p_param, int id, ServoType type)
        : ServoDm(p_device, nullptr, p_driver) {
        id_ = id;
        data_index_ = 0;
        type_ = type;
        p_servo_param_ = p_param;
    }

    bool has_arx_driver() const { return p_driver_can_ != nullptr; }
};

TEST(ServoPositionWrap, FreshArxCacheRecoversYambotOpenGripperReadingsAfterPowerCycle) {
    constexpr float kTwoPi = 6.283185307179586f;
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArx driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};

    for (const float absolute_position : {1.165f, 0.979f}) {
        DriverArxTestServoDm servo(&device, &driver, &param, 7, ServoType::DM_4310);
        servo.dir_invert_ = -1;
        servo.zero_pos_abs_ = 0.0f;
        servo.pos_min_rel_ = 0.0f;
        servo.pos_max_rel_ = 5.4f;
        servo.position_wrap_period_ = kTwoPi;
        driver.received_servo_data_[servo.data_index_].motor_id_ = servo.id_;
        driver.received_servo_data_[servo.data_index_].angle_actual_rad_ = absolute_position;

        ASSERT_EQ(driver.read_hardware_values(&servo), ReturnCode::SUCCESS);
        EXPECT_NEAR(servo.get_pos_rad_relative(), kTwoPi - absolute_position, 1e-5f);
        EXPECT_NEAR(servo.get_pos_rad_absolute(servo.get_pos_rad_relative()), absolute_position, 1e-5f);
    }
}

TEST(ServoPositionWrap, LeavesTrueOutOfRangePositionForNormalLimitHandling) {
    constexpr float kTwoPi = 6.283185307179586f;
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArx driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 7, ServoType::DM_4310);
    servo.dir_invert_ = -1;
    servo.zero_pos_abs_ = 0.0f;
    servo.pos_min_rel_ = 0.0f;
    servo.pos_max_rel_ = 5.4f;
    servo.position_wrap_period_ = kTwoPi;
    servo.curr_pos_abs_ = -12.0f;

    ASSERT_EQ(servo.initialize_position_wrap(), ReturnCode::SUCCESS);
    EXPECT_FLOAT_EQ(servo.position_wrap_offset_rel_, 0.0f);
    EXPECT_FLOAT_EQ(servo.get_pos_rad_relative(), 12.0f);
}

class SocketBackedDriverArx : public DriverArx {
   public:
    using DriverArx::DriverArx;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
};

class RestartFailingDriverArx : public DriverArx {
   public:
    using DriverArx::DriverArx;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }

    ReturnCode send_frame(void*, size_t) override { return ReturnCode::SUCCESS; }

    ReturnCode read_frame(void* p_data_buf, size_t data_buf_size) override {
        if (data_buf_size < sizeof(can_frame_t)) {
            return ReturnCode::INVALID_PARAM;
        }

        auto* frame = static_cast<can_frame_t*>(p_data_buf);
        *frame = {};
        frame->can_id = 0x11;
        frame->can_dlc = 8;
        frame->data[0] = 0x01;
        sock_ = -1;
        return ReturnCode::SUCCESS;
    }
};

class ZeroDescriptorDriverArx : public DriverArx {
   public:
    using DriverArx::DriverArx;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
    void abandon_socket() { sock_ = -1; }
    size_t send_count() const { return send_count_; }

    ReturnCode send_frame(void*, size_t) override {
        ++send_count_;
        return ReturnCode::SUCCESS;
    }

   private:
    size_t send_count_ = 0;
};

class EnableSendFailingDriverArx : public DriverArx {
   public:
    using DriverArx::DriverArx;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
    bool reception_running() const { return is_running_.load(std::memory_order_acquire); }

    ReturnCode send_frame(void*, size_t) override {
        return send_count_++ == 0 ? ReturnCode::SUCCESS : ReturnCode::FAIL;
    }

   private:
    size_t send_count_ = 0;
};

class FailingSendDriverArx : public DriverArx {
   public:
    using DriverArx::DriverArx;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
    void abandon_socket() { sock_ = -1; }
    size_t send_count() const { return send_count_; }

    ReturnCode send_frame(void*, size_t) override {
        ++send_count_;
        return ReturnCode::BUSY;
    }

   private:
    size_t send_count_ = 0;
};

class SetupSendFailingDriverArx : public DriverArx {
   public:
    SetupSendFailingDriverArx(Device* p_device, const CommandLineArgs& cla, size_t fail_on_send)
        : DriverArx(p_device, cla), fail_on_send_(fail_on_send) {}

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
    void abandon_socket() { sock_ = -1; }
    size_t send_count() const { return send_count_; }
    bool reception_running() const { return is_running_.load(std::memory_order_acquire); }

    ReturnCode send_frame(void*, size_t) override {
        ++send_count_;
        return send_count_ == fail_on_send_ ? ReturnCode::BUSY : ReturnCode::SUCCESS;
    }

    ReturnCode read_frame(void* p_data_buf, size_t data_buf_size) override {
        if (data_buf_size < sizeof(can_frame_t)) {
            return ReturnCode::INVALID_PARAM;
        }

        auto* frame = static_cast<can_frame_t*>(p_data_buf);
        *frame = {};
        frame->can_id = 0x11;
        frame->can_dlc = 8;
        frame->data[0] = 0x01;
        return ReturnCode::SUCCESS;
    }

   private:
    size_t fail_on_send_;
    size_t send_count_ = 0;
};

class ScriptedEnableStatusDriverArx : public DriverArx {
   public:
    ScriptedEnableStatusDriverArx(Device* p_device, const CommandLineArgs& cla, std::vector<uint8_t> statuses)
        : DriverArx(p_device, cla), statuses_(std::move(statuses)) {}

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
    size_t enable_frame_count() const { return enable_frame_count_; }
    size_t reset_frame_count() const { return reset_frame_count_; }

    ReturnCode send_frame(void* p_data_buf, size_t data_buf_size) override {
        if (p_data_buf == nullptr || data_buf_size < sizeof(can_frame_t)) {
            return ReturnCode::INVALID_PARAM;
        }
        const auto& frame = *static_cast<can_frame_t*>(p_data_buf);
        bool is_special_frame = frame.can_dlc == 8;
        for (size_t i = 0; i < 7 && is_special_frame; ++i) {
            is_special_frame = frame.data[i] == 0xFF;
        }
        if (is_special_frame && (frame.data[7] == 0xFC || frame.data[7] == 0xFD)) {
            ++enable_frame_count_;
        } else if (is_special_frame && frame.data[7] == 0xFB) {
            ++reset_frame_count_;
        }
        return ReturnCode::SUCCESS;
    }

    ReturnCode read_frame(void* p_data_buf, size_t data_buf_size) override {
        if (p_data_buf == nullptr || data_buf_size < sizeof(can_frame_t) || statuses_.empty()) {
            return ReturnCode::INVALID_PARAM;
        }
        const uint8_t status = statuses_[std::min(status_index_, statuses_.size() - 1)];
        ++status_index_;
        auto* frame = static_cast<can_frame_t*>(p_data_buf);
        *frame = {};
        frame->can_id = 0x11;
        frame->can_dlc = 8;
        frame->data[0] = static_cast<uint8_t>((status << 4) | 0x01);
        frame->data[7] = 25;
        return ReturnCode::SUCCESS;
    }

   private:
    std::vector<uint8_t> statuses_;
    size_t status_index_ = 0;
    size_t enable_frame_count_ = 0;
    size_t reset_frame_count_ = 0;
};

class ConcurrentEnableProbeDriverArx : public DriverArx {
   public:
    using DriverArx::DriverArx;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }
    bool overlap_detected() const { return overlap_detected_.load(std::memory_order_acquire); }

    ReturnCode send_frame(void* p_data_buf, size_t data_buf_size) override {
        if (p_data_buf == nullptr || data_buf_size < sizeof(can_frame_t)) {
            return ReturnCode::INVALID_PARAM;
        }

        const auto& frame = *static_cast<can_frame_t*>(p_data_buf);
        if (frame.can_dlc != 8 || frame.data[7] != 0xFD) {
            return ReturnCode::SUCCESS;
        }

        std::unique_lock<std::mutex> lock(probe_mutex_);
        const int motor_id = static_cast<int>(frame.can_id & 0x7FF);
        expected_id_by_thread_[std::this_thread::get_id()] = motor_id;
        if (handshake_owner_ < 0) {
            handshake_owner_ = motor_id;
            probe_cv_.wait_for(lock, std::chrono::milliseconds(500), [this] {
                return overlap_detected_.load(std::memory_order_acquire);
            });
            return ReturnCode::SUCCESS;
        }

        if (handshake_owner_ != motor_id) {
            overlap_detected_.store(true, std::memory_order_release);
            probe_cv_.notify_all();
            return ReturnCode::BUSY;
        }
        return ReturnCode::SUCCESS;
    }

    ReturnCode read_frame(void* p_data_buf, size_t data_buf_size) override {
        if (p_data_buf == nullptr || data_buf_size < sizeof(can_frame_t)) {
            return ReturnCode::INVALID_PARAM;
        }

        std::lock_guard<std::mutex> lock(probe_mutex_);
        const auto expected = expected_id_by_thread_.find(std::this_thread::get_id());
        if (expected == expected_id_by_thread_.end()) {
            return ReturnCode::FAIL;
        }

        auto* frame = static_cast<can_frame_t*>(p_data_buf);
        *frame = {};
        frame->can_dlc = 8;
        frame->data[0] = static_cast<uint8_t>(expected->second);
        handshake_owner_ = -1;
        return ReturnCode::SUCCESS;
    }

   private:
    mutable std::mutex probe_mutex_;
    std::condition_variable probe_cv_;
    std::map<std::thread::id, int> expected_id_by_thread_;
    int handshake_owner_ = -1;
    std::atomic<bool> overlap_detected_{false};
};

class EnableTransactionProbeDriverArx : public DriverArx {
   public:
    using DriverArx::DriverArx;

    void adopt_socket(int socket_fd) { sock_ = socket_fd; }

    bool wait_for_handshake_start() {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
        while (std::chrono::steady_clock::now() < deadline) {
            if (handshake_active_.load(std::memory_order_acquire)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        return false;
    }

    bool overlap_detected() const { return overlap_detected_.load(std::memory_order_acquire); }

    ReturnCode send_frame(void* p_data_buf, size_t data_buf_size) override {
        if (p_data_buf == nullptr || data_buf_size < sizeof(can_frame_t)) {
            return ReturnCode::INVALID_PARAM;
        }

        const auto& frame = *static_cast<can_frame_t*>(p_data_buf);
        bool is_enable_frame = frame.can_dlc == 8 && (frame.data[7] == 0xFC || frame.data[7] == 0xFD);
        for (size_t i = 0; i < 7 && is_enable_frame; ++i) {
            is_enable_frame = frame.data[i] == 0xFF;
        }

        if (is_enable_frame) {
            handshake_active_.store(true, std::memory_order_release);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            return ReturnCode::SUCCESS;
        }

        if (handshake_active_.load(std::memory_order_acquire)) {
            overlap_detected_.store(true, std::memory_order_release);
        }
        return ReturnCode::SUCCESS;
    }

    ReturnCode read_frame(void* p_data_buf, size_t data_buf_size) override {
        if (p_data_buf == nullptr || data_buf_size < sizeof(can_frame_t)) {
            return ReturnCode::INVALID_PARAM;
        }

        auto* frame = static_cast<can_frame_t*>(p_data_buf);
        *frame = {};
        frame->can_dlc = 8;
        frame->data[0] = 1;

        handshake_active_.store(false, std::memory_order_release);
        return ReturnCode::SUCCESS;
    }

   private:
    std::atomic<bool> handshake_active_{false};
    std::atomic<bool> overlap_detected_{false};
};

namespace {

// handle_received_message is protected (invoked by the receive loop only);
// expose it for direct exercise in tests.
class DriverArxMessageProbe : public DriverArx {
   public:
    using DriverArx::DriverArx;
    using DriverArx::handle_received_message;
};

}  // namespace

TEST(DriverArxTransport, RejectsTruncatedReceivedFrames) {
    CommandLineArgs cla{};
    DriverArxMessageProbe driver(nullptr, cla);
    std::vector<uint8_t> truncated_frame(1);

    driver.handle_received_message(truncated_frame.data(), truncated_frame.size(), truncated_frame.size());
}

namespace {

// handle_received_message is protected on the encoder driver too; expose it
// for direct frame-decoding assertions.
class DriverArxEncoderMessageProbe : public DriverArxEncoder {
   public:
    using DriverArxEncoder::DriverArxEncoder;
    using DriverArxEncoder::handle_received_message;
};

}  // namespace

TEST(DriverArxEncoderDecode, DecodesTwoByteAngleFrameIntoCacheSlot) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArxEncoderMessageProbe driver(&device, cla);
    ServoDmParam param{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1537, ServoType::ARX_ENCODER);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    // raw = ARX_ENCODER_ZERO_RAW + 2048 -> +2048 * pi / 4096 = +pi/2 rad.
    const uint16_t raw = ARX_ENCODER_ZERO_RAW + 2048;
    DriverCan::can_frame_t frame{};
    frame.can_id = 1537;
    frame.can_dlc = ARX_ENCODER_FEEDBACK_DLC;
    frame.data[0] = static_cast<uint8_t>(raw >> 8);
    frame.data[1] = static_cast<uint8_t>(raw & 0xFF);

    driver.handle_received_message(&frame, sizeof(frame), sizeof(frame));

    EXPECT_EQ(driver.received_servo_data_[servo.data_index_].motor_id_, 1537);
    EXPECT_NEAR(driver.received_servo_data_[servo.data_index_].angle_actual_rad_, 1.5707963f, 1e-5f);

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
}

TEST(DriverArxEncoderDecode, DropsFramesWithWrongDlcOrUnmappedIds) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArxEncoderMessageProbe driver(&device, cla);
    ServoDmParam param{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1537, ServoType::ARX_ENCODER);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    // Wrong DLC on a mapped id: not an encoder report, must not touch the slot.
    DriverCan::can_frame_t wrong_dlc{};
    wrong_dlc.can_id = 1537;
    wrong_dlc.can_dlc = 8;
    driver.handle_received_message(&wrong_dlc, sizeof(wrong_dlc), sizeof(wrong_dlc));
    EXPECT_EQ(driver.received_servo_data_[servo.data_index_].motor_id_, 0);

    // Correct DLC on an unmapped id: not for us, must not touch any slot.
    DriverCan::can_frame_t unmapped{};
    unmapped.can_id = 1600;
    unmapped.can_dlc = ARX_ENCODER_FEEDBACK_DLC;
    driver.handle_received_message(&unmapped, sizeof(unmapped), sizeof(unmapped));
    EXPECT_EQ(driver.received_servo_data_[servo.data_index_].motor_id_, 0);

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
}

TEST(DriverArxEncoderLifecycle, ActuationEntryPointsAreNoOps) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArxEncoder driver(&device, cla);
    ServoDmParam param{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1537, ServoType::ARX_ENCODER);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);
    // Pre-populate the slot so enable() returns without exhausting the warmup window.
    driver.received_servo_data_[servo.data_index_].motor_id_ = servo.id_;

    // No socket is open: a real command would fail, a no-op succeeds untouched.
    EXPECT_EQ(driver.send_command(&servo, 1.0f, 0.1f, 0.5f, 0.0f, 0.0f), ReturnCode::SUCCESS);
    EXPECT_EQ(driver.reset_zero_position(servo.id_, static_cast<int>(ServoType::ARX_ENCODER)), ReturnCode::SUCCESS);
    EXPECT_EQ(driver.enable(servo.id_, static_cast<int>(ServoType::ARX_ENCODER), true), ReturnCode::SUCCESS);

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
}

TEST(DriverArxEncoderLifecycle, EnableFailsFastForUnmappedIds) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArxEncoder driver(&device, cla);

    EXPECT_EQ(driver.enable(1999, static_cast<int>(ServoType::ARX_ENCODER), true), ReturnCode::FAIL);
}

TEST(ServoDmParser, DmStatusRejectsNullIndexLookup) {
    CommandLineArgs cla{};
    DriverArx driver(nullptr, cla);
    DriverCan::can_frame_t frame{};
    ReceivedServoData cache[MAX_SERVO_INFO_BUF_SIZE]{};
    frame.can_dlc = 8;
    frame.data[0] = 1;

    EXPECT_EQ(ServoDm::parse_dm_servo_status(&frame, cache, nullptr, &driver), ReturnCode::INVALID_PARAM);
}

TEST(ServoDmParser, EncosStatusRejectsNullIndexLookup) {
    DriverCan::can_frame_t frame{};
    ReceivedServoData cache[MAX_SERVO_INFO_BUF_SIZE]{};
    frame.can_dlc = 8;
    frame.can_id = 1;

    EXPECT_EQ(ServoDm::parser_encos_servo_status(&frame, cache, nullptr), ReturnCode::INVALID_PARAM);
}

TEST(ServoDmParser, DmStatusRejectsNullDriver) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArx driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::DM_4310);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    DriverCan::can_frame_t frame{};
    ReceivedServoData cache[MAX_SERVO_INFO_BUF_SIZE]{};
    frame.can_dlc = 8;
    frame.data[0] = static_cast<uint8_t>(servo.id_);

    EXPECT_EQ(ServoDm::parse_dm_servo_status(&frame, cache, &DriverArx::find_data_index, nullptr),
              ReturnCode::INVALID_PARAM);

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
}

TEST(DriverArxTransport, RejectsServoDataIndexOutsideReceiveCache) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArx driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::DM_4310);
    servo.data_index_ = MAX_SERVO_INFO_BUF_SIZE;

    EXPECT_EQ(driver.read_hardware_values(&servo), ReturnCode::INVALID_PARAM);
}

TEST(DriverArxTransport, RejectsNullEncoderSnapshotOutputs) {
    CommandLineArgs cla{};
    DriverArx driver(nullptr, cla);
    float position = 0;
    float velocity = 0;
    uint8_t digital_inputs = 0;
    uint32_t update_count = 0;

    EXPECT_FALSE(driver.get_received_encoder_data(0, nullptr, &velocity, &digital_inputs, &update_count));
    EXPECT_FALSE(driver.get_received_encoder_data(0, &position, nullptr, &digital_inputs, &update_count));
    EXPECT_FALSE(driver.get_received_encoder_data(0, &position, &velocity, nullptr, &update_count));
    EXPECT_FALSE(driver.get_received_encoder_data(0, &position, &velocity, &digital_inputs, nullptr));
}

TEST(DriverArxTransport, RejectsNonDmServoReads) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArx driver(&device, cla);
    ServoCanPassiveEncoder servo(&device, nullptr, &driver);
    servo.id_ = 1;
    servo.data_index_ = 0;

    EXPECT_EQ(driver.read_hardware_values(&servo), ReturnCode::INVALID_PARAM);
}

TEST(ServoDmConstruction, RejectsNonArxDrivers) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArxTestOtherDriver driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::DM_4310);

    ASSERT_FALSE(servo.has_arx_driver());
    EXPECT_EQ(servo.start_hardware(), ReturnCode::NOT_INITIALIZED);
}

TEST(ServoDmConstruction, RejectsNonArxDriversDuringModelInitialization) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    DriverArxTestOtherDriver driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::DM_4310);
    DeviceConfig config;
    const json servo_config = {
        {config.fn_servo_id, 1},
        {config.fn_servo_data_index, 0},
        {config.fn_servo_model, config.val_servo_model_dm_4310},
    };

    EXPECT_EQ(servo.init_config_model(servo_config, &config), ReturnCode::NOT_INITIALIZED);
}

TEST(DriverArxLifecycle, EnablePropagatesReceptionRestartFailure) {
    CommandLineArgs cla{};
    RestartFailingDriverArx driver(nullptr, cla);
    driver.adopt_socket(42);

    EXPECT_EQ(driver.enable(1, static_cast<int>(ServoType::DM_4310), false), ReturnCode::FAIL);
}

TEST(DriverArxLifecycle, EnableRecoversOneStaleCommunicationLossFault) {
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    ScriptedEnableStatusDriverArx driver(nullptr, cla, {0xD, 0x1});
    driver.adopt_socket(sockets[0]);

    EXPECT_EQ(driver.enable(1, static_cast<int>(ServoType::DM_4310), true), ReturnCode::SUCCESS);
    EXPECT_EQ(driver.enable_frame_count(), 2);
    EXPECT_EQ(driver.reset_frame_count(), 3);
    EXPECT_EQ(driver.last_enable_fault_status(), -1);

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
}

TEST(DriverArxLifecycle, EnableFailsAfterOneResetForPersistentCommunicationLoss) {
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    ScriptedEnableStatusDriverArx driver(nullptr, cla, {0xD});
    driver.adopt_socket(sockets[0]);

    EXPECT_EQ(driver.enable(1, static_cast<int>(ServoType::DM_4310), true), ReturnCode::HARDWARE_FAULT);
    EXPECT_EQ(driver.enable_frame_count(), 2);
    EXPECT_EQ(driver.reset_frame_count(), 3);
    EXPECT_EQ(driver.last_enable_fault_status(), 0xD);

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
}

TEST(DriverArxLifecycle, DisableDoesNotResetCommunicationLossFault) {
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    ScriptedEnableStatusDriverArx driver(nullptr, cla, {0xD});
    driver.adopt_socket(sockets[0]);

    EXPECT_EQ(driver.enable(1, static_cast<int>(ServoType::DM_4310), false), ReturnCode::HARDWARE_FAULT);
    EXPECT_EQ(driver.enable_frame_count(), 1);
    EXPECT_EQ(driver.reset_frame_count(), 0);
    EXPECT_EQ(driver.last_enable_fault_status(), 0xD);

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
}

TEST(DriverArxLifecycle, ResetZeroPositionAcceptsDescriptorZero) {
    CommandLineArgs cla{};
    ZeroDescriptorDriverArx driver(nullptr, cla);
    driver.adopt_socket(0);

    EXPECT_EQ(driver.reset_zero_position(1, static_cast<int>(ServoType::DM_4310)), ReturnCode::SUCCESS);
    EXPECT_EQ(driver.send_count(), 1);

    driver.abandon_socket();
}

TEST(DriverArxTransport, ResetZeroPositionPropagatesSendFailure) {
    CommandLineArgs cla{};
    FailingSendDriverArx driver(nullptr, cla);
    driver.adopt_socket(42);

    EXPECT_EQ(driver.reset_zero_position(1, static_cast<int>(ServoType::DM_4310)), ReturnCode::BUSY);

    driver.abandon_socket();
}

TEST(DriverArxTransport, SendCommandPropagatesSendFailure) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    FailingSendDriverArx driver(&device, cla);
    driver.adopt_socket(42);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::DM_4310);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    EXPECT_EQ(driver.send_command(&servo, 1.0f, 0.1f, 0.5f, 0.0f, 0.0f), ReturnCode::BUSY);

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
    driver.abandon_socket();
}

TEST(DriverArxTransport, SendCommandRejectsNullServo) {
    CommandLineArgs cla{};
    FailingSendDriverArx driver(nullptr, cla);
    driver.adopt_socket(42);

    EXPECT_EQ(driver.send_command(nullptr, 1.0f, 0.1f, 0.5f, 0.0f, 0.0f), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(driver.send_count(), 0);

    driver.abandon_socket();
}

TEST(DriverArxConcurrency, ConcurrentCloseAndSendCommandAreRaceFree) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed concurrency assertion currently runs on Linux";
#else
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    SocketBackedDriverArx driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::DM_4310);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    constexpr size_t kRoundCount = 32;
    constexpr size_t kSenderCount = 16;
    for (size_t round = 0; round < kRoundCount; ++round) {
        SCOPED_TRACE(round);
        int sockets[2];
        ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);
        driver.adopt_socket(sockets[0]);

        std::atomic<size_t> ready{0};
        std::atomic<bool> go{false};
        std::vector<ReturnCode> send_results(kSenderCount, ReturnCode::INVALID_PARAM);
        std::vector<std::thread> senders;
        senders.reserve(kSenderCount);
        for (size_t i = 0; i < kSenderCount; ++i) {
            senders.emplace_back([&, i] {
                ready.fetch_add(1, std::memory_order_release);
                while (!go.load(std::memory_order_acquire)) {
                    std::this_thread::yield();
                }
                send_results[i] = driver.send_command(&servo, 1.0f, 0.1f, 0.5f, 0.0f, 0.0f);
            });
        }

        ReturnCode close_result = ReturnCode::FAIL;
        std::thread closer([&] {
            ready.fetch_add(1, std::memory_order_release);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            close_result = driver.close();
        });

        while (ready.load(std::memory_order_acquire) != kSenderCount + 1) {
            std::this_thread::yield();
        }
        go.store(true, std::memory_order_release);
        for (auto& sender : senders) {
            sender.join();
        }
        closer.join();

        for (ReturnCode result : send_results) {
            EXPECT_TRUE(result == ReturnCode::SUCCESS || result == ReturnCode::BUSY ||
                        result == ReturnCode::FAIL || result == ReturnCode::NOT_INITIALIZED);
        }
        EXPECT_EQ(close_result, ReturnCode::SUCCESS);
        close(sockets[1]);
    }
#endif
}

TEST(DriverArxConcurrency, ConcurrentCloseAndResetZeroPositionAreRaceFree) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed concurrency assertion currently runs on Linux";
#else
    CommandLineArgs cla{};
    SocketBackedDriverArx driver(nullptr, cla);

    constexpr size_t kRoundCount = 32;
    constexpr size_t kResetterCount = 16;
    for (size_t round = 0; round < kRoundCount; ++round) {
        SCOPED_TRACE(round);
        int sockets[2];
        ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);
        driver.adopt_socket(sockets[0]);

        std::atomic<size_t> ready{0};
        std::atomic<bool> go{false};
        std::vector<ReturnCode> reset_results(kResetterCount, ReturnCode::INVALID_PARAM);
        std::vector<std::thread> resetters;
        resetters.reserve(kResetterCount);
        for (size_t i = 0; i < kResetterCount; ++i) {
            resetters.emplace_back([&, i] {
                ready.fetch_add(1, std::memory_order_release);
                while (!go.load(std::memory_order_acquire)) {
                    std::this_thread::yield();
                }
                reset_results[i] =
                    driver.reset_zero_position(1, static_cast<int>(ServoType::DM_4310));
            });
        }

        ReturnCode close_result = ReturnCode::FAIL;
        std::thread closer([&] {
            ready.fetch_add(1, std::memory_order_release);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            close_result = driver.close();
        });

        while (ready.load(std::memory_order_acquire) != kResetterCount + 1) {
            std::this_thread::yield();
        }
        go.store(true, std::memory_order_release);
        for (auto& resetter : resetters) {
            resetter.join();
        }
        closer.join();

        for (ReturnCode result : reset_results) {
            EXPECT_TRUE(result == ReturnCode::SUCCESS || result == ReturnCode::BUSY ||
                        result == ReturnCode::FAIL || result == ReturnCode::NOT_INITIALIZED);
        }
        EXPECT_EQ(close_result, ReturnCode::SUCCESS);
        close(sockets[1]);
    }
#endif
}

TEST(DriverArxConcurrency, ConcurrentCloseAndEnableAreRaceFree) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed concurrency assertion currently runs on Linux";
#else
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    SocketBackedDriverArx driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::ENCOS_A4310);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    constexpr size_t kRoundCount = 32;
    constexpr size_t kEnablerCount = 16;
    for (size_t round = 0; round < kRoundCount; ++round) {
        SCOPED_TRACE(round);
        int sockets[2];
        ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);
        driver.adopt_socket(sockets[0]);

        std::atomic<size_t> ready{0};
        std::atomic<bool> go{false};
        std::vector<ReturnCode> enable_results(kEnablerCount, ReturnCode::INVALID_PARAM);
        std::vector<std::thread> enablers;
        enablers.reserve(kEnablerCount);
        for (size_t i = 0; i < kEnablerCount; ++i) {
            enablers.emplace_back([&, i] {
                ready.fetch_add(1, std::memory_order_release);
                while (!go.load(std::memory_order_acquire)) {
                    std::this_thread::yield();
                }
                enable_results[i] =
                    driver.enable(1, static_cast<int>(ServoType::ENCOS_A4310), true);
            });
        }

        ReturnCode close_result = ReturnCode::FAIL;
        std::thread closer([&] {
            ready.fetch_add(1, std::memory_order_release);
            while (!go.load(std::memory_order_acquire)) {
                std::this_thread::yield();
            }
            close_result = driver.close();
        });

        while (ready.load(std::memory_order_acquire) != kEnablerCount + 1) {
            std::this_thread::yield();
        }
        go.store(true, std::memory_order_release);
        for (auto& enabler : enablers) {
            enabler.join();
        }
        closer.join();

        for (ReturnCode result : enable_results) {
            EXPECT_TRUE(result == ReturnCode::SUCCESS || result == ReturnCode::BUSY ||
                        result == ReturnCode::FAIL || result == ReturnCode::NOT_INITIALIZED);
        }
        EXPECT_EQ(close_result, ReturnCode::SUCCESS);
        close(sockets[1]);
    }

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
#endif
}

TEST(DriverArxConcurrency, ConcurrentDmEnablesDoNotInterleaveHandshakes) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed concurrency assertion currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    ConcurrentEnableProbeDriverArx driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    ReturnCode first_result = ReturnCode::FAIL;
    ReturnCode second_result = ReturnCode::FAIL;
    auto enable = [&](int motor_id, ReturnCode& result) {
        ready.fetch_add(1, std::memory_order_release);
        while (!go.load(std::memory_order_acquire)) {
            std::this_thread::yield();
        }
        result = driver.enable(motor_id, static_cast<int>(ServoType::DM_4310), false);
    };

    std::thread first(enable, 1, std::ref(first_result));
    std::thread second(enable, 2, std::ref(second_result));
    while (ready.load(std::memory_order_acquire) != 2) {
        std::this_thread::yield();
    }
    go.store(true, std::memory_order_release);
    first.join();
    second.join();

    EXPECT_FALSE(driver.overlap_detected())
        << "concurrent enable handshakes shared the synchronous CAN response path";
    EXPECT_EQ(first_result, ReturnCode::SUCCESS);
    EXPECT_EQ(second_result, ReturnCode::SUCCESS);

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
#endif
}

TEST(DriverArxConcurrency, SendCommandDoesNotInterleaveEnableHandshake) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed concurrency assertion currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    EnableTransactionProbeDriverArx driver(&device, cla);
    driver.adopt_socket(sockets[0]);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 2, ServoType::DM_4310);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    ReturnCode enable_result = ReturnCode::FAIL;
    std::thread enabler([&] {
        enable_result = driver.enable(1, static_cast<int>(ServoType::DM_4310), false);
    });
    EXPECT_TRUE(driver.wait_for_handshake_start());
    const ReturnCode send_result = driver.send_command(&servo, 1.0f, 0.1f, 0.5f, 0.0f, 0.0f);
    enabler.join();

    EXPECT_FALSE(driver.overlap_detected())
        << "a normal command was written while enable owned the synchronous response path";
    EXPECT_EQ(enable_result, ReturnCode::SUCCESS);
    EXPECT_EQ(send_result, ReturnCode::SUCCESS);

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
#endif
}

TEST(DriverArxConcurrency, ResetDoesNotInterleaveEnableHandshake) {
#if !defined(__linux__)
    GTEST_SKIP() << "socket-backed concurrency assertion currently runs on Linux";
#else
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    EnableTransactionProbeDriverArx driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    ReturnCode enable_result = ReturnCode::FAIL;
    std::thread enabler([&] {
        enable_result = driver.enable(1, static_cast<int>(ServoType::DM_4310), false);
    });
    EXPECT_TRUE(driver.wait_for_handshake_start());
    const ReturnCode reset_result =
        driver.reset_zero_position(2, static_cast<int>(ServoType::DM_4310));
    enabler.join();

    EXPECT_FALSE(driver.overlap_detected())
        << "a zero reset was written while enable owned the synchronous response path";
    EXPECT_EQ(enable_result, ReturnCode::SUCCESS);
    EXPECT_EQ(reset_result, ReturnCode::SUCCESS);

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
#endif
}

TEST(DriverRegistryLifetime, DestroyedServosAreUnregistered) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    FailingSendDriverArx driver(&device, cla);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    constexpr int kServoId = 13;

    {
        DriverArxTestServoDm servo(&device, &driver, &param, kServoId, ServoType::DM_4310);
        Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);
        ASSERT_EQ(Driver::find_servo(kServoId), &servo);
        ASSERT_EQ(Driver::find_data_index(kServoId), servo.data_index_);
    }

    EXPECT_EQ(Driver::find_servo(kServoId), nullptr);
    EXPECT_EQ(Driver::find_data_index(kServoId), -1);

    // Keep this regression isolated while the broken implementation still
    // leaves a dangling process-wide registry entry behind.
    if (Driver::find_servo(kServoId) != nullptr || Driver::find_data_index(kServoId) != -1) {
        Driver::register_servo_data_index(kServoId, -1, nullptr);
    }
}

TEST(DriverArxTransport, SendCommandRejectsInvalidCanFrameBeforeWrite) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    FailingSendDriverArx driver(&device, cla);
    driver.adopt_socket(42);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 15, ServoType::DM_4310);
    Driver::register_servo_data_index(servo.id_, -1, nullptr);

    EXPECT_EQ(driver.send_command(&servo, 1.0f, 0.1f, 0.5f, 0.0f, 0.0f), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(driver.send_count(), 0);

    driver.abandon_socket();
}

TEST(DriverArxTransport, EncosEnablePropagatesSetupSendFailure) {
    CommandLineArgs cla{};
    DriverArxTestDevice device(cla);
    SetupSendFailingDriverArx driver(&device, cla, 1);
    driver.adopt_socket(42);
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                       0.2f, 0.3f, 0.1f};
    DriverArxTestServoDm servo(&device, &driver, &param, 1, ServoType::ENCOS_A4310);
    Driver::register_servo_data_index(servo.id_, servo.data_index_, &servo);

    EXPECT_EQ(driver.enable(servo.id_, static_cast<int>(ServoType::ENCOS_A4310)), ReturnCode::BUSY);
    EXPECT_EQ(driver.send_count(), 1);

    Driver::register_servo_data_index(servo.id_, -1, nullptr);
    driver.abandon_socket();
}

TEST(DriverArxTransport, EncosEnableRejectsInvalidSetupFrameBeforeWrite) {
    CommandLineArgs cla{};
    ZeroDescriptorDriverArx driver(nullptr, cla);
    driver.adopt_socket(42);
    Driver::register_servo_data_index(31, -1, nullptr);

    EXPECT_EQ(driver.enable(31, static_cast<int>(ServoType::ENCOS_A4310)), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(driver.send_count(), 0);

    driver.abandon_socket();
}

TEST(DriverArxTransport, DmEnableStopsAfterFailedSetupWrite) {
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    SetupSendFailingDriverArx driver(nullptr, cla, 1);
    driver.adopt_socket(sockets[0]);

    EXPECT_EQ(driver.enable(1, static_cast<int>(ServoType::DM_4310), false), ReturnCode::BUSY);
    EXPECT_EQ(driver.send_count(), 1);
    EXPECT_FALSE(driver.reception_running());

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
}

TEST(DriverArxLifecycle, FailedEnableRestartsActiveReception) {
    int sockets[2];
    ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM, 0, sockets), 0);

    CommandLineArgs cla{};
    EnableSendFailingDriverArx driver(nullptr, cla);
    driver.adopt_socket(sockets[0]);

    std::atomic<bool> callback_entered{false};
    ASSERT_EQ(driver.start_reception([&](void*, size_t, size_t) {
                  callback_entered.store(true, std::memory_order_release);
                  while (driver.reception_running()) {
                      std::this_thread::yield();
                  }
              }),
              ReturnCode::SUCCESS);

    const uint8_t wake_byte = 0x42;
    ASSERT_EQ(write(sockets[1], &wake_byte, sizeof(wake_byte)), static_cast<ssize_t>(sizeof(wake_byte)));
    while (!callback_entered.load(std::memory_order_acquire)) {
        std::this_thread::yield();
    }

    EXPECT_EQ(driver.enable(1, static_cast<int>(ServoType::DM_4310), false), ReturnCode::FAIL);
    EXPECT_TRUE(driver.reception_running());

    EXPECT_EQ(driver.close(), ReturnCode::SUCCESS);
    close(sockets[1]);
}

}  // namespace
