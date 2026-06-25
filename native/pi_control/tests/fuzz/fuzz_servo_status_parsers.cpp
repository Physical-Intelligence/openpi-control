// libFuzzer harness for the wire-facing CAN status parsers:
// ServoDm::parse_dm_servo_status and ServoDm::parser_encos_servo_status.
// These consume raw bytes off the CAN bus on the reception thread, so the
// contract is: an arbitrary frame must never read or write out of bounds,
// whatever the registry happens to contain.
//
// The rig registers motors 1..6 the way Joint/Servo setup does (in-range data
// indices, to exercise the normal decode path), plus motor 7 with a
// data_index one slot past the cache bound. The registry is populated straight
// from config JSON (Servo::init_config_model), which performs no range check,
// so a misconfigured data_index in a config file produces exactly that registry
// state in production -- and the parsers index the feedback cache by it.
//
// The feedback cache is heap-allocated at exactly MAX_SERVO_INFO_BUF_SIZE
// entries so AddressSanitizer brackets it with redzones: any write the parser
// performs at an out-of-range data_index lands in a redzone and is reported on
// the first store. (A stack array would not work here -- a far out-of-range
// index writes into adjacent live stack slots, which ASan cannot flag.)
//
// Build with -DOPENPI_CONTROL_BUILD_FUZZERS=ON (requires Clang); run e.g.
//   ./fuzz_servo_status_parsers -max_total_time=300 -close_fd_mask=3 corpus/

#include <cstdint>
#include <cstring>

#include "pi_device.hpp"
#include "pi_driver_arx.hpp"
#include "pi_servo_dm.hpp"

namespace {

class FuzzDevice : public Device {
   public:
    explicit FuzzDevice(const CommandLineArgs& cla) : Device(cla) {}

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

class FuzzServo : public ServoDm {
   public:
    FuzzServo(Device* p_device, Driver* p_driver, const ServoParam* p_param, int id, int data_index)
        : ServoDm(p_device, nullptr, p_driver) {
        id_ = id;
        data_index_ = data_index;
        servo_model_ = "DM J4340";
        p_servo_param_ = p_param;
    }
};

CommandLineArgs make_cla() {
    CommandLineArgs cla;
    cla.device_model = "Yam";
    cla.device_id = "01";
    cla.safety_feature_off = false;
    cla.force_feedback = -1.0f;
    cla.control_frequency = 100;
    cla.moving_mode = MovingMode::PARALLEL;
    return cla;
}

struct FuzzRig {
    CommandLineArgs cla = make_cla();
    // DM J4340 ranges (pi_servo_dm.cpp g_servo_dm_param_4340).
    ServoDmParam param{0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -10.0f, 10.0f, -28.0f, 28.0f,
                       0.2f, 0.3f, 0.1f};
    FuzzDevice device{cla};
    DriverArx driver{&device, cla};
    std::vector<FuzzServo*> servos;

    FuzzRig() {
        for (int motor_id = 1; motor_id <= 6; ++motor_id) {
            auto* servo = new FuzzServo(&device, &driver, &param, motor_id, motor_id - 1);
            servos.push_back(servo);
            Driver::register_servo_data_index(motor_id, servo->data_index_, servo);
        }
        // A misconfigured config file can register any data_index
        // (Servo::init_config_model performs no range validation); the parsers
        // must stay in bounds even then. Map motor 7 one slot past the cache so
        // a status frame addressed to it indexes the first redzone byte.
        auto* out_of_range = new FuzzServo(&device, &driver, &param, 7, MAX_SERVO_INFO_BUF_SIZE);
        servos.push_back(out_of_range);
        Driver::register_servo_data_index(7, out_of_range->data_index_, out_of_range);
    }
};

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    static FuzzRig* rig = new FuzzRig();  // reachable singleton; intentionally never freed

    if (size < 2) {
        return 0;
    }

    DriverCan::can_frame_t frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = data[0];
    frame.can_dlc = data[1];
    const size_t payload = (size - 2) > sizeof(frame.data) ? sizeof(frame.data) : (size - 2);
    std::memcpy(frame.data, data + 2, payload);

    // Exactly-sized heap cache: ASan redzones bracket it, so any parser write at
    // an out-of-range data_index is reported on the first store. Freed each
    // iteration to keep the redzone state clean.
    auto* cache = new ReceivedServoData[MAX_SERVO_INFO_BUF_SIZE]();

    (void)ServoDm::parse_dm_servo_status(&frame, cache, &DriverArx::find_data_index, &rig->driver);
    (void)ServoDm::parser_encos_servo_status(&frame, cache, &DriverArx::find_data_index);

    delete[] cache;
    return 0;
}
