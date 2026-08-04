/*!
 * @file pi_command_line_args.cpp
 * @brief Implementation of the CommandLineArgs class for parsing and validating
 * command-line arguments.
 */

#include "pi_command_line_args.hpp"

#include <cctype>
#include <cmath>
#include <cstddef>
#include <sstream>

#include <boost/program_options.hpp>

#include "pi_device.hpp"
#include "pi_device_config.hpp"
#include "pi_topic.hpp"

namespace po = boost::program_options;

std::vector<float> CommandLineArgs::parse_torq_rescale_csv(const std::string& csv) {
    std::vector<float> values;
    std::istringstream stream(csv);
    std::string token;
    while (std::getline(stream, token, ',')) {
        std::size_t consumed = 0;
        float value = 0.0f;
        try {
            value = std::stof(token, &consumed);
        } catch (const std::invalid_argument&) {
            return {};
        } catch (const std::out_of_range&) {
            return {};
        }
        // Reject trailing garbage (e.g. "0.8x") and non-physical values.
        while (consumed < token.size() && std::isspace(static_cast<unsigned char>(token[consumed]))) consumed++;
        if (consumed != token.size() || !std::isfinite(value) || value < 0.0f) {
            return {};
        }
        values.push_back(value);
    }
    return values;
}

CommandLineArgs::CommandLineArgs(int argc, char** argv) {
    // Parse command line arguments
    po::options_description desc("Allowed options");

    desc.add_options()(
        OPT_DEVICE_TYPE,
        po::value<std::string>()->default_value(OPT_DEVICE_TYPE_ARM),
        "The type of the device")(OPT_DEVICE_MODEL, po::value<std::string>(),
                                  "The model name of the device")(
        OPT_DEVICE_ID, po::value<std::string>(), "The ID of the device")(
        OPT_LOGICAL_NAME, po::value<std::string>(), "Session-unique logical arm name")(
        OPT_ROLE, po::value<std::string>(),
        "The role of the device: leader or follower")(
        OPT_CONTROL_PORT, po::value<std::string>(),
        "The SocketCAN interface used to control the device, for example can0")(
        OPT_CONTROL_FREQUENCY, po::value<int>(),
        "The frequency (Hz) of the main control loop")(
        OPT_EFFECTOR_MODEL, po::value<std::string>()->default_value(""),
        "The model name of the device")(
        OPT_EFFECTOR_ID, po::value<std::string>()->default_value("01"),
        "The ID of the device")(
        OPT_TOPIC_JOINT, po::value<std::string>()->default_value("joint_info"),
        "The topic name for joint information")(
        OPT_TOPIC_JOYSTICK,
        po::value<std::string>()->default_value(OPT_DEFAULT_NONE),
        "The topic name for joystick information")(
        OPT_INFO_GROUPS, po::value<std::string>()->default_value(""),
        "The groups to see the information messages")(
        OPT_INFO_LEVEL, po::value<int>()->default_value(0),
        "The level of information message to see: lower value shows less "
        "frequent messages")(
        OPT_BAUD_RATE, po::value<int>()->default_value(DEFAULT_BAUD_RATE),
        "To set the baud rate of the serial port (bps)")(
        OPT_DOF_ARM, po::value<int>()->default_value(DEFAULT_DOF_ARM),
        "To set the DOF of the arm")(
        OPT_SERVO_NUM_ARM,
        po::value<int>()->default_value(DEFAULT_SERVO_NUM_ARM),
        "To set the number of servos in the arm")(
        OPT_DOF_EFFECTOR, po::value<int>()->default_value(DEFAULT_DOF_EFFECTOR),
        "To set the DOF of effector")(
        OPT_SERVO_NUM_EFFECTOR,
        po::value<int>()->default_value(DEFAULT_SERVO_NUM_EFFECTOR),
        "To set the number of servos in the effector")(
        OPT_INIT_JOINTS_SEQUENTIAL, po::bool_switch()->default_value(false),
        "To initialize joint positions sequentially")(
        OPT_MSG_TYPE,
        po::value<std::string>()->default_value(OPT_MSG_TYPE_JOINT_INFO),
        "Message type between leader and follower")(
        OPT_JOYSTICK_NUM_CHANNEL,
        po::value<int>()->default_value(DEFAULT_JOYSTICK_NUM_CHANNEL),
        "Number of joystick channels")(
        OPT_JOYSTICK_NUM_BUTTON,
        po::value<int>()->default_value(DEFAULT_JOYSTICK_NUM_BUTTON),
        "Number of joystick buttons")(
        OPT_TOPIC_TYPE,
        po::value<std::string>()->default_value(OPT_TOPIC_TYPE_UNDEFINED),
        "Topic type: ZMQ")(
        OPT_ALGO_TYPE,
        po::value<std::string>()->default_value(OPT_ALGO_TYPE_UNDEFINED),
        "Algo type: Pinocchio")(OPT_DONT_GO_TO_HOME_POS,
                                  po::bool_switch()->default_value(false),
                                  "Not to go to home position")(
        OPT_LEADER_GRAVITY_COMPENSATION,
        po::bool_switch()->default_value(false),
        "Enable leader gravity compensation")(
        OPT_EFFECTOR_MAX_POS,
        po::value<float>()->default_value(OPT_EFFECTOR_MAX_POS_DEFAULT),
        "Maximum position of the effector")(
        OPT_EFFECTOR_MIN_POS,
        po::value<float>()->default_value(OPT_EFFECTOR_MIN_POS_DEFAULT),
        "Minimum position of the effector")(
        OPT_DISTANCE_TO_TORQUE,
        po::value<float>()->default_value(OPT_DISTANCE_TO_TORQUE_DEFAULT),
        "Distance to torque conversion factor")(
        OPT_EFFECTOR_CONTROL_MODE,
        po::value<std::string>()->default_value(
            OPT_EFFECTOR_CONTROL_MODE_UNDEFINED),
        "Effector control mode: torque or position")(
        OPT_EFFECTOR_KD,
        po::value<float>()->default_value(OPT_EFFECTOR_KD_DEFAULT),
        "Effector Kd gain")(OPT_SAFETY_FEATURE_OFF,
                             po::bool_switch()->default_value(false),
                             "To disable all safety features")(
        OPT_SAFETY_TORQUE_MODE, po::bool_switch()->default_value(false),
        "Enable sustained measured-torque protective stops")(
        OPT_PLANING_TYPE,
        po::value<std::string>()->default_value(OPT_PLANING_TYPE_DEFAULT),
        "Planning type of waypoint generation")(
        OPT_FOLLOWER_GRAVITY_COMPENSATION,
        po::value<std::string>()->default_value(OPT_FOLLOWER_GRAVITY_COMPENSATION_DEFAULT),
        "Follower gravity feed-forward override: config (arm individual JSON decides), true, or false")(
        OPT_TORQ_RESCALE, po::value<std::string>()->default_value(""),
        "Per-joint torq_rescale override, comma-separated (one value per arm joint); "
        "empty leaves the model/individual config values")(OPT_FORCE_FEEDBACK,
                                                               po::value<float>()->default_value(0.0f),
                                                               "Force feedback parameter")(
        OPT_TOPIC_STATE, po::value<std::string>()->default_value(""),
        "Unique state output topic")(
        OPT_TOPIC_LIVE_COMMAND, po::value<std::string>()->default_value(""),
        "Unique leader-to-follower live command topic")(
        OPT_TOPIC_DIRECT_COMMAND, po::value<std::string>()->default_value(""),
        "Unique direct Python command topic")(
        OPT_TOPIC_LIFECYCLE, po::value<std::string>()->default_value(""),
        "Unique lifecycle command topic")(
        OPT_TOPIC_STATUS, po::value<std::string>()->default_value(""),
        "Unique status and acknowledgement topic")(
        OPT_PAIRED_FOLLOWER_STATE_TOPIC, po::value<std::string>()->default_value(""),
        "Follower state topic consumed by a leader for bilateral feedback")(
        OPT_ARM_MODEL_CONFIG, po::value<std::string>()->default_value(""),
        "Explicit arm model JSON path")(
        OPT_ARM_INSTANCE_CONFIG, po::value<std::string>()->default_value(""),
        "Explicit arm instance JSON path")(
        OPT_EFFECTOR_MODEL_CONFIG, po::value<std::string>()->default_value(""),
        "Explicit effector model JSON path")(
        OPT_EFFECTOR_INSTANCE_CONFIG, po::value<std::string>()->default_value(""),
        "Explicit effector instance JSON path")(
        OPT_URDF_PATH, po::value<std::string>()->default_value(""),
        "Explicit URDF path")(
        OPT_PARENT_LIVENESS_FD, po::value<int>()->default_value(-1),
        "Inherited read end of the supervising process's liveness pipe")(
        OPT_FR3_ADDRESS, po::value<std::string>()->default_value(""),
        "FR3 controller hostname or IP address")(
        OPT_FR3_RESET_POSE,
        po::value<std::string>()->default_value("0,-0.785398,0,-2.356194,0,1.570796,0.785398"),
        "Seven comma-separated FR3 reset joint positions in radians")(
        OPT_ROBOTIQ_TRANSPORT, po::value<std::string>()->default_value(""),
        "Robotiq transport: rtu or tcp")(
        OPT_ROBOTIQ_ENDPOINT, po::value<std::string>()->default_value(""),
        "Robotiq serial device or TCP hostname/address")(
        OPT_ROBOTIQ_PORT, po::value<int>()->default_value(502),
        "Robotiq Modbus TCP port")(
        OPT_ROBOTIQ_BAUD_RATE, po::value<int>()->default_value(115200),
        "Robotiq Modbus RTU baud rate")(
        OPT_ROBOTIQ_SLAVE_ID, po::value<int>()->default_value(9),
        "Robotiq Modbus slave id")(
        OPT_ROBOTIQ_POLL_FREQUENCY, po::value<int>()->default_value(50),
        "Robotiq status polling frequency in Hz")(
        OPT_ROBOTIQ_TIMEOUT_MS, po::value<int>()->default_value(200),
        "Robotiq Modbus response timeout in milliseconds")(
        OPT_ROBOTIQ_MIN_POSITION_RAW, po::value<int>()->default_value(3),
        "Robotiq raw register value for fully open")(
        OPT_ROBOTIQ_MAX_POSITION_RAW, po::value<int>()->default_value(230),
        "Robotiq raw register value for fully closed")(
        OPT_ROBOTIQ_DEFAULT_SPEED, po::value<float>()->default_value(1.0f),
        "Robotiq speed for position commands, normalized to [0, 1]")(
        OPT_ROBOTIQ_DEFAULT_FORCE, po::value<float>()->default_value(1.0f),
        "Robotiq force for position commands, normalized to [0, 1]")(
        OPT_MOVE_TO_READY_VEL_RAD_S_NORMAL,
        po::value<float>()->default_value(MOVE_TO_READY_VEL_RAD_S_NORMAL),
        "Healthy move-to-ready angular speed (rad/s). Used by startup, command-driven "
        "MOVE_TO_READY_POS, and MOVE_TO_READY_AND_STOP. Step = max_vel * loop_dt so the "
        "speed is bounded regardless of distance from home")(
        OPT_MOVE_TO_READY_VEL_RAD_S_ERROR,
        po::value<float>()->default_value(MOVE_TO_READY_VEL_RAD_S_ERROR),
        "Emergency-recovery move-to-ready angular speed (rad/s). Used after CAN loss, "
        "over-temp, or any joint error. Should be <= the normal speed (~1.5x slower is "
        "recommended)")(
        OPT_EMERGENCY_SHUTDOWN_TIMEOUT_MS,
        po::value<int>()->default_value(DEVICE_EMERGENCY_TIMEOUT_MS_DEFAULT),
        "Heartbeat-staleness watchdog (ms) for any active move-to-ready: if the control "
        "loop stops refreshing its step heartbeat for this long, the device is force-"
        "parked. The move itself is allowed to run as long as it keeps stepping, so a "
        "heavy arm doing a slow ready move can take tens of seconds without being killed");

    // Parse command line options
    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
        po::notify(vm);
    } catch (const po::error& ex) {
        PI_ERROR("Error parsing command line arguments: %s", ex.what());
        exit(2);
    }

    PI_INFO("main()", InfoLevel::ESSENTIAL_0,
            "Processing command line arguments...");

    // Role setup
    role = Role::LEADER;
    if (vm.count(OPT_ROLE)) {
        std::string arg_role = vm[OPT_ROLE].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Device role: %s",
                arg_role.c_str());

        if (arg_role == OPT_ROLE_LEADER) {
            role = Role::LEADER;
        } else if (arg_role == OPT_ROLE_FOLLOWER) {
            role = Role::FOLLOWER;
        } else {
            PI_ERROR(
                "Invalid value for --%s: '%s' (must be 'leader' or 'follower')",
                OPT_ROLE, arg_role.c_str());
        }
    } else {
        PI_ERROR("--%s is not set", OPT_ROLE);
        exit(2);
    }

    // Device type setup
    if (vm.count(OPT_DEVICE_TYPE)) {
        device_type = vm[OPT_DEVICE_TYPE].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Device type: %s",
                device_type.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_DEVICE_TYPE);
        exit(2);
    }

    if (device_type == OPT_DEVICE_TYPE_ARM) {
        device_config_type = DeviceConfigType::ARM;
    } else {
        PI_ERROR("Invalid device type: %s (only arms are supported)", device_type.c_str());
        exit(2);
    }

    // Device model name setup
    if (vm.count(OPT_DEVICE_MODEL)) {
        device_model = vm[OPT_DEVICE_MODEL].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Device model name: %s",
                device_model.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_DEVICE_MODEL);
        exit(2);
    }

    // Device ID setup
    if (vm.count(OPT_DEVICE_ID)) {
        device_id = vm[OPT_DEVICE_ID].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Device ID: %s",
                device_id.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_DEVICE_ID);
        exit(2);
    }
    if (vm.count(OPT_LOGICAL_NAME)) {
        logical_name = vm[OPT_LOGICAL_NAME].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Logical arm name: %s", logical_name.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_LOGICAL_NAME);
        exit(2);
    }

    // Effector model name setup
    if (vm.count(OPT_EFFECTOR_MODEL)) {
        effector_model = vm[OPT_EFFECTOR_MODEL].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Effector model name: %s",
                effector_model.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_EFFECTOR_MODEL);
        exit(2);
    }

    // Effector ID setup
    if (vm.count(OPT_EFFECTOR_ID)) {
        effector_id = vm[OPT_EFFECTOR_ID].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Effector ID: %s",
                effector_id.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_EFFECTOR_ID);
        exit(2);
    }

    // Topic joint topic setup
    if (vm.count(OPT_TOPIC_JOINT)) {
        topic_joint = vm[OPT_TOPIC_JOINT].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Topic name for joint information: %s", topic_joint.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_TOPIC_JOINT);
        exit(2);
    }

    // Topic joystick topic setup
    if (vm.count(OPT_TOPIC_JOYSTICK)) {
        topic_joystick = vm[OPT_TOPIC_JOYSTICK].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Topic name for joystick information: %s",
                topic_joystick.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_TOPIC_JOYSTICK);
        exit(2);
    }

    // Control frequency setup
    if (vm.count(OPT_CONTROL_FREQUENCY)) {
        control_frequency = vm[OPT_CONTROL_FREQUENCY].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Control frequency was set to %d Hz", control_frequency);
    } else {
        PI_ERROR("--%s is not set", OPT_CONTROL_FREQUENCY);
        exit(2);
    }

    if (vm.count(OPT_CONTROL_PORT)) {
        control_port_name = vm[OPT_CONTROL_PORT].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Control port name: %s",
                control_port_name.c_str());
    } else if (device_model == "FR3") {
        control_port_name.clear();
    } else {
        PI_ERROR("--%s is not set", OPT_CONTROL_PORT);
        exit(2);
    }

    if (vm.count(OPT_INFO_LEVEL)) {
        info_level = vm[OPT_INFO_LEVEL].as<int>();
        if (info_level > static_cast<int>(InfoLevel::FREQUENT_3)) {
            PI_ERROR("--%s has invalid value: %d (maximum: %d)", OPT_INFO_LEVEL,
                     info_level, static_cast<int>(InfoLevel::FREQUENT_3));
            exit(2);
        } else {
            PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Information level: %d",
                    info_level);
        }
    } else {
        PI_ERROR("--%s is not set", OPT_INFO_LEVEL);
        exit(2);
    }

    if (vm.count(OPT_INFO_GROUPS)) {
        info_groups = vm[OPT_INFO_GROUPS].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Info groups: %s",
                info_groups.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_INFO_GROUPS);
        exit(2);
    }

    if (vm.count(OPT_BAUD_RATE)) {
        baud_rate = vm[OPT_BAUD_RATE].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Baud rate: %d bps",
                baud_rate);
    } else {
        PI_ERROR("--%s is not set", OPT_BAUD_RATE);
        exit(2);
    }

    if (vm.count(OPT_DOF_ARM)) {
        dof_arm = vm[OPT_DOF_ARM].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Arm degrees of freedom: %d",
                dof_arm);
    } else {
        PI_ERROR("--%s is not set", OPT_DOF_ARM);
        exit(2);
    }

    if (vm.count(OPT_DOF_EFFECTOR)) {
        dof_effector = vm[OPT_DOF_EFFECTOR].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Effector degrees of freedom: %d", dof_effector);
    } else {
        PI_ERROR("--%s is not set", OPT_DOF_EFFECTOR);
        exit(2);
    }

    if (vm.count(OPT_SERVO_NUM_ARM)) {
        servo_num_arm = vm[OPT_SERVO_NUM_ARM].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Number of servos in arm: %d",
                servo_num_arm);
    } else {
        PI_ERROR("--%s is not set", OPT_SERVO_NUM_ARM);
        exit(2);
    }

    if (vm.count(OPT_SERVO_NUM_EFFECTOR)) {
        servo_num_effector = vm[OPT_SERVO_NUM_EFFECTOR].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Number of servos in effector: %d", servo_num_effector);
    } else {
        PI_ERROR("--%s is not set", OPT_SERVO_NUM_EFFECTOR);
        exit(2);
    }

    if (vm.count(OPT_INIT_JOINTS_SEQUENTIAL)) {
        init_joints_sequential = vm[OPT_INIT_JOINTS_SEQUENTIAL].as<bool>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Initialize joints sequentially: %s",
                init_joints_sequential ? "enabled" : "disabled");
    } else {
        PI_ERROR("--%s is not set", OPT_INIT_JOINTS_SEQUENTIAL);
        exit(2);
    }
    moving_mode = (init_joints_sequential == true) ? MovingMode::SEQUENTIAL
                                                   : MovingMode::PARALLEL;

    if (vm.count(OPT_MSG_TYPE)) {
        msg_type_str = vm[OPT_MSG_TYPE].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Teleoperation message type: %s", msg_type_str.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_MSG_TYPE);
        exit(2);
    }

    if (msg_type_str == OPT_MSG_TYPE_JOINT_INFO) {
        msg_type = MsgType::JOINT_INFO;
    } else {
        PI_ERROR("Unsupported message type for teleoperation: %s",
                 msg_type_str.c_str());
        exit(2);
    }

    if (vm.count(OPT_JOYSTICK_NUM_CHANNEL)) {
        joystick_num_channels = vm[OPT_JOYSTICK_NUM_CHANNEL].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Joystick number of channels: %d", joystick_num_channels);
    } else {
        PI_ERROR("--%s is not set", OPT_JOYSTICK_NUM_CHANNEL);
        exit(2);
    }

    if (vm.count(OPT_JOYSTICK_NUM_BUTTON)) {
        joystick_num_buttons = vm[OPT_JOYSTICK_NUM_BUTTON].as<int>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Joystick number of buttons: %d", joystick_num_buttons);
    } else {
        PI_ERROR("--%s is not set", OPT_JOYSTICK_NUM_BUTTON);
        exit(2);
    }

    if (vm.count(OPT_TOPIC_TYPE)) {
        topic_type = vm[OPT_TOPIC_TYPE].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Topic type: %s",
                topic_type.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_TOPIC_TYPE);
        exit(2);
    }
    if (topic_type != OPT_TOPIC_TYPE_ZMQ) {
        PI_ERROR("pi_control_node supports ZMQ topics only");
        exit(2);
    }

    if (vm.count(OPT_ALGO_TYPE)) {
        algo_type = vm[OPT_ALGO_TYPE].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Algorithm type: %s",
                algo_type.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_ALGO_TYPE);
        exit(2);
    }

    if (vm.count(OPT_DONT_GO_TO_HOME_POS)) {
        dont_go_to_home_pos = vm[OPT_DONT_GO_TO_HOME_POS].as<bool>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Skip home position: %s",
                dont_go_to_home_pos ? "enabled" : "disabled");
    } else {
        PI_ERROR("--%s is not set", OPT_DONT_GO_TO_HOME_POS);
        exit(2);
    }

    if (vm.count(OPT_LEADER_GRAVITY_COMPENSATION)) {
        leader_gravity_compensation = vm[OPT_LEADER_GRAVITY_COMPENSATION].as<bool>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Leader gravity compensation: %s",
                leader_gravity_compensation ? "enabled" : "disabled");
    } else {
        PI_ERROR("--%s is not set", OPT_LEADER_GRAVITY_COMPENSATION);
        exit(2);
    }

    if (vm.count(OPT_EFFECTOR_MAX_POS)) {
        effector_max_pos = vm[OPT_EFFECTOR_MAX_POS].as<float>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Command line argument: Effector maximum position : %f",
                effector_max_pos);
    } else {
        PI_ERROR("--%s is not set", OPT_EFFECTOR_MAX_POS);
        exit(2);
    }

    if (vm.count(OPT_EFFECTOR_MIN_POS)) {
        effector_min_pos = vm[OPT_EFFECTOR_MIN_POS].as<float>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Command line argument: Effector minimum position : %f",
                effector_min_pos);
    } else {
        PI_ERROR("--%s is not set", OPT_EFFECTOR_MIN_POS);
        exit(2);
    }

    if (vm.count(OPT_DISTANCE_TO_TORQUE)) {
        distance_to_torque = vm[OPT_DISTANCE_TO_TORQUE].as<float>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Distance to torque conversion factor: %f", distance_to_torque);
    } else {
        PI_ERROR("--%s is not set", OPT_DISTANCE_TO_TORQUE);
        exit(2);
    }

    if (vm.count(OPT_EFFECTOR_CONTROL_MODE)) {
        effector_control_mode = vm[OPT_EFFECTOR_CONTROL_MODE].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Effector control mode: %s",
                effector_control_mode.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_EFFECTOR_CONTROL_MODE);
        exit(2);
    }

    if (vm.count(OPT_EFFECTOR_KD)) {
        effector_kd = vm[OPT_EFFECTOR_KD].as<float>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Effector Kd gain: %f",
                effector_kd);
    } else {
        PI_ERROR("--%s is not set", OPT_EFFECTOR_KD);
        exit(2);
    }

    if (vm.count(OPT_SAFETY_FEATURE_OFF)) {
        safety_feature_off = vm[OPT_SAFETY_FEATURE_OFF].as<bool>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Safety features: %s",
                safety_feature_off ? "disabled" : "enabled");
    } else {
        PI_ERROR("--%s is not set", OPT_SAFETY_FEATURE_OFF);
        exit(2);
    }

    if (vm.count(OPT_SAFETY_TORQUE_MODE)) {
        safety_torque_mode = vm[OPT_SAFETY_TORQUE_MODE].as<bool>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Safety torque mode: %s",
                safety_torque_mode ? "enabled" : "disabled (warning-only)");
    } else {
        PI_ERROR("--%s is not set", OPT_SAFETY_TORQUE_MODE);
        exit(2);
    }

    if (vm.count(OPT_PLANING_TYPE)) {
        planning_type = vm[OPT_PLANING_TYPE].as<std::string>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Planning type: %s",
                planning_type.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_PLANING_TYPE);
        exit(2);
    }

    if (vm.count(OPT_FOLLOWER_GRAVITY_COMPENSATION)) {
        follower_gravity_compensation_override = vm[OPT_FOLLOWER_GRAVITY_COMPENSATION].as<std::string>();
        if (follower_gravity_compensation_override != OPT_FOLLOWER_GRAVITY_COMPENSATION_DEFAULT &&
            follower_gravity_compensation_override != "true" && follower_gravity_compensation_override != "false") {
            PI_ERROR("--%s must be one of config/true/false, got '%s'", OPT_FOLLOWER_GRAVITY_COMPENSATION,
                     follower_gravity_compensation_override.c_str());
            exit(2);
        }
        PI_INFO("main()", InfoLevel::ESSENTIAL_0, "Follower gravity compensation override: %s",
                follower_gravity_compensation_override.c_str());
    } else {
        PI_ERROR("--%s is not set", OPT_FOLLOWER_GRAVITY_COMPENSATION);
        exit(2);
    }

    if (vm.count(OPT_TORQ_RESCALE)) {
        const std::string torq_rescale_csv = vm[OPT_TORQ_RESCALE].as<std::string>();
        if (!torq_rescale_csv.empty()) {
            torq_rescale_override = parse_torq_rescale_csv(torq_rescale_csv);
            if (torq_rescale_override.empty()) {
                PI_ERROR("--%s must be comma-separated nonnegative finite floats, got '%s'", OPT_TORQ_RESCALE,
                         torq_rescale_csv.c_str());
                exit(2);
            }
            PI_INFO("main()", InfoLevel::ESSENTIAL_0, "torq_rescale override requested: %s",
                    torq_rescale_csv.c_str());
        }
    }

    if (vm.count(OPT_FORCE_FEEDBACK)) {
        force_feedback = vm[OPT_FORCE_FEEDBACK].as<float>();
        PI_INFO("main()", InfoLevel::ESSENTIAL_0,
                "Force feedback parameter: %f", force_feedback);
    } else {
        PI_ERROR("--%s is not set", OPT_FORCE_FEEDBACK);
        exit(2);
    }

    topic_state = vm[OPT_TOPIC_STATE].as<std::string>();
    topic_live_command = vm[OPT_TOPIC_LIVE_COMMAND].as<std::string>();
    topic_direct_command = vm[OPT_TOPIC_DIRECT_COMMAND].as<std::string>();
    topic_lifecycle = vm[OPT_TOPIC_LIFECYCLE].as<std::string>();
    topic_status = vm[OPT_TOPIC_STATUS].as<std::string>();
    paired_follower_state_topic = vm[OPT_PAIRED_FOLLOWER_STATE_TOPIC].as<std::string>();
    arm_model_config = vm[OPT_ARM_MODEL_CONFIG].as<std::string>();
    arm_instance_config = vm[OPT_ARM_INSTANCE_CONFIG].as<std::string>();
    effector_model_config = vm[OPT_EFFECTOR_MODEL_CONFIG].as<std::string>();
    effector_instance_config = vm[OPT_EFFECTOR_INSTANCE_CONFIG].as<std::string>();
    urdf_path = vm[OPT_URDF_PATH].as<std::string>();
    parent_liveness_fd = vm[OPT_PARENT_LIVENESS_FD].as<int>();
    if (parent_liveness_fd < -1) {
        PI_ERROR("--%s=%d must be -1 or a nonnegative file descriptor",
                 OPT_PARENT_LIVENESS_FD, parent_liveness_fd);
        exit(2);
    }
    fr3_address = vm[OPT_FR3_ADDRESS].as<std::string>();
    fr3_reset_pose = vm[OPT_FR3_RESET_POSE].as<std::string>();
    robotiq_transport = vm[OPT_ROBOTIQ_TRANSPORT].as<std::string>();
    robotiq_endpoint = vm[OPT_ROBOTIQ_ENDPOINT].as<std::string>();
    robotiq_port = vm[OPT_ROBOTIQ_PORT].as<int>();
    robotiq_baud_rate = vm[OPT_ROBOTIQ_BAUD_RATE].as<int>();
    robotiq_slave_id = vm[OPT_ROBOTIQ_SLAVE_ID].as<int>();
    robotiq_poll_frequency = vm[OPT_ROBOTIQ_POLL_FREQUENCY].as<int>();
    robotiq_timeout_ms = vm[OPT_ROBOTIQ_TIMEOUT_MS].as<int>();
    robotiq_min_position_raw = vm[OPT_ROBOTIQ_MIN_POSITION_RAW].as<int>();
    robotiq_max_position_raw = vm[OPT_ROBOTIQ_MAX_POSITION_RAW].as<int>();
    robotiq_default_speed = vm[OPT_ROBOTIQ_DEFAULT_SPEED].as<float>();
    robotiq_default_force = vm[OPT_ROBOTIQ_DEFAULT_FORCE].as<float>();
    if (!topic_live_command.empty()) {
        topic_joint = topic_live_command;
    }
    if (device_config_type != DeviceConfigType::ARM) {
        PI_ERROR("pi_control_node only owns arm processes; effectors must be attached to an arm");
        exit(2);
    }
    if (topic_state.empty() || topic_live_command.empty() || topic_direct_command.empty() ||
        topic_lifecycle.empty() || topic_status.empty()) {
        PI_ERROR("Standalone state/live/direct/lifecycle/status topics are all required");
        exit(2);
    }
    if (arm_model_config.empty() || arm_instance_config.empty() ||
        (device_model != "FR3" && urdf_path.empty())) {
        PI_ERROR("Explicit arm model/instance configs are required; non-FR3 arms also require a URDF");
        exit(2);
    }
    if (!effector_model.empty() &&
        (effector_model_config.empty() || effector_instance_config.empty())) {
        PI_ERROR("Attached effectors require explicit model and instance configuration paths");
        exit(2);
    }
    if (device_model == "FR3") {
        if (role != Role::FOLLOWER || fr3_address.empty()) {
            PI_ERROR("FR3 requires follower role and --%s", OPT_FR3_ADDRESS);
            exit(2);
        }
        if (!robotiq_transport.empty()) {
            if ((robotiq_transport != "rtu" && robotiq_transport != "tcp") || robotiq_endpoint.empty()) {
                PI_ERROR("Robotiq requires transport rtu/tcp and a non-empty endpoint");
                exit(2);
            }
            if (robotiq_port < 1 || robotiq_port > 65535 || robotiq_baud_rate <= 0 ||
                robotiq_slave_id < 0 || robotiq_slave_id > 247 || robotiq_poll_frequency <= 0 ||
                robotiq_timeout_ms <= 0 || robotiq_min_position_raw < 0 ||
                robotiq_max_position_raw > 255 || robotiq_min_position_raw >= robotiq_max_position_raw ||
                !std::isfinite(robotiq_default_speed) || robotiq_default_speed < 0.0f ||
                robotiq_default_speed > 1.0f || !std::isfinite(robotiq_default_force) ||
                robotiq_default_force < 0.0f || robotiq_default_force > 1.0f) {
                PI_ERROR("Invalid Robotiq transport parameters");
                exit(2);
            }
        }
    }

    // Unified move-to-ready / emergency-recovery options. All have defaults so existing
    // call sites are unaffected. Velocity-bounded stepping (step = max_vel * loop_dt) makes
    // motion safe at any distance from home; ERROR is the slower emergency speed.
    move_to_ready_vel_rad_s_normal = vm[OPT_MOVE_TO_READY_VEL_RAD_S_NORMAL].as<float>();
    if (move_to_ready_vel_rad_s_normal <= 0.0f) {
        PI_ERROR("--%s=%f must be > 0", OPT_MOVE_TO_READY_VEL_RAD_S_NORMAL,
                 move_to_ready_vel_rad_s_normal);
        exit(2);
    }
    move_to_ready_vel_rad_s_error = vm[OPT_MOVE_TO_READY_VEL_RAD_S_ERROR].as<float>();
    if (move_to_ready_vel_rad_s_error <= 0.0f) {
        PI_ERROR("--%s=%f must be > 0", OPT_MOVE_TO_READY_VEL_RAD_S_ERROR,
                 move_to_ready_vel_rad_s_error);
        exit(2);
    }
    if (move_to_ready_vel_rad_s_error > move_to_ready_vel_rad_s_normal) {
        PI_ERROR("--%s=%f must be <= --%s=%f (ERROR is the slower emergency speed)",
                 OPT_MOVE_TO_READY_VEL_RAD_S_ERROR, move_to_ready_vel_rad_s_error,
                 OPT_MOVE_TO_READY_VEL_RAD_S_NORMAL, move_to_ready_vel_rad_s_normal);
        exit(2);
    }
    PI_INFO("main()", InfoLevel::ESSENTIAL_0,
            "Move-to-ready speeds: normal=%.3f rad/s, error=%.3f rad/s",
            move_to_ready_vel_rad_s_normal, move_to_ready_vel_rad_s_error);

    emergency_shutdown_timeout_ms = vm[OPT_EMERGENCY_SHUTDOWN_TIMEOUT_MS].as<int>();
    if (emergency_shutdown_timeout_ms <= 0) {
        PI_ERROR("--%s=%d must be > 0", OPT_EMERGENCY_SHUTDOWN_TIMEOUT_MS,
                 emergency_shutdown_timeout_ms);
        exit(2);
    }
    PI_INFO("main()", InfoLevel::ESSENTIAL_0,
            "Emergency shutdown timeout: %d ms", emergency_shutdown_timeout_ms);
}
