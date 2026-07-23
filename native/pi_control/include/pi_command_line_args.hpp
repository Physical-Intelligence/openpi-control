/*!
 * @file pi_command_line_args.hpp
 * @brief Command-line argument parsing and storage.
 */
#pragma once
#include <string>

#define OPT_ROLE                                  "role"              ///< Device role option.
#define OPT_ROLE_LEADER                           "leader"            ///< Leader role value.
#define OPT_ROLE_FOLLOWER                         "follower"         ///< Follower role value.
#define OPT_DEVICE_TYPE                           "device_type"      ///< Device type option.
#define OPT_DEVICE_TYPE_ARM                       "arms"             ///< Arm device type value.
#define OPT_DEVICE_MODEL                          "device_model"     ///< Device model option.
#define OPT_DEVICE_ID                             "device_id"        ///< Device ID option.
#define OPT_LOGICAL_NAME                          "logical_name"     ///< Session-unique logical process identity.
#define OPT_EFFECTOR_MODEL                        "effector_model"   ///< Effector model option.
#define OPT_EFFECTOR_ID                           "effector_id"      ///< Effector ID option.
#define OPT_CONTROL_PORT                          "control_port"     ///< Control port option.
#define OPT_INFO_LEVEL                            "info_level"       ///< Logging level option.
#define OPT_INFO_GROUPS                           "info_groups"      ///< Information groups option.
#define OPT_TOPIC_JOINT                           "topic_joint"      ///< Joint topic option.
#define OPT_TOPIC_JOYSTICK                        "topic_joystick"   ///< Joystick topic option.
#define OPT_CONTROL_FREQUENCY                     "control_frequency"  ///< Control frequency option (Hz).
#define OPT_DOF_ARM                               "dof_arm"          ///< Arm DOF option.
#define OPT_SERVO_NUM_ARM                         "servo_num_arm"    ///< Arm servo count option.
#define OPT_DOF_EFFECTOR                          "dof_effector"     ///< Effector DOF option.
#define OPT_SERVO_NUM_EFFECTOR                    "servo_num_effector"  ///< Effector servo count option.
#define OPT_INIT_JOINTS_SEQUENTIAL                "init_joints_sequential"  ///< Sequential joint initialization flag.
#define OPT_MSG_TYPE                              "msg_type"         ///< Message type option.
#define OPT_MSG_TYPE_JOINT_INFO                   "joint_info"       ///< Joint info message type value.
#define OPT_JOYSTICK_NUM_CHANNEL                  "joystick_num_channel"  ///< Joystick channel count option.
#define OPT_JOYSTICK_NUM_BUTTON                   "joystick_num_button"  ///< Joystick button count option.
#define OPT_TOPIC_TYPE                            "topic_type"       ///< Topic type option.
#define OPT_TOPIC_TYPE_ZMQ                        "ZMQ"              ///< ZMQ topic type value.
#define OPT_TOPIC_TYPE_UNDEFINED                  "undefined"        ///< Undefined topic type value.
#define OPT_ALGO_TYPE                             "algo_type"        ///< Algorithm type option.
#define OPT_ALGO_TYPE_PINO                        "Pinocchio"        ///< Pinocchio algorithm type value.
#define OPT_ALGO_TYPE_UNDEFINED                   "undefined"        ///< Undefined algorithm type value.
#define OPT_DONT_GO_TO_HOME_POS                   "dont_go_to_home_pos"  ///< Skip home position flag.
#define OPT_LEADER_GRAVITY_COMPENSATION           "leader_gravity_compensation"  ///< Enable leader gravity compensation.
#define OPT_EFFECTOR_MAX_POS                      "effector_max_pos"  ///< Effector max position option (radians).
#define OPT_EFFECTOR_MAX_POS_DEFAULT              -9999.0            ///< Default effector max position value.
#define OPT_EFFECTOR_MIN_POS                      "effector_min_pos"  ///< Effector min position option (radians).
#define OPT_EFFECTOR_MIN_POS_DEFAULT              9999.0             ///< Default effector min position value.
#define OPT_DISTANCE_TO_TORQUE                    "distance_to_torque"  ///< Distance-to-torque conversion factor option.
#define OPT_DISTANCE_TO_TORQUE_DEFAULT            2.0                ///< Default distance-to-torque value.
#define OPT_EFFECTOR_CONTROL_MODE                 "effector_control_mode"  ///< Effector control mode option.
#define OPT_EFFECTOR_CONTROL_MODE_TORQUE          "torque"           ///< Torque control mode value.
#define OPT_EFFECTOR_CONTROL_MODE_POSITION        "position"        ///< Position control mode value.
#define OPT_EFFECTOR_CONTROL_MODE_UNDEFINED       "undefined"       ///< Undefined control mode value.
#define OPT_EFFECTOR_KD                           "effector_kd"      ///< Effector Kd gain option.
#define OPT_EFFECTOR_KD_DEFAULT                   0.1                ///< Default effector Kd value.
#define OPT_SAFETY_FEATURE_OFF                    "safety_feature_off"  ///< Disable safety features flag.
#define OPT_SAFETY_TORQUE_MODE                    "safety_torque_mode"  ///< Enable sustained measured-torque protective stops.
#define OPT_DEFAULT_NONE                          "None"             ///< Default "none" value.
#define OPT_PLANING_TYPE                          "planning_type"    ///< Planning type option.
#define OPT_PLANING_TYPE_DEFAULT                  "config"           ///< Default planning type value.
#define OPT_ARM_PLANNING_TYPE                     "arm_planning_type"  ///< Planning type override applied to ARM devices only.
#define OPT_FORCE_FEEDBACK                        "force_feedback"   ///< Force feedback option.
#define OPT_TOPIC_STATE                           "topic_state"
#define OPT_TOPIC_LIVE_COMMAND                    "topic_live_command"
#define OPT_TOPIC_DIRECT_COMMAND                  "topic_direct_command"
#define OPT_TOPIC_LIFECYCLE                       "topic_lifecycle"
#define OPT_TOPIC_STATUS                          "topic_status"
#define OPT_PAIRED_FOLLOWER_STATE_TOPIC           "paired_follower_state_topic"
#define OPT_ARM_MODEL_CONFIG                      "arm_model_config"
#define OPT_ARM_INSTANCE_CONFIG                   "arm_instance_config"
#define OPT_EFFECTOR_MODEL_CONFIG                 "effector_model_config"
#define OPT_EFFECTOR_INSTANCE_CONFIG              "effector_instance_config"
#define OPT_URDF_PATH                             "urdf_path"
// Read end of the supervising Python process's lifeline pipe.
#define OPT_PARENT_LIVENESS_FD                    "parent_liveness_fd"


// Unified move-to-ready / emergency-recovery options. Every "move to ready" path (startup,
// command-driven MOVE_TO_READY_POS / MOVE_TO_READY_AND_STOP, emergency recovery on CAN loss /
// over-temp / exception) uses velocity-bounded stepping (step = max_vel * loop_dt). NORMAL is
// used by healthy stops; ERROR (the slower speed) is used during emergency recovery.
#define OPT_MOVE_TO_READY_VEL_RAD_S_NORMAL        "move_to_ready_vel_rad_s_normal"  ///< Healthy move-to-ready speed (rad/s).
#define OPT_MOVE_TO_READY_VEL_RAD_S_ERROR         "move_to_ready_vel_rad_s_error"   ///< Emergency-recovery move-to-ready speed (rad/s).
#define OPT_EMERGENCY_SHUTDOWN_TIMEOUT_MS         "emergency_shutdown_timeout_ms"   ///< Max gap (ms) between step heartbeats during any active move-to-ready.

enum class DeviceConfigType;
enum class Role;
enum class MovingMode;
enum class MsgType;

/*!
 * @class CommandLineArgs
 * @brief Container for parsed command-line arguments.
 */
class CommandLineArgs {
   public:
    std::string device_type;              ///< Device type string.
    DeviceConfigType device_config_type;  ///< Device configuration type.
    std::string device_model;             ///< Device model name.
    std::string device_id;                ///< Device identifier.
    std::string logical_name;              ///< Session-unique logical process identity.
    Role role;                            ///< Device role (LEADER or FOLLOWER).
    int control_frequency;                ///< Control loop frequency (Hz).
    std::string effector_model;           ///< Effector model name.
    std::string effector_id;              ///< Effector identifier.
    std::string topic_joint;              ///< Joint topic name.
    std::string topic_joystick;           ///< Joystick topic name.
    std::string info_groups;              ///< Information groups (comma-separated).
    std::string control_port_name;        ///< Control port name.
    int info_level;                       ///< Logging level.
    int dof_arm;                          ///< Arm DOF.
    int servo_num_arm;                    ///< Arm servo count.
    int dof_effector;                     ///< Effector DOF.
    int servo_num_effector;               ///< Effector servo count.
    bool init_joints_sequential;          ///< Sequential joint initialization flag.
    MovingMode moving_mode;               ///< Moving mode.
    std::string msg_type_str;             ///< Message type string.
    MsgType msg_type;                     ///< Message type.
    int joystick_num_channels;            ///< Joystick channel count.
    int joystick_num_buttons;             ///< Joystick button count.
    std::string topic_type;               ///< Topic type.
    std::string algo_type;                ///< Algorithm type.
    bool dont_go_to_home_pos;             ///< Skip home position flag.
    bool leader_gravity_compensation = false;  ///< Enable leader gravity feedforward torque.
    float effector_max_pos;               ///< Effector max position (radians).
    float effector_min_pos;               ///< Effector min position (radians).
    float distance_to_torque;             ///< Distance-to-torque conversion factor.
    std::string effector_control_mode;    ///< Effector control mode.
    float effector_kd;                    ///< Effector Kd gain.
    bool safety_feature_off;              ///< Disable safety features flag.
    bool safety_torque_mode = false;       ///< Enable sustained measured-torque protective stops.
    std::string planning_type;            ///< Planning type.
    std::string arm_planning_type;        ///< Planning type override for ARM devices only (empty/None = model config).
    float force_feedback;                 ///< Force feedback parameter.
    std::string topic_state;
    std::string topic_live_command;
    std::string topic_direct_command;
    std::string topic_lifecycle;
    std::string topic_status;
    std::string paired_follower_state_topic;
    std::string arm_model_config;
    std::string arm_instance_config;
    std::string effector_model_config;
    std::string effector_instance_config;
    std::string urdf_path;
    int parent_liveness_fd = -1;  ///< Inherited lifeline pipe; EOF means the Python owner exited.

    // Unified move-to-ready / emergency-recovery options. See macros above for descriptions.
    float move_to_ready_vel_rad_s_normal; ///< Healthy move-to-ready angular speed (rad/s).
    float move_to_ready_vel_rad_s_error;  ///< Emergency-recovery move-to-ready angular speed (rad/s).
    int emergency_shutdown_timeout_ms;    ///< Heartbeat-staleness threshold (ms) for the move-to-ready watchdog.

    /*!
     * @brief Constructs a CommandLineArgs instance by parsing command-line arguments.
     *
     * @param argc Argument count.
     * @param argv Argument values.
     */
    CommandLineArgs(int argc, char** argv);

    // Default constructor.
    CommandLineArgs() = default;
};
