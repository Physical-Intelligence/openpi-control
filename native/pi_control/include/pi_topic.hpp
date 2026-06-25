/*!
 * @file pi_topic.hpp
 * @brief Topic base class and message types for robot control communication.
 */

#pragma once
#include <memory>
#include <string>

#include "pi_algo.hpp"
#include "pi_control.hpp"

#define TOPIC_FOLLOWER_JOINT_INFO "follower_joint_info"  ///< Topic name for publishing follower's joint information.
#define TOPIC_COMMAND_INFO        "command_info"         ///< Topic name for sending command information.
#define TOPIC_DEVICE_INFO         "device_info"          ///< Topic name for sending device information.
#define TOPIC_EFFECTOR_INFO       "effector_info"        ///< Topic name for sending effector information.
#define TOPIC_JOYSTICK_INFO_LEFT  "joystick_info_left"   ///< Topic name for sending left joystick information.
#define TOPIC_JOYSTICK_INFO_RIGHT "joystick_info_right"  ///< Topic name for sending right joystick information.

#define DEVICE_COMMAND_STOP                   1  ///< Command code to stop the device.
#define DEVICE_COMMAND_SET_EFFECTOR_MIN_MAX_POS 10  ///< Command code to set the effector minimum and maximum position limits.
#define DEVICE_COMMAND_SET_DISTANCE_TO_TORQUE   11  ///< Command code to set the distance-to-torque conversion factor.
#define DEVICE_COMMAND_SET_EFFECTOR_KD           12  ///< Command code to set the effector derivative gain (Kd).
#define DEVICE_COMMAND_MOVE_TO_READY_POS         13  ///< Command code to re-enter ready/home position from current pose.
#define DEVICE_COMMAND_PAUSE_LEADER_COMMAND_LISTENING  14  ///< Follower-only: ignore leader joint commands.
#define DEVICE_COMMAND_RESUME_LEADER_COMMAND_LISTENING 15  ///< Follower-only: resume leader joint commands.
#define DEVICE_COMMAND_MOVE_TO_READY_POS_AND_PAUSE_LEADER_COMMAND_LISTENING 16  ///< Follower-only: pause leader cmds and move-to-ready.
#define DEVICE_COMMAND_MOVE_TO_READY_AND_STOP 17  ///< Pause leader cmds, move to ready, then exit.
#define DEVICE_COMMAND_ENTER_GRAVITY_COMPENSATION 30
#define DEVICE_COMMAND_ENABLE_FORCE_FEEDBACK       31
#define DEVICE_COMMAND_SET_FORCE_FEEDBACK_GAIN     32
#define DEVICE_COMMAND_HOLD                        33
#define DEVICE_COMMAND_HEARTBEAT                   34  ///< Client-liveness heartbeat; payload-free, arms the dead-client watchdog.

#define DEVICE_INFO_READY_NOW 1  ///< Device is ready; param_int[0] is the completed lifecycle request id, if any.
#define DEVICE_INFO_EFFECTOR  10 ///< Device info code indicating that the device is an effector.

// Graceful shutdown / emergency recovery notifications.
//
// Payload convention for the codes below (carried in ZmqDeviceInfo / DeviceInfo.msg):
//   param_int[0]   = ReturnCode value (cause of the error)
//   param_int[1]   = First failed joint id (-1 if unknown)
//   param_int[2]   = Failed joint bitmask (for multi-joint failures; 0 if none reported)
//   param_float[0] = Recovery progress in [0.0, 1.0]
//
// MUST stay in sync with pi_lib/pi_lib/constants.py mirrors of the same names.
#define DEVICE_INFO_ERROR_DETECTED       20  ///< Joint error detected; entering graceful recovery (slow ready move).
#define DEVICE_INFO_RECOVERY_IN_PROGRESS 21  ///< Periodic progress update while moving to ready during emergency recovery.
#define DEVICE_INFO_SHUTDOWN_AFTER_ERROR 22  ///< Recovery complete (or timed out); device is about to terminate.

// Unified move-to-ready heartbeat. Published every EMERGENCY_PROGRESS_PUBLISH_INTERVAL_MS by every
// active move_to_ready_position() (startup, command-driven, emergency recovery). Lets Python wait
// unbounded for DEVICE_INFO_READY_NOW while still detecting genuine hangs (no heartbeat for the
// configured stale threshold).
//   param_int[0]   = source enum (0=startup, 1=command, 2=emergency)
//   param_int[1]   = is_error (0=NORMAL speed, 1=ERROR speed)
//   param_float[0] = best-effort progress in [0.0, 1.0]
#define DEVICE_INFO_READY_MOVE_IN_PROGRESS 23
#define DEVICE_INFO_PROTOCOL_HANDSHAKE 30
#define DEVICE_INFO_COMMAND_ACK        31
#define DEVICE_INFO_RUNTIME_MODE       32

#define PI_CONTROL_PROTOCOL_VERSION_MAJOR 1
#define PI_CONTROL_PROTOCOL_VERSION_MINOR 1
#define PI_CONTROL_CAP_DIRECT_COMMAND      (1 << 0)
#define PI_CONTROL_CAP_LIVE_INPUT          (1 << 1)
#define PI_CONTROL_CAP_GRAVITY_COMP        (1 << 2)
#define PI_CONTROL_CAP_FORCE_FEEDBACK      (1 << 3)
#define PI_CONTROL_CAP_MOVE_TO_READY       (1 << 4)

// Source enum for DEVICE_INFO_READY_MOVE_IN_PROGRESS param_int[0]. Concrete devices pass
// these via Device::publish_ready_move_progress() so Python can distinguish startup vs
// command-driven vs emergency moves for logging/UI purposes.
#define READY_MOVE_SOURCE_STARTUP   0
#define READY_MOVE_SOURCE_COMMAND   1
#define READY_MOVE_SOURCE_EMERGENCY 2

class Device;
class DeviceConfig;

/*!
 * @brief Message types for teleoperation.
 */
enum class MsgType {
    INVALID     = 0, ///< Invalid or uninitialized message type.
    JOINT_INFO  = 1  ///< Joint state information (positions, velocities, torques).
};

/*!
 * @brief Single joint state data.
 */
class DataJoint {
   public:
    float curr_pos_;    ///< Current relative position of the joint (radians).
    float curr_vel_;    ///< Current velocity of the joint (rad/s).
    float curr_tor_;    ///< Current torque of the joint (Nm).
    float temperature_; ///< Current temperature of the joint (degrees Celsius).
    float idc_current_; ///< Input DC current of the joint (A).
};

/*!
 * @brief Joint state information for all joints.
 */
class MsgJoints {
   public:
    int msg_id_;                      ///< Message ID.
    std::vector<DataJoint> joints_;  ///< Vector containing state information for all joints.
    float measured_idc_current_;      ///< Measured total input DC current (A).

    /*!
     * @brief Adds joint state information to the message.
     * @param curr_pos_rel Joint position (radians).
     * @param curr_vel Joint velocity (rad/s).
     * @param curr_tor Joint torque (Nm).
     * @param temperature Joint temperature (degrees Celsius).
     * @param idc_current Joint DC current (A).
     */
    void add_joint_info(float curr_pos_rel, float curr_vel, float curr_tor, float temperature, float idc_current) {
        DataJoint data_joint;
        data_joint.curr_pos_ = curr_pos_rel;
        data_joint.curr_vel_ = curr_vel;
        data_joint.curr_tor_ = curr_tor;
        data_joint.temperature_ = temperature;
        data_joint.idc_current_ = idc_current;
        joints_.push_back(data_joint);
    }
};

/*!
 * @brief Device status and configuration information.
 */
class MsgDeviceInfo {
   public:
    int info_key_;                  ///< Information type identifier (see DEVICE_INFO_* macros).
    std::vector<float> float_data_; ///< Vector of floating-point parameter values.
    std::vector<int> int_data_;     ///< Vector of integer parameter values.

    /*!
     * @brief Constructor.
     * @param info_key Information type identifier.
     * @param p_float_data Optional floating-point data vector.
     * @param p_int_data Optional integer data vector.
     */
    MsgDeviceInfo(int info_key, std::vector<float>* p_float_data = nullptr, std::vector<int>* p_int_data = nullptr) {
        info_key_ = info_key;
        if (p_float_data != nullptr) {
            float_data_ = *p_float_data;
        }
        if (p_int_data != nullptr) {
            int_data_ = *p_int_data;
        }
    }
};

/*!
 * @brief End-effector state and control information.
 */
class MsgEffectorInfo {
   public:
    std::vector<float> param_float_; ///< Vector of floating-point effector parameters.

    /*!
     * @brief Constructor.
     * @param p_param_float Optional floating-point parameter vector.
     */
    MsgEffectorInfo(std::vector<float>* p_param_float = nullptr) {
        if (p_param_float != nullptr) {
            param_float_ = *p_param_float;
        }
    }
};

/*!
 * @brief Operator inputs published by the YAM teaching handle.
 *
 * The retained handle publishes its two digital buttons in ``button_[0..1]``.
 * ``channel_`` and ``raw_channel_`` remain in the stable wire layout for
 * forwards-compatible operator inputs.
 */
class MsgJoystick {
   public:
    enum Side { LEFT = 0, RIGHT = 1 } side_; ///< Joystick side identifier (left or right).
    std::vector<float> channel_;     ///< Analog channel values (see class docstring for layout).
    std::vector<int8_t> button_;     ///< Button states (0 = released, 1 = pressed; see class docstring for layout).
    /*!
     * @brief Raw per-channel reading straight from the device, before any normalization
     * or deadband processing.
     *
     * Same index layout as ``channel_`` (entry [i] is the raw counterpart to ``channel_[i]``).
     * Populated when an input device surfaces raw data so consumers can display it
     * without re-deriving it from normalized values.
     * ``int16_t`` is used so future >8-bit ADC devices fit without a wire change. Empty
     * when the publisher does not surface raw data.
     */
    std::vector<int16_t> raw_channel_;
    int8_t mode_ = 0;                ///< Joystick operational mode (reserved for future use).
};

/*!
 * @brief Control commands to the device.
 */
class MsgCommand {
   public:
    int command_;                  ///< Command type identifier.
    int num_param_float_;          ///< Number of floating-point parameters used.
    int num_param_int_;            ///< Number of integer parameters used.
    std::vector<float> param_float_; ///< Vector of floating-point parameter values.
    std::vector<int> param_int_;     ///< Vector of integer parameter values.
};

/*!
 * @brief Abstract base class for robot control topic management.
 */
class Topic {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the connected device.
     * @param cla Command-line arguments.
     */
    Topic(Device* p_device, const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    virtual ~Topic();

    /*!
     * @brief Performs one step of topic processing.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode step() = 0;

    /*!
     * @brief Sleeps for the control period.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode sleep() = 0;

    /*!
     * @brief Stops the topic and closes communication channels.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode stop() = 0;

    /*!
     * @brief Publishes joint state information.
     * @param msg Joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode publish(const MsgJoints& msg);

    /*!
     * @brief Publishes joystick input information.
     * @param msg Joystick information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode publish(const MsgJoystick& msg) = 0;

    /*!
     * @brief Publishes device status and configuration information.
     * @param msg Device information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode publish(const MsgDeviceInfo& msg) = 0;

    /*!
     * @brief Publishes end-effector state and control information.
     * @param msg Effector information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode publish(const MsgEffectorInfo& msg) = 0;

    /*!
     * @brief Checks if the topic is running.
     * @return True if running, false otherwise.
     */
    virtual bool is_running() { return is_running_; }

    /*!
     * @brief Returns the ID of the most recently received message.
     * @return The received message ID.
     */
    virtual int get_received_msg_id() { return msg_id_received_; }

    /*!
     * @brief Generates a unique topic name.
     * @param base_topic_name Base topic name.
     * @param device_model Device model identifier.
     * @param device_id Device identifier.
     * @return Generated unique topic name.
     */
    virtual std::string generate_unique_topic_name(const std::string& base_topic_name, const std::string& device_model,
                                                   const std::string& device_id);

    /*!
     * @brief Factory method to create a Topic instance.
     * @param p_device Pointer to the device.
     * @param p_config Pointer to the device configuration.
     * @param cla Command-line arguments.
     * @param argc Argument count.
     * @param argv Argument values.
     * @return Pointer to the created Topic instance, or nullptr on failure.
     */
    static std::shared_ptr<Topic> new_topic(Device* p_device, const DeviceConfig* p_config, const CommandLineArgs& cla,
                                            int argc, char** argv);

   protected:

    /*!
     * @brief Processes a received command message.
     * @param msg Command message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode process_leader_msg(const MsgCommand& msg);

    /*!
     * @brief Processes a received joint information message.
     * @param msg Joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode process_leader_msg(const MsgJoints& msg);

    ReturnCode process_direct_msg(const MsgJoints& msg);

    /*!
     * @brief Processes a received joystick input message.
     * @param msg Joystick information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode process_leader_msg(const MsgJoystick& msg);

    /*!
     * @brief Processes a received follower joint information message.
     * @param msg Follower joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode process_follower_msg(const MsgJoints& msg);

    Role role_;                           ///< Device role in teleoperation (leader or follower).
    std::string topic_joint_;             ///< Topic name identifier for joint information.
    Device* p_device_ = nullptr;         ///< Pointer to the connected device object.
    int control_frequency_;               ///< Control loop frequency in Hz.
    unsigned int msg_id_generation_ = 0;  ///< Counter for generating successive unique message IDs.
    unsigned int msg_id_received_ = 0;   ///< ID of the most recently received message.
    bool is_running_ = true;              ///< Flag indicating whether the topic is active and running.
    MsgType msg_type_ = MsgType::INVALID; ///< Current message type for teleoperation.

    // Follower-only gate:
    // When true, incoming leader joint commands are dropped.
    // This is used for pause/resume behavior (e.g. to temporarily stop reacting to leader commands).
    bool pause_leader_command_listening_ = false;

   public:
    /*!
     * @brief Forces ``pause_leader_command_listening_`` from outside the topic.
     *
     * Used by ``Device::enter_emergency_recovery()`` so that the topic immediately
     * stops feeding incoming leader joint commands to the device once recovery
     * begins. The pause flag is a no-op for LEADER devices.
     */
    void set_pause_leader_command_listening(bool pause) { pause_leader_command_listening_ = pause; }
};
