/*!
 * @file pi_topic_zmq.hpp
 * @brief TopicZmq class and ZMQ message types for inter-process communication.
 */

#pragma once
#include <cstdint>
#include <vector>
#include <zmq.hpp>

#include "pi_topic.hpp"

/*!
 * @brief ZMQ message structure for transmitting joint state information.
 */
class ZmqJointInfo {
   public:
    float joint_pos[10];          ///< Joint positions (radians or meters).
    float joint_vel[10];          ///< Joint velocities (rad/s or m/s).
    float joint_tor[10];          ///< Joint torques (Nm).
    float temperature[10];        ///< Joint temperatures (degrees Celsius).
    float idc_current[10];        ///< Estimated DC current for each joint (A).
    /*!
     * @brief Age of the newest hardware feedback frame backing each joint's
     *        published position, in milliseconds (-1 = unknown/unsupported).
     *
     * The published positions come from a driver-side cache that the CAN RX
     * thread refreshes; when frames stop arriving the cache silently repeats
     * the last value. This field lets subscribers (Python evaluate loop)
     * distinguish live telemetry from a stale cache. Kept before ``msg_id``
     * so the trailing scalar layout matches the Python JOINT_STRUCT mirror.
     */
    float joint_age_ms[10];
    int32_t msg_id;                  ///< Unique message identifier for sequencing.
    int16_t joint_num;                ///< Number of active joints in this message.
    int16_t msg_type;                ///< Message type identifier.
    float measured_idc_current;   ///< Measured DC current value (A).
};

/*!
 * @brief ZMQ message structure for sending control commands to the robot.
 */
class ZmqCommand {
   public:
    int16_t command;          ///< Command type identifier.
    int16_t num_param_float;  ///< Number of floating-point parameters used.
    int16_t num_param_int;    ///< Number of integer parameters used.
    float param_float[10];    ///< Array of floating-point parameters.
    int32_t param_int[10];    ///< Array of integer parameters.
};

/*!
 * @brief ZMQ message structure for device status and configuration information.
 */
class ZmqDeviceInfo {
   public:
    int16_t info_key;         ///< Information type identifier (key).
    int16_t num_param_float;  ///< Number of floating-point parameters used.
    int16_t num_param_int;    ///< Number of integer parameters used.
    float param_float[10];    ///< Array of floating-point parameter values.
    int32_t param_int[10];    ///< Array of integer parameter values.
};

/*!
 * @brief ZMQ message structure for end-effector state and control information.
 */
class ZmqEffectorInfo {
   public:
    int16_t num_param_float;   ///< Number of floating-point parameters used.
    float param_float[30];     ///< Array of floating-point parameter values.
};

static_assert(sizeof(ZmqJointInfo) == 252, "ZmqJointInfo ABI changed");
static_assert(sizeof(ZmqCommand) == 88, "ZmqCommand ABI changed");
static_assert(sizeof(ZmqDeviceInfo) == 88, "ZmqDeviceInfo ABI changed");

/*!
 * @brief ZMQ message structure for joystick input data.
 *
 * Wire layout (sizeof == 44 bytes; native alignment, matches the Python
 * struct format in ``pi_control_agent.py:ZMQ_JOYSTICK_INFO_FORMAT``):
 *
 *   offset  size  field
 *   ------  ----  ---------------
 *    0       1    mode             (int8)
 *    1       1    side             (int8: 0 = LEFT, 1 = RIGHT)
 *    2       2    <pad>            (alignment for channel[])
 *    4      20    channel[5]       (float32)
 *   24       1    channel_num      (int8)
 *   25       5    button[5]        (int8)
 *   30       1    button_num       (int8)
 *   31       1    <pad>            (alignment for raw_channel[])
 *   32      10    raw_channel[5]   (int16)
 *   42       2    <trailing pad>   (struct alignment to 4)
 *
 * The ``side`` slot reuses 1 byte of what used to be leading padding after
 * ``mode``, so the total size stays at 44 bytes — subscribers built before
 * ``side`` was added simply read padding (zero) here, which decodes as LEFT
 * (the previous topic-name-only default). Subscribers built after the
 * change read the publisher's reconciled side directly.
 */
class ZmqJoystickInfo {
   public:
    int8_t mode;          ///< Joystick operational mode (legacy OP_MODE register on supported firmware).
    /*!
     * @brief Joystick side as reconciled by the publisher.
     *
     * Matches :class:`MsgJoystick::Side` (``0`` = LEFT, ``1`` = RIGHT). This
     * carries the side selected by the publisher; subscribers should prefer
     * this over the topic name.
     */
    int8_t side;
    float channel[5];     ///< Analog channel values (normalized to [-1.0, 1.0]).
    int8_t channel_num;   ///< Number of active analog channels.
    int8_t button[5];      ///< Button states (0 = released, 1 = pressed).
    int8_t button_num;    ///< Number of active buttons.
    /*!
     * @brief Raw per-channel device reading before any normalization (see
     * :class:`MsgJoystick::raw_channel_` in ``pi_topic.hpp`` for semantics).
     *
     * ``int16_t`` so future >8-bit ADC sensors fit without a wire format change.
     * Zero-padded to ``[5]`` to mirror ``channel`` / ``button``. Reuses
     * ``channel_num`` for the populated count — no separate ``raw_channel_num``
     * is needed because the publisher always fills the same index range it
     * filled in ``channel``. The Python mirror in ``pi_control_agent.py``
     * must be kept in sync with this layout.
     */
    int16_t raw_channel[5];
};

/*!
 * @brief Enumeration for ZMQ socket port types.
 */
enum class ZmqPortType {
    PUB, ///< Publisher port type for sending messages.
    SUB  ///< Subscriber port type for receiving messages.
};

/*!
 * @brief ZMQ-based implementation of the Topic interface for robot control communication.
 */
class TopicZmq : public Topic {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the device.
     * @param cla Command-line arguments.
     * @param argc Argument count.
     * @param argv Argument values.
     */
    TopicZmq(Device* p_device, const CommandLineArgs& cla, int argc, char** argv);

    /*!
     * @brief Destructor.
     */
    ~TopicZmq();

    /*!
     * @brief Performs one step of topic processing.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode step() override;

    /*!
     * @brief Sleeps for the control period.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode sleep() override;

    /*!
     * @brief Stops the topic and closes communication channels.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode stop() override;

    /*!
     * @brief Publishes joint state information.
     * @param msg Joint information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode publish(const MsgJoints& msg) override;

    /*!
     * @brief Publishes device status and configuration information.
     * @param msg Device information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode publish(const MsgDeviceInfo& msg) override;

    /*!
     * @brief Publishes end-effector state and control information.
     * @param msg Effector information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode publish(const MsgEffectorInfo& msg) override;

    /*!
     * @brief Publishes joystick input information.
     *
     * Serializes ``msg`` into a ``ZmqJoystickInfo`` and sends it on
     * ``pub_joystick_``. The publisher is bound to the per-side topic
     * (``joystick_info_left`` or ``joystick_info_right``) chosen via
     * ``--topic_joystick``; **independent of that topic name**, the
     * ``side`` byte of the published payload is filled from
     * :member:`MsgJoystick::side_`. Subscribers should prefer the in-payload
     * side regardless of which topic the launcher wired up.
     *
     * @param msg Joystick information message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode publish(const MsgJoystick& msg) override;

   private:

    /*!
     * @brief Processes a received joint information message.
     * @param p_zmq_msg Pointer to the ZmqJointInfo message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode process_leader_msg_joint(ZmqJointInfo* p_zmq_msg, bool direct = false);

    /*!
     * @brief Processes a received joystick input message.
     *
     * The in-payload ``side`` byte (filled by the publisher from the
     * reconciled :member:`MsgJoystick::side_`) is the source of truth.
     * The ``topic_side`` argument is only used as a fallback for legacy
     * publishers that pre-date the in-payload ``side`` and therefore
     * leave that byte at the leading-pad default (zero).
     *
     * @param p_zmq_msg Pointer to the ZmqJoystickInfo message.
     * @param topic_side Side derived from the topic name; used as a
     *        fallback when the payload's own ``side`` byte is unknown.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode process_leader_msg_joystick(ZmqJoystickInfo* p_zmq_msg, MsgJoystick::Side topic_side);

    /*!
     * @brief Processes a received follower joint information message.
     * @param p_zmq_msg Pointer to the ZmqJointInfo message.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode process_follower_msg_joint(ZmqJointInfo* p_zmq_msg);

    /*!
     * @brief Resolve the ZMQ connect host for a subscriber topic.
     *
     * ZMQ sockets need an explicit connect address. This function encodes the
     * topic-level ownership contract (who publishes which topic) for ZMQ.
     *
     * @param topic_name Topic name.
     * @return Hostname/IP for `tcp://<host>:<port>` connect.
     * @throws std::runtime_error if the topic is not recognized as a SUB topic.
     */

    /*!
     * @brief Generates a port number from a topic name.
     *
     * NOTE:
     * A single hash→port mapping can collide inside the constrained range.
     * Use generate_port_candidates_from_topic() for collision-resilient bind/connect.
     * @param topic_name Topic name.
     * @return Generated port number.
     */
    int generate_port_from_topic(const std::string& topic_name);

    /*!
     * @brief Generate a deterministic probe sequence of candidate ports for a topic.
     *
     * Design:
     * - Deterministic across processes/languages (must match Python's port_hash.py).
     * - PUB sockets can bind the first available candidate.
     * - SUB sockets can connect to multiple candidates to remain robust even if the
     *   publisher had to skip the first port due to EADDRINUSE.
     */
    std::vector<int> generate_port_candidates_from_topic(const std::string& topic_name, int max_candidates);

    /*!
     * @brief Initializes a ZMQ socket port.
     * @param port ZMQ socket to initialize.
     * @param port_type Port type (PUB or SUB).
     * @param topic_name Topic name.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode init_port(zmq::socket_t& port, const ZmqPortType& port_type, const std::string& topic_name);

    static constexpr int kZmqSndHwm = 2;
    static constexpr int kZmqRcvHwm = 2;
    static constexpr int kZmqPortProbeCandidatesPub = 16;
    // Subscribers must probe the publisher's full bind-fallback window or a
    // publisher that fell back past the subscriber's last candidate becomes
    // silently unreachable; connecting to unused ports is harmless.
    static constexpr int kZmqPortProbeCandidatesSub = kZmqPortProbeCandidatesPub;


    zmq::context_t context_;            ///< ZMQ context managing all socket resources.
    zmq::socket_t pub_joint_;           ///< Publisher socket for joint state messages.
    zmq::socket_t pub_follower_joint_;  ///< Publisher socket for follower joint state messages.
    zmq::socket_t pub_device_info_;     ///< Publisher socket for device information messages.
    zmq::socket_t pub_effector_info_;   ///< Publisher socket for effector information messages.
    zmq::socket_t pub_joystick_;        ///< Publisher socket for joystick information messages (leader-only).
    std::string topic_joystick_;        ///< Topic name used by pub_joystick_ (e.g. joystick_info_left).
    zmq::socket_t sub_joint_;           ///< Subscriber socket for joint state messages.
    zmq::socket_t sub_direct_joint_;    ///< Follower-only direct Python command input.
    zmq::socket_t sub_follower_joint_;  ///< Subscriber socket for follower joint state messages.
    zmq::socket_t sub_command_;         ///< Subscriber socket for command messages.
    zmq::socket_t sub_joystick_left_;   ///< Subscriber socket for left joystick messages.
    zmq::socket_t sub_joystick_right_; ///< Subscriber socket for right joystick messages.
    int control_frequency_;             ///< Control loop frequency in Hz.
    prof_time_msec_t control_period_;   ///< Control period duration in milliseconds.
    prof_time_t prev_time_;             ///< Timestamp of the previous sleep() call.
    std::string topic_joint_;           ///< Topic name identifier for joint information.
    std::string topic_follower_joint_;  ///< Topic name identifier for follower joint information.
    std::string topic_state_;           ///< Unique public state topic.
    std::string topic_direct_command_;  ///< Unique direct Python command topic.
    std::string topic_lifecycle_;       ///< Unique lifecycle command topic.
    std::string topic_status_;          ///< Unique status and acknowledgement topic.
};
