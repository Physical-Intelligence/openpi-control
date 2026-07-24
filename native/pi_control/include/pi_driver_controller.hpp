/*!
 * @file pi_driver_controller.hpp
 * @brief DriverController -- vendor-neutral base class for whole-arm robot controllers.
 *
 * Covers arms that are NOT driven servo-by-servo over CAN/serial but through a
 * vendor controller box exposing a whole-arm API over Ethernet (Trossen iNerve,
 * and structurally similar stacks like Franka FCI or UR RTDE). The class keeps
 * the existing Driver group read/write contract so DeviceArm / DeviceEffector /
 * Joint / Servo code runs unchanged:
 *
 *  - group_read_hardware_values()  -> one whole-arm state fetch into a cache;
 *    read_hardware_values(Servo*) copies the per-joint slice into ServoController.
 *  - ServoController::move()/apply_torque() queue per-joint commands here;
 *    group_write_hardware_values() flushes them as one whole-arm transaction.
 *  - Per-joint control modes (position / external effort) are buffered and
 *    applied lazily right before the next command flush, so a leader/follower
 *    switch costs one vendor configuration call instead of one per joint.
 *
 * Lifecycle safety (vendor-neutral):
 *  - open() is a template method: vendor_connect() -> joint count validation ->
 *    initial state read, so a half-configured controller can never enter the
 *    control loop.
 *  - make_safe() idles the whole arm exactly once (idempotent, thread-safe);
 *    close() and the stall watchdog both funnel into it.
 *  - arm_comm_loss_protection() starts a stall watchdog thread: if the host
 *    control loop stops calling group read/write while the vendor daemon keeps
 *    streaming the last command (host hang), the watchdog idles the arm.
 */

#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "pi_driver.hpp"

class ServoController;

/*!
 * @brief Vendor-neutral base class for whole-arm controller drivers.
 */
class DriverController : public Driver {
   public:
    /*!
     * @brief Per-joint control mode requested from the vendor controller.
     */
    enum class JointCommandMode {
        UNSET,            ///< No mode selected yet (joint not started).
        POSITION,         ///< Controller-side position servoing (follower / ready moves).
        EXTERNAL_EFFORT,  ///< Torque on top of the controller's own gravity/friction compensation (leader).
    };

    /*!
     * @brief Per-joint command buffered until the next group write flush.
     */
    struct JointCommand {
        JointCommandMode mode = JointCommandMode::UNSET;  ///< Mode the command was queued for.
        float position = 0;                               ///< Goal position (rad; gripper joints use meters).
        float velocity_ff = 0;                            ///< Feedforward velocity (rad/s), 0 when unused.
        float external_effort = 0;                        ///< External effort (Nm; gripper joints use N).
        bool pending = false;                             ///< True when queued and not yet flushed.
    };

    /*!
     * @brief Per-joint state cache filled by group_read_hardware_values().
     */
    struct JointState {
        float position = 0;     ///< Position (rad; gripper joints use meters).
        float velocity = 0;     ///< Velocity (rad/s).
        float effort = 0;       ///< Effort (Nm).
        float temperature = 0;  ///< Motor temperature (degrees Celsius).
        bool valid = false;     ///< True once at least one whole-arm read succeeded.
    };

    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device instance.
     * @param cla Command-line arguments (control_port_name carries the controller IP address).
     */
    DriverController(Device* p_device, const CommandLineArgs& cla);

    /*!
     * @brief Destructor. Funnels into close() so the arm is idled even when the
     *        owning Device skips its stop path (idempotent).
     */
    ~DriverController() override;

    /*!
     * @brief Connects to the controller (template method around vendor_connect()).
     *
     * Fast-fails when the vendor reports fewer joints than the model config
     * registered, and performs one initial whole-arm read so the position
     * caches are valid before any servo is started.
     * @param baud_rate Unused for Ethernet controllers (Driver interface compatibility).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode open(int baud_rate) override;

    /*!
     * @brief Idles the arm and disconnects. Idempotent: safe to call repeatedly
     *        and from the destructor.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode close() override;

    /*!
     * @brief Copies the cached whole-arm state slice of one joint into its ServoController.
     * @param p_servo Pointer to the Servo instance (must be a ServoController).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode read_hardware_values(Servo* p_servo) override;

    /*!
     * @brief Fetches the whole-arm state from the controller into the joint state cache.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode group_read_hardware_values() override;

    /*!
     * @brief Applies pending mode changes, then flushes the queued per-joint
     *        commands as whole-arm vendor transactions.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode group_write_hardware_values() override;

    /*!
     * @brief Starts the stall watchdog. Called once by the node right before
     *        the main control loop starts.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode arm_comm_loss_protection() override;

    /*!
     * @brief Requests a control mode for one joint. Buffered; the vendor
     *        configuration call happens once in the next group write flush.
     * @param data_index Joint slot (Servo::data_index_).
     * @param mode Requested mode.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode set_joint_command_mode(int data_index, JointCommandMode mode);

    /*!
     * @brief Queues a position command for one joint.
     * @param data_index Joint slot (Servo::data_index_).
     * @param position Goal position (rad; gripper joints use meters).
     * @param velocity_ff Feedforward velocity (rad/s), 0 when unused.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode queue_position(int data_index, float position, float velocity_ff);

    /*!
     * @brief Queues an external effort command for one joint.
     * @param data_index Joint slot (Servo::data_index_).
     * @param external_effort External effort (Nm; gripper joints use N).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode queue_external_effort(int data_index, float external_effort);

    /*!
     * @brief Puts the whole arm in the vendor's safe passive state exactly once.
     *
     * Thread-safe and idempotent: park_safely() of every ServoController, close(),
     * the stall watchdog, and the destructor all funnel into this method; only the
     * first caller performs the vendor call.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode make_safe();

    /*!
     * @brief Number of joints the vendor controller reports (arm + gripper).
     */
    int controller_joint_count() const { return joint_count_; }

    /*!
     * @brief True once open() completed and close() has not run.
     */
    bool is_open() const { return opened_; }

    /*!
     * @brief True once at least one whole-arm state read succeeded.
     */
    bool has_valid_state() const { return read_success_count_ > 0; }

   protected:
    // ---- Vendor hooks (implemented by DriverTrossen and future controller drivers) ----

    /*!
     * @brief Establishes the connection to the controller and returns its joint count.
     * @param address Controller network address (from --control_port_name).
     * @param joint_count Output: number of joints the controller manages.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode vendor_connect(const std::string& address, int& joint_count) = 0;

    /*!
     * @brief Tears the connection down. Must tolerate being called when the
     *        connection is already gone.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode vendor_disconnect() = 0;

    /*!
     * @brief Reads the whole-arm state.
     * @param states Output vector sized to controller_joint_count().
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode vendor_read_state(std::vector<JointState>& states) = 0;

    /*!
     * @brief Sends the pending commands (only entries with pending == true).
     * @param commands Command buffer sized to controller_joint_count().
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode vendor_write_commands(const std::vector<JointCommand>& commands) = 0;

    /*!
     * @brief Applies the requested per-joint control modes on the controller.
     * @param modes Mode vector sized to controller_joint_count(); UNSET entries
     *        must be mapped to the vendor's passive/idle mode.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode vendor_apply_command_modes(const std::vector<JointCommandMode>& modes) = 0;

    /*!
     * @brief Puts every joint in the vendor's safe passive state (e.g. idle with
     *        controller-side gravity compensation). Must not throw.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode vendor_make_safe() = 0;

   private:
    /*!
     * @brief Validates a joint slot index against the vendor joint count.
     */
    ReturnCode validate_data_index(int data_index) const;

    /*!
     * @brief Stall watchdog loop: idles the arm when the host control loop
     *        stops interacting with the driver while commands may still stream.
     */
    void watchdog_loop();

    /*!
     * @brief Records a successful driver interaction for the stall watchdog.
     */
    void feed_watchdog();

    /*!
     * @brief Stops the watchdog thread (idempotent).
     */
    void stop_watchdog();

    int joint_count_ = 0;                       ///< Joint count reported by the vendor controller.
    bool opened_ = false;                       ///< True between successful open() and close().
    std::atomic<uint64_t> read_success_count_{0};  ///< Successful whole-arm reads since open().

    std::mutex command_mutex_;                  ///< Guards commands_, desired_modes_, modes_dirty_.
    std::vector<JointCommand> commands_;        ///< Per-joint command buffer (indexed by data_index).
    std::vector<JointCommandMode> desired_modes_;  ///< Per-joint requested modes.
    bool modes_dirty_ = false;                  ///< True when desired_modes_ changed since the last flush.

    std::mutex state_mutex_;                    ///< Guards states_.
    std::vector<JointState> states_;            ///< Per-joint state cache (indexed by data_index).

    std::mutex safe_mutex_;                     ///< Serializes make_safe() / close().
    bool made_safe_ = false;                    ///< True once vendor_make_safe() ran.

    std::thread watchdog_thread_;               ///< Stall watchdog thread.
    std::atomic<bool> watchdog_running_{false};  ///< Watchdog thread lifecycle flag.
    std::atomic<bool> watchdog_tripped_{false};  ///< Latched when the watchdog idled the arm.
    std::atomic<int64_t> last_interaction_ms_{0};  ///< Monotonic ms of the last successful read/write.
};
