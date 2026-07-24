/*!
 * @file pi_servo_controller.hpp
 * @brief ServoController -- per-joint view onto a whole-arm controller (DriverController).
 *
 * Joints of controller-managed arms (Trossen iNerve etc.) are not individual
 * bus servos; the vendor controller owns the servo loops. This class adapts
 * one controller joint to the Servo interface so Joint / DeviceArm /
 * DeviceEffector logic (leash, safe mode, ready moves, torque grippers) runs
 * unchanged:
 *
 *  - move() queues a position target into the DriverController command buffer
 *    (flushed once per cycle as a whole-arm transaction).
 *  - apply_torque() queues an external effort on top of the controller's own
 *    gravity/friction compensation (leader spring / force feedback / torque
 *    gripper path).
 *  - change_control_mode_for_{leader,follower}() request per-joint mode
 *    switches, applied lazily as one vendor configuration call.
 *
 * Positions are exchanged in the controller's native joint space (radians;
 * gripper joints use meters), so model configs use zero_pos = 0 and the
 * controller-side position offset is the single zeroing authority.
 */

#pragma once
#include "pi_driver_controller.hpp"
#include "pi_servo.hpp"

/*!
 * @brief Parameter container class for controller-managed joint configuration.
 */
class ServoControllerParam : public ServoParam {
   public:
    /*!
     * @brief Constructor.
     * @param tolerable_pos_difference_rad Tolerable position difference threshold in radians.
     * @param max_pos_difference_rad Maximum position difference threshold in radians.
     * @param velocity_threshold_rad_sec Velocity threshold in rad/sec.
     */
    ServoControllerParam(float tolerable_pos_difference_rad, float max_pos_difference_rad,
                         float velocity_threshold_rad_sec)
        : ServoParam(tolerable_pos_difference_rad, max_pos_difference_rad, velocity_threshold_rad_sec) {}
};

/*!
 * @brief Manages one joint of a whole-arm controller through the Servo interface.
 */
class ServoController : public Servo {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device object.
     * @param p_joint Pointer to the Joint object.
     * @param p_driver Pointer to the Driver object (must be a DriverController).
     */
    ServoController(Device* p_device, Joint* p_joint, Driver* p_driver);

    /*!
     * @brief Destructor.
     */
    ~ServoController();

    /*!
     * @brief Initializes the servo with model configuration.
     * @param servo_config JSON object containing the servo model configuration.
     * @param p_config Pointer to the device model configuration object.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode init_config_model(const json& servo_config, const DeviceConfig* p_config) override;

    /*!
     * @brief Starts the joint: requests POSITION mode so the controller holds
     *        the current pose until ready moves take over.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode start_hardware() override;

    /*!
     * @brief Parks the joint: funnels into DriverController::make_safe(), which
     *        idles the whole arm exactly once (idempotent across joints).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode park_safely() override;

    /*!
     * @brief Fails fast when no whole-arm state read has succeeded yet.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode verify_position_fresh() override;

    /*!
     * @brief Queues a position target for the next whole-arm command flush.
     * @param target_pos Target position in relative radians (gripper: meters).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos) override;

    /*!
     * @brief Queues a position target with a feedforward velocity.
     *
     * The controller runs its own servo loops, so target_tor cannot bound the
     * motion per-command and is not consumed (joint limits on the controller
     * bound the effort).
     * @param target_pos Target position in relative radians (gripper: meters).
     * @param target_vel Target velocity in relative rad/sec (feedforward).
     * @param target_tor Target torque in Nm (not consumed).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode move(float target_pos, float target_vel, float target_tor) override;

    /*!
     * @brief Queues an external effort on top of the controller's own
     *        gravity/friction compensation.
     * @param torque Torque in relative Nm (gripper: N).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode apply_torque(float torque) override;

    /*!
     * @brief Same as apply_torque(); the controller applies its own damping.
     * @param torque Torque in relative Nm (gripper: N).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode apply_torque_with_damping(float torque) override;

    /*!
     * @brief Requests EXTERNAL_EFFORT mode for leader (teleoperation) operation.
     *
     * The controller keeps its own gravity/friction compensation active in this
     * mode; the model's spring / force-feedback torques ride on top as external
     * efforts, so the C++ Pinocchio gravity path must stay disabled
     * (individual config gravity_compensation = false).
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode change_control_mode_for_leader() override;

    /*!
     * @brief Requests POSITION mode for follower operation.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode change_control_mode_for_follower() override;

    /*!
     * @brief Gets the joint's current position in the controller's native unit.
     * @return Current position (radians; gripper joints use meters).
     */
    float get_pos_servo() override { return curr_pos_abs_; }

    /*!
     * @brief Copies one whole-arm state slice into this servo's caches.
     *        Called by DriverController::read_hardware_values().
     * @param position Position (rad; gripper: meters).
     * @param velocity Velocity (rad/s).
     * @param effort Effort (Nm).
     * @param temperature Motor temperature (degrees Celsius).
     */
    void update_from_controller_state(float position, float velocity, float effort, float temperature);

   protected:
    DriverController* p_driver_controller_ = nullptr;  ///< Pointer to the whole-arm controller driver.
};
