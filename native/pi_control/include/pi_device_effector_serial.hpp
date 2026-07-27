/*!
 * @file pi_device_effector_serial.hpp
 * @brief serial bus-servo effector device implementation.
 */
#pragma once
#include "pi_device_effector.hpp"

/*!
 * @brief serial bus-servo effector device implementation.
 */
class DeviceEffectorSerial : public DeviceEffector {
   public:
    /*!
     * @brief Constructor.
     * @param cla Command-line arguments.
     */
    DeviceEffectorSerial(const CommandLineArgs& cla);

    /*!
     * @brief Destructor.
     */
    ~DeviceEffectorSerial();

    //
    // Override functions
    //

    /*!
     * @brief Initializes the serial bus-servo effector device.
     * @param cla Command-line arguments.
     * @param argc Argument count.
     * @param argv Argument values.
     * @param p_topic Topic instance for communication.
     * @param p_driver Driver instance for hardware communication.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode init(const CommandLineArgs& cla, int argc, char** argv, std::shared_ptr<Topic> p_topic,
                            std::shared_ptr<Driver> p_driver) override;

    /*!
     * @brief Moves the effector to the ready position.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode move_to_ready_position() override;

    /*!
     * @brief Moves a joint using torque control.
     * @param p_joint Pointer to the joint.
     * @param target_pos Target position (relative radians).
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    virtual ReturnCode move_joint_with_torque(Joint* p_joint, float target_pos) override;

    /*!
     * @brief Sets control mode for serial bus-servo effector.
     *
     * Serial bus-servo devices: leader and follower require different servo operation modes, and follower behavior also depends on
     * effector control type (torque vs position). READY_MOVE_OVERRIDE forces a safe position-based mode via the
     * base-class override flag.
     */
    ReturnCode set_control_mode(Role target_role, ControlModeIntent intent) override;

    /*!
     * @brief Resets ready state so a commanded move-to-ready re-engages torque once.
     */
    void reset_ready_state_for_move_to_ready() override {
        DeviceEffector::reset_ready_state_for_move_to_ready();
        ready_move_torque_engaged_ = false;
    }

   private:
    /// True once the ready move has engaged servo torque; prevents per-cycle
    /// enable(true) re-sends that race the leader-passive torque disable.
    bool ready_move_torque_engaged_ = false;
};

