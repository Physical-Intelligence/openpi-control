/*!
 * @file pi_driver_trossen.hpp
 * @brief DriverTrossen -- Trossen iNerve arm controller driver (via TrossenArmShim over Ethernet).
 *
 * Implements the DriverController vendor hooks against TrossenArmShim, the
 * isolation boundary around the prebuilt libtrossen_arm SDK (see
 * pi_trossen_shim.hpp for why the SDK cannot be linked into the node
 * directly):
 *  - vendor_connect() -> shim configure() (TCP handshake + UDP stream)
 *  - vendor_read_state() -> cached whole-arm output from the SDK daemon thread
 *  - vendor_write_commands() -> set_arm_positions / set_arm_external_efforts /
 *    set_gripper_* streaming calls (goal_time 0, non-blocking)
 *  - vendor_apply_command_modes() -> set_joint_modes() (one TCP configuration call)
 *  - vendor_make_safe() -> set_all_modes_idle(); the controller then holds the
 *    arm passively with its own gravity/friction compensation
 *
 * All positions/velocities/efforts are exchanged in the controller's native
 * joint space (radians and Nm; the gripper joint uses meters and Newtons).
 * Zeroing is owned by the controller (position_offset joint characteristic,
 * managed from the Python setup backend), so model configs use zero_pos = 0.
 *
 * The SDK reports failures as C++ exceptions (including UDP stream loss
 * detected by its daemon thread, rethrown on the next SDK call); the shim
 * catches them and every hook converts to fast-failing ReturnCodes.
 */

#pragma once
#include <memory>
#include <string>
#include <vector>

#include "pi_driver_controller.hpp"
#include "pi_trossen_shim.hpp"

/*!
 * @brief Trossen iNerve whole-arm controller driver.
 */
class DriverTrossen : public DriverController {
   public:
    /*!
     * @brief Constructor.
     * @param p_device Pointer to the Device instance.
     * @param cla Command-line arguments (control_port_name carries the controller
     *        IP address; role selects the leader/follower end-effector properties).
     * @param controller_model Trossen model string from the model config (e.g. "wxai_v0").
     */
    DriverTrossen(Device* p_device, const CommandLineArgs& cla, const std::string& controller_model);

    /*!
     * @brief Destructor. Idles the arm and disconnects (idempotent).
     */
    ~DriverTrossen() override;

   protected:
    ReturnCode vendor_connect(const std::string& address, int& joint_count) override;
    ReturnCode vendor_disconnect() override;
    ReturnCode vendor_read_state(std::vector<JointState>& states) override;
    ReturnCode vendor_write_commands(const std::vector<JointCommand>& commands) override;
    ReturnCode vendor_apply_command_modes(const std::vector<JointCommandMode>& modes) override;
    ReturnCode vendor_make_safe() override;

   private:
    std::string controller_model_;              ///< Trossen model string from the model config.
    bool is_leader_role_ = false;               ///< True when this device runs as a leader.
    std::unique_ptr<TrossenArmShim> p_arm_;     ///< Shim around the vendor SDK (see pi_trossen_shim.hpp).
};
