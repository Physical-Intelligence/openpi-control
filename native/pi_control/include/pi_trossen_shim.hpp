/*!
 * @file pi_trossen_shim.hpp
 * @brief TrossenArmShim -- isolation boundary around the prebuilt libtrossen_arm SDK.
 *
 * libtrossen_arm.a statically bundles its own copies of pinocchio, urdfdom and
 * tinyxml2. Linking that archive directly into pi_control_node makes those
 * symbols coexist (and interpose) with the shared pinocchio the node links for
 * AlgoPino, which mixes two different pinocchio builds inside one process and
 * crashes (SIGSEGV in pinocchio::ModelTpl::addJoint during
 * TrossenArmDriver::configure). The shim compiles the vendor archive into a
 * separate shared library (libpi_trossen_shim) whose archive symbols are
 * hidden (Linux: -Wl,--exclude-libs,ALL), so the vendor's pinocchio copy is
 * invisible to the rest of the process.
 *
 * This header is the ONLY interface the node sees: no vendor types, no vendor
 * headers. Every vendor exception is caught inside the shim and surfaced as a
 * false return + last_error() message so no vendor typeinfo crosses the
 * library boundary.
 */

#pragma once
#include <memory>
#include <string>
#include <vector>

/*!
 * @brief Thin client for one Trossen iNerve controller connection.
 *
 * All methods return true on success. On failure they return false and
 * last_error() carries the vendor error message (fast-fail: the caller is
 * expected to convert to its own error handling immediately).
 */
class TrossenArmShim {
   public:
    /*! @brief Joint command mode understood by the controller. */
    enum class Mode {
        IDLE = 0,             ///< Passive hold (controller-side gravity/friction compensation).
        POSITION = 1,         ///< Streamed position control.
        EXTERNAL_EFFORT = 2,  ///< Streamed external-effort (torque) control.
    };

    TrossenArmShim();
    ~TrossenArmShim();

    TrossenArmShim(const TrossenArmShim&) = delete;
    TrossenArmShim& operator=(const TrossenArmShim&) = delete;

    /*!
     * @brief Connect and configure the controller (TCP handshake + UDP stream).
     * @param model Controller model string from the model config (e.g. "wxai_v0").
     * @param leader_end_effector True selects the vendor leader end-effector mass
     *        properties (teleop handle); false selects the follower set.
     * @param address Controller IPv4 address.
     * @param clear_error True clears a stale latched controller error before configuring.
     * @param timeout_s TCP handshake timeout in seconds.
     * @return True on success.
     */
    bool configure(const std::string& model, bool leader_end_effector, const std::string& address, bool clear_error,
                   double timeout_s);

    /*! @brief Release the controller connection (idempotent). @return True on success. */
    bool cleanup();

    /*! @brief Number of joints reported by the controller (arm + gripper). */
    int num_joints() const;

    /*! @brief Vendor driver (SDK) version string. */
    std::string driver_version() const;

    /*! @brief Controller firmware version string. */
    std::string controller_version() const;

    /*!
     * @brief Read the cached whole-arm state (positions, velocities, efforts, rotor temperatures).
     * @return True on success; all vectors are resized to num_joints().
     */
    bool read_state(std::vector<double>& positions, std::vector<double>& velocities, std::vector<double>& efforts,
                    std::vector<double>& temperatures);

    /*!
     * @brief Stream position targets for all arm joints (gripper excluded).
     */
    bool set_arm_positions(const std::vector<double>& positions, double goal_time, bool blocking,
                           const std::vector<double>& velocity_ffs);

    /*!
     * @brief Stream external-effort targets for all arm joints (gripper excluded).
     */
    bool set_arm_external_efforts(const std::vector<double>& efforts, double goal_time, bool blocking);

    /*! @brief Stream a position target for one joint. */
    bool set_joint_position(int joint_index, double position, double goal_time, bool blocking, double velocity_ff);

    /*! @brief Stream an external-effort target for one joint. */
    bool set_joint_external_effort(int joint_index, double effort, double goal_time, bool blocking);

    /*! @brief Stream a position target for the gripper joint. */
    bool set_gripper_position(double position, double goal_time, bool blocking, double velocity_ff);

    /*! @brief Stream an external-effort target for the gripper joint. */
    bool set_gripper_external_effort(double effort, double goal_time, bool blocking);

    /*!
     * @brief Apply per-joint command modes (one TCP configuration call).
     * @param modes One mode per controller joint.
     */
    bool set_joint_modes(const std::vector<Mode>& modes);

    /*! @brief Idle every joint (controller-side passive hold). */
    bool set_all_modes_idle();

    /*! @brief Message of the most recent failure (empty when the last call succeeded). */
    const std::string& last_error() const;

   private:
    struct Impl;                  ///< Hides every vendor type behind the shim boundary.
    std::unique_ptr<Impl> impl_;  ///< Vendor driver instance (pimpl).
    std::string last_error_;      ///< Last failure message (vendor exception text).
};
