/*!
 * @file pi_algo.hpp
 * @brief Defines the Algo base class and related types for implementing robot control algorithms.
 */
#pragma once
#include <string>
#include <vector>

#include "pi_control.hpp"
#include "pi_device_config.hpp"
#include "pi_profile.hpp"

class Joint;

///< @todo Need to change to accept any link names from URDF. Otherwise, manual update is required when link names change.
#define LINK_NAME_END "end_link"     ///< Default name for the end-effector link in the URDF model.

/*!
 * @enum TrajectoryPlanningType
 * @brief Enumeration for different trajectory planning strategies.
 */
enum class TrajectoryPlanningType {
    NONE,                     ///< No trajectory planning; target position is sent directly to the servo.
    SLEW_POS_GRAVITY          ///< Velocity-limited follower tracking with gravity compensation.
};

/*!
 * @enum SpringMode
 * @brief Enumeration for spring effect control modes.
 */
enum class SpringMode {
    PASSIVE,   ///< Passive mode; no spring effect is applied. The arm moves freely without resistance.
    POSITION,  ///< Position-based mode; spring effect is disabled when the arm is stable.
    SPRING     ///< Active spring mode; spring effect is applied to provide haptic feedback and stability.
};

class Device;

/*!
 * @class Algo
 * @brief Abstract base class for implementing robot control algorithms.
 */
class Algo {
   public:
    /*!
     * @brief Constructs a new Algo instance.
     * @param p_device Pointer to the Device instance that this algorithm will control.
     *                 Must not be nullptr.
     * @param cla Command-line arguments containing configuration parameters such as URDF file path,
     */
    Algo(Device* p_device, const CommandLineArgs& cla);

    // Virtual destructor ensures proper cleanup of derived class instances.
    virtual ~Algo();

    /*!
     * @brief Calculates gravity compensation torques for the given joint positions.
     * @param joint_positions Vector of current joint positions in radians. The size must match
     *                        the number of joints in the robot model.
     * @param calculated_torques Output vector that will be populated with the calculated gravity
     *                           compensation torques in Nm. The vector will be resized if necessary.
     * @return ReturnCode::SUCCESS if the calculation succeeds (or if not implemented), otherwise
     */
    virtual ReturnCode gravity_compensation(const std::vector<float>& joint_positions,
                                            std::vector<float>& calculated_torques) {
        (void)joint_positions;
        (void)calculated_torques;
        return ReturnCode::SUCCESS;
    }

    /*!
     * @brief Initializes the algorithm with the given device configuration.
     * @param p_config_model Pointer to the device model configuration containing URDF path, joint
     *                       limits, and other model-specific parameters. Must not be nullptr.
     * @param p_config_individual Pointer to the individual device configuration containing
     *                           device-specific calibration and tuning parameters. May be nullptr.
     * @param cla Command-line arguments containing additional runtime configuration such as control
     *            frequency and algorithm-specific options.
     * @return ReturnCode::SUCCESS if initialization succeeds, otherwise an appropriate error code
     */
    virtual ReturnCode init(const DeviceConfig* p_config_model, const DeviceConfig* p_config_individual,
                            const CommandLineArgs& cla);

    /*!
     * @brief Ramps the current position towards the goal position at a specified rate.
     * @param goal The target position in radians.
     * @param current The current position in radians.
     * @param ramp_rate The maximum rate of change in radians per control cycle.
     * @return The new position after applying the ramp constraint, in radians. The returned value
     */
    float ramp_pos(float goal, float current, float ramp_rate);

    /*!
     * @brief Determines the next target position based on the current and target positions.
     * @param p_joint Pointer to the Joint instance for which to calculate the next target position.
     *                Must not be nullptr.
     * @param previous_target Previous target position in radians (relative to joint zero position).
     * @param target_pos Desired target position in radians (relative to joint zero position).
     * @param safe_mode_derating Scaling factor (0.0 to 1.0) to apply to maximum velocity and
     *                           acceleration limits during safe mode or initialization.
     * @return The next target position in radians that respects velocity/acceleration limits and
     */
    float get_next_target_pos(Joint* p_joint, float previous_target, float target_pos, float safe_mode_derating);

    /*!
     * @brief Checks and applies stability control with spring effects for leader arms.
     * @param joints Vector of pointers to Joint instances that form the arm. Must not be empty.
     * @param step_start_time Starting time of the current control step, used for timing stability
     *                        evaluations and mode transitions.
     * @return ReturnCode::SUCCESS if stability control is applied successfully, otherwise an error
     */
    ReturnCode check_stability_control(std::vector<Joint*>& joints, const prof_time_t& step_start_time);

    /*!
     * @brief Changes the arm control mode to the specified spring mode.
     * @param joints Vector of pointers to Joint instances that form the arm. Must not be empty.
     * @param new_mode The new SpringMode to apply (PASSIVE, POSITION, or SPRING).
     * @return ReturnCode::SUCCESS if the mode transition succeeds, otherwise an error code
     */
    virtual ReturnCode set_control_mode(std::vector<Joint*>& joints, SpringMode new_mode);

    /*!
     * @brief Factory method to create a new Algo instance based on the device configuration.
     * @param p_device Pointer to the Device instance that the algorithm will control. Must not be
     *                nullptr.
     * @param p_config Pointer to the device configuration containing algorithm selection parameters
     *                 and model information. Must not be nullptr.
     * @param cla Command-line arguments containing additional runtime configuration.
     * @return Pointer to the newly created Algo instance. The caller is responsible for managing
     */
    static Algo* new_algo(Device* p_device, const DeviceConfig* p_config, const CommandLineArgs& cla);

   protected:
    std::string urdf_path_;  ///< Path to the URDF file describing the robot model structure.

    int init_count_ = 0;  ///< Counter tracking the number of times init() has been called. Used for debugging.

    int calc_done_t_ = 0;  ///< Timestamp indicating when the last calculation was completed. Used for performance monitoring.

    int control_frequency_ = 50;  ///< The main control loop frequency in Hz. Reinitialized in init() with config parameter.

    Device* p_device_ = nullptr;  ///< Pointer to the Device instance that this algorithm controls. Set during construction.

    std::vector<float> base_rpy_;  ///< Base frame rotation angles [roll, pitch, yaw] in radians, used for coordinate transformations.

    // std::vector<std::pair<int, int>> collision_pairs_; ///< Reserved for future use: pairs of joint indices that
    // should avoid collisions.
};
