/*!
 * @file pi_force_feedback.hpp
 * @brief ForceFeedback class for haptic force feedback in teleoperation.
 */

#pragma once

#include <boost/circular_buffer.hpp>

#include "pi_control.hpp"
#include "pi_filter.hpp"
#include "pi_stall_detect.hpp"
#include "pi_command_line_args.hpp"

/*!
 * @brief Class for calculating haptic force feedback in teleoperation systems.
 */
class ForceFeedback {
   public:
    /*!
     * @brief Constructor.
     */
    ForceFeedback();

    // Copy constructor is deleted to prevent accidental filter state duplication.
    ForceFeedback(const ForceFeedback& other) = delete;

    // Copy assignment operator is deleted to prevent accidental filter state duplication.
    ForceFeedback& operator=(const ForceFeedback& other) = delete;

    // Move constructor is deleted to prevent accidental filter state transfer.
    ForceFeedback(ForceFeedback&& other) noexcept = delete;

    // Move assignment operator is deleted to prevent accidental filter state transfer.
    ForceFeedback& operator=(ForceFeedback&& other) noexcept = delete;

    /*!
     * @brief Destructor.
     */
    ~ForceFeedback();

    /*!
     * @brief Initializes the force feedback system.
     * @param cla Command-line arguments.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode init(const CommandLineArgs& cla);

    /*!
     * @brief Gets the latest filtered force feedback torque value.
     * @return Latest filtered force feedback torque in Nm.
     */
    float get_force_feedback_torque() { return force_feedback_torque_filter_.get_latest_value(); }

    /*!
     * @brief Updates follower state information and calculates force feedback.
     * @param leader_pos Current leader position in relative radians.
     * @param follower_pos Current follower position in relative radians.
     * @param follower_vel Current follower velocity in rad/s.
     * @param follower_tor Current follower torque in Nm.
     * @param follower_temperature Current follower temperature in degrees Celsius.
     * @param follower_idc_current Current follower input DC current in Amperes.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode update_follower_info(float leader_pos, float follower_pos, float follower_vel, float follower_tor,
                                    float follower_temperature, float follower_idc_current);

    /*!
     * @brief Gets the latest filtered stall torque value.
     * @return Latest filtered stall torque in Nm.
     */
    float get_stall_tor() { return stall_tor_filter_.get_latest_value(); }

    /*!
     * @brief Gets the latest filtered follower position value.
     * @return Latest filtered follower position in relative radians.
     */
    float get_follower_pos() { return follower_pos_filter_.get_latest_value(); }

    /*!
     * @brief Gets the latest filtered follower velocity value.
     * @return Latest filtered follower velocity in rad/s.
     */
    float get_follower_vel() { return follower_vel_filter_.get_latest_value(); }

    /*!
     * @brief Gets the latest filtered follower torque value.
     * @return Latest filtered follower torque in Nm.
     */
    float get_follower_tor() { return follower_tor_filter_.get_latest_value(); }

    /*!
     * @brief Gets the current follower temperature value.
     * @return Current follower temperature in degrees Celsius.
     */
    int get_follower_temperature() { return follower_temperature_; }

    /*!
     * @brief Gets the current follower input DC current value.
     * @return Current follower input DC current in Amperes.
     */
    float get_follower_idc_current() { return follower_idc_current_; }

   private:
    /*!
     * @brief Calculates the force feedback torque based on leader and follower positions.
     * @param leader_pos Current leader position in relative radians.
     * @return Calculated force feedback torque in Nm.
     */
    float calc_force_feedback_torque(float leader_pos);

    StallDetect stall_detect_;  ///< Stall detection object.
    float follower_pos_ = 0.0f;  ///< Current follower position in relative radians.
    float follower_vel_ = 0.0f;  ///< Current follower velocity in rad/s.
    float follower_tor_ = 0.0f;  ///< Current follower torque in Nm.
    float follower_temperature_ = 0.0f;  ///< Current follower temperature in degrees Celsius.
    float follower_idc_current_ = 0.0f;  ///< Current follower input DC current in Amperes.
    float prev_leader_pos_ = 0.0f;  ///< Previous leader position in relative radians.
    Filter follower_pos_filter_;  ///< Moving average filter for follower position signals.
    Filter follower_vel_filter_;  ///< Moving average filter for follower velocity signals.
    Filter follower_tor_filter_;  ///< Moving average filter for follower torque signals.
    Filter stall_tor_filter_;  ///< Moving average filter for stall torque signals.
    Filter force_feedback_torque_filter_;  ///< Moving average filter for force feedback torque signals.
    CommandLineArgs cla_;  ///< Command-line arguments containing force feedback configuration parameters.
};

