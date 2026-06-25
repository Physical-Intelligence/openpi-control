/*!
 * @file pi_stall_detect.hpp
 * @brief StallDetect class for detecting servo motor stall conditions.
 */

#pragma once
#include <boost/circular_buffer.hpp>

#include "pi_control.hpp"
#include "pi_command_line_args.hpp"

/*!
 * @brief Detects servo motor stall conditions based on position, velocity, and torque.
 */
class StallDetect {
   public:
    /*!
     * @brief Constructor.
     */
    StallDetect();

    /*!
     * @brief Destructor.
     */
    ~StallDetect();

    /*!
     * @brief Initializes the stall detection system.
     * @param cla Command line arguments containing configuration parameters.
     * @return ReturnCode indicating success or failure.
     */
    ReturnCode init(const CommandLineArgs& cla);

    /*!
     * @brief Detects stall condition based on current servo state.
     * @param pos Current position in radians.
     * @param vel Current velocity in rad/sec.
     * @param tor Current torque in Nm.
     * @return Average stall torque in Nm (0.0 if no stall detected).
     */
    float detect_stall(float pos, float vel, float tor);

    /*!
     * @brief Gets the average stall torque from the last detected stall.
     * @return Average stall torque in Nm (0.0 if no stall has been detected).
     */
    float get_stall_tor() { return stall_tor_; }

    /*!
     * @brief Checks if a stall condition is currently detected.
     * @return True if a stall is currently detected, false otherwise.
     */
    bool is_stall_detected() { return is_stall_detected_; }

   private:
    bool is_stall_detected_ = false;                    ///< Flag indicating whether a stall condition is currently detected.
    int stall_count_ = 0;                                ///< Consecutive count of control cycles meeting stall conditions.
    float stall_tor_ = 0.0f;                             ///< Average stall torque in Nm.
    float pos_prev_ = 0.0f;                              ///< Previous position in radians.
    boost::circular_buffer<float> stall_tor_buffer_;     ///< Circular buffer storing recent torque values during potential stall conditions.
    CommandLineArgs cla_;                                ///< Command line arguments.
};

