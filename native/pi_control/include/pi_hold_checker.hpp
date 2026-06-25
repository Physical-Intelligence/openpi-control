/*!
 * @file pi_hold_checker.hpp
 * @brief HoldChecker class for state persistence checking with hysteresis.
 */

#pragma once

/*!
 * @brief Hysteresis-based state checker for detecting persistent conditions.
 */
class HoldChecker {
   public:
    /*!
     * @brief Constructor.
     * @param hold_count_threshold Number of consecutive true conditions required to enter the holding state.
     */
    HoldChecker(int hold_count_threshold);

    /*!
     * @brief Destructor.
     */
    ~HoldChecker();

    /*!
     * @brief Checks if the condition has persisted long enough to be in holding state.
     * @param condition Boolean condition to check.
     * @return True if in holding state, false otherwise.
     */
    bool is_holding(bool condition);

    /*!
     * @brief Resets the hold checker to initial state.
     */
    void reset();

    /*!
     * @brief Sets a new hold count threshold and resets the checker.
     * @param hold_count_threshold New threshold for entering holding state.
     */
    void set_hold_count_threshold(int hold_count_threshold);

   private:
    int hold_count_;  ///< Counter tracking consecutive true conditions.
    int exit_count_;  ///< Counter tracking consecutive false conditions.
    bool holding_status_;  ///< Current holding state.
    int hold_count_threshold_;  ///< Threshold for entering holding state.
};

