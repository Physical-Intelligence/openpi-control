/*!
 * @file pi_info.hpp
 * @brief Logging functions and information levels.
 */

#pragma once
#include <string>
#include <unordered_set>

/*!
 * @brief Information levels for categorizing log messages.
 */
enum class InfoLevel {
    ESSENTIAL_0 = 0,  ///< Essential information that should always be displayed.
    HELPFUL_1 = 1,    ///< Helpful information for understanding system behavior.
    DETAIL_2 = 2,     ///< Detailed information for debugging.
    FREQUENT_3 = 3,   ///< Frequent information that may be verbose.
    UI = 100          ///< UI messages displayed without scrolling.
};

/*!
 * @brief Logs an informational message with a specific group ID and information level.
 * @param group_id The group identifier for categorizing the log message.
 * @param info_level The information level of the log message.
 * @param format The format string for the message (supports printf-style formatting).
 * @param ... Additional arguments to format the message.
 */
void PI_INFO(const std::string& group_id, const InfoLevel info_level, const std::string format, ...);

/*!
 * @brief Logs an informational message with scroll control.
 * @param no_scroll If true, clears the screen before displaying the message to prevent scrolling.
 * @param group_id The group identifier for categorizing the log message.
 * @param info_level The information level of the log message.
 * @param format The format string for the message (supports printf-style formatting).
 * @param ... Additional arguments to format the message.
 */
void PI_INFO(bool no_scroll, const std::string& group_id, const InfoLevel info_level, const std::string format, ...);

/*!
 * @brief Logs a warning message.
 * @param format The format string for the message (supports printf-style formatting).
 * @param ... Additional arguments to format the message.
 */
void PI_WARN(const std::string format, ...);

/*!
 * @brief Logs an error message.
 * @param format The format string for the message (supports printf-style formatting).
 * @param ... Additional arguments to format the message.
 */
void PI_ERROR(const std::string format, ...);

/*!
 * @brief Manages log message filtering and output control.
 */
class Info {
   public:
    /*!
     * @brief Adds multiple groups to the output filter.
     * @param groups A string containing group names separated by the specified delimiter.
     * @param delimiter The character used to separate group names.
     */
    void add_groups(const std::string& groups, char delimiter);

    /*!
     * @brief Adds a single group to the output filter.
     * @param group_id The group identifier to add.
     */
    void add_group(const std::string& group_id) { group_set_.insert(group_id); }

    /*!
     * @brief Removes a group from the output filter.
     * @param group_id The group identifier to remove.
     */
    void remove_group(const std::string& group_id) { group_set_.erase(group_id); }

    /*!
     * @brief Sets the current information level threshold for log output.
     * @param info_level The maximum information level to display.
     */
    void set_info_level(InfoLevel info_level) { info_level_ = info_level; }

    /*!
     * @brief Determines whether a log message should be displayed based on filtering criteria.
     * @param group_id The group identifier for the log message.
     * @param info_level The information level of the log message.
     * @return True if the message should be displayed, false otherwise.
     */
    bool is_in_output_group(const std::string& group_id, InfoLevel info_level) {
        bool is_in = false;

        if (info_level == InfoLevel::ESSENTIAL_0 || info_level == InfoLevel::UI) {
            // Always output ESSENTIAL_0 and UI level messages regardless of group or level settings
            is_in = true;
        } else if ((int)info_level <= (int)info_level_) {
            if (group_set_.empty())
                is_in = true;  // If no groups are specified, display messages from all groups
            else
                is_in = group_set_.find(group_id) != group_set_.end();
        }

        return is_in;
    }

    Info();
    /*!
     * @brief Destructor.
     */
    ~Info();

   private:
    std::unordered_set<std::string> group_set_;  ///< Set of group names whose messages should be displayed.
    InfoLevel info_level_ = InfoLevel::FREQUENT_3;  ///< Current maximum information level threshold.
};

/*!
 * @brief Global instance of the Info class for managing log output.
 */
extern Info g_info_manager;

/*!
 * @brief Temporary field-debug helper for tracking joystick LEFT/RIGHT detection,
 * joystick-leader lifecycle and bus-exclusivity races reported
 * from field deployments.
 *
 * Routes through PI_INFO at ESSENTIAL_0 level (so the message is always emitted
 * regardless of the runner's ``--info_level 0`` filter) under the "FieldDbg"
 * group, and carries a "[FIELD_DBG]" line prefix so the resulting tee logs can
 * be filtered with a single grep. To bulk-remove later:
 *   * grep ``PI_DBG_FIELD`` to find every call site and delete the line.
 *
 * The string-literal concatenation (``"[FIELD_DBG] " fmt``) requires ``fmt`` to
 * be a string literal at the call site, which is consistent with every other
 * PI_* call in this codebase.
 */
#define PI_DBG_FIELD(fmt, ...) \
    PI_INFO("FieldDbg", InfoLevel::ESSENTIAL_0, "[FIELD_DBG] " fmt, ##__VA_ARGS__)
