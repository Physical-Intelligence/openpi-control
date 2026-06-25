/*!
 * @file pi_info.cpp
 * @brief Implementation of logging and information output system for PI Control.
 */

#include <cstdarg>
#include <iostream>
#include <sstream>

#include "pi_info.hpp"


#define MSG_STRING_BUF_SIZE 1024

#define COLOR_RESET "\033[0m"    ///< Reset terminal color to default
#define COLOR_RED "\033[31m"     ///< Red color for error messages
#define COLOR_YELLOW "\033[33m"  ///< Yellow color for warning messages
#define COLOR_GREEN "\033[32m"   ///< Green color for info messages
#define COLOR_BLUE "\033[34m"    ///< Blue color (reserved for future use)

#define CLEAR_SCREEN "\033[2J\033[H"

Info g_info_manager;

void output_message(bool no_scroll, const std::string& group_id, InfoLevel info_level, const char* buffer) {
    if (info_level == InfoLevel::UI) {
        if (no_scroll) {
            std::cout << CLEAR_SCREEN;
            std::cout << buffer << std::endl;
        } else {
            std::cout << buffer << std::endl;
        }
    } else {
        std::string colored_msg = std::string(COLOR_GREEN) + "[INFO] [" + group_id + ":info_level_" +
                                  std::to_string(static_cast<int>(info_level)) + "] " + buffer + COLOR_RESET;
        std::cout << colored_msg << std::endl;
    }
}

void PI_INFO(bool no_scroll, const std::string& group_id, InfoLevel info_level, const std::string msg, ...) {
    if (g_info_manager.is_in_output_group(group_id, info_level)) {
        char buffer[MSG_STRING_BUF_SIZE];
        va_list args;
        va_start(args, msg);
        vsnprintf(buffer, sizeof(buffer), msg.c_str(), args);
        va_end(args);

        output_message(no_scroll, group_id, info_level, buffer);
    }
}

void PI_INFO(const std::string& group_id, InfoLevel info_level, const std::string msg, ...) {
    if (g_info_manager.is_in_output_group(group_id, info_level)) {
        char buffer[MSG_STRING_BUF_SIZE];
        va_list args;
        va_start(args, msg);
        vsnprintf(buffer, sizeof(buffer), msg.c_str(), args);
        va_end(args);

        output_message(false, group_id, info_level, buffer);
    }
}

void PI_WARN(const std::string msg, ...) {
    char buffer[MSG_STRING_BUF_SIZE];
    va_list args;
    va_start(args, msg);
    vsnprintf(buffer, sizeof(buffer), msg.c_str(), args);
    va_end(args);

    std::string colored_msg = std::string(COLOR_YELLOW) + "[WARNING] " + buffer + COLOR_RESET;
    std::cout << colored_msg << std::endl;
}

void PI_ERROR(const std::string msg, ...) {
    char buffer[MSG_STRING_BUF_SIZE];
    va_list args;
    va_start(args, msg);
    vsnprintf(buffer, sizeof(buffer), msg.c_str(), args);
    va_end(args);

    std::string colored_msg = std::string(COLOR_RED) + "[ERROR] " + buffer + COLOR_RESET;
    std::cerr << colored_msg << std::endl;

    ///< @note Exception throwing is currently disabled to avoid abrupt program termination
    ///< @todo Implement proper try-catch blocks in acceptable/retriable error paths
    // PiException::pi_error(colored_msg);
}

Info::Info() {}

Info::~Info() {}

void Info::add_groups(const std::string& groups, char delimiter) {
    if (groups == "") {
        return;
    }

    std::stringstream ss(groups);
    std::string group_name;

    while (std::getline(ss, group_name, delimiter)) {
        group_name.erase(0, group_name.find_first_not_of(' '));

        group_name.erase(group_name.find_last_not_of(' ') + 1);

        add_group(group_name);
    }
}
