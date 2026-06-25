/*!
 * @file pi_servo_dm_status.hpp
 * @brief Human-readable DM servo status and fault-code descriptions.
 */

#pragma once

#include <cstdint>

struct DmServoStatusInfo {
    const char* description;
    const char* action;
    bool is_fault;
    bool is_thermal_fault;
    bool is_resettable_on_enable;
};

/*!
 * @brief Decodes the high status nibble returned by a DM servo.
 * @param status_code Four-bit DM status code.
 * @return Description, operator action, and fault/recovery classification.
 */
const DmServoStatusInfo& dm_servo_status_info(uint8_t status_code);
