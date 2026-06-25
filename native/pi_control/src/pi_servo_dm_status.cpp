/*!
 * @file pi_servo_dm_status.cpp
 * @brief Human-readable DM servo status and fault-code descriptions.
 */

#include "pi_servo_dm_status.hpp"

namespace {

constexpr DmServoStatusInfo kDisabled{
    "disabled", "No action required.", false, false, false};
constexpr DmServoStatusInfo kEnabled{
    "enabled", "No action required.", false, false, false};
constexpr DmServoStatusInfo kReserved{
    "reserved/non-normal status",
    "Retry once; if the status persists, consult the motor manual.", false, false, false};
constexpr DmServoStatusInfo kOvervoltage{
    "overvoltage",
    "Stop operation and verify the supply voltage before retrying.", true, false, false};
constexpr DmServoStatusInfo kUndervoltage{
    "undervoltage",
    "Stop operation and verify the supply voltage and power wiring before retrying.", true, false, false};
constexpr DmServoStatusInfo kOvercurrent{
    "overcurrent",
    "Stop operation and inspect power wiring and mechanical load before retrying.", true, false, false};
constexpr DmServoStatusInfo kMosOvertemperature{
    "MOSFET overtemperature",
    "Stop commands and allow the drive electronics to cool before retrying.", true, true, false};
constexpr DmServoStatusInfo kMotorCoilOvertemperature{
    "motor coil overtemperature",
    "Stop commands and allow the motor to cool; inspect for mechanical binding or sustained load before retrying.",
    true, true, false};
constexpr DmServoStatusInfo kCommunicationLoss{
    "communication loss",
    "Stop operation and check CAN wiring, termination, and bus health before retrying.", true, false, true};
constexpr DmServoStatusInfo kOverload{
    "overload",
    "Stop operation and inspect for mechanical binding or sustained load before retrying.", true, false, false};
constexpr DmServoStatusInfo kUnknownFault{
    "unknown DM hardware fault",
    "Stop operation and consult the motor manual before retrying.", true, false, false};

}  // namespace

const DmServoStatusInfo& dm_servo_status_info(uint8_t status_code) {
    switch (status_code & 0x0F) {
        case 0x0:
            return kDisabled;
        case 0x1:
            return kEnabled;
        case 0x8:
            return kOvervoltage;
        case 0x9:
            return kUndervoltage;
        case 0xA:
            return kOvercurrent;
        case 0xB:
            return kMosOvertemperature;
        case 0xC:
            return kMotorCoilOvertemperature;
        case 0xD:
            return kCommunicationLoss;
        case 0xE:
            return kOverload;
        case 0xF:
            return kUnknownFault;
        default:
            return kReserved;
    }
}
