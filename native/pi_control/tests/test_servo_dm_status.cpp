#include <gtest/gtest.h>

#include "pi_servo_dm_status.hpp"

TEST(DmServoStatus, DecodesMotorCoilOvertemperature) {
    const DmServoStatusInfo& status = dm_servo_status_info(0xC);

    EXPECT_TRUE(status.is_fault);
    EXPECT_TRUE(status.is_thermal_fault);
    EXPECT_FALSE(status.is_resettable_on_enable);
    EXPECT_STREQ(status.description, "motor coil overtemperature");
    EXPECT_NE(std::string(status.action).find("allow the motor to cool"), std::string::npos);
}

TEST(DmServoStatus, DistinguishesNormalStatesFromFaults) {
    EXPECT_FALSE(dm_servo_status_info(0x0).is_fault);
    EXPECT_FALSE(dm_servo_status_info(0x1).is_fault);
    EXPECT_TRUE(dm_servo_status_info(0xB).is_thermal_fault);
    EXPECT_TRUE(dm_servo_status_info(0xC).is_thermal_fault);
    EXPECT_FALSE(dm_servo_status_info(0xA).is_thermal_fault);
    EXPECT_FALSE(dm_servo_status_info(0xE).is_thermal_fault);
    EXPECT_TRUE(dm_servo_status_info(0x8).is_fault);
    EXPECT_TRUE(dm_servo_status_info(0xE).is_fault);
    EXPECT_TRUE(dm_servo_status_info(0xF).is_fault);
    EXPECT_TRUE(dm_servo_status_info(0xD).is_resettable_on_enable);
    EXPECT_FALSE(dm_servo_status_info(0xC).is_resettable_on_enable);
    EXPECT_FALSE(dm_servo_status_info(0xE).is_resettable_on_enable);
}
