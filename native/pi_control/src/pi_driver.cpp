/*!
 * @file pi_driver.cpp
 * @brief Implementation of the Driver base class and factory method for robot servo motor drivers.
 */

#include "pi_driver.hpp"

#include "pi_device_config.hpp"
#include "pi_driver_arx.hpp"
#include "pi_driver_arx_encoder.hpp"
#include "pi_servo.hpp"

Driver::Driver(Device* p_device, const CommandLineArgs& cla) {
    control_port_name_ = cla.control_port_name;
    p_device_ = p_device;
}

std::shared_ptr<Driver> Driver::new_driver(Device* p_device, const DeviceConfig* p_config, const CommandLineArgs& cla) {
    if (p_config == nullptr) {
        PI_ERROR("Configuration pointer is null in new_driver()");
        return nullptr;
    }

    std::string driver_type;
    ReturnCode return_code = p_config->get_field_value(p_config->values_, p_config->fn_driver_type, driver_type);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Driver type is not defined in configuration file");
        return nullptr;
    }

    std::shared_ptr<Driver> p_driver = nullptr;
    if (driver_type == p_config->val_driver_type_can) {
        auto p_driver_arx = std::make_shared<DriverArx>(p_device, cla);
        p_driver = p_driver_arx;
        PI_INFO("Driver", InfoLevel::HELPFUL_1, "Created CAN 2.0 driver (DriverArx)");

    } else if (driver_type == p_config->val_driver_type_can_encoder) {
        auto p_driver_encoder = std::make_shared<DriverArxEncoder>(p_device, cla);
        p_driver = p_driver_encoder;
        PI_INFO("Driver", InfoLevel::HELPFUL_1, "Created CAN read-only encoder driver (DriverArxEncoder)");

    } else {
        PI_ERROR("Unsupported driver type: '%s' (supported types: CAN, CAN_ENCODER)", driver_type.c_str());
        return nullptr;
    }

    return p_driver;
}

ReturnCode Driver::read_hardware_values(Servo* p_servo) {
    ///< @note Default implementation for interface compatibility (can be overridden by derived classes)
    (void)p_servo;
    return ReturnCode::SUCCESS;
}

std::map<int, int> Driver::map_id_to_data_index_;

// Must use find() rather than operator[] -- the latter auto-inserts unknown
// keys with value 0, which silently turns "unknown CAN ID" into "valid slot 0"
// in handle_received_message(), allowing frames from non-servo bus participants
// (e.g. the A6 power-control board's PubSub on CAN ID 0x80) to corrupt J0's
// position/velocity feedback. Returning -1 here makes the existing
// `if (data_index == -1) return;` guard actually reject those frames.
int Driver::find_data_index(int servo_id) {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = map_id_to_data_index_.find(servo_id);
    return (it == map_id_to_data_index_.end()) ? -1 : it->second;
}

std::map<int, class Servo*> Driver::map_id_to_servo_;

class Servo* Driver::find_servo(int servo_id) {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = map_id_to_servo_.find(servo_id);
    return (it == map_id_to_servo_.end()) ? nullptr : it->second;
}

void Driver::unregister_servo(Servo* p_servo) {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    for (auto it = map_id_to_servo_.begin(); it != map_id_to_servo_.end();) {
        if (it->second != p_servo) {
            ++it;
            continue;
        }

        map_id_to_data_index_.erase(it->first);
        it = map_id_to_servo_.erase(it);
    }
}

Driver::RegisteredServo Driver::lock_registered_servo(int servo_id) {
    std::unique_lock<std::mutex> lock(registry_mutex_);
    auto it = map_id_to_servo_.find(servo_id);
    Servo* p_servo = (it == map_id_to_servo_.end()) ? nullptr : it->second;
    return RegisteredServo(std::move(lock), p_servo);
}
