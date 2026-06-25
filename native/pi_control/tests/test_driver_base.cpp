#include "pi_driver.hpp"

Driver::Driver(Device* p_device, const CommandLineArgs& cla) {
    control_port_name_ = cla.control_port_name;
    p_device_ = p_device;
}

std::shared_ptr<Driver> Driver::new_driver(
    Device* p_device, const DeviceConfig* p_config, const CommandLineArgs& cla) {
    (void)p_device;
    (void)p_config;
    (void)cla;
    return nullptr;
}

ReturnCode Driver::read_hardware_values(Servo* p_servo) {
    (void)p_servo;
    return ReturnCode::SUCCESS;
}

std::map<int, int> Driver::map_id_to_data_index_;
std::map<int, Servo*> Driver::map_id_to_servo_;

int Driver::find_data_index(int servo_id) {
    std::lock_guard<std::mutex> lock(registry_mutex_);
    auto it = map_id_to_data_index_.find(servo_id);
    return (it == map_id_to_data_index_.end()) ? -1 : it->second;
}

Servo* Driver::find_servo(int servo_id) {
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
