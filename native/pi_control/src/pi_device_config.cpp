/*!
 * @file pi_device_config.cpp
 * @brief Implementation of the DeviceConfig class for device configuration file management.
 */

#include <fstream>

#include "pi_info.hpp"
#include "pi_device_config.hpp"

DeviceConfig::DeviceConfig() {}

DeviceConfig::~DeviceConfig() {}

ReturnCode DeviceConfig::init_config_model(const CommandLineArgs& cla) {
    if (cla.device_config_type == DeviceConfigType::ARM && !cla.arm_model_config.empty()) {
        return init_config(cla.arm_model_config);
    }
    if (cla.device_config_type == DeviceConfigType::EFFECTOR && !cla.effector_model_config.empty()) {
        return init_config(cla.effector_model_config);
    }
    PI_ERROR("Explicit model configuration path is required");
    return ReturnCode::INVALID_PARAM;
}

ReturnCode DeviceConfig::init_config_individual(const CommandLineArgs& cla) {
    if (cla.device_config_type == DeviceConfigType::ARM && !cla.arm_instance_config.empty()) {
        return init_config(cla.arm_instance_config);
    }
    if (cla.device_config_type == DeviceConfigType::EFFECTOR && !cla.effector_instance_config.empty()) {
        ReturnCode return_code = init_config(cla.effector_instance_config);
        if (return_code == ReturnCode::SUCCESS) {
            cascade_effector_open_at_min_to_servos();
        }
        return return_code;
    }
    PI_ERROR("Explicit individual configuration path is required");
    return ReturnCode::INVALID_PARAM;
}

void DeviceConfig::cascade_effector_open_at_min_to_servos() {
    if (!values_.contains(fn_effector_open_at_min)) return;
    const auto& top_value = values_[fn_effector_open_at_min];
    if (!top_value.is_boolean()) return;
    if (!values_.contains(fn_joints) || !values_[fn_joints].is_array()) return;

    const bool top_open_at_min = top_value.get<bool>();
    int injected_count = 0;
    for (auto& joint : values_[fn_joints]) {
        if (!joint.contains(fn_servos) || !joint[fn_servos].is_array()) continue;
        for (auto& servo : joint[fn_servos]) {
            if (servo.contains(fn_effector_open_at_min)) continue;
            servo[fn_effector_open_at_min] = top_open_at_min;
            ++injected_count;
        }
    }
    PI_INFO("DeviceConfig", InfoLevel::HELPFUL_1,
            "Cascaded top-level '%s'=%d into %d servo dict(s) for downstream readers",
            fn_effector_open_at_min.c_str(), top_open_at_min ? 1 : 0, injected_count);
}

ReturnCode DeviceConfig::init_config(const std::string& file_path) {
    // ESSENTIAL_0: seeing exactly which model/instance config file was loaded is a
    // required pre-flight check before driving real hardware (limits come from here).
    PI_INFO("DeviceConfig", InfoLevel::ESSENTIAL_0, "Loading configuration file: %s", file_path.c_str());

    std::ifstream config_file(file_path);
    if (!config_file.is_open()) {
        PI_ERROR("Configuration file not found: %s", file_path.c_str());
        return ReturnCode::NOT_FOUND;
    }

    try {
        config_file >> values_;
    } catch (const nlohmann::json::exception& e) {
        PI_ERROR("Configuration file '%s' is not valid JSON: %s", file_path.c_str(), e.what());
        return ReturnCode::INVALID_PARAM;
    }

    std::string config_version;
    ReturnCode return_code = get_field_value(values_, fn_config_version, config_version);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Configuration version field '%s' is not defined in configuration file", fn_config_version.c_str());
        return ReturnCode::NOT_FOUND;
    }

    if (config_version != CURRENT_CONFIG_VERSION) {
        PI_ERROR("Configuration version '%s' is not supported (found version '%s', required version '%s')",
                 fn_config_version.c_str(), config_version.c_str(), CURRENT_CONFIG_VERSION);
        return ReturnCode::NOT_SUPPORTED;
    }

    return ReturnCode::SUCCESS;
}
