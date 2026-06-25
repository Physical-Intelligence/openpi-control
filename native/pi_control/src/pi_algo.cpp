/*!
 * @file pi_algo.cpp
 * @brief Implementation of the Algo base class and factory method for robot control algorithms.
 */

#include "pi_algo.hpp"

#include "pi_device.hpp"
#include "pi_device_config.hpp"
#include "pi_info.hpp"
#include "pi_joint.hpp"

#ifdef ENABLE_ALGO_PINO
#include "pi_algo_pino.hpp"
#endif  // ENABLE_ALGO_PINO

///< @todo These constants are from legacy implementations. Need to find better values and a better way to manage them.
#define ALGO_RESPONSE_TIME_CONSTANT 6
#define ALGO_RESPONSE_DELAY 0.025

Algo::Algo(Device* p_device, const CommandLineArgs& cla) {
    (void)cla;
    p_device_ = p_device;
}

Algo::~Algo() {}

ReturnCode Algo::init(const DeviceConfig* p_config_model, const DeviceConfig* p_config_individual,
                      const CommandLineArgs& cla) {
    control_frequency_ = cla.control_frequency;

    if (p_config_model == nullptr || p_config_individual == nullptr) {
        PI_ERROR("Configuration pointer is null");
        return ReturnCode::NOT_INITIALIZED;
    }

    if (cla.urdf_path.empty()) {
        PI_ERROR("Explicit URDF path is required");
        return ReturnCode::INVALID_PARAM;
    }
    urdf_path_ = cla.urdf_path;

    PI_INFO("Algo", InfoLevel::ESSENTIAL_0, "URDF path: %s", urdf_path_.c_str());

    if (!p_config_individual->values_.contains(p_config_individual->fn_base_rpy)) {
        PI_ERROR("Base rotation vector (base_rpy) not found in individual configuration file");
        return ReturnCode::INVALID_PARAM;
    }

    int rpy_size = (int)p_config_individual->values_[p_config_individual->fn_base_rpy].size();
    if (rpy_size != 3) {
        PI_ERROR("Base rotation vector must have exactly 3 elements (roll, pitch, yaw in radians), but found %d",
                 rpy_size);
        return ReturnCode::INVALID_PARAM;
    }

    for (float value : p_config_individual->values_[p_config_individual->fn_base_rpy]) {
        base_rpy_.push_back(value);
    }
    PI_INFO("Algo", InfoLevel::ESSENTIAL_0, "Base rotation: roll=%f, pitch=%f, yaw=%f", base_rpy_[0], base_rpy_[1],
            base_rpy_[2]);

    return ReturnCode::SUCCESS;
}

Algo* Algo::new_algo(Device* p_device, const DeviceConfig* p_config_model, const CommandLineArgs& cla) {
    if (p_config_model == nullptr) {
        PI_ERROR("Configuration pointer is null");
        return nullptr;
    }

    std::string algo_type;

    ReturnCode return_code =
        p_config_model->get_field_value(p_config_model->values_, p_config_model->fn_algo_type, algo_type);
    if (return_code != ReturnCode::SUCCESS) {
        PI_WARN("Algorithm type not defined in configuration file");
        return nullptr;
    }

    if (algo_type != p_config_model->val_algo_type_algo && cla.algo_type != OPT_ALGO_TYPE_UNDEFINED) {
        algo_type = cla.algo_type;
        PI_INFO("Algo", InfoLevel::ESSENTIAL_0, "Algorithm type set from command line: %s", algo_type.c_str());
    } else {
        PI_INFO("Algo", InfoLevel::ESSENTIAL_0, "Algorithm type set from configuration file: %s", algo_type.c_str());
    }

    Algo* p_algo = nullptr;
    if (algo_type == p_config_model->val_algo_type_algo) {
        p_algo = new Algo(p_device, cla);
        PI_INFO("Algo", InfoLevel::HELPFUL_1, "Created base Algo class instance");
#ifdef ENABLE_ALGO_PINO
    } else if (algo_type == p_config_model->val_algo_type_pinocchio) {
        p_algo = new AlgoPino(p_device, cla);
        PI_INFO("Algo", InfoLevel::HELPFUL_1, "Created AlgoPino class instance (Pinocchio-based)");
#endif  // ENABLE_ALGO_PINO
    } else {
        PI_ERROR("Unsupported algorithm type: %s", algo_type.c_str());
        return nullptr;
    }

    return p_algo;
}

float Algo::ramp_pos(float goal, float current, float ramp_rate) {
    float ramped_pos = goal;
    float delta = goal - current;

    if (delta > ramp_rate) {
        ramped_pos = current + ramp_rate;
    } else if (delta < -ramp_rate) {
        ramped_pos = current - ramp_rate;
    }

    return ramped_pos;
}

float Algo::get_next_target_pos(Joint* p_joint, float previous_target_pos, float target_pos, float safe_mode_derating) {
    if (control_frequency_ == 0) {
        PI_ERROR("Control frequency not set correctly");
        return target_pos;
    }

    float acceleration = p_joint->accel_max_ * safe_mode_derating;
    float max_velocity = p_joint->vel_max_ * safe_mode_derating;

    float acceleration_distance = (max_velocity * max_velocity) / (2 * acceleration);
    float deceleration_distance = acceleration_distance;

    float update_interval =
        1.0f / control_frequency_ + p_joint->get_response_delay();  ///< @todo Option 1: add response time delay
    // float update_interval = 1.0f / control_frequency_ * ALGO_RESPONSE_TIME_CONSTANT;  ///< @todo Option 2: multiply
    // by response time constant

    float distance = target_pos - previous_target_pos;
    float distance_abs = fabs(distance);

    float velocity;

    if (distance_abs < acceleration_distance) {
        velocity = sqrt(2 * acceleration * distance_abs);
    } else if (distance_abs > (acceleration_distance + deceleration_distance)) {
        velocity = max_velocity;
    } else {
        velocity = sqrt(max_velocity * max_velocity - 2 * acceleration * (distance_abs - acceleration_distance));
    }

    velocity *= (distance >= 0 ? 1 : -1);

    float next_position = previous_target_pos + velocity * update_interval;

    PI_INFO("Algo", InfoLevel::FREQUENT_3, "get_next_target_pos: previous=%.3f, max_vel=%.3f, next=%.3f, target=%.3f",
            previous_target_pos, max_velocity, next_position, target_pos);

    return next_position;
}

ReturnCode Algo::check_stability_control(std::vector<Joint*>& joints, const prof_time_t& step_start_time) {
    bool is_stable = true;

    for (Joint* p_joint : joints) {
        bool is_joint_stable = p_joint->update_stability(step_start_time);
        if (!is_joint_stable) {
            is_stable = false;
        }
    }

    if (is_stable) {
        set_control_mode(joints, SpringMode::POSITION);
    } else {
        set_control_mode(joints, SpringMode::SPRING);
    }

    return ReturnCode::SUCCESS;
}

ReturnCode Algo::set_control_mode(std::vector<Joint*>& joints, SpringMode new_mode) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    if (new_mode == SpringMode::PASSIVE) {
        for (Joint* p_joint : joints) {
            return_code = p_joint->apply_torque(0);
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to set joint %d to passive mode", p_joint->id_);
                return return_code;
            }
        }
        PI_INFO("Algo", InfoLevel::FREQUENT_3, "Switched to passive mode");
    } else if (new_mode == SpringMode::SPRING) {
        for (Joint* p_joint : joints) {
            return_code = p_joint->change_control_mode_for_spring();
            if (return_code != ReturnCode::SUCCESS) {
                PI_ERROR("Failed to set joint %d to spring mode", p_joint->id_);
                return return_code;
            }
        }
        PI_INFO("Algo", InfoLevel::FREQUENT_3, "Switched to spring mode");
    } else if (new_mode == SpringMode::POSITION) {
        for (Joint* p_joint : joints) {
            p_joint->spring_enabled_ = false;
        }
        PI_INFO("Algo", InfoLevel::FREQUENT_3, "Switched to position mode (spring disabled, position not held)");
    } else {
        PI_ERROR("Unsupported control mode: %d", (int)new_mode);
        return ReturnCode::INVALID_PARAM;
    }
    return return_code;
}
