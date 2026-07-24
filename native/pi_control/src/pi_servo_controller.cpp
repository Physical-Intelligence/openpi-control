/*!
 * @file pi_servo_controller.cpp
 * @brief Implementation of the ServoController per-joint view onto a whole-arm controller.
 */

#include "pi_servo_controller.hpp"

#include "pi_joint.hpp"

const ServoControllerParam g_servo_controller_param(DEFAULT_TOLERABLE_POS_DIFFERENCE_RAD, MAX_POS_DIFFERENCE_RAD,
                                                    DEFAULT_VELOCITY_THRESHOLD_RAD_SEC);

ServoController::ServoController(Device* p_device, Joint* p_joint, Driver* p_driver)
    : Servo(p_device, p_joint, p_driver) {
    p_driver_controller_ = dynamic_cast<DriverController*>(p_driver);
}

ServoController::~ServoController() {}

ReturnCode ServoController::init_config_model(const json& servo_config, const DeviceConfig* p_config) {
    p_servo_param_ = &g_servo_controller_param;

    ReturnCode return_code = Servo::init_config_model(servo_config, p_config);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (servo_model_ == p_config->val_servo_model_trossen_wxai) {
        type_ = ServoType::CONTROLLER_JOINT;
    } else {
        PI_ERROR("Unsupported servo model '%s' (servo ID %d)", servo_model_.c_str(), id_);
        return ReturnCode::NOT_SUPPORTED;
    }

    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Servo ID %d requires a DriverController-based driver (check driver_type in the model config)", id_);
        return ReturnCode::INVALID_PARAM;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode ServoController::start_hardware() {
    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    if (p_device_ == nullptr) {
        PI_ERROR("Device pointer is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    if (p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: device is read only, skipping start hardware", id_);
        return ReturnCode::SUCCESS;
    }

    // POSITION mode makes the controller hold the current pose; the mode is
    // applied together with the hold-at-current seed in the first command flush,
    // so there is no window where a stale target could be executed.
    ReturnCode return_code =
        p_driver_controller_->set_joint_command_mode(data_index_, DriverController::JointCommandMode::POSITION);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to request POSITION mode (servo ID %d)", id_);
        return return_code;
    }

    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo %d started hardware (controller joint slot %d)", id_,
            data_index_);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoController::park_safely() {
    if (parked_ == true) {
        return ReturnCode::SUCCESS;
    }
    if (p_device_ != nullptr && p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d: device is read only, skipping park safely", id_);
        return ReturnCode::SUCCESS;
    }
    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Controller driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    // Whole-arm controllers park all joints at once; make_safe() is idempotent
    // so the first parked joint idles the arm and the rest are no-ops.
    ReturnCode return_code = p_driver_controller_->make_safe();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to park controller joint (servo ID %d)", id_);
        return return_code;
    }

    parked_ = true;
    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d parked safely (controller idled)", id_);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoController::verify_position_fresh() {
    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Controller driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    if (!p_driver_controller_->has_valid_state()) {
        PI_ERROR("Servo ID %d: no whole-arm state has been received from the controller yet", id_);
        return ReturnCode::NO_RESPONSE;
    }
    return ReturnCode::SUCCESS;
}

ReturnCode ServoController::move(float target_pos) {
    return move(target_pos, 0, 0);
}

ReturnCode ServoController::move(float target_pos, float target_vel, float target_tor) {
    // The controller runs its own servo loops; per-command torque bounds are
    // not part of the whole-arm command set (joint limits bound the effort).
    (void)target_tor;
    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    float clipped_pos = clipping(target_pos, pos_min_rel_, pos_max_rel_);
    float target_pos_absolute = get_pos_rad_absolute(clipped_pos);
    float velocity_ff = target_vel * dir_invert_;

    ReturnCode return_code = p_driver_controller_->queue_position(data_index_, target_pos_absolute, velocity_ff);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to queue position command (servo ID %d)", id_);
        return return_code;
    }

    PI_INFO("Servo", InfoLevel::FREQUENT_3, "Joint ID %d, Servo ID %d: move target_pos=%.3f, absolute=%.3f",
            p_joint_->id_, id_, target_pos, target_pos_absolute);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoController::apply_torque(float torque) {
    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    ReturnCode return_code = p_driver_controller_->queue_external_effort(data_index_, torque * dir_invert_);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to queue external effort command (servo ID %d)", id_);
        return return_code;
    }

    PI_INFO("Servo", InfoLevel::FREQUENT_3, "Joint ID %d, Servo ID %d: external effort %.3f", p_joint_->id_, id_,
            torque);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoController::apply_torque_with_damping(float torque) {
    // Damping runs inside the vendor controller's servo loops.
    return apply_torque(torque);
}

ReturnCode ServoController::change_control_mode_for_leader() {
    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    if (p_device_ != nullptr && p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                "Servo ID %d: device is read only, skipping change_control_mode_for_leader", id_);
        return ReturnCode::SUCCESS;
    }

    PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
            "Servo ID %d: leader mode, requesting EXTERNAL_EFFORT (controller gravity compensation stays active)",
            id_);
    return p_driver_controller_->set_joint_command_mode(data_index_,
                                                        DriverController::JointCommandMode::EXTERNAL_EFFORT);
}

ReturnCode ServoController::change_control_mode_for_follower() {
    if (p_driver_controller_ == nullptr) {
        PI_ERROR("Driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    if (p_device_ != nullptr && p_device_->is_read_only() == true) {
        PI_INFO("Servo", InfoLevel::ESSENTIAL_0,
                "Servo ID %d: device is read only, skipping change_control_mode_for_follower", id_);
        return ReturnCode::SUCCESS;
    }

    return p_driver_controller_->set_joint_command_mode(data_index_, DriverController::JointCommandMode::POSITION);
}

void ServoController::update_from_controller_state(float position, float velocity, float effort, float temperature) {
    curr_pos_abs_ = position;
    curr_vel_ = velocity;
    curr_tor_ = effort;
    temperature_ = temperature;
}
