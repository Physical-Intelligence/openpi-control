/*!
 * @file pi_servo_dm.cpp
 * @brief Implementation of the ServoDm class for managing DM and ENCOS type servos controlled via CAN interface.
 */

#include <unistd.h>
#include <iomanip>
#include <sstream>

#include "can/math_ops.h"
#include "pi_joint.hpp"
#include "pi_servo_dm.hpp"
#include "pi_servo_dm_status.hpp"

#define DM_CMD_WRITE 0x55  ///< DM command code for writing register values

const ServoDmParam g_servo_dm_param_4340(0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -10.0f, 10.0f, -28.0f, 28.0f,
                                         DEFAULT_TOLERABLE_POS_DIFFERENCE_RAD, MAX_POS_DIFFERENCE_RAD,
                                         DEFAULT_VELOCITY_THRESHOLD_RAD_SEC);

const ServoDmParam g_servo_dm_param_4310(0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -30.0f, 30.0f, -10.0f, 10.0f,
                                         DEFAULT_TOLERABLE_POS_DIFFERENCE_RAD, MAX_POS_DIFFERENCE_RAD,
                                         DEFAULT_VELOCITY_THRESHOLD_RAD_SEC);

const ServoDmParam g_servo_dm_param_encos_A4310(0.0f, 500.0f, 0.0f, 5.0f, -12.5f, 12.5f, -18.0f, 18.0f, -30.0f, 30.0f,
                                                DEFAULT_TOLERABLE_POS_DIFFERENCE_RAD, MAX_POS_DIFFERENCE_RAD,
                                                DEFAULT_VELOCITY_THRESHOLD_RAD_SEC);

#define MAX_CNT_MOTOR_NO_RESPONSE_INITIAL \
    500  ///< Threshold count for detecting motor no-response during initial period
#define MAX_CNT_MOTOR_NO_RESPONSE_NORMAL \
    10000  ///< Threshold count for detecting motor no-response during normal operation

ServoDm::ServoDm(Device* p_device, Joint* p_joint, Driver* p_driver)
    : Servo(p_device, p_joint, p_driver), checker_motor_no_response_(MAX_CNT_MOTOR_NO_RESPONSE_INITIAL) {
    p_driver_can_ = dynamic_cast<DriverArx*>(p_driver);
}

ServoDm::~ServoDm() {
    // Retire the derived object before its fields begin destruction. Registry
    // users hold the same lock through every access to this pointer.
    Driver::unregister_servo(this);
}

ReturnCode ServoDm::reject_if_thermal_fault_latched() const {
    return thermal_fault_latched_ ? ReturnCode::HARDWARE_FAULT : ReturnCode::SUCCESS;
}

ReturnCode ServoDm::latch_effector_thermal_fault(uint8_t status_code, const char* description, const char* action,
                                                 const char* trigger) {
    if (thermal_fault_latched_) {
        return ReturnCode::HARDWARE_FAULT;
    }

    thermal_fault_latched_ = true;
    // Mark parked before attempting I/O so teardown cannot retry or re-enable
    // a motor that has reported a thermal fault.
    parked_ = true;

    ReturnCode zero_rc = ReturnCode::NOT_INITIALIZED;
    ReturnCode disable_rc = ReturnCode::NOT_INITIALIZED;
    if (p_driver_can_ != nullptr) {
        zero_rc = p_driver_can_->send_command(this, 0, 0, 0, 0, 0);
        disable_rc = p_driver_can_->send_disable_once(id_, static_cast<int>(type_));
    }

    const float requested_target = p_joint_ != nullptr ? p_joint_->get_last_grip_requested_target_pos() : 0.0f;
    const float applied_target = p_joint_ != nullptr ? p_joint_->get_last_grip_applied_target_pos() : 0.0f;
    const bool limiter_active = p_joint_ != nullptr && p_joint_->is_grip_limiter_active();
    const float raw_pos_min = p_joint_ != nullptr ? p_joint_->get_pos_min_relative() : 0.0f;
    const float raw_pos_max = p_joint_ != nullptr ? p_joint_->get_pos_max_relative() : 0.0f;
    const float normalized_pos_min = p_joint_ != nullptr ? p_joint_->get_normalized_pos_min_relative() : 0.0f;
    const float normalized_pos_max = p_joint_ != nullptr ? p_joint_->get_normalized_pos_max_relative() : 0.0f;
    PI_ERROR("HARDWARE FAULT: DM servo id=%d reported status 0x%X (%s); thermal stop latched "
             "(trigger=%s, requested_target=%.3f rad, applied_target=%.3f rad, limiter_active=%d, "
             "measured_position=%.3f rad, effort=%.3f Nm, current=%.3f A, temperature=%.0f C, "
             "raw_range=[%.3f, %.3f] rad, normalized_range=[%.3f, %.3f] rad, "
             "zero_output_rc=%d, disable_rc=%d). Action: %s",
             id_, static_cast<unsigned>(status_code), description, trigger, requested_target, applied_target,
             static_cast<int>(limiter_active), get_pos_rad_relative(), curr_tor_, idc_current_, temperature_,
             raw_pos_min, raw_pos_max, normalized_pos_min, normalized_pos_max,
             static_cast<int>(zero_rc), static_cast<int>(disable_rc), action);
    last_reported_fault_code_ = status_code;
    return ReturnCode::HARDWARE_FAULT;
}

ReturnCode ServoDm::park_safely() {
    if (parked_ == true) {
        return ReturnCode::SUCCESS;
    }

    ReturnCode return_code;

    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    return_code = p_driver_can_->send_command(this, 0, pos_kd_, 0, 0, 0);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to send park command to servo ID %d", id_);
        return return_code;
    }
    usleep(100);

    return_code = p_driver_can_->enable(id_, static_cast<int>(type_), false);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to disable servo ID %d during safe parking", id_);
        return return_code;
    }
    usleep(100);

    parked_ = true;

    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Servo ID %d parked safely", id_);

    return return_code;
}

ReturnCode ServoDm::init_current_estimation(std::string& servo_model, const DeviceConfig* p_config) {
    if (servo_model == p_config->val_servo_model_dm_4340) {
        motor_params_current_estimation_.kt0_ = 0.1087;
        motor_params_current_estimation_.r0_ = 0.9292449;
        motor_params_current_estimation_.t0_ = 25.0;
        motor_params_current_estimation_.beta_ = -0.0015;
        motor_params_current_estimation_.alpha_ = 0.0039;
        motor_params_current_estimation_.c1_ = 0.0;
        motor_params_current_estimation_.c2_ = 0.0;
        motor_params_current_estimation_.eta_inv_ = 0.96;
        motor_params_current_estimation_.p_drv_ = 2.0;
        motor_params_current_estimation_.eta_g_ = 0.90;
        motor_params_current_estimation_.gear_ratio_ = 40.0;

    } else if (servo_model == p_config->val_servo_model_dm_4310) {
        motor_params_current_estimation_.kt0_ = 0.1032;
        motor_params_current_estimation_.r0_ = 0.8413959;
        motor_params_current_estimation_.t0_ = 25.0;
        motor_params_current_estimation_.beta_ = -0.0015;
        motor_params_current_estimation_.alpha_ = 0.0039;
        motor_params_current_estimation_.c1_ = 0.0;
        motor_params_current_estimation_.c2_ = 0.0;
        motor_params_current_estimation_.eta_inv_ = 0.96;
        motor_params_current_estimation_.p_drv_ = 2.0;
        motor_params_current_estimation_.eta_g_ = 0.90;
        motor_params_current_estimation_.gear_ratio_ = 10.0;

    }

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::init_config_model(const json& servo_config, const DeviceConfig* p_config) {
    if (p_driver_can_ == nullptr) {
        PI_ERROR("DM servo requires an ARX CAN driver");
        return ReturnCode::NOT_INITIALIZED;
    }

    ReturnCode return_code = Servo::init_config_model(servo_config, p_config);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (servo_model_ == p_config->val_servo_model_dm_4340) {
        type_ = ServoType::DM_4340;
        p_servo_param_ = &g_servo_dm_param_4340;
    } else if (servo_model_ == p_config->val_servo_model_dm_4310) {
        type_ = ServoType::DM_4310;
        p_servo_param_ = &g_servo_dm_param_4310;
    } else if (servo_model_ == p_config->val_servo_model_encos_A4310) {
        type_ = ServoType::ENCOS_A4310;
        p_servo_param_ = &g_servo_dm_param_encos_A4310;
    } else {
        PI_ERROR("Unsupported servo model '%s' (servo ID %d)", servo_model_.c_str(), id_);
        return ReturnCode::NOT_SUPPORTED;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::init_config_individual(const json& servo_config, const DeviceConfig* p_config) {
    ReturnCode return_code = Servo::init_config_individual(servo_config, p_config);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::start_hardware() {
    if (reject_if_thermal_fault_latched() != ReturnCode::SUCCESS) {
        return ReturnCode::HARDWARE_FAULT;
    }
    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    ReturnCode return_code = p_driver_can_->enable(
        id_, static_cast<int>(type_), true, get_device_type_belong_to() == DeviceType::EFFECTOR);
    if (return_code != ReturnCode::SUCCESS) {
        const int status_code = p_driver_can_->last_enable_fault_status();
        if (return_code == ReturnCode::HARDWARE_FAULT && status_code >= 0 &&
            get_device_type_belong_to() == DeviceType::EFFECTOR) {
            const DmServoStatusInfo& status = dm_servo_status_info(static_cast<uint8_t>(status_code));
            if (status.is_thermal_fault) {
                // DriverArx cached the fault response before returning. Refresh the
                // servo fields so the terminal fault message reports that snapshot.
                p_driver_can_->read_hardware_values(this);
                idc_current_ = current_estimation_.estimate_idc_calibrated(
                    motor_params_current_estimation_, curr_tor_, curr_vel_, temperature_,
                    DEFAULT_VDC, get_tor_max());
                return latch_effector_thermal_fault(static_cast<uint8_t>(status_code), status.description,
                                                    status.action, "firmware status during enable");
            }
        }
        PI_ERROR("Failed to enable servo ID %d (type=%d): return_code=%d", id_, static_cast<int>(type_),
                 static_cast<int>(return_code));
        return return_code;
    }

    PI_INFO("Servo", InfoLevel::DETAIL_2, "Enable command sent to servo ID %d", id_);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::verify_position_fresh() {
    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }
    // DM/ENCOS slot is considered fresh once the asynchronous CAN parser (or the enable
    // response path in DriverArx::enable()) has written into received_servo_data_. The
    // motor_id_ field is zero-initialised and motor IDs start at 1, so a non-zero value
    // proves at least one status frame was parsed.
    const int cached_id = p_driver_can_->get_received_motor_id(data_index_);
    if (cached_id == 0) {
        PI_ERROR("Servo ID %d: position cache is stale (data_index=%d, motor_id=0); "
                 "no status frame ever parsed",
                 id_, data_index_);
        return ReturnCode::FAIL;
    }
    return ReturnCode::SUCCESS;
}

#define SERVO_DM_DEFAULT_KP 10  ///< Default proportional gain for position control when configuration value is zero

float ServoDm::get_effective_pos_kp() const { return (pos_kp_ == 0) ? SERVO_DM_DEFAULT_KP : pos_kp_; }

ReturnCode ServoDm::move(float target_pos) {
    if (reject_if_thermal_fault_latched() != ReturnCode::SUCCESS) {
        return ReturnCode::HARDWARE_FAULT;
    }
    if (p_driver_can_ == nullptr) {
        PI_ERROR("CAN driver is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    float clipped_pos = clipping(target_pos, pos_min_rel_, pos_max_rel_);
    float pos_absolute = get_pos_rad_absolute(clipped_pos);

    float new_kp = 0.0f;
    if (p_device_->is_force_feedback_enabled()) {
        new_kp = get_adjusted_pos_kp();
    } else {
        new_kp = get_effective_pos_kp();
    }

    ReturnCode return_code = p_driver_can_->send_command(this, new_kp, get_adjusted_pos_kd(), pos_absolute, 0.0, 0.0);
    usleep(100);

    PI_INFO("Servo", InfoLevel::FREQUENT_3, "Servo ID %d: Move to position=%.3f rad (absolute), kp=%.3f, new_kp=%.3f",
            id_, pos_absolute, pos_kp_, new_kp);
    return return_code;
}

ReturnCode ServoDm::move(float target_pos, float target_vel, float target_tor) {
    if (reject_if_thermal_fault_latched() != ReturnCode::SUCCESS) {
        return ReturnCode::HARDWARE_FAULT;
    }
    if (p_driver_can_ == nullptr || p_joint_ == nullptr) {
        PI_ERROR("Driver or joint pointer is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    // This is unconditionally a position command: a target of exactly 0.0 rad
    // is as legitimate as any other (it is the YAM home pose). The old
    // `target_pos != 0.0` guard silently turned such frames into torque-only
    // commands -- callers that want a torque-only frame use apply_torque().
    float clipped_pos = clipping(target_pos, pos_min_rel_, pos_max_rel_);
    float pos_absolute = get_pos_rad_absolute(clipped_pos);
    float pos_kp = 0.0;

    if (p_device_->is_force_feedback_enabled()) {
        pos_kp = get_adjusted_pos_kp();
    } else {
        pos_kp = get_effective_pos_kp();
    }

    float clipped_tor = clipping(target_tor, p_joint_->torq_min_, p_joint_->torq_max_);

    ReturnCode return_code =
        p_driver_can_->send_command(this, pos_kp, get_adjusted_pos_kd(), pos_absolute, target_vel, clipped_tor);
    PI_INFO("Servo", InfoLevel::FREQUENT_3,
            "Servo ID %d: Move with position=%.3f rad, velocity=%.3f rad/s, torque=%.3f Nm, pos_kp=%.3f, pos_kd=%.3f",
            id_, pos_absolute, target_vel, clipped_tor, pos_kp, pos_kd_);
    usleep(100);
    return return_code;
}

ReturnCode ServoDm::apply_torque(float torque) {
    if (reject_if_thermal_fault_latched() != ReturnCode::SUCCESS) {
        return ReturnCode::HARDWARE_FAULT;
    }
    if (p_driver_can_ == nullptr || p_joint_ == nullptr) {
        PI_ERROR("Driver or joint pointer is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    float rescaled_torque = torque * p_joint_->torq_rescale_;

    float clipped_tor = clipping(rescaled_torque, p_joint_->torq_min_, p_joint_->torq_max_);

    ReturnCode return_code = p_driver_can_->send_command(this, 0, 0, 0.0, 0.0, clipped_tor);
    usleep(200);
    PI_INFO("Servo", InfoLevel::FREQUENT_3, "Servo ID %d: Apply torque=%.3f Nm", id_, clipped_tor);

    return return_code;
}

ReturnCode ServoDm::apply_torque_with_damping(float torque) {
    if (reject_if_thermal_fault_latched() != ReturnCode::SUCCESS) {
        return ReturnCode::HARDWARE_FAULT;
    }
    if (p_driver_can_ == nullptr || p_joint_ == nullptr) {
        PI_ERROR("Driver or joint pointer is not initialized (servo ID %d)", id_);
        return ReturnCode::NOT_INITIALIZED;
    }

    const float rescaled_torque = torque * p_joint_->torq_rescale_;
    const float clipped_tor = clipping(rescaled_torque, p_joint_->torq_min_, p_joint_->torq_max_);

    ReturnCode return_code =
        p_driver_can_->send_command(this, 0, get_adjusted_pos_kd(), 0.0, 0.0, clipped_tor);
    usleep(200);
    PI_INFO("Servo", InfoLevel::FREQUENT_3,
            "Servo ID %d: Apply torque=%.3f Nm with pos_kd=%.3f", id_, clipped_tor, get_adjusted_pos_kd());

    return return_code;
}

ReturnCode ServoDm::parse_dm_servo_status(DriverCan::can_frame_t* p_frame, ReceivedServoData* p_received_servo_data,
                                          DriverArx::func_find_data_index_t p_find_data_index,
                                          DriverArx* p_driver_arx) {
    if (p_frame == nullptr) {
        PI_ERROR("Invalid CAN frame pointer");
        return ReturnCode::INVALID_PARAM;
    }

    if (p_received_servo_data == nullptr) {
        PI_ERROR("Invalid received servo data pointer");
        return ReturnCode::INVALID_PARAM;
    }

    if (p_find_data_index == nullptr) {
        PI_ERROR("Invalid data-index lookup callback");
        return ReturnCode::INVALID_PARAM;
    }

    if (p_driver_arx == nullptr) {
        PI_ERROR("Invalid ARX driver pointer");
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t data_len = p_frame->can_dlc;
    if (data_len < 8) {
        PI_ERROR("Invalid CAN frame data length: %d bytes (expected 8)", data_len);
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t* p_data = p_frame->data;

    uint8_t motor_id = p_data[0] & 0x0F;

    int data_index = (*p_find_data_index)(motor_id);
    if (data_index < 0 || data_index >= MAX_SERVO_INFO_BUF_SIZE) {
        PI_ERROR("Invalid data index %d for motor ID %d", data_index, motor_id);
        return ReturnCode::INVALID_PARAM;
    }

    if (p_data[2] == DM_CMD_WRITE) {
        return ReturnCode::SUCCESS;
    } else {
        auto registered_servo = Driver::lock_registered_servo(motor_id);
        ServoDm* p_servo = dynamic_cast<ServoDm*>(registered_servo.get());
        if (p_servo == nullptr) {
            PI_ERROR("Invalid servo pointer for motor ID %d", motor_id);
            return ReturnCode::INVALID_PARAM;
        }

        const ServoDmParam* p_servo_param = (const ServoDmParam*)p_servo->p_servo_param_;
        if (p_servo_param == nullptr) {
            PI_ERROR("Invalid servo parameter pointer for motor ID %d", motor_id);
            return ReturnCode::INVALID_PARAM;
        }

        p_received_servo_data[data_index].motor_id_ = motor_id;
        p_received_servo_data[data_index].error_ = p_data[0] >> 4;

        p_received_servo_data[data_index].angle_actual_rad_ =
            uint_to_float((p_data[1] << 8) | p_data[2], p_servo_param->pos_min_, p_servo_param->pos_max_, 16);

        p_received_servo_data[data_index].speed_actual_rad_ =
            uint_to_float((p_data[3] << 4) | (p_data[4] >> 4), p_servo_param->vel_min_, p_servo_param->vel_max_, 12);

        p_received_servo_data[data_index].current_actual_float_ =
            uint_to_float(((p_data[4] & 0x0F) << 8) | p_data[5], p_servo_param->tor_min_, p_servo_param->tor_max_, 12);

        p_received_servo_data[data_index].temperature_ = p_data[7];

    }

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::parser_encos_servo_status(DriverCan::can_frame_t* p_frame, ReceivedServoData* p_received_servo_data,
                                              DriverArx::func_find_data_index_t p_find_data_index) {
    if (p_frame == nullptr) {
        PI_ERROR("Invalid CAN frame pointer");
        return ReturnCode::INVALID_PARAM;
    }

    if (p_received_servo_data == nullptr) {
        PI_ERROR("Invalid received servo data pointer");
        return ReturnCode::INVALID_PARAM;
    }

    if (p_find_data_index == nullptr) {
        PI_ERROR("Invalid data-index lookup callback");
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t data_len = p_frame->can_dlc;
    if (data_len < 8) {
        PI_ERROR("Invalid CAN frame data length: %d bytes (expected 8)", data_len);
        return ReturnCode::INVALID_PARAM;
    }

    uint8_t* p_data = p_frame->data;

    uint8_t ack_status = p_data[0] >> 5;
    uint8_t motor_id = p_frame->can_id;
    // Use int (not uint8_t) so -1 from find_data_index stays -1 instead of
    // wrapping to 255 and writing out-of-bounds into p_received_servo_data.
    int data_index = (*p_find_data_index)(motor_id);
    if (data_index < 0 || data_index >= MAX_SERVO_INFO_BUF_SIZE) {
        PI_ERROR("ENCOS status frame with invalid data index %d (motor ID %d)", data_index, motor_id);
        return ReturnCode::INVALID_PARAM;
    }

    p_received_servo_data[data_index].motor_id_ = motor_id;
    p_received_servo_data[data_index].error_ = p_data[0] & 0x1F;

    if (ack_status == 1) {
        auto registered_servo = Driver::lock_registered_servo(motor_id);
        ServoDm* p_servo = dynamic_cast<ServoDm*>(registered_servo.get());
        if (p_servo == nullptr) {
            PI_ERROR("Invalid servo pointer for motor ID %d", motor_id);
            return ReturnCode::INVALID_PARAM;
        }

        const ServoDmParam* p_servo_param = (const ServoDmParam*)p_servo->p_servo_param_;
        if (p_servo_param == nullptr) {
            PI_ERROR("Invalid servo parameter pointer for motor ID %d", motor_id);
            return ReturnCode::INVALID_PARAM;
        }

        p_received_servo_data[data_index].angle_actual_rad_ =
            uint_to_float((p_data[1] << 8) | p_data[2], p_servo_param->pos_min_, p_servo_param->pos_max_, 16);

        p_received_servo_data[data_index].speed_actual_rad_ =
            uint_to_float((p_data[3] << 4) | (p_data[4] >> 4), p_servo_param->vel_min_, p_servo_param->vel_max_, 12);

        p_received_servo_data[data_index].current_actual_float_ =
            uint_to_float(((p_data[4] & 0x0F) << 8) | p_data[5], p_servo_param->tor_min_, p_servo_param->tor_max_, 12);

        p_received_servo_data[data_index].temperature_ = (p_data[6] - 50) / 2;

    } else if (ack_status == 2) {
        union RV_TypeConvert {
            float to_float;
            uint8_t buf[4];
        } rv_type_convert;

        rv_type_convert.buf[0] = p_data[4];
        rv_type_convert.buf[1] = p_data[3];
        rv_type_convert.buf[2] = p_data[2];
        rv_type_convert.buf[3] = p_data[1];
        p_received_servo_data[data_index].angle_actual_rad_ = rv_type_convert.to_float;

        p_received_servo_data[data_index].current_actual_float_ = ((p_data[5] << 8) | p_data[6]) / 100.0f;

        p_received_servo_data[data_index].temperature_ = (p_data[7] - 50) / 2;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::can_frame_to_command_encos_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id, float kp,
                                                     float kd, float pos, float spd, float tor) {
    // Zero the whole frame first: the builders below set can_id/can_dlc/data, but
    // struct can_frame also carries __pad/__res bytes that otherwise leave the
    // stack uninitialized and go onto the bus verbatim (flagged by SIL valgrind).
    can_frame = {};
    can_frame.can_dlc = 8;
    can_frame.can_id = motor_id;

    auto registered_servo = Driver::lock_registered_servo(motor_id);
    ServoDm* p_servo = dynamic_cast<ServoDm*>(registered_servo.get());
    if (p_servo == nullptr) {
        PI_ERROR("Invalid servo pointer for motor ID %d", motor_id);
        return ReturnCode::INVALID_PARAM;
    }

    const ServoDmParam* p_servo_param = (const ServoDmParam*)p_servo->p_servo_param_;
    if (p_servo_param == nullptr) {
        PI_ERROR("Invalid servo parameter pointer for motor ID %d", motor_id);
        return ReturnCode::INVALID_PARAM;
    }

    kp = (kp > p_servo_param->kp_max_) ? p_servo_param->kp_max_
                                       : ((kp < p_servo_param->kp_min_) ? p_servo_param->kp_min_ : kp);
    kd = (kd > p_servo_param->kd_max_) ? p_servo_param->kd_max_
                                       : ((kd < p_servo_param->kd_min_) ? p_servo_param->kd_min_ : kd);
    pos = (pos > p_servo_param->pos_max_) ? p_servo_param->pos_max_
                                          : ((pos < p_servo_param->pos_min_) ? p_servo_param->pos_min_ : pos);
    spd = (spd > p_servo_param->vel_max_) ? p_servo_param->vel_max_
                                          : ((spd < p_servo_param->vel_min_) ? p_servo_param->vel_min_ : spd);
    tor = (tor > p_servo_param->tor_max_) ? p_servo_param->tor_max_
                                          : ((tor < p_servo_param->tor_min_) ? p_servo_param->tor_min_ : tor);

    int kp_int = float_to_uint(kp, p_servo_param->kp_min_, p_servo_param->kp_max_, 12);
    int kd_int = float_to_uint(kd, p_servo_param->kd_min_, p_servo_param->kd_max_, 9);
    int pos_int = float_to_uint(pos, p_servo_param->pos_min_, p_servo_param->pos_max_, 16);
    int spd_int = float_to_uint(spd, p_servo_param->vel_min_, p_servo_param->vel_max_, 12);
    int tor_int = float_to_uint(tor, p_servo_param->tor_min_, p_servo_param->tor_max_, 12);

    can_frame.data[0] = 0x00 | (kp_int >> 7);
    can_frame.data[1] = ((kp_int & 0x7F) << 1) | ((kd_int & 0x100) >> 8);
    can_frame.data[2] = kd_int & 0xFF;
    can_frame.data[3] = pos_int >> 8;
    can_frame.data[4] = pos_int & 0xFF;
    can_frame.data[5] = spd_int >> 4;
    can_frame.data[6] = (spd_int & 0x0F) << 4 | (tor_int >> 8);
    can_frame.data[7] = tor_int & 0xFF;

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::can_frame_to_command_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id, float kp,
                                                  float kd, float pos, float spd, float tor) {
    can_frame = {};
    can_frame.can_dlc = 8;
    can_frame.can_id = motor_id;

    auto registered_servo = Driver::lock_registered_servo(motor_id);
    ServoDm* p_servo = dynamic_cast<ServoDm*>(registered_servo.get());
    if (p_servo == nullptr) {
        PI_ERROR("Invalid servo pointer for motor ID %d", motor_id);
        return ReturnCode::INVALID_PARAM;
    }

    const ServoDmParam* p_servo_param = (const ServoDmParam*)p_servo->p_servo_param_;
    if (p_servo_param == nullptr) {
        PI_ERROR("Invalid servo parameter pointer for motor ID %d", motor_id);
        return ReturnCode::INVALID_PARAM;
    }

    kp = (kp > p_servo_param->kp_max_) ? p_servo_param->kp_max_
                                       : ((kp < p_servo_param->kp_min_) ? p_servo_param->kp_min_ : kp);
    kd = (kd > p_servo_param->kd_max_) ? p_servo_param->kd_max_
                                       : ((kd < p_servo_param->kd_min_) ? p_servo_param->kd_min_ : kd);
    pos = (pos > p_servo_param->pos_max_) ? p_servo_param->pos_max_
                                          : ((pos < p_servo_param->pos_min_) ? p_servo_param->pos_min_ : pos);
    spd = (spd > p_servo_param->vel_max_) ? p_servo_param->vel_max_
                                          : ((spd < p_servo_param->vel_min_) ? p_servo_param->vel_min_ : spd);
    tor = (tor > p_servo_param->tor_max_) ? p_servo_param->tor_max_
                                          : ((tor < p_servo_param->tor_min_) ? p_servo_param->tor_min_ : tor);

    uint16_t pos_tmp = float_to_uint(pos, p_servo_param->pos_min_, p_servo_param->pos_max_, 16);
    uint16_t vel_tmp = float_to_uint(spd, p_servo_param->vel_min_, p_servo_param->vel_max_, 12);
    uint16_t kp_tmp = float_to_uint(kp, p_servo_param->kp_min_, p_servo_param->kp_max_, 12);
    uint16_t kd_tmp = float_to_uint(kd, p_servo_param->kd_min_, p_servo_param->kd_max_, 12);
    uint16_t tor_tmp = float_to_uint(tor, p_servo_param->tor_min_, p_servo_param->tor_max_, 12);

    can_frame.data[0] = (pos_tmp >> 8);
    can_frame.data[1] = (pos_tmp & 0xFF);
    can_frame.data[2] = (vel_tmp >> 4);
    can_frame.data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    can_frame.data[4] = (kp_tmp & 0xFF);
    can_frame.data[5] = (kd_tmp >> 4);
    can_frame.data[6] = (((kd_tmp & 0xF) << 4) | (tor_tmp >> 8));
    can_frame.data[7] = (tor_tmp & 0xFF);

    return ReturnCode::SUCCESS;
}

std::string byte_to_hex(uint8_t byte) {
    std::stringstream ss;
    ss << std::hex << std::setw(2) << std::setfill('0') << (int)byte << " ";
    return ss.str();
}

ReturnCode ServoDm::can_frame_to_write_register_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                         RegAddr register_id, uint32_t register_value) {
    can_frame = {};
    can_frame.can_dlc = 8;
    can_frame.can_id = 0x7FF;

    can_frame.data[0] = motor_id & 0xFF;
    can_frame.data[1] = motor_id >> 8;
    can_frame.data[2] = 0x55;
    can_frame.data[3] = (uint8_t)register_id;
    can_frame.data[4] = register_value;
    can_frame.data[5] = register_value >> 8;
    can_frame.data[6] = register_value >> 16;
    can_frame.data[7] = register_value >> 24;

    PI_INFO("Servo", InfoLevel::ESSENTIAL_0, "Register write command: motor ID=%d, register address=%d, value=%u",
            motor_id, (uint32_t)register_id, register_value);

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::can_frame_to_set_operation_mode_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                             OperationMode operation_mode) {
    return can_frame_to_write_register_dm_servo(can_frame, motor_id, RegAddr::CONTROL_MODE, (uint32_t)operation_mode);
}

ReturnCode ServoDm::can_frame_to_enable_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id,
                                                 bool enable_flag) {
    can_frame = {};
    can_frame.can_dlc = 8;
    can_frame.can_id = motor_id;

    can_frame.data[0] = 0xFF;
    can_frame.data[1] = 0xFF;
    can_frame.data[2] = 0xFF;
    can_frame.data[3] = 0xFF;
    can_frame.data[4] = 0xFF;
    can_frame.data[5] = 0xFF;
    can_frame.data[6] = 0xFF;
    can_frame.data[7] = (enable_flag) ? 0xFC : 0xFD;

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::can_frame_to_reset_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id) {
    can_frame = {};
    can_frame.can_dlc = 8;
    can_frame.can_id = motor_id;

    can_frame.data[0] = 0xFF;
    can_frame.data[1] = 0xFF;
    can_frame.data[2] = 0xFF;
    can_frame.data[3] = 0xFF;
    can_frame.data[4] = 0xFF;
    can_frame.data[5] = 0xFF;
    can_frame.data[6] = 0xFF;
    can_frame.data[7] = 0xFB;

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::can_frame_to_set_zero_dm_servo(DriverCan::can_frame_t& can_frame, uint16_t motor_id) {
    can_frame = {};
    can_frame.can_dlc = 8;
    can_frame.can_id = motor_id;

    can_frame.data[0] = 0xFF;
    can_frame.data[1] = 0xFF;
    can_frame.data[2] = 0xFF;
    can_frame.data[3] = 0xFF;
    can_frame.data[4] = 0xFF;
    can_frame.data[5] = 0xFF;
    can_frame.data[6] = 0xFF;
    can_frame.data[7] = 0xFE;

    return ReturnCode::SUCCESS;
}

ReturnCode ServoDm::read_hardware_values() {
    if (checker_motor_no_response_.is_holding((curr_pos_abs_ == 0 && curr_vel_ == 0 && curr_tor_ == 0))) {
        int cnt = (motor_moved_) ? MAX_CNT_MOTOR_NO_RESPONSE_NORMAL : MAX_CNT_MOTOR_NO_RESPONSE_INITIAL;
        PI_ERROR(
            "Servo ID %d: velocity and torque unchanged for %d control loops (position=%.3f rad, "
            "velocity=%.3f rad/s, torque=%.3f Nm)",
            id_, cnt, curr_pos_abs_, curr_vel_, curr_tor_);
        return safe_mode_.graceful_management(this, ReturnCode::SAFE_MODE_SIG);
    } else {
        safe_mode_.exit_safe_mode(this, ReturnCode::SAFE_MODE_SIG);
    }

    if (!motor_moved_) {
        if (curr_pos_abs_ != 0 || curr_vel_ != 0 || curr_tor_ != 0) {
            motor_moved_ = true;
            checker_motor_no_response_.set_hold_count_threshold(MAX_CNT_MOTOR_NO_RESPONSE_NORMAL);
        }
    }

    ReturnCode return_code = Servo::read_hardware_values();

    if (type_ != ServoType::ENCOS_A4310) {
        const DmServoStatusInfo& status = dm_servo_status_info(motor_error_code_);
        if (get_device_type_belong_to() == DeviceType::EFFECTOR && status.is_thermal_fault) {
            return latch_effector_thermal_fault(motor_error_code_, status.description, status.action,
                                                "firmware status during operation");
        }
        if (get_device_type_belong_to() == DeviceType::EFFECTOR &&
            temperature_ > TEMPERATURE_THRESHOLD_FORCE_STOP) {
            return latch_effector_thermal_fault(
                motor_error_code_, "temperature exceeds force-stop limit",
                "Stop commands and allow the motor to cool; inspect for mechanical binding or sustained load before retrying.",
                "first sample above 93 C");
        }
        if (status.is_fault) {
            if (last_reported_fault_code_ != motor_error_code_) {
                PI_ERROR("HARDWARE FAULT: DM servo id=%d reported status 0x%X (%s) during operation "
                         "(reported temperature=%.0f C). Action: %s",
                         id_, static_cast<unsigned>(motor_error_code_), status.description, temperature_,
                         status.action);
                last_reported_fault_code_ = motor_error_code_;
            }
            if (return_code == ReturnCode::SUCCESS) {
                return ReturnCode::HARDWARE_FAULT;
            }
        } else {
            last_reported_fault_code_ = 0;
        }
    }

    return return_code;
}
