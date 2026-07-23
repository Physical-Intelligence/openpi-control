/*!
 * @file pi_device_config.hpp
 * @brief Device configuration management.
 */
#pragma once
#include <string>

#include "json/json.hpp"
#include "pi_command_line_args.hpp"
#include "pi_control.hpp"
#include "pi_info.hpp"

using json = nlohmann::json;

/*!
 * @enum DeviceConfigType
 * @brief Device configuration types.
 */
enum class DeviceConfigType {
    ARM,                 ///< Robotic arm device configuration.
    EFFECTOR             ///< Attached end-effector configuration.
};

#define CURRENT_CONFIG_VERSION "1.1.1"  ///< Currently supported configuration file format version.

/*!
 * @class DeviceConfig
 * @brief Device configuration management class.
 */
class DeviceConfig {
   public:
    const std::string fn_config_version      = "config_version";      ///< Field name for config file format version.
    const std::string fn_device_model         = "device_model";       ///< Field name for device model name.
    const std::string fn_device_id            = "device_id";          ///< Field name for device id.
    const std::string fn_spring_effect        = "spring_effect";      ///< Field name for spring_effect on/off.
    const std::string fn_gravity_compensation = "gravity_compensation"; ///< Field name for gravity_compensation on/off.
    const std::string fn_read_only            = "read_only";            ///< Field name for read_only on/off.
    const std::string fn_publishes_joystick   = "publishes_joystick";  ///< Top-level boolean: declares this effector hosts a joystick servo and must publish MsgJoystick. When true, pi_control_node auto-derives `--topic_joystick` from the per-handle `joystick_side`. Absent/false => legacy servo_model scan fallback.

    const std::string fn_device_type            = "device_type";    ///< Field name for device type.
    const std::string val_device_type_arm       = "arm";            ///< Value for device type arm.
    const std::string val_device_type_effector  = "effector";       ///< Value for device type effector.

    const std::string fn_arm_type         = "arm_type";  ///< Field name for arm type.
    const std::string val_arm_type_arx    = "arx";      ///< Value for arm type arx.

    const std::string fn_effector_type                    = "effector_type";     ///< Field name for effector type.
    const std::string val_effector_type_arx                   = "arx";               ///< Value for effector type arx.
    const std::string val_effector_type_none                  = "None";             ///< Value for effector type none.
    const std::string fn_effector_control_mode            = "control_mode";      ///< Field name for effector control mode.
    const std::string val_effector_control_mode_torque        = "torque";            ///< Value for effector control mode torque.
    const std::string val_effector_control_mode_position      = "position";         ///< Value for effector control mode position.
    const std::string fn_effector_dist_to_torque_const        = "dist_to_torque_const"; ///< Field name for effector distance to torque constant.
    const std::string fn_effector_grip_spring_offset           = "grip_spring_offset"; ///< Field name for the torque-mode spring offset (rad), subtracted from the position error (monopi ControlFollowGripper "offset"). Optional; default 0.
    const std::string fn_effector_open_at_min                = "open_at_min";    ///< Field name to specify the open side is at min position. (default is false)


    const std::string fn_topic_type      = "topic_type";  ///< Field name for topic type.
    const std::string val_topic_type_zmq = "ZMQ";        ///< Value for topic type ZMQ.

    const std::string fn_driver_type         = "driver_type";  ///< Field name for driver type.
    const std::string val_driver_type_can        = "CAN";          ///< Value for driver type CAN.
    const std::string val_driver_type_can_encoder = "CAN_ENCODER"; ///< Value for driver type CAN read-only encoder (DriverArxEncoder).

    const std::string fn_algo_type            = "algo_type";   ///< Field name for algorithm type.
    const std::string val_algo_type_algo          = "Algo";        ///< Value for algorithm type Algo.
    const std::string val_algo_type_pinocchio     = "Pinocchio";   ///< Value for algorithm type Pinocchio.

    const std::string fn_base_rpy   = "base_rpy";    ///< Field name for base axes rotation (roll, pitch, yaw), radian.

    const std::string fn_planning_type                        = "planning_type";                    ///< Field name for moving trajectory planning type.
    const std::string val_planning_type_none                      = "None";                            ///< Value for no planning.
    const std::string val_planning_type_slew_pos_gravity          = "slew_pos_gravity";                 ///< Value for synchronized velocity-limited position tracking + gravity compensation torque.

    const std::string fn_joint_init_sequence    = "joint_init_sequence";  ///< Field name for joint initialization sequence.
    const std::string fn_joints                 = "joints";              ///< Field name for joints.
    const std::string fn_joint_id               = "joint_id";            ///< Field name for joint ID.
    const std::string fn_joint_pos_rescale      = "pos_rescale";         ///< Field name for joint position rescale.
    const std::string fn_joint_safe_mode_derating = "safe_mode_derating"; ///< Field name for joint safe mode derating.
    const std::string fn_joint_pos_error_margin = "pos_error_margin";    ///< Field name for joint position error margin.
    const std::string fn_joint_pos_max_safety_margin = "pos_max_safety_margin";  ///< Field name for joint maximum-position safety margin (rad). See Joint::pos_max_safety_margin_.
    const std::string fn_joint_normalized_pos_min = "normalized_pos_min";  ///< Optional lower relative-radian bound used by normalized mapping.
    const std::string fn_joint_normalized_pos_max = "normalized_pos_max";  ///< Optional upper relative-radian bound used by normalized mapping.
    const std::string fn_joint_torq_min         = "torq_min";            ///< Field name for joint torque minimum.
    const std::string fn_joint_torq_max         = "torq_max";            ///< Field name for joint torque maximum.
    const std::string fn_joint_safe_torq_min    = "safe_torq_min";       ///< Field name for joint safe torque minimum.
    const std::string fn_joint_safe_torq_max    = "safe_torq_max";       ///< Field name for joint safe torque maximum.
    const std::string fn_joint_torq_rescale     = "torq_rescale";        ///< Field name for joint torque rescale.
    const std::string fn_joint_vel_max          = "vel_max";             ///< Field name for joint velocity maximum.
    const std::string fn_joint_follow_vel_max   = "follow_vel_max";      ///< Optional follower slew velocity maximum; defaults to vel_max.
    const std::string fn_joint_follow_viscous_damping = "follow_viscous_damping";  ///< Optional motor-side follower viscous damping coefficient.
    const std::string fn_joint_grip_torque_limit = "grip_torque_limit";  ///< Field name for the symmetric grip torque bound (Nm). In position mode, clamps target error to +/-limit/kp; in torque mode it directly clips the spring torque. 0/absent disables.
    const std::string fn_joint_accel_max        = "accel_max";            ///< Field name for joint acceleration maximum.
    const std::string fn_joint_gravity_comp_factor = "gravity_comp_factor";  ///< Field name for per-joint gravity feedforward scale.

    const std::string fn_joint_reference_servo_index = "reference_servo_index";  ///< Field name for reference servo index.

    const std::string fn_joint_spring_constant      = "spring_constant";      ///< Field name for spring constant.
    const std::string fn_joint_spring_preload       = "spring_preload";       ///< Field name for spring preload.
    const std::string fn_joint_spring_force_config  = "spring_force_config"; ///< Field name for spring force config.
    const std::string fn_joint_spring_type          = "spring_type";          ///< Field name for spring type.
    const std::string fn_joint_threshold_angle_change = "threshold_angle_change"; ///< Field name for threshold of angle change.
    const std::string fn_joint_threshold_time_sec   = "threshold_time_sec";   ///< Field name for threshold of stable time.
    const std::string fn_joint_spring_invert        = "spring_invert";         ///< Field name for spring invert flag.

    const std::string fn_servos                        = "servos";              ///< Field name for servos.
    const std::string fn_servo_model                   = "servo_model";          ///< Field name for servo model.
    // Servo strings used by the retained ARX/YAM model JSON files.
    const std::string val_servo_model_dm_4340              = "DM J4340";              ///< Damiao J4340 (CAN).
    const std::string val_servo_model_dm_4310              = "DM J4310";              ///< Damiao J4310 (CAN).
    const std::string val_servo_model_encos_A4310          = "Encos EC-A4310-P2-36";  ///< Encos EC-A4310-P2-36 (CAN).
    const std::string val_servo_model_can_passive_encoder  = "CAN Passive Encoder";   ///< YAM teaching-handle trigger encoder (CAN request/response poll, read-only).
    const std::string val_servo_model_arx_encoder           = "ARX Remote Encoder";     ///< ARX read-only joint encoder (CAN, 2-byte angle, no actuation).
    const std::string fn_servo_id                      = "servo_id";            ///< Field name for servo ID.
    const std::string fn_servo_data_index               = "data_index";          ///< Field name for servo data index.
    const std::string fn_servo_pos_min                  = "pos_min";             ///< Field name for servo position minimum (relative radian).
    const std::string fn_servo_pos_max                  = "pos_max";             ///< Field name for servo position maximum (relative radian).
    const std::string fn_servo_pos_kp                   = "pos_kp";              ///< Field name for servo position KP.
    const std::string fn_servo_pos_ki                   = "pos_ki";              ///< Field name for servo position KI.
    const std::string fn_servo_pos_kd                   = "pos_kd";              ///< Field name for servo position KD.
    const std::string fn_servo_pos_imax                 = "pos_imax";            ///< Field name for servo position KI maximum value.
    const std::string fn_servo_vel_kp                   = "vel_kp";              ///< Field name for servo velocity KP.
    const std::string fn_servo_vel_ki                   = "vel_ki";              ///< Field name for servo velocity KI.
    const std::string fn_servo_vel_kd                   = "vel_kd";              ///< Field name for servo velocity KD.
    const std::string fn_servo_vel_imax                 = "vel_imax";            ///< Field name for servo velocity KI maximum value.
    const std::string fn_servo_vel_max                  = "vel_max";             ///< Field name for servo velocity maximum.
    const std::string fn_servo_tor_max                  = "tor_max";             ///< Field name for servo torque maximum.
    const std::string fn_servo_current_max              = "current_max";         ///< Field name for servo current max (mA).
    const std::string fn_servo_current_min              = "current_min";         ///< Field name for servo current min (mA).
    const std::string fn_servo_current_target           = "current_target";       ///< Field name for servo target current (mA).
    const std::string fn_servo_kt                        = "kT";                  ///< Field name for servo current value to torque (Nm) conversion constant.
    const std::string fn_servo_ka                        = "kA";                  ///< Field name for servo current value to mA conversion constant.
    const std::string fn_servo_kv                        = "kV";                  ///< Field name for servo velocity value to rpm conversion constant.
    const std::string fn_servo_resolution                = "servo_resolution";    ///< Field name for servo resolution.
    const std::string fn_servo_dir_invert                = "dir_invert";          ///< Field name for servo direction invert: inverted = -1, not inverted = 1.
    const std::string fn_servo_zero_pos                  = "zero_pos";            ///< Field name for servo zero position (absolute radian).
    const std::string fn_servo_position_wrap_period      = "position_wrap_period"; ///< Optional single-turn feedback wrap period (relative radian).
    const std::string fn_servo_spring_home_pos           = "spring_home_pos";     ///< Field name for servo home position (relative radian).
    const std::string fn_servo_home_pos                   = "home_pos";           ///< Field name for home position (relative radian).
    const std::string fn_servo_response_delay             = "response_delay";     ///< Field name for response_delay (sec).
    const std::string fn_servo_response_can_id             = "response_can_id";    ///< Field name for the CAN id a passive encoder answers on (optional; default encoder id + 1, the i2rt ``plus_one`` firmware receive mode).
    const std::string fn_passive_encoder_firmware_compat  = "passive_encoder_firmware_compat";  ///< Field name (bool, passive encoders only; optional, default false). When true the driver tolerates multiple teaching-handle firmware revisions during EEPROM/frequency setup (both the <=2.2.x READINGS and >=2.3.x GET_EEPROM reply formats, plus the legacy <=2.2.12 8-bit frequency layout). Isolated to the YAM handle: ARX effectors have no passive encoder and never read this.
    const std::string fn_servo_reverse                    = "reverse_flag";       ///< Field name for direction reverse flag (bool).

    const std::string fn_joystick_deadband              = "joystick_deadband";              ///< Field name for joystick deadband (raw byte half-width applied symmetrically around the X/Y channel center).
    const std::string fn_joystick_trigger_deadband      = "trigger_deadband";                ///< Field name for the trigger-channel one-sided open-end deadband, in normalized units [0, 1] (same convention used by the VR mapper's ``VrJoystickMappingConfig.trigger_deadband``). Snaps the released (open) end only; the squeezed (close) side stays strictly linear. Optional; 0.0 (default) disables.
    const std::string fn_joystick_channel_max            = "joystick_channel_max";            ///< Field name for joystick channel max value.
    const std::string fn_joystick_channel_min            = "joystick_channel_min";            ///< Field name for joystick channel min value.
    const std::string fn_joystick_button_pressed_value   = "joystick_button_pressed_value";   ///< Field name for value when joystick button is pressed.
    const std::string fn_joystick_button_unpressed_value = "joystick_button_unpressed_value"; ///< Field name for value when joystick button is not pressed.
    const std::string fn_joystick_channel_num            = "joystick_channel_num";            ///< Field name for number of joystick channels.
    const std::string fn_joystick_button_num             = "joystick_button_num";             ///< Field name for number of joystick buttons.
    const std::string fn_joystick_channel_center         = "joystick_channel_center";         ///< Field name for joystick channel center value (used for symmetric normalization around center).
    const std::string fn_joystick_side                   = "joystick_side";                   ///< Field name for joystick side, "LEFT" or "RIGHT".
    const std::string fn_brake_button                    = "brake_button";                    ///< Field name for brake-toggle button index in MsgJoystick.button_ array (null = disabled).
    const std::string val_joystick_side_left             = "LEFT";                            ///< Value for joystick_side = LEFT.
    const std::string val_joystick_side_right            = "RIGHT";                           ///< Value for joystick_side = RIGHT.

    json values_;  ///< JSON object storing the loaded configuration values.

    /*!
     * @brief Constructs a new DeviceConfig instance.
     */
    DeviceConfig();

    // Destroys the DeviceConfig instance.
    ~DeviceConfig();

    /*!
     * @brief Initializes device model configuration from a JSON file.
     * @param cla Command-line arguments.
     */
    ReturnCode init_config_model(const CommandLineArgs& cla);

    /*!
     * @brief Initializes device individual configuration from a JSON file.
     * @param cla Command-line arguments.
     */
    ReturnCode init_config_individual(const CommandLineArgs& cla);

    /*!
     * @brief Extracts a typed value from JSON data for the specified field.
     * @tparam T Type of the value to extract (e.g., int, float, std::string, bool).
     * @param json_data JSON object containing the configuration data.
     * @param field_name Name of the field to extract (should use one of the fn_* constants).
     * @param value Output parameter that will be populated with the extracted value.
     */
    template <typename T>
    ReturnCode get_field_value(const json& json_data, const std::string& field_name, T& value) const {
        if (json_data.contains(field_name)) {
            try {
                value = json_data[field_name].get<T>();
            } catch (const nlohmann::json::type_error& e) {
                std::string what = e.what();
                PI_ERROR("Type Error: %s", what.c_str());
                return ReturnCode::INVALID_PARAM;
            }
        } else {
            PI_WARN("%s is not defined in the config file", field_name.c_str());
            return ReturnCode::INVALID_PARAM;
        }

        return ReturnCode::SUCCESS;
    }

   private:
    /*!
     * @brief Loads and parses a configuration file from the given file path.
     * @param file_path Absolute or relative filesystem path to the configuration file.
     */
    ReturnCode init_config(const std::string& file_path);

    /*!
     * @brief Cascades the effector top-level ``open_at_min`` into every servo dict that
     *        does not already carry it.
     *
     * ``open_at_min`` is a mechanical fact declared once at the effector's top level.
     * Cascading it keeps per-servo normalized-position mapping consistent while preserving
     * a single source of truth.
     *
     * No-op when the top-level field is absent (e.g. non-effector configs, legacy files).
     * Explicit servo-level overrides are preserved.
     */
    void cascade_effector_open_at_min_to_servos();

};
