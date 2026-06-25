/*!
 * @file pi_control.hpp
 * @brief Core definitions, constants, and enumerations.
 */
#pragma once

#define DEFAULT_DOF_ARM                           6                  ///< Default arm DOF.
#define DEFAULT_SERVO_NUM_ARM                     DEFAULT_DOF_ARM    ///< Default number of servos in arm.
#define DEFAULT_DOF_EFFECTOR                      1                  ///< Default effector DOF.
#define DEFAULT_SERVO_NUM_EFFECTOR                DEFAULT_DOF_EFFECTOR  ///< Default number of servos in effector.
#define DEFAULT_JOYSTICK_NUM_CHANNEL              5                  ///< Default number of joystick channels.
#define DEFAULT_JOYSTICK_NUM_BUTTON               5                  ///< Default number of joystick buttons.

#define DEFAULT_VDC                               24.0               ///< Default DC bus voltage (volts).

#define MAX_VELOCITY_THRESHOLD                    30.0               ///< Maximum velocity threshold (rad/s).
#define TEMPERATURE_THRESHOLD_CAUTIOUS            60.0                ///< Cautious temperature threshold (degrees Celsius). Lower bound of the derating ramp.
#define TEMPERATURE_THRESHOLD_CRITICAL            90.0                ///< Critical temperature threshold (degrees Celsius). Upper bound of the 60-90 derating ramp; clamps targets to current pose.
#define TEMPERATURE_THRESHOLD_RANGE                (TEMPERATURE_THRESHOLD_CRITICAL - TEMPERATURE_THRESHOLD_CAUTIOUS)  ///< Temperature threshold range (degrees Celsius).
#define TEMPERATURE_THRESHOLD_FORCE_STOP          93.0                ///< Force-stop temperature threshold (degrees Celsius). Single source of truth: exceeding this triggers emergency recovery (slow ready move + process exit) so the servo cannot burn. Must stay >= TEMPERATURE_THRESHOLD_CRITICAL.
#define MAX_POS_DIFFERENCE_RAD                    0.3                ///< Maximum position difference threshold (radians).
#define DEFAULT_TOLERABLE_POS_DIFFERENCE_RAD      0.2                ///< Default tolerable position difference (radians).
#define DEFAULT_VELOCITY_THRESHOLD_RAD_SEC        0.1                ///< Default velocity threshold (rad/s).

// Velocity-bounded move-to-ready speeds (rad/s). Every "move to ready" path (startup, command-driven
// MOVE_TO_READY_POS, MOVE_TO_READY_AND_STOP, emergency recovery on CAN loss / over-temp / exception) uses
// step = max_vel * loop_dt so the angular speed is bounded regardless of how far the arm is from home.
// NORMAL is used for healthy stops (UI stop, episode end, command); ERROR is the slower speed used during
// emergency recovery. Configurable via --move_to_ready_vel_rad_s_{normal,error}.
#define MOVE_TO_READY_VEL_RAD_S_NORMAL            0.30f              ///< NORMAL speed for healthy move-to-ready.
#define MOVE_TO_READY_VEL_RAD_S_ERROR             0.20f              ///< ERROR speed for emergency recovery (~1.5x slower than NORMAL).
#define MOVE_TO_READY_VEL_RAD_S_NORMAL_EFFECTOR   MOVE_TO_READY_VEL_RAD_S_NORMAL  ///< Effector NORMAL speed; alias for now.
#define MOVE_TO_READY_VEL_RAD_S_ERROR_EFFECTOR    MOVE_TO_READY_VEL_RAD_S_ERROR   ///< Effector ERROR speed; alias for now.

// Stuck detection thresholds for ready movement exit. A joint that has been commanded toward the
// ready position but has not moved more than READY_MOVE_STUCK_POS_DELTA_RAD per iteration for
// READY_MOVE_STUCK_ITER_THRESHOLD consecutive iterations is treated as stuck so the ready
// movement can exit instead of waiting forever. Command frames keep flowing while stuck so a
// physically released joint can still catch up. The threshold only applies while the joint is
// outside ``pos_error_margin_`` of its target.
#define READY_MOVE_STUCK_POS_DELTA_RAD            0.0001f             ///< Per-iteration measured-position floor (radians).
#define READY_MOVE_STUCK_ITER_THRESHOLD           200                  ///< Consecutive iterations with sub-floor motion that mark a joint as stuck (~2s @ 100Hz).
#define READY_MOVE_TOLERANCE_HYSTERESIS_RAD       0.001f               ///< Settling hysteresis added only to ready-completion checks.

// Hard iteration budget for an entire move-to-ready. The stuck detector above resets whenever a
// joint twitches past the motion floor, so a joint oscillating against an external load (e.g.
// gravity sag fighting the position hold) can reset both the stuck counter and the arrival
// confirmation window indefinitely -- the ready move then runs at full control-loop rate for
// minutes (observed as the left-YAM startup retry storm, issue #5). The budget is derived from
// the actual travel distance at init (so it is NOT a fixed timeout: long moves get long budgets)
// and only fires when the move is far beyond the velocity-bounded travel time, at which point the
// device completes best-effort and names the joints that never settled.
#define READY_MOVE_BUDGET_TRAVEL_SCALE            3                    ///< Budget = scale x nominal velocity-bounded travel iterations.
#define READY_MOVE_BUDGET_MIN_ITERS               1500                 ///< Budget floor so short moves still get settle time (~15s @ 100Hz).
#define READY_MOVE_PROGRESS_LOG_PERIOD_ITERS      500                  ///< Period for in-progress laggard logging (~5s @ 100Hz).

// Short wait after sending the ENCOS zero-effort enable command so the asynchronous CAN receive
// thread has time to parse the first status frame into ``received_servo_data_``. Without this
// wait the subsequent ``read_hardware_values()`` can race the parser and return pos=0.
#define ENABLE_ENCOS_CACHE_WAIT_US                5000                 ///< Microseconds to wait after ENCOS enable.

/*!
 * @enum Role
 * @brief Device roles in teleoperation.
 */
enum class Role {
    LEADER = 0,   ///< Leader role.
    FOLLOWER = 1, ///< Follower role.
    UNKNOWN = 2   ///< Unknown role.
};

/*!
 * @enum ReturnCode
 * @brief Return code enumeration for function results and error handling.
 */
enum class ReturnCode {
    // General success and error codes
    SUCCESS = 0,         ///< Operation completed successfully.
    FAIL = -1,           ///< Operation failed.
    NOT_SUPPORTED = -2,  ///< Operation not supported.
    INVALID_PARAM = -3,  ///< Invalid parameters.
    NOT_INITIALIZED = -4,  ///< Resource not initialized.
    NOT_FOUND = -5,        ///< Resource not found.
    NO_RESPONSE = -6,      ///< No response from hardware.
    BUSY = -7,             ///< Resource is busy.
    HARDWARE_FAULT = -8,   ///< Hardware reported a specific fault condition.

    // Safe mode condition codes
    SAFE_MODE = -100,  ///< Generic safe mode condition.
    SAFE_MODE_POS_BEHIND = -101,  ///< Position safe mode: position behind target.
    SAFE_MODE_POS_EXCEED = -102,  ///< Position safe mode: position exceeds limits.
    SAFE_MODE_VEL = -103,  ///< Velocity safe mode: velocity exceeds threshold.
    SAFE_MODE_TOR = -104,  ///< Torque safe mode: torque exceeds limits.
    SAFE_MODE_SIG = -105,  ///< Signal safe mode: communication signal loss.
    SAFE_MODE_TEMPERATURE = -106,  ///< Temperature safe mode: temperature exceeds critical threshold.

    // Special condition codes
    STALL = -200  ///< Stall condition detected.
};
