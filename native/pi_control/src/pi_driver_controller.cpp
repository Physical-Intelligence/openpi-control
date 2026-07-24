/*!
 * @file pi_driver_controller.cpp
 * @brief Implementation of the DriverController vendor-neutral whole-arm controller base class.
 */

#include "pi_driver_controller.hpp"

#include <chrono>

#include "pi_servo_controller.hpp"

namespace {

/*!
 * @brief Monotonic milliseconds for the stall watchdog.
 */
int64_t steady_now_ms() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace

DriverController::DriverController(Device* p_device, const CommandLineArgs& cla) : Driver(p_device, cla) {}

DriverController::~DriverController() {
    // Vendor hooks are gone once the derived destructor ran, so the derived
    // class must call close() in its own destructor. This base destructor only
    // stops the watchdog thread as a last line of defense.
    stop_watchdog();
}

ReturnCode DriverController::open(int baud_rate) {
    (void)baud_rate;  // Ethernet controllers have no baud rate.

    if (opened_) {
        PI_INFO("DriverController", InfoLevel::HELPFUL_1, "open() called on an already-open controller connection");
        return ReturnCode::SUCCESS;
    }

    if (control_port_name_.empty()) {
        PI_ERROR("Controller address is empty (--control_port_name must carry the controller IP address)");
        return ReturnCode::INVALID_PARAM;
    }

    int joint_count = 0;
    ReturnCode return_code = vendor_connect(control_port_name_, joint_count);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to connect to the arm controller at '%s'", control_port_name_.c_str());
        return return_code;
    }
    if (joint_count <= 0) {
        PI_ERROR("Arm controller at '%s' reported an invalid joint count: %d", control_port_name_.c_str(),
                 joint_count);
        vendor_disconnect();
        return ReturnCode::HARDWARE_FAULT;
    }
    joint_count_ = joint_count;

    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        commands_.assign(joint_count_, JointCommand{});
        desired_modes_.assign(joint_count_, JointCommandMode::UNSET);
        modes_dirty_ = false;
    }
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        states_.assign(joint_count_, JointState{});
    }
    read_success_count_ = 0;
    made_safe_ = false;
    watchdog_tripped_ = false;

    opened_ = true;

    // One initial whole-arm read so every ServoController position cache is
    // valid before start_hardware() / hold-at-current seeding runs.
    return_code = group_read_hardware_values();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Initial whole-arm state read failed for controller at '%s'", control_port_name_.c_str());
        close();
        return return_code;
    }

    PI_INFO("DriverController", InfoLevel::ESSENTIAL_0, "Connected to arm controller at '%s' (%d joints)",
            control_port_name_.c_str(), joint_count_);

    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::close() {
    stop_watchdog();

    if (!opened_) {
        return ReturnCode::SUCCESS;
    }

    // Idle the arm before dropping the connection so the controller never
    // keeps executing the last streamed command.
    make_safe();

    ReturnCode return_code = vendor_disconnect();
    opened_ = false;

    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Controller disconnect failed for '%s'", control_port_name_.c_str());
        return return_code;
    }

    PI_INFO("DriverController", InfoLevel::ESSENTIAL_0, "Disconnected from arm controller at '%s'",
            control_port_name_.c_str());

    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::read_hardware_values(Servo* p_servo) {
    ServoController* p_servo_controller = dynamic_cast<ServoController*>(p_servo);
    if (p_servo_controller == nullptr) {
        PI_ERROR("read_hardware_values() called with a non-ServoController servo");
        return ReturnCode::INVALID_PARAM;
    }

    const int data_index = p_servo_controller->data_index_;
    ReturnCode return_code = validate_data_index(data_index);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    JointState state;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = states_[data_index];
    }
    if (!state.valid) {
        PI_ERROR("Joint slot %d has no valid controller state yet (servo ID %d)", data_index,
                 p_servo_controller->id_);
        return ReturnCode::NO_RESPONSE;
    }

    p_servo_controller->update_from_controller_state(state.position, state.velocity, state.effort, state.temperature);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::group_read_hardware_values() {
    if (!opened_) {
        PI_ERROR("group_read_hardware_values() called before open()");
        return ReturnCode::NOT_INITIALIZED;
    }

    std::vector<JointState> states(joint_count_);
    ReturnCode return_code = vendor_read_state(states);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Whole-arm state read failed for controller at '%s'", control_port_name_.c_str());
        return return_code;
    }

    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        states_ = std::move(states);
    }
    read_success_count_++;
    feed_watchdog();

    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::group_write_hardware_values() {
    if (!opened_) {
        PI_ERROR("group_write_hardware_values() called before open()");
        return ReturnCode::NOT_INITIALIZED;
    }
    if (watchdog_tripped_) {
        PI_ERROR("Stall watchdog idled the arm; refusing further commands (restart required)");
        return ReturnCode::SAFE_MODE_SIG;
    }

    std::vector<JointCommand> commands;
    std::vector<JointCommandMode> modes;
    bool apply_modes = false;
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (modes_dirty_) {
            modes = desired_modes_;
            apply_modes = true;
            modes_dirty_ = false;
        }
        commands = commands_;
        for (auto& command : commands_) {
            command.pending = false;
        }
    }

    if (apply_modes) {
        ReturnCode return_code = vendor_apply_command_modes(modes);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Failed to apply joint control modes on controller at '%s'", control_port_name_.c_str());
            {
                // Restore the dirty flag so the next flush retries the mode change
                // before any command is interpreted under a stale mode.
                std::lock_guard<std::mutex> lock(command_mutex_);
                modes_dirty_ = true;
            }
            return return_code;
        }
        {
            // The arm is active again; a later shutdown must run vendor_make_safe()
            // even when an earlier make_safe() already latched.
            std::lock_guard<std::mutex> lock(safe_mutex_);
            made_safe_ = false;
        }
    }

    bool any_pending = false;
    for (const auto& command : commands) {
        if (command.pending) {
            any_pending = true;
            break;
        }
    }
    if (any_pending) {
        ReturnCode return_code = vendor_write_commands(commands);
        if (return_code != ReturnCode::SUCCESS) {
            PI_ERROR("Whole-arm command write failed for controller at '%s'", control_port_name_.c_str());
            return return_code;
        }
    }

    feed_watchdog();
    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::arm_comm_loss_protection() {
    if (!opened_) {
        PI_ERROR("arm_comm_loss_protection() called before open()");
        return ReturnCode::NOT_INITIALIZED;
    }
    if (watchdog_running_) {
        return ReturnCode::SUCCESS;
    }

    last_interaction_ms_ = steady_now_ms();
    watchdog_running_ = true;
    watchdog_thread_ = std::thread(&DriverController::watchdog_loop, this);

    PI_INFO("DriverController", InfoLevel::ESSENTIAL_0,
            "Stall watchdog armed (timeout %d ms): the arm is idled when the host control loop stops",
            CONTROLLER_STALL_WATCHDOG_TIMEOUT_MS);

    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::set_joint_command_mode(int data_index, JointCommandMode mode) {
    ReturnCode return_code = validate_data_index(data_index);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }
    if (mode == JointCommandMode::UNSET) {
        PI_ERROR("Joint slot %d: UNSET is not a requestable control mode", data_index);
        return ReturnCode::INVALID_PARAM;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (desired_modes_[data_index] == mode) {
        return ReturnCode::SUCCESS;
    }
    desired_modes_[data_index] = mode;
    modes_dirty_ = true;
    // Drop a command queued for the previous mode; it must not be sent under
    // the new mode.
    commands_[data_index] = JointCommand{};
    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::queue_position(int data_index, float position, float velocity_ff) {
    ReturnCode return_code = validate_data_index(data_index);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (desired_modes_[data_index] != JointCommandMode::POSITION) {
        PI_ERROR("Joint slot %d: position command queued while the joint is not in POSITION mode", data_index);
        return ReturnCode::INVALID_PARAM;
    }
    JointCommand& command = commands_[data_index];
    command.mode = JointCommandMode::POSITION;
    command.position = position;
    command.velocity_ff = velocity_ff;
    command.external_effort = 0;
    command.pending = true;
    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::queue_external_effort(int data_index, float external_effort) {
    ReturnCode return_code = validate_data_index(data_index);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

    std::lock_guard<std::mutex> lock(command_mutex_);
    if (desired_modes_[data_index] != JointCommandMode::EXTERNAL_EFFORT) {
        PI_ERROR("Joint slot %d: external effort queued while the joint is not in EXTERNAL_EFFORT mode", data_index);
        return ReturnCode::INVALID_PARAM;
    }
    JointCommand& command = commands_[data_index];
    command.mode = JointCommandMode::EXTERNAL_EFFORT;
    command.external_effort = external_effort;
    command.position = 0;
    command.velocity_ff = 0;
    command.pending = true;
    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::make_safe() {
    std::lock_guard<std::mutex> lock(safe_mutex_);
    if (made_safe_ || !opened_) {
        return ReturnCode::SUCCESS;
    }

    ReturnCode return_code = vendor_make_safe();
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Failed to put the arm controller at '%s' into its safe passive state", control_port_name_.c_str());
        return return_code;
    }

    made_safe_ = true;
    {
        std::lock_guard<std::mutex> command_lock(command_mutex_);
        for (auto& mode : desired_modes_) {
            mode = JointCommandMode::UNSET;
        }
        for (auto& command : commands_) {
            command = JointCommand{};
        }
        modes_dirty_ = false;
    }

    PI_INFO("DriverController", InfoLevel::ESSENTIAL_0, "Arm controller at '%s' put into safe passive state",
            control_port_name_.c_str());

    return ReturnCode::SUCCESS;
}

ReturnCode DriverController::validate_data_index(int data_index) const {
    if (data_index < 0 || data_index >= joint_count_) {
        PI_ERROR("Joint slot %d is out of range (controller manages %d joints)", data_index, joint_count_);
        return ReturnCode::INVALID_PARAM;
    }
    return ReturnCode::SUCCESS;
}

void DriverController::watchdog_loop() {
    while (watchdog_running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(CONTROLLER_STALL_WATCHDOG_PERIOD_MS));
        if (!watchdog_running_) {
            break;
        }

        const int64_t idle_ms = steady_now_ms() - last_interaction_ms_;
        if (idle_ms <= CONTROLLER_STALL_WATCHDOG_TIMEOUT_MS) {
            continue;
        }
        if (watchdog_tripped_) {
            continue;
        }

        PI_ERROR(
            "Stall watchdog: no driver interaction for %lld ms (host control loop hang?); "
            "idling the arm controller at '%s'",
            (long long)idle_ms, control_port_name_.c_str());
        watchdog_tripped_ = true;
        make_safe();
    }
}

void DriverController::feed_watchdog() {
    last_interaction_ms_ = steady_now_ms();
}

void DriverController::stop_watchdog() {
    watchdog_running_ = false;
    if (watchdog_thread_.joinable()) {
        watchdog_thread_.join();
    }
}
