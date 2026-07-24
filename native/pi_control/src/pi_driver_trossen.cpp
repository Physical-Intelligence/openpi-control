/*!
 * @file pi_driver_trossen.cpp
 * @brief Implementation of the DriverTrossen Trossen iNerve controller driver.
 */

#include "pi_driver_trossen.hpp"

// Streaming command interpolation horizon. 0 executes the target immediately
// (the host-side trajectory planning / leash already shapes the motion); the
// SDK treats sub-millisecond horizons as direct linear stepping.
#define TROSSEN_STREAM_GOAL_TIME_S 0.0

// TCP handshake timeout for configure(). Generous enough for a controller
// that is still booting, small enough to fail fast on a wrong IP.
#define TROSSEN_CONFIGURE_TIMEOUT_S 10.0

DriverTrossen::DriverTrossen(Device* p_device, const CommandLineArgs& cla, const std::string& controller_model)
    : DriverController(p_device, cla), controller_model_(controller_model) {
    is_leader_role_ = (cla.role == Role::LEADER);
}

DriverTrossen::~DriverTrossen() {
    // Must run here (not only in the base destructor): close() calls the
    // vendor hooks, which are gone once this destructor finished.
    close();
}

ReturnCode DriverTrossen::vendor_connect(const std::string& address, int& joint_count) {
    p_arm_ = std::make_unique<TrossenArmShim>();
    // clear_error = true: recover from a stale error latched by a previous
    // session (e.g. a hard-killed host) instead of refusing to configure.
    // Leader and follower differ in end-effector mass properties (the leader
    // carries the teleop handle); using the wrong set skews the controller's
    // own gravity compensation.
    if (!p_arm_->configure(controller_model_, is_leader_role_, address, true, TROSSEN_CONFIGURE_TIMEOUT_S)) {
        PI_ERROR("libtrossen_arm configure failed for '%s': %s", address.c_str(), p_arm_->last_error().c_str());
        p_arm_.reset();
        return ReturnCode::NO_RESPONSE;
    }

    joint_count = p_arm_->num_joints();
    PI_INFO("DriverTrossen", InfoLevel::ESSENTIAL_0,
            "Configured Trossen controller at '%s': model=%s, driver=%s, controller_firmware=%s, joints=%d",
            address.c_str(), controller_model_.c_str(), p_arm_->driver_version().c_str(),
            p_arm_->controller_version().c_str(), joint_count);
    return ReturnCode::SUCCESS;
}

ReturnCode DriverTrossen::vendor_disconnect() {
    if (p_arm_ == nullptr) {
        return ReturnCode::SUCCESS;
    }

    const bool clean = p_arm_->cleanup();
    if (!clean) {
        PI_ERROR("libtrossen_arm cleanup failed: %s", p_arm_->last_error().c_str());
    }
    p_arm_.reset();
    return clean ? ReturnCode::SUCCESS : ReturnCode::FAIL;
}

ReturnCode DriverTrossen::vendor_read_state(std::vector<JointState>& states) {
    if (p_arm_ == nullptr) {
        PI_ERROR("vendor_read_state() called without a configured Trossen driver");
        return ReturnCode::NOT_INITIALIZED;
    }

    std::vector<double> positions;
    std::vector<double> velocities;
    std::vector<double> efforts;
    std::vector<double> temperatures;
    if (!p_arm_->read_state(positions, velocities, efforts, temperatures)) {
        PI_ERROR("libtrossen_arm state read failed: %s", p_arm_->last_error().c_str());
        return ReturnCode::NO_RESPONSE;
    }

    if (positions.size() != states.size() || velocities.size() != states.size() || efforts.size() != states.size() ||
        temperatures.size() != states.size()) {
        PI_ERROR("Trossen state size mismatch: expected %zu joints, got pos=%zu vel=%zu eff=%zu temp=%zu",
                 states.size(), positions.size(), velocities.size(), efforts.size(), temperatures.size());
        return ReturnCode::HARDWARE_FAULT;
    }

    for (size_t i = 0; i < states.size(); i++) {
        states[i].position = (float)positions[i];
        states[i].velocity = (float)velocities[i];
        states[i].effort = (float)efforts[i];
        states[i].temperature = (float)temperatures[i];
        states[i].valid = true;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DriverTrossen::vendor_write_commands(const std::vector<JointCommand>& commands) {
    if (p_arm_ == nullptr) {
        PI_ERROR("vendor_write_commands() called without a configured Trossen driver");
        return ReturnCode::NOT_INITIALIZED;
    }

    const int joint_count = (int)commands.size();
    const int arm_joint_count = joint_count - 1;  // Trossen layout: N-1 arm joints + 1 gripper (last index).
    const int gripper_index = joint_count - 1;

    // Whole-group transactions when every arm joint carries the same pending
    // command type; per-joint fallback otherwise (e.g. during staggered
    // start-up moves).
    int arm_pending_positions = 0;
    int arm_pending_efforts = 0;
    for (int i = 0; i < arm_joint_count; i++) {
        if (!commands[i].pending) {
            continue;
        }
        if (commands[i].mode == JointCommandMode::POSITION) {
            arm_pending_positions++;
        } else if (commands[i].mode == JointCommandMode::EXTERNAL_EFFORT) {
            arm_pending_efforts++;
        }
    }

    bool write_ok = true;
    if (arm_pending_positions == arm_joint_count) {
        std::vector<double> positions(arm_joint_count);
        std::vector<double> velocity_ffs(arm_joint_count);
        for (int i = 0; i < arm_joint_count; i++) {
            positions[i] = commands[i].position;
            velocity_ffs[i] = commands[i].velocity_ff;
        }
        write_ok = p_arm_->set_arm_positions(positions, TROSSEN_STREAM_GOAL_TIME_S, false, velocity_ffs);
    } else if (arm_pending_efforts == arm_joint_count) {
        std::vector<double> efforts(arm_joint_count);
        for (int i = 0; i < arm_joint_count; i++) {
            efforts[i] = commands[i].external_effort;
        }
        write_ok = p_arm_->set_arm_external_efforts(efforts, TROSSEN_STREAM_GOAL_TIME_S, false);
    } else {
        for (int i = 0; i < arm_joint_count && write_ok; i++) {
            if (!commands[i].pending) {
                continue;
            }
            if (commands[i].mode == JointCommandMode::POSITION) {
                write_ok = p_arm_->set_joint_position(i, commands[i].position, TROSSEN_STREAM_GOAL_TIME_S, false,
                                                      commands[i].velocity_ff);
            } else if (commands[i].mode == JointCommandMode::EXTERNAL_EFFORT) {
                write_ok = p_arm_->set_joint_external_effort(i, commands[i].external_effort,
                                                             TROSSEN_STREAM_GOAL_TIME_S, false);
            }
        }
    }

    if (write_ok && commands[gripper_index].pending) {
        const JointCommand& gripper = commands[gripper_index];
        if (gripper.mode == JointCommandMode::POSITION) {
            write_ok = p_arm_->set_gripper_position(gripper.position, TROSSEN_STREAM_GOAL_TIME_S, false,
                                                    gripper.velocity_ff);
        } else if (gripper.mode == JointCommandMode::EXTERNAL_EFFORT) {
            write_ok = p_arm_->set_gripper_external_effort(gripper.external_effort, TROSSEN_STREAM_GOAL_TIME_S, false);
        }
    }

    if (!write_ok) {
        PI_ERROR("libtrossen_arm command write failed: %s", p_arm_->last_error().c_str());
        return ReturnCode::FAIL;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DriverTrossen::vendor_apply_command_modes(const std::vector<JointCommandMode>& modes) {
    if (p_arm_ == nullptr) {
        PI_ERROR("vendor_apply_command_modes() called without a configured Trossen driver");
        return ReturnCode::NOT_INITIALIZED;
    }

    std::vector<TrossenArmShim::Mode> shim_modes(modes.size(), TrossenArmShim::Mode::IDLE);
    for (size_t i = 0; i < modes.size(); i++) {
        switch (modes[i]) {
            case JointCommandMode::POSITION:
                shim_modes[i] = TrossenArmShim::Mode::POSITION;
                break;
            case JointCommandMode::EXTERNAL_EFFORT:
                shim_modes[i] = TrossenArmShim::Mode::EXTERNAL_EFFORT;
                break;
            case JointCommandMode::UNSET:
                shim_modes[i] = TrossenArmShim::Mode::IDLE;
                break;
        }
    }

    if (!p_arm_->set_joint_modes(shim_modes)) {
        PI_ERROR("libtrossen_arm set_joint_modes failed: %s", p_arm_->last_error().c_str());
        return ReturnCode::FAIL;
    }

    return ReturnCode::SUCCESS;
}

ReturnCode DriverTrossen::vendor_make_safe() {
    if (p_arm_ == nullptr) {
        return ReturnCode::SUCCESS;
    }

    // idle: the controller stops executing streamed commands and holds the
    // arm passively with its own gravity/friction compensation.
    if (!p_arm_->set_all_modes_idle()) {
        PI_ERROR("libtrossen_arm set_all_modes(idle) failed: %s", p_arm_->last_error().c_str());
        return ReturnCode::FAIL;
    }

    return ReturnCode::SUCCESS;
}
