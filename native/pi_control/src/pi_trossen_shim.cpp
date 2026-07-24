/*!
 * @file pi_trossen_shim.cpp
 * @brief Implementation of TrossenArmShim (compiled into libpi_trossen_shim).
 *
 * This is the only translation unit that includes vendor headers. It is built
 * into a shared library together with the prebuilt libtrossen_arm.a; the
 * archive's symbols (including its bundled pinocchio/urdfdom/tinyxml2 copies)
 * are hidden from the rest of the process (see CMakeLists.txt), which is what
 * prevents the ODR clash with the node's own shared pinocchio.
 */

#include "pi_trossen_shim.hpp"

#include <exception>

#include "libtrossen_arm/trossen_arm.hpp"

// Trossen model strings accepted in model configs (fn_controller_model).
#define TROSSEN_MODEL_WXAI_V0 "wxai_v0"

struct TrossenArmShim::Impl {
    trossen_arm::TrossenArmDriver driver;
};

namespace {

trossen_arm::Mode to_vendor_mode(TrossenArmShim::Mode mode) {
    switch (mode) {
        case TrossenArmShim::Mode::POSITION:
            return trossen_arm::Mode::position;
        case TrossenArmShim::Mode::EXTERNAL_EFFORT:
            return trossen_arm::Mode::external_effort;
        case TrossenArmShim::Mode::IDLE:
            break;
    }
    return trossen_arm::Mode::idle;
}

}  // namespace

TrossenArmShim::TrossenArmShim() = default;

TrossenArmShim::~TrossenArmShim() {
    if (impl_ != nullptr) {
        try {
            impl_->driver.cleanup();
        } catch (const std::exception&) {
            // Destructor: nothing actionable; the connection dies with the process.
        }
    }
}

bool TrossenArmShim::configure(const std::string& model, bool leader_end_effector, const std::string& address,
                               bool clear_error, double timeout_s) {
    trossen_arm::Model vendor_model;
    if (model == TROSSEN_MODEL_WXAI_V0) {
        vendor_model = trossen_arm::Model::wxai_v0;
    } else {
        last_error_ = "unsupported Trossen controller model '" + model + "' (supported: " TROSSEN_MODEL_WXAI_V0 ")";
        return false;
    }

    const trossen_arm::EndEffector end_effector = leader_end_effector
                                                      ? trossen_arm::StandardEndEffector::wxai_v0_leader
                                                      : trossen_arm::StandardEndEffector::wxai_v0_follower;

    try {
        impl_ = std::make_unique<Impl>();
        impl_->driver.configure(vendor_model, end_effector, address, clear_error, timeout_s);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        impl_.reset();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::cleanup() {
    if (impl_ == nullptr) {
        return true;
    }

    try {
        impl_->driver.cleanup();
    } catch (const std::exception& e) {
        last_error_ = e.what();
        impl_.reset();
        return false;
    }

    impl_.reset();
    last_error_.clear();
    return true;
}

int TrossenArmShim::num_joints() const {
    if (impl_ == nullptr) {
        return 0;
    }
    return (int)impl_->driver.get_num_joints();
}

std::string TrossenArmShim::driver_version() const {
    if (impl_ == nullptr) {
        return "";
    }
    return impl_->driver.get_driver_version();
}

std::string TrossenArmShim::controller_version() const {
    if (impl_ == nullptr) {
        return "";
    }
    return impl_->driver.get_controller_version();
}

bool TrossenArmShim::read_state(std::vector<double>& positions, std::vector<double>& velocities,
                                std::vector<double>& efforts, std::vector<double>& temperatures) {
    if (impl_ == nullptr) {
        last_error_ = "read_state() called without a configured driver";
        return false;
    }

    try {
        positions = impl_->driver.get_all_positions();
        velocities = impl_->driver.get_all_velocities();
        efforts = impl_->driver.get_all_efforts();
        temperatures = impl_->driver.get_all_rotor_temperatures();
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_arm_positions(const std::vector<double>& positions, double goal_time, bool blocking,
                                       const std::vector<double>& velocity_ffs) {
    if (impl_ == nullptr) {
        last_error_ = "set_arm_positions() called without a configured driver";
        return false;
    }

    try {
        impl_->driver.set_arm_positions(positions, goal_time, blocking, velocity_ffs);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_arm_external_efforts(const std::vector<double>& efforts, double goal_time, bool blocking) {
    if (impl_ == nullptr) {
        last_error_ = "set_arm_external_efforts() called without a configured driver";
        return false;
    }

    try {
        impl_->driver.set_arm_external_efforts(efforts, goal_time, blocking);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_joint_position(int joint_index, double position, double goal_time, bool blocking,
                                        double velocity_ff) {
    if (impl_ == nullptr) {
        last_error_ = "set_joint_position() called without a configured driver";
        return false;
    }

    try {
        impl_->driver.set_joint_position((uint8_t)joint_index, position, goal_time, blocking, velocity_ff);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_joint_external_effort(int joint_index, double effort, double goal_time, bool blocking) {
    if (impl_ == nullptr) {
        last_error_ = "set_joint_external_effort() called without a configured driver";
        return false;
    }

    try {
        impl_->driver.set_joint_external_effort((uint8_t)joint_index, effort, goal_time, blocking);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_gripper_position(double position, double goal_time, bool blocking, double velocity_ff) {
    if (impl_ == nullptr) {
        last_error_ = "set_gripper_position() called without a configured driver";
        return false;
    }

    try {
        impl_->driver.set_gripper_position(position, goal_time, blocking, velocity_ff);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_gripper_external_effort(double effort, double goal_time, bool blocking) {
    if (impl_ == nullptr) {
        last_error_ = "set_gripper_external_effort() called without a configured driver";
        return false;
    }

    try {
        impl_->driver.set_gripper_external_effort(effort, goal_time, blocking);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_joint_modes(const std::vector<Mode>& modes) {
    if (impl_ == nullptr) {
        last_error_ = "set_joint_modes() called without a configured driver";
        return false;
    }

    std::vector<trossen_arm::Mode> vendor_modes(modes.size(), trossen_arm::Mode::idle);
    for (size_t i = 0; i < modes.size(); i++) {
        vendor_modes[i] = to_vendor_mode(modes[i]);
    }

    try {
        impl_->driver.set_joint_modes(vendor_modes);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

bool TrossenArmShim::set_all_modes_idle() {
    if (impl_ == nullptr) {
        last_error_ = "set_all_modes_idle() called without a configured driver";
        return false;
    }

    try {
        impl_->driver.set_all_modes(trossen_arm::Mode::idle);
    } catch (const std::exception& e) {
        last_error_ = e.what();
        return false;
    }

    last_error_.clear();
    return true;
}

const std::string& TrossenArmShim::last_error() const { return last_error_; }
