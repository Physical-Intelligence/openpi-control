/*!
 * @file pi_algo_pino.cpp
 * @brief Implementation of the AlgoPino class for Pinocchio-based robot kinematics and dynamics algorithms.
 */

#include <Eigen/Core>
#include <cmath>
#include <cstring>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/inertia.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <vector>

#include "pi_command_line_args.hpp"
#include "pi_device.hpp"
#include "pi_algo_pino.hpp"

AlgoPino::AlgoPino(Device* p_device, const CommandLineArgs& cla) : Algo(p_device, cla) {}

AlgoPino::~AlgoPino() {}

ReturnCode AlgoPino::init(const DeviceConfig* p_config_model, const DeviceConfig* p_config_individual,
                          const CommandLineArgs& cla) {
    ReturnCode return_code = Algo::init(p_config_model, p_config_individual, cla);
    if (return_code != ReturnCode::SUCCESS) {
        PI_ERROR("Algo::init() failed");
        return return_code;
    }

    try {
        pinocchio::urdf::buildModel(urdf_path_, model_);
    } catch (const std::exception& e) {
        PI_ERROR("Failed to build robot model from URDF file '%s': %s", urdf_path_.c_str(), e.what());
        return ReturnCode::FAIL;
    }

    try {
        data_ = pinocchio::Data(model_);
    } catch (const std::exception& e) {
        PI_ERROR("Failed to create Pinocchio data structure: %s", e.what());
        return ReturnCode::FAIL;
    }

    end_link_index_ = model_.getFrameId(LINK_NAME_END);
    if (end_link_index_ == (pinocchio::FrameIndex)(-1)) {
        PI_ERROR("Failed to find end-effector frame ID for link '%s'", LINK_NAME_END);
        return ReturnCode::FAIL;
    }

    const int device_dof = p_device_->get_dof();
    if (device_dof < 0 || static_cast<size_t>(device_dof) >= data_.oMi.size()) {
        PI_ERROR("Device DOF %d is outside Pinocchio joint placement range [0, %d)", device_dof,
                 (int)data_.oMi.size());
        return ReturnCode::INVALID_PARAM;
    }

    PI_INFO("Algo", InfoLevel::ESSENTIAL_0,
            "Pinocchio model: njoints=%d, nq=%d, nv=%d, nframes=%d, device_dof=%d, end_link_frame_id=%d",
            model_.njoints, model_.nq, model_.nv, (int)model_.nframes,
            device_dof, (int)end_link_index_);

    const auto& frame = model_.frames[end_link_index_];
    PI_INFO("Algo", InfoLevel::ESSENTIAL_0,
            "end_link frame: parentJoint=%d, name=%s",
            (int)frame.parentJoint, frame.name.c_str());

    if ((int)frame.parentJoint != device_dof) {
        PI_WARN("Joint index mismatch! IK uses oMi[%d] but end_link parent joint is %d",
                device_dof, (int)frame.parentJoint);
    }

    {
        Eigen::VectorXd q0 = pinocchio::neutral(model_);
        pinocchio::forwardKinematics(model_, data_, q0);
        pinocchio::updateFramePlacements(model_, data_);
        Eigen::Vector3d pos_oMi = data_.oMi[device_dof].translation();
        Eigen::Vector3d pos_oMf = data_.oMf[end_link_index_].translation();
        double diff = (pos_oMi - pos_oMf).norm();
        PI_INFO("Algo", InfoLevel::ESSENTIAL_0,
                "Neutral pose check: oMi[%d]=(%.4f,%.4f,%.4f) oMf[end_link]=(%.4f,%.4f,%.4f) diff=%.6f",
                device_dof,
                pos_oMi(0), pos_oMi(1), pos_oMi(2),
                pos_oMf(0), pos_oMf(1), pos_oMf(2), diff);
    }

    // Effector inertia is already merged into the URDF's end link by the Python
    // launcher (openpi_control.urdf_inertial), so the model is complete as parsed.
    model_.gravity.linear() = Eigen::Vector3d(0, 0, -9.81);
    model_.gravity.angular() = Eigen::Vector3d::Zero();

    if ((int)base_rpy_.size() != 3) {
        PI_ERROR("Base rotation vector not initialized");
        return ReturnCode::NOT_INITIALIZED;
    }

    rotation_matrix_base_ = Eigen::AngleAxisd(static_cast<double>(base_rpy_[0]), Eigen::Vector3d::UnitX()) *
                            Eigen::AngleAxisd(static_cast<double>(base_rpy_[1]), Eigen::Vector3d::UnitY()) *
                            Eigen::AngleAxisd(static_cast<double>(base_rpy_[2]), Eigen::Vector3d::UnitZ());

    model_.gravity.linear() = rotation_matrix_base_ * model_.gravity.linear();

    PI_INFO("Algo", InfoLevel::ESSENTIAL_0, "Gravity vector after rotation: %f, %f, %f", model_.gravity.linear()(0),
            model_.gravity.linear()(1), model_.gravity.linear()(2));

    return ReturnCode::SUCCESS;
}

ReturnCode AlgoPino::gravity_compensation(const std::vector<float>& joint_positions,
                                          std::vector<float>& calculated_torques) {
    ReturnCode return_code = ReturnCode::SUCCESS;

    const size_t model_dof = static_cast<size_t>(model_.nv);
    const size_t device_dof_total = static_cast<size_t>(p_device_->get_dof_total());
    if (joint_positions.size() != model_dof && joint_positions.size() != device_dof_total) {
        PI_ERROR("Joint positions size (%d) does not match model degrees of freedom (%d) or total device degrees of "
                 "freedom (%d)",
                 (int)joint_positions.size(), model_.nv, (int)device_dof_total);
        return ReturnCode::INVALID_PARAM;
    }

    std::vector<float> positions(joint_positions.begin(), joint_positions.begin() + model_dof);
    std::vector<float> velocities(model_.nv, 0.0f);
    std::vector<float> accelerations(model_.nv, 0.0f);

    Eigen::VectorXf q = Eigen::Map<const Eigen::VectorXf>(positions.data(), positions.size());
    Eigen::VectorXf v = Eigen::Map<const Eigen::VectorXf>(velocities.data(), velocities.size());
    Eigen::VectorXf a = Eigen::Map<const Eigen::VectorXf>(accelerations.data(), accelerations.size());

    Eigen::VectorXf tau =
        pinocchio::rnea(model_, data_, q.cast<double>(), v.cast<double>(), a.cast<double>()).cast<float>();

    calculated_torques.assign(joint_positions.size(), 0.0f);
    Eigen::VectorXf::Map(calculated_torques.data(), tau.size()) = tau;

    std::string torque_info;
    for (int i = 0; i < (int)calculated_torques.size(); i++) {
        torque_info += std::to_string(calculated_torques[i]) + ", ";
    }
    PI_INFO("Algo", InfoLevel::FREQUENT_3, "Gravity compensation torques: " + torque_info);

    return return_code;
}

