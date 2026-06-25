/*!
 * @file pi_algo_pino.cpp
 * @brief Implementation of the AlgoPino class for Pinocchio-based robot kinematics and dynamics algorithms.
 */

#include <Eigen/Core>
#include <cmath>
#include <cstring>
#include <fstream>
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

#include "json/json.hpp"
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

    return_code = append_effector_mass(cla);
    if (return_code != ReturnCode::SUCCESS) {
        return return_code;
    }

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

ReturnCode AlgoPino::append_effector_mass(const CommandLineArgs& cla) {
    if (cla.effector_model_config.empty() || cla.effector_model_config == OPT_DEFAULT_NONE) {
        return ReturnCode::SUCCESS;
    }
    const std::string suffix = ".json";
    std::string mass_path = cla.effector_model_config;
    if (mass_path.size() <= suffix.size() ||
        mass_path.compare(mass_path.size() - suffix.size(), suffix.size(), suffix) != 0) {
        PI_WARN("Effector model config '%s' has no .json suffix; skipping effector mass model",
                mass_path.c_str());
        return ReturnCode::SUCCESS;
    }
    mass_path = mass_path.substr(0, mass_path.size() - suffix.size()) + "_mass.json";

    std::ifstream mass_file(mass_path);
    if (!mass_file.is_open()) {
        PI_INFO("Algo", InfoLevel::ESSENTIAL_0,
                "No effector mass model at %s; gravity model covers the bare arm only",
                mass_path.c_str());
        return ReturnCode::SUCCESS;
    }

    try {
        json mass_json = json::parse(mass_file);
        const double mass = mass_json.at("mass").get<double>();
        const auto& com_json = mass_json.at("center_of_mass");
        const auto& inertia_json = mass_json.at("inertia");
        Eigen::Vector3d com(com_json.at(0).get<double>(), com_json.at(1).get<double>(),
                            com_json.at(2).get<double>());
        Eigen::Matrix3d rotational_inertia;
        const double ixx = inertia_json.at("ixx").get<double>();
        const double iyy = inertia_json.at("iyy").get<double>();
        const double izz = inertia_json.at("izz").get<double>();
        const double ixy = inertia_json.at("ixy").get<double>();
        const double ixz = inertia_json.at("ixz").get<double>();
        const double iyz = inertia_json.at("iyz").get<double>();
        rotational_inertia << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;

        // The mass model is expressed in the effector mount frame (the URDF's end
        // link); transform onto the supporting joint before adding so RNEA sees
        // the load where it physically hangs.
        const auto& frame = model_.frames[end_link_index_];
        const pinocchio::Inertia effector_inertia(mass, com, pinocchio::Symmetric3(rotational_inertia));
        model_.inertias[frame.parentJoint] += frame.placement.act(effector_inertia);
        data_ = pinocchio::Data(model_);

        PI_INFO("Algo", InfoLevel::ESSENTIAL_0,
                "Added effector mass model to gravity model: %.3f kg at %s (from %s)",
                mass, frame.name.c_str(), mass_path.c_str());
    } catch (const std::exception& e) {
        PI_ERROR("Failed to load effector mass model '%s': %s", mass_path.c_str(), e.what());
        return ReturnCode::INVALID_PARAM;
    }

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

