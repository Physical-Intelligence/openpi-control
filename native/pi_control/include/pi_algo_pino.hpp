/*!
 * @file pi_algo_pino.hpp
 * @brief Pinocchio-based control algorithm implementation.
 */
#pragma once
#include <Eigen/Core>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <vector>

#include "pi_algo.hpp"
#include "pi_control.hpp"

/*!
 * @class AlgoPino
 * @brief Pinocchio-based control algorithm implementation.
 */
class AlgoPino : public Algo {
   public:
    /*!
     * @brief Constructs a new AlgoPino instance.
     *
     * @param p_device Pointer to the device.
     * @param cla Command-line arguments.
     */
    AlgoPino(Device* p_device, const CommandLineArgs& cla);

    // Destroys the AlgoPino instance.
    ~AlgoPino();

    /*!
     * @brief Initializes the algorithm.
     *
     * @param p_config_model Device model configuration.
     * @param p_config_individual Individual device configuration (optional).
     * @param cla Command-line arguments.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode init(const DeviceConfig* p_config_model, const DeviceConfig* p_config_individual,
                    const CommandLineArgs& cla) override;

    /*!
     * @brief Folds the attached effector's mass model into the gravity model.
     *
     * Loads ``<effector_model>_mass.json`` (sibling of the effector model config
     * named in ``cla``) and adds its inertia to the link the effector bolts onto.
     * Without this the URDF covers the bare arm only and gravity compensation
     * underestimates the load on every joint proximal to the wrist.
     *
     * @param cla Command-line arguments (provides the effector model config path).
     * @return ReturnCode::SUCCESS (also when no effector or no mass file exists),
     *         otherwise a parse/configuration error.
     */
    ReturnCode append_effector_mass(const CommandLineArgs& cla);

    /*!
     * @brief Calculates gravity compensation torques.
     *
     * @param joint_positions Joint positions (radians), optionally followed by positions for an attached effector.
     * @param calculated_torques Output vector for gravity compensation torques (Nm). Effector torque entries are zero.
     * @return ReturnCode::SUCCESS if successful, otherwise an error code.
     */
    ReturnCode gravity_compensation(const std::vector<float>& joint_positions,
                                    std::vector<float>& calculated_torques) override;

   private:
    pinocchio::Model model_;                    ///< Pinocchio model structure.
    pinocchio::Data data_;                      ///< Pinocchio data structure.
    pinocchio::FrameIndex end_link_index_;      ///< End-effector frame index.
    Eigen::Matrix3d rotation_matrix_base_;      ///< Base frame to Pinocchio frame rotation matrix.
};
