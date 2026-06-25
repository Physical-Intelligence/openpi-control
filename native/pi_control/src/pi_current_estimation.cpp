/*!
 * @file pi_current_estimation.cpp
 * @brief Implementation of the CurrentEstimation class for DC current estimation in robot motors.
 */

#include <cmath>

#include "pi_current_estimation.hpp"

CurrentEstimation::CurrentEstimation() {
    calib_params_.a_gain_ = 0.256;
    calib_params_.b_offset_ = 0.0;
    calib_params_.i_baseline_ = 0.016;
    calib_params_.alpha_scale_ = 2.9;
    calib_params_.beta_scale_ = 0.90;
    calib_params_.delta_t_winding_ = 5;
    calib_params_.stall_gain_ = 2.5;
    calib_params_.stall_omega_thr_ = 0.05;
}

CurrentEstimation::~CurrentEstimation() {}

float CurrentEstimation::kt_at_T(const MotorParams4CurrentEstimation& motor_params, float temperature, float beta_scale,
                                 float delta_T_winding) const {
    float T_w = temperature + delta_T_winding;
    return motor_params.kt0_ * (1.0 + (motor_params.beta_ * beta_scale) * (T_w - motor_params.t0_));
}

float CurrentEstimation::R_at_T_scaled(const MotorParams4CurrentEstimation& motor_params, float temperature,
                                       float alpha_scale, float delta_T_winding) const {
    float T_w = temperature + delta_T_winding;
    return motor_params.r0_ * (1.0 + (motor_params.alpha_ * alpha_scale) * (T_w - motor_params.t0_));
}

float CurrentEstimation::iron_loss(const MotorParams4CurrentEstimation& motor_params, float velocity) const {
    if (motor_params.c1_ == 0.0 && motor_params.c2_ == 0.0) {
        return 0.0;
    }
    return motor_params.c1_ * std::abs(velocity) + motor_params.c2_ * (velocity * velocity);
}

float CurrentEstimation::electromagnetic_torque(float base_tau_em, float velocity, float J, float domega_dt, float B,
                                                float tau_coulomb, float tau_gravity, float eta_g) const {
    float tau_inertia = J * domega_dt;
    float tau_friction = B * velocity + tau_coulomb;
    float tau_total = base_tau_em + tau_inertia + tau_friction + tau_gravity;

    return tau_total / eta_g;
}

float CurrentEstimation::estimate_idc_calibrated(const MotorParams4CurrentEstimation& motor_params, float torque,
                                                 float velocity, float temperature, float vdc, float max_torque,
                                                 bool use_load_torque, float J, float domega_dt, float B,
                                                 float tau_coulomb, float tau_gravity) const {
    float tau_em;
    if (use_load_torque) {
        float gear_ratio = motor_params.gear_ratio_;
        float base_tau_em = torque / std::max(gear_ratio, 1e-6f);
        tau_em = electromagnetic_torque(base_tau_em, velocity, J, domega_dt, B, tau_coulomb, tau_gravity,
                                        motor_params.eta_g_);
    } else {
        tau_em = torque;
    }

    float tau_em_abs = std::abs(tau_em);
    float velocity_abs = std::abs(velocity);
    float Kt =
        std::max(kt_at_T(motor_params, temperature, calib_params_.beta_scale_, calib_params_.delta_t_winding_), 1e-6f);
    float Rph = R_at_T_scaled(motor_params, temperature, calib_params_.alpha_scale_, calib_params_.delta_t_winding_);

    float P_mech = tau_em_abs * velocity_abs;
    float P_fe = iron_loss(motor_params, velocity_abs);
    float P_cu = 3.0 * Rph * (tau_em_abs / Kt) * (tau_em_abs / Kt);
    float P_drv = motor_params.p_drv_;

    float I_model = (P_mech + P_cu + P_fe + P_drv) / std::max(motor_params.eta_inv_ * vdc, 1e-6f);

    bool is_stall = (fabs(torque) > max_torque) && (fabs(velocity) < calib_params_.stall_omega_thr_);
    if (is_stall) {
        I_model *= calib_params_.stall_gain_;
    }

    float I_hat = calib_params_.a_gain_ * I_model + calib_params_.b_offset_ + calib_params_.i_baseline_;
    return std::max(I_hat, 0.0f);
}
