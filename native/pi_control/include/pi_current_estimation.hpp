/*!
 * @file pi_current_estimation.hpp
 * @brief Servo motor DC current estimation.
 */
#pragma once

/*!
 * @class MotorParams4CurrentEstimation
 * @brief Motor physical parameters for current estimation.
 */
class MotorParams4CurrentEstimation {
   public:
    float kt0_;        ///< Torque constant at reference temperature (Nm/A).
    float r0_;          ///< Phase resistance at reference temperature (Ohms).
    float t0_;          ///< Reference temperature (degrees Celsius).
    float beta_;        ///< Temperature coefficient for torque constant (1/°C).
    float alpha_;       ///< Temperature coefficient for copper resistance (1/°C).
    float c1_;          ///< Linear coefficient for iron loss (W/(rad/s)).
    float c2_;          ///< Quadratic coefficient for iron loss (W/(rad/s)²).
    float eta_inv_;     ///< Inverter efficiency (0.0 to 1.0).
    float p_drv_;       ///< Driver overhead power loss (Watts).
    float eta_g_;       ///< Gearbox efficiency (0.0 to 1.0).
    float gear_ratio_;  ///< Gear ratio.

    /*!
     * @brief Constructor with default values.
     */
    MotorParams4CurrentEstimation() {
        // Default values that won't cause division errors but indicate uninitialized state
        // Note: These should be properly initialized by init_current_estimation() for each servo model
        kt0_        = 0.0;      // Will cause inaccurate calculations if not initialized (protected by std::max in kt_at_T)
        r0_         = 0.0;      // Will cause P_cu = 0 if not initialized, leading to inaccurate current estimation
        t0_         = 25.0;     // Reasonable default temperature reference
        beta_       = -0.0015;  // Typical temperature coefficient for Kt (was 1.0, which is wrong)
        alpha_      = 0.0039;   // Typical temperature coefficient for copper resistance (was 1.0, which is wrong)
        c1_         = 0.0;      // Iron loss coefficients (0 is acceptable if iron loss is negligible)
        c2_         = 0.0;      // Iron loss coefficients (0 is acceptable if iron loss is negligible)
        eta_inv_    = 0.96;     // Typical inverter efficiency (was 0.0, which causes division protection)
        p_drv_      = 0.0;      // Driver overhead loss (0 is acceptable if negligible)
        eta_g_      = 1.0;      // Gearbox efficiency (1.0 is correct for no gearbox)
        gear_ratio_ = 1.0;      // Gear ratio (1.0 is correct for direct drive)
    }
};

/*!
 * @class CalibParams4CurrentEstimation
 * @brief Calibration parameters for current estimation.
 */
class CalibParams4CurrentEstimation {
   public:
    float a_gain_;          ///< Linear gain factor (unitless).
    float b_offset_;        ///< Linear offset (Amperes).
    float i_baseline_;      ///< System baseline current (Amperes).
    float beta_scale_;      ///< Scaling factor for beta (unitless).
    float alpha_scale_;     ///< Scaling factor for alpha (unitless).
    float delta_t_winding_; ///< Temperature difference between case and winding (degrees Celsius).
    float stall_gain_;      ///< Gain factor for stall condition (unitless).
    float stall_omega_thr_; ///< Velocity threshold for stall (rad/s).

    /*!
     * @brief Constructor with default calibration values.
     */
    CalibParams4CurrentEstimation() {
        a_gain_          = 1.0;
        b_offset_        = 0.0;
        i_baseline_      = 0.0;
        beta_scale_      = 1.0;
        alpha_scale_     = 1.0;
        delta_t_winding_ = 0.0;
        stall_gain_      = 1.0;
        stall_omega_thr_ = 0.05;
    }
};

/*!
 * @class CurrentEstimation
 * @brief Servo motor DC current estimation.
 */
class CurrentEstimation {
   public:
    /*!
     * @brief Constructor.
     */
    CurrentEstimation();

    /*!
     * @brief Destructor.
     */
    ~CurrentEstimation();

    /*!
     * @brief Estimates input DC current consumption.
     * @param motor_params Motor physical parameters.
     * @param torque Torque (Nm).
     * @param velocity Motor angular velocity (rad/s).
     * @param temperature Motor case temperature (degrees Celsius).
     * @param vdc DC bus voltage (Volts).
     * @param max_torque Maximum torque threshold (Nm).
     * @param use_load_torque If true, torque is load torque.
     * @param J Rotational inertia (kg·m²).
     * @param domega_dt Angular acceleration (rad/s²).
     * @param B Back EMF constant (Nm/(rad/s)).
     * @param tau_coulomb Coulomb friction torque (Nm).
     * @param tau_gravity Gravity compensation torque (Nm).
     * @return Estimated input DC current (Amperes).
     */
    float estimate_idc_calibrated(const MotorParams4CurrentEstimation& motor_params,
                                  float torque,
                                  float velocity,
                                  float temperature,
                                  float vdc,
                                  float max_torque,
                                  bool use_load_torque = true,
                                  float J = 0.0,
                                  float domega_dt = 0.0,
                                  float B = 0.0,
                                  float tau_coulomb = 0.0,
                                  float tau_gravity = 0.0) const;

   private:
    /*!
     * @brief Calculates torque constant at given temperature.
     * @param motor_params Motor parameters.
     * @param temperature Motor case temperature (degrees Celsius).
     * @param beta_scale Scaling factor for beta.
     * @param delta_T_winding Temperature difference between case and winding (degrees Celsius).
     * @return Torque constant (Nm/A).
     */
    float kt_at_T(const MotorParams4CurrentEstimation& motor_params,
                  float temperature,
                  float beta_scale = 1.0,
                  float delta_T_winding = 0.0) const;

    /*!
     * @brief Calculates phase resistance at given temperature.
     * @param motor_params Motor parameters.
     * @param temperature Motor case temperature (degrees Celsius).
     * @param alpha_scale Scaling factor for alpha.
     * @param delta_T_winding Temperature difference between case and winding (degrees Celsius).
     * @return Phase resistance (Ohms).
     */
    float R_at_T_scaled(const MotorParams4CurrentEstimation& motor_params,
                        float temperature,
                        float alpha_scale = 1.0,
                        float delta_T_winding = 0.0) const;

    /*!
     * @brief Calculates iron loss.
     * @param motor_params Motor parameters.
     * @param velocity Motor angular velocity (rad/s).
     * @return Iron loss power (Watts).
     */
    float iron_loss(const MotorParams4CurrentEstimation& motor_params, float velocity) const;

    /*!
     * @brief Calculates required electromagnetic torque.
     * @param base_tau_em Base electromagnetic torque (Nm).
     * @param velocity Motor angular velocity (rad/s).
     * @param J Rotational inertia (kg·m²).
     * @param domega_dt Angular acceleration (rad/s²).
     * @param B Back EMF constant (Nm/(rad/s)).
     * @param tau_coulomb Coulomb friction torque (Nm).
     * @param tau_gravity Gravity compensation torque (Nm).
     * @param eta_g Gearbox efficiency (0.0 to 1.0).
     * @return Required electromagnetic torque (Nm).
     */
    float electromagnetic_torque(float base_tau_em,
                                 float velocity,
                                 float J = 0.0,
                                 float domega_dt = 0.0,
                                 float B = 0.0,
                                 float tau_coulomb = 0.0,
                                 float tau_gravity = 0.0,
                                 float eta_g = 1.0) const;

    CalibParams4CurrentEstimation calib_params_;  ///< Calibration parameters.
};
