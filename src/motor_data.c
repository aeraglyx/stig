// Copyright 2024 Lukas Hrazky
// Copyright 2024 Vladislav Macicek
//
// This file is part of the Stig VESC package.
//
// Stig VESC package is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by the
// Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// Stig VESC package is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
// or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along with
// this program. If not, see <http://www.gnu.org/licenses/>.

#include "motor_data.h"
#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

void motor_data_init(MotorData *m) {
    m->erpm = 0.0f;
    m->speed = 0.0f;
    m->speed_last = 0.0f;
    m->wheel_accel = 0.0f;
    m->accel_smooth = 0.0f;
    m->current_smooth = 0.0f;
    m->duty_cycle = 0.0f;
    m->slope = 0.0f;
    m->board_speed = 0.0f;
}

void motor_data_configure(
    MotorData *m,
    const CfgTune *cfg,
    const CfgHardware *hw,
    const CfgRider *rider,
    float dt
) {
    m->data_filter_alpha = half_time_to_alpha(cfg->traction.filter, dt);
    m->mod_filter_alpha = half_time_to_alpha(0.5f, dt);  // used only for curent_smooth
    m->board_speed_alpha = half_time_to_alpha(cfg->modifiers.board_speed_filter, dt);

    m->acceleration_source = cfg->modifiers.acceleration_source;
    m->use_erpm_correction = cfg->modifiers.use_erpm_correction;

    // min current is a positive value here!
    m->current_max = VESC_IF->get_cfg_float(CFG_PARAM_l_current_max);
    m->current_min = fabsf(VESC_IF->get_cfg_float(CFG_PARAM_l_current_min));
    m->duty_max = VESC_IF->get_cfg_float(CFG_PARAM_l_max_duty);

    uint8_t pole_pairs = VESC_IF->get_cfg_int(CFG_PARAM_si_motor_poles) / 2;
    float wheel_radius = 0.5f * VESC_IF->get_cfg_float(CFG_PARAM_si_wheel_diameter);

    m->erpm_gyro_ratio = 60.0f * pole_pairs / 360.0f;
    m->speed_erpm_ratio = TAU * wheel_radius / (pole_pairs * 60.0f);

    // const float flux_linkage = 0.030f;  // [Wb]
    // m->c_torque = 1.5f * flux_linkage * pole_pairs;
    m->c_torque = hw->motor.torque_constant;
    slope_configure(&m->slope_data, hw, rider);

    m->dt = dt;
}

static float get_erpm_confidence(const MotorData *m, const IMUData *imu) {
    float accel_confidence = bell_curve(1.0f * (m->wheel_accel - imu->board_accel));
    // const float speed_confidence = bell_curve(0.5f * (m->speed - m->board_speed));
    // return accel_confidence * speed_confidence;
    return accel_confidence;
}

void motor_data_update(MotorData *m, uint16_t frequency, const IMUData *imu) {
    float erpm_raw = VESC_IF->mc_get_rpm();
    float erpm_correction = imu->gyro[1] * m->erpm_gyro_ratio;

    float erpm_corrected = erpm_raw;
    if (m->use_erpm_correction) {
        erpm_corrected -= erpm_correction;
    }
    // const float erpm_corrected = erpm_raw - erpm_correction;
    const float speed_corrected = erpm_corrected * m->speed_erpm_ratio;

    filter_ema(&m->erpm, erpm_corrected, m->data_filter_alpha);

    filter_ema(&m->speed, speed_corrected, m->data_filter_alpha);
    m->speed_abs = fabsf(m->speed);
    m->speed_sign = sign(m->speed);

    float erpm_confidence = get_erpm_confidence(m, imu);
    float alpha = m->board_speed_alpha * erpm_confidence;
    float speed_imu = m->board_speed + g_to_mps2(imu->board_accel) * m->dt;
    m->board_speed = speed_imu + alpha * (m->speed - speed_imu);

    // 0 at 0 m/s, asymptotically approaches 1 at high speeds on both sides
    m->fast_boi = 1.0f - bell_curve(0.2f * m->board_speed);



    // Wheel acceleration in [g]
    float accel_raw = mps2_to_g((speed_corrected - m->speed_last) * frequency);
    // const float accel_raw_clamped = clamp_sym(accel_raw, 0.5f);

    filter_ema(&m->wheel_accel, accel_raw, m->data_filter_alpha);
    // TODO incorporate confidence information from traction
    // TODO keep wheel and board accelerations separate and switch later
    // TODO rename accel_smooth
    switch (m->acceleration_source) {
        case ACCELERATION_SOURCE_ERPM:
            m->accel_smooth = m->wheel_accel;
            // filter_ema(&m->accel_smooth, accel_raw_clamped, m->mod_filter_alpha);
            break;
        case ACCELERATION_SOURCE_IMU:
            m->accel_smooth = imu->board_accel;
            // filter_ema(&m->accel_smooth, imu->board_accel, m->mod_filter_alpha);
            break;
    }



    // m->current = VESC_IF->mc_get_tot_current_directional_filtered();
    float current_raw = VESC_IF->mc_get_tot_current_directional();
    filter_ema(&m->current, current_raw, m->data_filter_alpha);
    filter_ema(&m->current_smooth, current_raw, m->mod_filter_alpha);
    m->torque = m->current * m->c_torque;
    // m->torque_smooth = m->current_smooth * m->c_torque;

    m->braking = sign(m->torque) != m->speed_sign;

    float duty_cycle_raw = fabsf(VESC_IF->mc_get_duty_cycle_now());
    filter_ema(&m->duty_cycle, duty_cycle_raw, 0.05f);

    m->slope = slope_estimate(&m->slope_data, m->torque, m->board_speed, m->accel_smooth);

    m->speed_last = speed_corrected;
    m->debug = erpm_confidence;
}
