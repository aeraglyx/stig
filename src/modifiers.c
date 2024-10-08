// Copyright 2022 Dado Mista
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

#include "modifiers.h"

#include "utils.h"

#include <math.h>

void modifiers_configure(Modifiers *mod, const CfgTune *cfg, float dt) {
    mod->winddown_alpha = half_time_to_alpha(cfg->traction.mod_winddown, dt);
    // mod->tilt_alpha = half_time_to_alpha(cfg->modifiers.filter, dt);
    mod->tilt_alpha = half_time_to_alpha_iir2(cfg->modifiers.filter, dt);
    mod->tilt_step = cfg->modifiers.speed_limit * dt;
    mod->fusion_alpha = half_time_to_alpha(cfg->torque_tilt.filter, dt);
}

void modifiers_reset(Modifiers *mod, float alpha) {
    filter_ema(&mod->interpolated, 0.0f, alpha);
    filter_ema(&mod->tmp, 0.0f, alpha);
    filter_ema(&mod->accel_offset_smooth, 0.0f, alpha);
}

static float get_boost(float boost, float fast_boi) {
    return 1.0f + fast_boi * (boost - 1.0f);
}

static float atr_tilt(const CfgAtr *cfg, const MotorData *mot) {
    const bool uphill = sign(mot->slope) == sign(mot->board_speed);
    const float strength = uphill ? cfg->strength_up : cfg->strength_down;
    // const float strength_boost = powf(cfg->strength_boost, mot->fast_boi);
    const float strength_boost = get_boost(cfg->strength_boost, mot->fast_boi);

    float target = dead_zone(mot->slope, cfg->threshold);
    target *= strength * strength_boost;
    target = clamp_sym(target, cfg->angle_limit);

    return target;
}

static float torque_tilt(const CfgTorqueTilt *cfg, const MotorData *mot, float yaw_rate, Modifiers *mod) {
    // TODO test using above/below "setpoint"
    const float torque = mot->torque;
    const float ratio = mot->slope_data.k_accel / mot->slope_data.k_drive;
    const float torque_based_on_accel = mot->accel_smooth * ratio;

    const float accel_offset = torque_based_on_accel - torque;
    filter_ema(&mod->accel_offset_smooth, accel_offset, mod->fusion_alpha);
    
    const float strength = mot->braking ? cfg->strength_regen : cfg->strength;
    // const float strength_boost = powf(cfg->strength_boost, mot->fast_boi);
    const float strength_boost = get_boost(cfg->strength_boost, mot->fast_boi);
    const float turn_boost = 1.0f + fabsf(yaw_rate) * cfg->turn_boost * 0.00125f;

    // TODO feed forward resistances
    float target = torque + mod->accel_offset_smooth * cfg->method;
    target = dead_zone(target, cfg->threshold);
    target *= strength * strength_boost * turn_boost;
    target = clamp_sym(target, cfg->angle_limit);

    return target;
}

static float speed_tilt(const CfgSpeedTilt *cfg, float speed) {
    // TODO more user friendly low speed and high speed setting

    const float constant_lerp = clamp_sym(speed * 2.0f, 1.0f);
    const float constant = constant_lerp * cfg->constant;
    const float variable = speed * cfg->variable;

    return constant + variable;
}

// static float turn_tilt_update(const CfgTurnTilt *cfg, const MotorData *mot, float yaw_rate) {
//     float speed_boost = powf(cfg->strength_boost, mot->fast_boi);
//     float start_speed = fmaxf(cfg->start_erpm, 10) * 0.001f;
//     float direction = clamp_sym(mot->board_speed / start_speed, 1.0f);

//     float target = fabsf(yaw_rate) * cfg->strength * 0.00125;
//     target *= speed_boost;
//     target *= direction;
//     target = clamp_sym(tt->target, cfg->angle_limit);
// }

void modifiers_update(
    Modifiers *mod,
    const CfgTune *cfg,
    const MotorData *motor,
    const IMUData *imu,
    float confidence
) {
    // TODO we only need one thing from IMUData

    float target = 0.0f;

    target += atr_tilt(&cfg->atr, motor);
    target += torque_tilt(&cfg->torque_tilt, motor, imu->gyro[2], mod);
    target += speed_tilt(&cfg->speed_tilt, motor->board_speed);
    // TODO turn tilt

    target = clamp_sym(target, cfg->modifiers.angle_limit);
    mod->target = target;

    float interpolated_last = mod->interpolated;

    float alpha = mod->tilt_alpha * confidence;
    // filter_ema_clamp(&mod->interpolated, target, alpha, mod->tilt_step);
    filter_iir2_clamp(&mod->interpolated, &mod->tmp, target, alpha, mod->tilt_step);
    modifiers_reset(mod, mod->winddown_alpha * (1.0f - confidence));

    mod->speed = (mod->interpolated - interpolated_last) * 800.0f;  // TODO freq
}
