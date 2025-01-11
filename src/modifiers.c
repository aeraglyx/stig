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

#include <math.h>

void modifiers_configure(Modifiers *mod, const CfgTune *cfg, float dt) {
    gaussian_configure(&mod->filter, cfg->modifiers.filter);

    mod->winddown_alpha = half_time_to_alpha(cfg->traction.mod_winddown, dt);
    mod->fusion_alpha = half_time_to_alpha(cfg->torque_tilt.filter, dt);
}

void modifiers_reset(Modifiers *mod, float alpha) {
    gaussian_reset(&mod->filter, 0.0f, 0.0f);
    filter_ema(&mod->accel_offset_smooth, 0.0f, alpha);
}

static float get_boost(float boost, float fast_boi) {
    return 1.0f + fast_boi * (boost - 1.0f);
}

static float atr_tilt(const CfgAtr *cfg, const MotorData *mot) {
    if (!cfg->enabled) {
        return 0.0f;
    }

    bool uphill = sign(mot->slope_data.slope) == sign(mot->board_speed);
    float strength = uphill ? cfg->strength_up : cfg->strength_down;
    // float strength_boost = powf(cfg->strength_boost, mot->fast_boi);
    float strength_boost = get_boost(cfg->strength_boost, mot->fast_boi);

    float target = dead_zone(mot->slope_data.slope, cfg->threshold);
    target *= strength * strength_boost;
    target = clamp_sym(target, cfg->angle_limit);

    return target;
}

static float torque_tilt(const CfgTorqueTilt *cfg, const MotorData *mot, float yaw_rate, Modifiers *mod) {
    if (!cfg->enabled) {
        return 0.0f;
    }

    // TODO test using above/below "setpoint"
    float torque = mot->torque;
    float ratio = mot->slope_data.k_accel / mot->slope_data.k_drive;
    float torque_based_on_accel = mot->accel_smooth * ratio;

    float accel_offset = torque_based_on_accel - torque;
    filter_ema(&mod->accel_offset_smooth, accel_offset, mod->fusion_alpha);
    
    float strength = mot->braking ? cfg->strength_regen : cfg->strength;
    // float strength_boost = powf(cfg->strength_boost, mot->fast_boi);
    float strength_boost = get_boost(cfg->strength_boost, mot->fast_boi);
    float turn_boost = 1.0f + fabsf(yaw_rate) * cfg->turn_boost * 0.00125f;

    // TODO feed forward resistances
    float target = torque + mod->accel_offset_smooth * cfg->method;
    target = dead_zone(target, cfg->threshold);
    target *= strength * strength_boost * turn_boost;
    target = clamp_sym(target, cfg->angle_limit);

    return target;
}

static float speed_tilt(const CfgSpeedTilt *cfg, float speed) {
    // TODO more user friendly low speed and high speed setting

    float constant_lerp = clamp_sym(speed * 2.0f, 1.0f);
    float constant = constant_lerp * cfg->constant;
    float variable = speed * cfg->variable;

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
    float dt
) {
    // TODO we only need one thing from IMUData

    mod->target = 0.0f;

    mod->target += atr_tilt(&cfg->atr, motor);
    mod->target += torque_tilt(&cfg->torque_tilt, motor, imu->gyro[2], mod);
    mod->target += speed_tilt(&cfg->speed_tilt, motor->board_speed);

    float confidence = motor->traction.confidence_soft;
    gaussian_update(&mod->filter, mod->target, dt * confidence);

    // modifiers_reset(mod, mod->winddown_alpha * (1.0f - confidence));
    float traction_mult = 1.0f - mod->winddown_alpha * (1.0f - confidence);
    mod->filter.value *= traction_mult;
    mod->filter.speed *= traction_mult;
    mod->filter.accel *= traction_mult;
    mod->accel_offset_smooth *= traction_mult;
}
