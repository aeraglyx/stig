// Copyright 2022 Dado Mista
// Copyright 2024 Lukas Hrazky
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

#include "pid.h"
#include "utils.h"

#include <math.h>
#include <stdint.h>

void pid_configure(PID *pid, const CfgPid *cfg, float dt) {
    pid->kd_alpha = half_time_to_alpha(cfg->kd_filter, dt);
    pid->ki = cfg->ki * dt;
    pid->soft_start_step_size = dt / max(cfg->soft_start, dt);
}

void pid_reset(PID *pid, const CfgPid *cfg, float alpha) {
    pid->pid_value = 0.0f;

    pid->proportional = 0.0f;
    filter_ema(&pid->integral, 0.0f, alpha);
    pid->derivative = 0.0f;

    pid->kd_filtered = 0.0f;

    filter_ema(&pid->kp_scale, cfg->kp, alpha);
    filter_ema(&pid->kd_scale, cfg->kd, alpha);

    pid->soft_start_factor = 0.0f;
}

static void p_update(PID *pid, const CfgPid *cfg, float pitch_offset, int8_t direction, float speed_factor) {
    float kp = cfg->kp;
    if (sign(pitch_offset) != direction) {
        // TODO only when kp_brake_scale != 0 ?
        float kp_brake_scale = 1.0f + (cfg->kp_brake - 1.0f) * speed_factor;
        kp *= kp_brake_scale;
    }
    filter_ema(&pid->kp_scale, kp, 0.01f);
    pid->proportional = pitch_offset * pid->kp_scale;
}

static void i_update(PID *pid, const CfgPid *cfg, float pitch_offset, float confidence) {
    // TODO slowly winddown i term in wheelslip
    pid->integral += pitch_offset * pid->ki * confidence;
    if (cfg->ki_limit > 0.0f && fabsf(pid->integral) > cfg->ki_limit) {
        pid->integral = cfg->ki_limit * sign(pid->integral);
    }
}

static void d_update(PID *pid, const CfgPid *cfg, float gyro_y, int8_t direction, float speed_factor) {
    filter_ema(&pid->kd_filtered, -gyro_y, pid->kd_alpha); 
    float kd_input = pid->kd_filtered;
    float kd = cfg->kd;
    if (sign(kd_input) != direction) {
        float kd_brake_scale = 1.0f + (cfg->kd_brake - 1.0f) * speed_factor;
        kd *= kd_brake_scale;
    }
    filter_ema(&pid->kd_scale, kd, 0.01f);
    pid->derivative = kd_input * pid->kd_scale;
}

// static void f_update(PID *pid, const CfgPid *cfg, float gyro_y, int8_t direction, float speed_factor) {
//     float speed = 
//     pid->feed_forward = speed * pid->kf_scale;
// }

void pid_update(
    PID *pid, const IMUData *imu, const MotorData *mot, const CfgPid *cfg, float setpoint
) {
    float speed_factor = clamp(fabsf(mot->board_speed), 0.0f, 1.0f);
    float pitch_offset = setpoint - imu->pitch_balance;
    int8_t direction = sign(mot->board_speed);

    p_update(pid, cfg, pitch_offset, direction, speed_factor);
    i_update(pid, cfg, pitch_offset, mot->traction.confidence_soft);
    d_update(pid, cfg, imu->gyro[1], direction, speed_factor);
    // TODO FEED FORWARD
    
    // float windup_input = pid->proportional
    // float current_limit = mot->braking ? mot->current_min : mot->current_max;
    // new_pid_value = clamp_sym(new_pid_value, current_limit);

    float pid_sum = pid->proportional + pid->integral + pid->derivative;

    // CURRENT LIMITING
    // TODO remove redundant limiting
    float current_limit = mot->braking ? mot->current_min : mot->current_max;
    float new_pid_value = clamp_sym(pid_sum, current_limit);
    // TODO soft max?

    // float i_overshoot = pid_sum - pid_sum_saturated;
    // pid->integral -= i_overshoot;

    // TODO speed boost

    // SOFT START
    // TODO move outside pid.c
    // after limiting, otherwise soft start wouldn't be effective with aggressive PIDs
    if (pid->soft_start_factor < 1.0f) {
        float factor_new = pid->soft_start_factor + pid->soft_start_step_size;
        pid->soft_start_factor = clamp(factor_new, 0.0f, 1.0f);
        new_pid_value *= pid->soft_start_factor;
    }

    filter_ema(&pid->pid_value, new_pid_value, 0.2f);
}
