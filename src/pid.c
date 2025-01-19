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
    pid->p_alpha = half_time_to_alpha(cfg->kp_filter, dt);
    pid->d_alpha = half_time_to_alpha(cfg->kd_filter, dt);
    pid->ki = cfg->ki * dt;
    pid->soft_start_step_size = dt / max(cfg->soft_start, dt);
}

void pid_reset(PID *pid, const IMUData *imu, float alpha) {
    pid->pid_value = 0.0f;

    pid->proportional = 0.0f;
    filter_ema(&pid->integral, 0.0f, alpha);
    pid->derivative = 0.0f;

    pid->p_input = imu->pitch_balance;
    pid->d_input = -imu->gyro[1];

    filter_ema(&pid->kp_scale, 1.0f, alpha);
    filter_ema(&pid->kd_scale, 1.0f, alpha);

    pid->soft_start_factor = 0.0f;
}

static void p_update(PID *pid, const CfgPid *cfg, float pitch_offset, int8_t direction, float speed_factor) {
    // TODO only when kp_brake_scale != 1?
    // TODO refactor scaling into a function?
    float kp_brake_scale = 1.0f;
    if (sign(pitch_offset) != direction) {
        kp_brake_scale = 1.0f + (cfg->kp_brake - 1.0f) * speed_factor;
    }
    filter_ema(&pid->kp_scale, kp_brake_scale, 0.01f);

    pid->proportional = pitch_offset * cfg->kp;

    float pitch_offset_2 = pitch_offset * fabsf(pitch_offset);
    float pitch_offset_3 = pitch_offset * pitch_offset * pitch_offset;

    pid->proportional += pitch_offset_2 * cfg->kp_2nd_order;
    pid->proportional += pitch_offset_3 * cfg->kp_3rd_order;

    pid->proportional *= pid->kp_scale;
}

static void i_update(PID *pid, const CfgPid *cfg, float pitch_offset, float confidence) {
    // TODO slowly winddown i term in wheelslip
    // TODO use dt
    pid->integral += pitch_offset * pid->ki * confidence;

    if (cfg->ki_limit > 0.0f && fabsf(pid->integral) > cfg->ki_limit) {
        pid->integral = cfg->ki_limit * sign(pid->integral);
    }
}

static void d_update(PID *pid, const CfgPid *cfg, float d_input, int8_t direction, float speed_factor) {
    float kd_brake_scale = 1.0f;
    if (sign(d_input) != direction) {
        kd_brake_scale = 1.0f + (cfg->kd_brake - 1.0f) * speed_factor;
    }
    filter_ema(&pid->kd_scale, kd_brake_scale, 0.01f);

    pid->derivative = d_input * cfg->kd;

    float d_input_2 = d_input * fabsf(d_input);
    float d_input_3 = d_input * d_input * d_input;

    pid->derivative += d_input_2 * cfg->kd_2nd_order * 0.01f;
    pid->derivative += d_input_3 * cfg->kd_3rd_order * 0.0001f;

    pid->derivative *= pid->kd_scale;
}

static void f_update(PID *pid, const CfgPid *cfg, float setpoint_speed) {
    pid->feed_forward = setpoint_speed * cfg->kf;
}

void pid_update(
    PID *pid,
    const IMUData *imu,
    const MotorData *mot,
    const CfgPid *cfg,
    float setpoint,
    float setpoint_speed
) {
    filter_ema(&pid->p_input, imu->pitch_balance, pid->p_alpha); 
    filter_ema(&pid->d_input, -imu->gyro[1], pid->d_alpha); 

    float speed_factor = clamp(fabsf(mot->board_speed), 0.0f, 1.0f);
    float pitch_offset = setpoint - pid->p_input;
    int8_t direction = sign(mot->board_speed);

    p_update(pid, cfg, pitch_offset, direction, speed_factor);
    i_update(pid, cfg, pitch_offset, mot->traction.confidence);
    d_update(pid, cfg, pid->d_input, direction, speed_factor);
    f_update(pid, cfg, setpoint_speed);
    
    float pid_sum = pid->proportional + pid->integral + pid->derivative + pid->feed_forward;

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

    pid->pid_value = new_pid_value;
}
