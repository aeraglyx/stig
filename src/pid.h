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

#pragma once

#include "motor_data.h"
#include "imu_data.h"

#include "conf/datatypes.h"

typedef struct {
    float pid_value;

    float proportional;
    float integral;
    float derivative;
    float feed_forward;

    float d_alpha;
    float d_filtered_input;

    float kp_scale;
    float kd_scale;

    float ki;

    float soft_start_step_size;
    float soft_start_factor;
} PID;

void pid_configure(PID *pid, const CfgPid *cfg, float dt);

void pid_reset(PID *pid, float alpha);

void pid_update(
    PID *pid,
    const IMUData *imu,
    const MotorData *mot,
    const CfgPid *cfg,
    float setpoint,
    float setpoint_speed
);
