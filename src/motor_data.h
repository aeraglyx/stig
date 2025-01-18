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

#pragma once

#include "conf/datatypes.h"
#include "slope_estimation.h"
#include "traction.h"
#include "imu_data.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float erpm, erpm_tmp;
    float erpm_gyro_ratio;
    float speed_erpm_ratio;

    float speed;
    float speed_abs;
    int8_t speed_sign;
    float fast_boi;
    float speed_last;

    float board_speed;

    float current, current_tmp;
    float current_smooth;
    bool braking;

    float c_torque;
    float torque;

    float duty_cycle;

    float wheel_accel, wheel_accel_tmp;
    float accel_final;
    AccelerationSource acceleration_source;
    bool use_erpm_correction;
    float board_accel;

    Traction traction;
    SlopeData slope_data;

    float data_filter_alpha;
    float mod_filter_alpha;
    float board_speed_alpha;

    float current_min;
    float current_max;
    float duty_max;

    float dt;
    float debug;
} MotorData;

void motor_data_init(MotorData *m);

void motor_data_configure(
    MotorData *m,
    const CfgTune *cfg,
    const CfgHardware *hw,
    const CfgRider *rider,
    float dt
);

void motor_data_update(MotorData *m, uint16_t frequency, const IMUData *imu);
