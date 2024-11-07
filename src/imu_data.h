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

#include "balance_filter.h"
#include "conf/datatypes.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float pitch, pitch_tmp;
    float roll, roll_tmp;

    float gyro[3];
    float accel[3];

    float pitch_balance;

    float alpha;
    float frequency;
    float x_offset;

    // filtered data
    float ax, ax_tmp;
    float ay, ay_tmp;
    float az, az_tmp;

    float gy, gy_tmp;
    float gz, gz_tmp;

    // derivative data
    float gy_last;
    float gz_last;

    // output data
    float accel_mag;
    float board_accel;
} IMUData;

void imu_data_init(IMUData *data);

void imu_data_configure(IMUData *data, const CfgTraction *cfg, float x_offset, float dt);

void imu_data_update(IMUData *data, BalanceFilterData *balance_filter);
