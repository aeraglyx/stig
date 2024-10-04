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

#include "imu_data.h"
#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

void imu_data_init(IMUData *data) {
    data->ax = 0.0f;
    data->ay = 0.0f;
    data->az = 0.0f;

    data->gy = 0.0f;
    data->gz = 0.0f;

    data->gy_last = 0.0f;
    data->gz_last = 0.0f;
}

void imu_data_configure(IMUData *data, const CfgTraction *cfg, float x_offset, float dt) {
    data->imu_alpha = half_time_to_alpha(cfg->filter, dt);
    data->x_offset = x_offset;
    data->frequency = 1.0f / dt;
}

static float magnitude(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

void imu_data_update(IMUData *data, BalanceFilterData *balance_filter) {
    data->pitch = rad2deg(VESC_IF->imu_get_pitch());
    data->roll = rad2deg(VESC_IF->imu_get_roll());
    data->yaw = rad2deg(VESC_IF->imu_get_yaw());

    VESC_IF->imu_get_gyro(data->gyro);
    VESC_IF->imu_get_accel(data->accel);

    // TODO separate things only needed when running
    data->pitch_balance = rad2deg(balance_filter_get_pitch(balance_filter));

    // pre-filter accelerometer and gyroscope
    filter_ema(&data->ax, data->accel[0], data->imu_alpha);
    filter_ema(&data->ay, data->accel[1], data->imu_alpha);
    filter_ema(&data->az, data->accel[2], data->imu_alpha);

    filter_ema(&data->gy, deg2rad(data->gyro[1]), data->imu_alpha);
    filter_ema(&data->gz, deg2rad(data->gyro[2]), data->imu_alpha);

    // Acceleration is affected by centripetal force and angular acceleration.
    // For reference, IMU 0.2 m away from the axle would yield:
    //     0.06 g at 100 °/s from centripetal force (but it's quadratic)
    //     0.18 g at 500 °/s2 from angular acceleration

    // Briefly tested on an ADV, other controllers / orientations might not work
    // But is it even worth correcting for like 0.1 g here and there?

    // TODO refactor imu correction to its own function

    const float gy_sq = data->gy * data->gy;
    const float gz_sq = data->gz * data->gz;
    const float ang_vel_sq = gy_sq + gz_sq;

    const float ang_acc_y = (data->gy - data->gy_last) * data->frequency;
    const float ang_acc_z = (data->gz - data->gz_last) * data->frequency;

    const float ax_correction = data->x_offset * ang_vel_sq;
    const float ay_correction = data->x_offset * ang_acc_z;
    const float az_correction = data->x_offset * ang_acc_y;
    
    const float ax = data->ax - mps2_to_g(ax_correction);
    const float ay = data->ay + mps2_to_g(ay_correction);
    const float az = data->az - mps2_to_g(az_correction);

    data->gy_last = data->gy;
    data->gz_last = data->gz;

    data->accel_mag = magnitude(ax, ay, az);
    // data->accel_mag = magnitude(ax, ay, fmaxf(az, 0.0f));
    // for drop, negative Z accel is permitted (pushing down)

    const float pitch_rad = VESC_IF->imu_get_pitch();
    const float imu_accel_x = ax * cosf(pitch_rad) + az * sinf(pitch_rad);
    // const float imu_accel_x = ax + az * pitch;  // approximation

    // TODO use smooth slope
    // const float slope = deg2rad(mot->slope);
    // const float imu_accel_long = imu_accel_x / cosf(slope);
    // const float imu_accel_long = imu_accel_x * sec_approx(slope);
    // const float imu_accel_long = imu_accel_x;  // ignoring slope
    data->board_accel = - imu_accel_x;
}
