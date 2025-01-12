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
    data->pitch = 0.0f;
    data->roll = 0.0f;

    data->pitch_tmp = 0.0f;
    data->roll_tmp = 0.0f;

    data->ax = 0.0f;
    data->ay = 0.0f;
    data->az = 0.0f;

    data->gy = 0.0f;
    data->gz = 0.0f;

    data->ax_tmp = 0.0f;
    data->ay_tmp = 0.0f;
    data->az_tmp = 0.0f;

    data->gy_tmp = 0.0f;
    data->gz_tmp = 0.0f;

    data->gy_last = 0.0f;
    data->gz_last = 0.0f;
}

void imu_data_configure(IMUData *data, const CfgTraction *cfg, float x_offset, float dt) {
    data->alpha = half_time_to_alpha_iir2(cfg->filter, dt);
    data->x_offset = x_offset;
    data->frequency = 1.0f / dt;
}

void acceleration_correction(IMUData *data, float *ax, float *ay, float *az) {
    // Acceleration is affected by centripetal force and angular acceleration.
    // For reference, IMU 0.2 m away from the axle would yield:
    //     0.06 g at 100 °/s from centripetal force (but it's quadratic)
    //     0.18 g at 500 °/s2 from angular acceleration

    // Briefly tested on an ADV, other controllers / orientations might not work
    // But is it even worth correcting for like 0.1 g here and there?

    float gy_sq = data->gy * data->gy;
    float gz_sq = data->gz * data->gz;
    float ang_vel_sq = gy_sq + gz_sq;

    // TODO div by dt instead?
    float ang_acc_y = (data->gy - data->gy_last) * data->frequency;
    float ang_acc_z = (data->gz - data->gz_last) * data->frequency;

    float ax_correction = data->x_offset * ang_vel_sq;
    float ay_correction = data->x_offset * ang_acc_z;
    float az_correction = data->x_offset * ang_acc_y;
    
    *ax -= mps2_to_g(ax_correction);
    *ay += mps2_to_g(ay_correction);
    *az -= mps2_to_g(az_correction);

    data->gy_last = data->gy;
    data->gz_last = data->gz;
}

static float magnitude(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

void imu_data_update(IMUData *data, BalanceFilterData *balance_filter) {
    // data->pitch = rad2deg(VESC_IF->imu_get_pitch());
    // data->roll = rad2deg(VESC_IF->imu_get_roll());

    // TODO do I need this filtering?
    filter_iir2(&data->pitch, &data->pitch_tmp, rad2deg(VESC_IF->imu_get_pitch()), data->alpha);
    filter_iir2(&data->roll, &data->roll_tmp, rad2deg(VESC_IF->imu_get_roll()), data->alpha);

    VESC_IF->imu_get_gyro(data->gyro);
    VESC_IF->imu_get_accel(data->accel);

    // TODO separate things only needed when running
    data->pitch_balance = rad2deg(balance_filter_get_pitch(balance_filter));

    filter_iir2(&data->ax, &data->ax_tmp, data->accel[0], data->alpha);
    filter_iir2(&data->ay, &data->ay_tmp, data->accel[1], data->alpha);
    filter_iir2(&data->az, &data->az_tmp, data->accel[2], data->alpha);

    filter_iir2(&data->gy, &data->gy_tmp, deg2rad(data->gyro[1]), data->alpha);
    filter_iir2(&data->gz, &data->gz_tmp, deg2rad(data->gyro[2]), data->alpha);

    // TODO move corrected acceleration to IMUData for comparison
    float ax = data->ax;
    float ay = data->ay;
    float az = data->az;

    if (data->x_offset != 0.0f) {
        acceleration_correction(data, &ax, &ay, &az);
    }

    data->accel_mag = magnitude(ax, ay, az);
    // data->accel_mag = magnitude(ax, ay, fmaxf(az, 0.0f));
    // for drop, negative Z accel is permitted (pushing down)

    // float pitch_rad = VESC_IF->imu_get_pitch();
    float pitch_rad = deg2rad(data->pitch);
    float imu_accel_x = ax * cosf(pitch_rad) + az * sinf(pitch_rad);
    // float imu_accel_x = ax + az * pitch;  // approximation

    // TODO use smooth slope
    // float slope = deg2rad(mot->slope);
    // float imu_accel_long = imu_accel_x / cosf(slope);
    // float imu_accel_long = imu_accel_x * sec_approx(slope);
    // float imu_accel_long = imu_accel_x;  // ignoring slope
    data->board_accel = - imu_accel_x;
}
