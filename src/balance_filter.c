// Copyright 2023 - 2024 Lukas Hrazky
// Copyright 2025 Vladislav Macicek
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

//=====================================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date         Author          Notes
// 29/09/2011   SOH Madgwick    Initial release
// 02/10/2011   SOH Madgwick    Optimized for reduced CPU load
// 26/01/2014   Benjamin V      Adaption to our platform
// 20/02/2017   Benjamin V      Added Madgwick algorithm and refactoring
// 17/09/2023   Lukas Hrazky    Adopted from vedderb/bldc, modified for self-balancing skateboard
// 01/02/2025   Vladislav M     Further self-balancing skateboard modifications
//
//=====================================================================================================

#include "balance_filter.h"
#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

static float calculate_acc_confidence(float new_acc_mag, BalanceFilterData *data) {
    // G.K. Egan (C) computes confidence in accelerometers when
    // aircraft is being accelerated over and above that due to gravity
    if (data->acc_confidence_decay == 0.0f) {
        return 1.0f;
    }

    data->acc_mag += 0.2f * (new_acc_mag - data->acc_mag);
    return bell_curve((data->acc_mag - 1.0f) * data->acc_confidence_decay);
}

void balance_filter_init(BalanceFilterData *data) {
    // Init with internal filter orientation, otherwise the AHRS would need a while to stabilize
    float quat[4];
    VESC_IF->imu_get_quaternions(quat);
    data->q0 = quat[0];
    data->q1 = quat[1];
    data->q2 = quat[2];
    data->q3 = quat[3];

    data->az_filtered = 1.0f;
    data->az_filtered_tmp = 1.0f;

    data->acc_mag = 1.0f;
    data->kp_mult = 1.0f;
}

void balance_filter_configure(BalanceFilterData *data, const CfgBalanceFilter *cfg) {
    data->acc_confidence_decay = cfg->accel_confidence_decay;

    data->kp_pitch = cfg->mahony_kp;
    data->kp_roll = cfg->mahony_kp_roll;
    data->kp_yaw = 0.5f * (cfg->mahony_kp + cfg->mahony_kp_roll);

    data->az_filter = cfg->az_filter;
}

void balance_filter_update(BalanceFilterData *data, float *gyro_xyz, float *accel_xyz, float dt) {
    float gx = gyro_xyz[0];
    float gy = gyro_xyz[1];
    float gz = gyro_xyz[2];

    float ax = accel_xyz[0];
    float ay = accel_xyz[1];
    float az = accel_xyz[2];

    // TODO test if 1st order IIR is enough
    float alpha_z = half_time_to_alpha_fast(0.414f * data->az_filter, dt);
    filter_iir2(&data->az_filtered, &data->az_filtered_tmp, az, alpha_z);
    az = data->az_filtered;

    float accel_norm = magnitude_3d(ax, ay, az);

    // Compute feedback only if accelerometer magnitude is not too small
    // to avoid a division by a small number
    if (accel_norm > 0.01f) {
        float accel_confidence = calculate_acc_confidence(accel_norm, data);
        accel_confidence *= data->kp_mult;

        float two_kp_pitch = 2.0f * data->kp_pitch * accel_confidence;
        float two_kp_roll = 2.0f * data->kp_roll * accel_confidence;
        float two_kp_yaw = 2.0f * data->kp_yaw * accel_confidence;

        // Normalize accelerometer measurement
        float recip_norm = 1.0f / accel_norm;
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // Estimated direction of gravity
        float half_vector_x = data->q1 * data->q3 - data->q0 * data->q2;
        float half_vector_y = data->q0 * data->q1 + data->q2 * data->q3;
        float half_vector_z = data->q0 * data->q0 - 0.5f + data->q3 * data->q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        float half_error_x = ay * half_vector_z - az * half_vector_y;
        float half_error_y = az * half_vector_x - ax * half_vector_z;
        float half_error_z = ax * half_vector_y - ay * half_vector_x;

        // Apply proportional feedback
        gx += two_kp_roll * half_error_x;
        gy += two_kp_pitch * half_error_y;
        gz += two_kp_yaw * half_error_z;
    }

    // Integrate rate of change of quaternion
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    float qa = data->q0;
    float qb = data->q1;
    float qc = data->q2;
    float qd = data->q3;
    data->q0 += -qb * gx - qc * gy - qd * gz;
    data->q1 += qa * gx + qc * gz - qd * gy;
    data->q2 += qa * gy - qb * gz + qd * gx;
    data->q3 += qa * gz + qb * gy - qc * gx;

    // Normalize quaternion
    float recip_norm = 1.0f / magnitude_4d(data->q0, data->q1, data->q2, data->q3);
    data->q0 *= recip_norm;
    data->q1 *= recip_norm;
    data->q2 *= recip_norm;
    data->q3 *= recip_norm;
}

float balance_filter_get_roll(BalanceFilterData *data) {
    const float q0 = data->q0;
    const float q1 = data->q1;
    const float q2 = data->q2;
    const float q3 = data->q3;

    return -atan2f(q0 * q1 + q2 * q3, 0.5f - (q1 * q1 + q2 * q2));
}

float balance_filter_get_pitch(BalanceFilterData *data) {
    float sin = -2.0f * (data->q1 * data->q3 - data->q0 * data->q2);

    if (sin < -1.0f) {
        return -M_PI / 2.0f;
    } else if (sin > 1.0f) {
        return M_PI / 2.0f;
    }

    return asinf(sin);
}

float balance_filter_get_yaw(BalanceFilterData *data) {
    const float q0 = data->q0;
    const float q1 = data->q1;
    const float q2 = data->q2;
    const float q3 = data->q3;

    return -atan2f(q0 * q3 + q1 * q2, 0.5f - (q2 * q2 + q3 * q3));
}
