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

#include "slope_estimation.h"
#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

static float estimate_frontal_area(float rider_mass) {
    float rider_area = 0.02f * powf(rider_mass, 0.75f);
    float board_area = 0.03f;  // hard-coded for now
    return rider_area + board_area;
}

void slope_configure(SlopeData *data, const CfgHardware *hw, const CfgRider *rider) {
    float m = hw->mass + rider->mass;
    float g = 9.807f;  // earth gravity
    float r = 0.5f * VESC_IF->get_cfg_float(CFG_PARAM_si_wheel_diameter);
    float rho = 1.2f;  // air density
    float c_drag = rider->drag_coefficient;
    float area = estimate_frontal_area(rider->mass);

    data->k_drive = 1.0f / (r * m * g);
    data->k_drag = 0.5f * rho * c_drag * area / (m * g);
    data->k_roll = hw->motor.rolling_resistance;
    data->k_accel = hw->motor.acceleration_resistance;
}

void slope_update(SlopeData *data, float torque, float speed, float accel) {
    // torque in [Nm], speed in [m/s], accel in [g]

    // accelerations in [g] produced by the respective forces
    float a_drive = data->k_drive * torque;
    float a_drag = data->k_drag * speed * fabsf(speed);
    float a_roll = data->k_roll * clamp_sym(16.0f * speed, 1.0f);
    float a_accel = data->k_accel * accel;

    float x = a_drive - a_drag - a_roll - a_accel;
    data->slope = rad2deg(asin_approx(x));
}
