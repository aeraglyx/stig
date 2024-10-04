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

#include <math.h>

void slope_configure(SlopeData *data, const CfgHardware *hw, const CfgRider *rider) {
    const float m = hw->mass + rider->mass;
    const float g = 9.807f;  // earth gravity
    const float r = 0.145f;  // TODO get wheel radius
    const float rho = 1.225f;  // air density, should be a function of temperature
    const float c_drag = rider->drag_coefficient;
    const float area = 0.025f + 0.02f * powf(rider->mass, 0.75f);

    data->k_drive = 1.0f / (r * m * g);
    data->k_drag = 0.5f * rho * c_drag * area / (m * g);
    data->k_roll = hw->motor.rolling_resistance;
    data->k_accel = hw->motor.acceleration_resistance;
}

float slope_estimate(SlopeData *data, float torque, float speed, float accel) {
    // torque in [Nm], speed in [m/s], accel in [g]

    // accelerations in [g] produced by the respective forces
    const float a_drive = data->k_drive * torque;
    const float a_drag = data->k_drag * speed * fabsf(speed);
    const float a_roll = data->k_roll * clamp_sym(4.0f * speed, 1.0f);
    const float a_accel = data->k_accel * accel;

    const float x = a_drive - a_drag - a_roll - a_accel;
    return rad2deg(asin_approx(x));
}
