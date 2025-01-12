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

#include "traction.h"
#include "utils.h"

#include <math.h>

void traction_init(Traction *data) {
    data->multiplier = 1.0f;
    data->confidence = 0.0f;
}

void traction_configure(Traction *data, const CfgTraction *cfg) {
    data->slip_sensitivity = cfg->slip_sensitivity;
    data->drop_sensitivity = cfg->drop_sensitivity;
    data->conf_sensitivity = cfg->conf_sensitivity;
}

void traction_update(Traction *data, float accel_diff, float accel_mag) {
    float drop_factor = bell_curve(accel_mag * data->drop_sensitivity);
    data->multiplier = bell_curve(accel_diff * data->slip_sensitivity * drop_factor);
    data->confidence = bell_curve(accel_diff * data->conf_sensitivity);
}
