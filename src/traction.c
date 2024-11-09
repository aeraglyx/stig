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
#include <stdint.h>

void traction_init(Traction *data) {
    data->confidence_soft = 0.0f;
}

void traction_configure(Traction *data, const CfgTraction *cfg, float dt) {
    data->slip_sensitivity = cfg->slip_sensitivity;
    data->drop_sensitivity = cfg->drop_sensitivity;
    data->conf_sensitivity = cfg->conf_sensitivity;
    data->winddown_alpha = half_time_to_alpha(cfg->soft_release_winddown, dt);
}

void traction_update(Traction *data, float accel_diff, float accel_mag) {
    float drop_factor = bell_curve(accel_mag * data->drop_sensitivity);
    data->multiplier = bell_curve(accel_diff * data->slip_sensitivity * drop_factor);

    data->confidence = bell_curve(data->conf_sensitivity * accel_diff);
    if (data->confidence > data->confidence_soft) {
        data->confidence_soft = data->confidence;
    } else {
        filter_ema(&data->confidence_soft, data->confidence, data->winddown_alpha);
    }
}
