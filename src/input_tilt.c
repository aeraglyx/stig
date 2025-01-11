// Copyright 2022 Dado Mista
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

#include "input_tilt.h"

#include <math.h>

void input_tilt_configure(InputTilt *data, const CfgInputTilt *cfg) {
    gaussian_configure(&data->filter, cfg->filter);
}

void input_tilt_reset(InputTilt *data) {
    // TODO init to remote throttle?
    gaussian_reset(&data->filter, 0.0f, 0.0f);
}

static float dead_band(float value, float threshold) {
    if (fabsf(value) < threshold) {
        return 0.0f;
    }
    return sign(value) * (fabsf(value) - threshold) / (1.0f - threshold);
}

void input_tilt_update(
    InputTilt *data,
    const CfgInputTilt *cfg,
    const RemoteData *remote,
    float dt
) {
    float target = remote->throttle * cfg->angle_limit;
    target = dead_band(target, cfg->threshold);

    gaussian_update(&data->filter, target, dt);
}
