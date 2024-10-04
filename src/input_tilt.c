// Copyright 2022 Dado Mista
// Copyright 2024 Lukas Hrazky
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
#include "utils.h"

#include <math.h>

void input_tilt_reset(InputTilt *data) {
    // TODO init to remote throttle?
    data->interpolated = 0.0f;
    data->tmp = 0.0f;
}

void input_tilt_configure(InputTilt *data, const CfgInputTilt *cfg, float dt) {
    // data->tilt_alpha = half_time_to_alpha(cfg->filter, dt);
    data->tilt_alpha = half_time_to_alpha_iir2(cfg->filter, dt);
    data->tilt_step = cfg->speed_limit * dt;
}

static float dead_band(float value, float threshold) {
    if (fabsf(value) < threshold) {
        return 0.0f;
    }
    return sign(value) * (fabsf(value) - threshold) / (1.0f - threshold);
}

void input_tilt_update(InputTilt *data, const CfgInputTilt *cfg, const RemoteData *remote) {
    float target = remote->throttle * cfg->angle_limit;
    target = dead_band(target, cfg->threshold);

    // filter_ema_clamp(&data->interpolated, target, data->tilt_alpha, data->tilt_step);
    filter_iir2_clamp(&data->interpolated, &data->tmp, target, data->tilt_alpha, data->tilt_step);
}
