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

#include "remote_data.h"

#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

// void remote_data_init(RemoteData *data) {
// }

// void remote_data_configure(RemoteData *data, const CfgInputTilt *cfg, float dt) {
//     // TODO store a flag whether we should update remote data?
// }

void remote_data_update(RemoteData *data, const CfgHwRemote *cfg) {
    data->connected = false;
    data->throttle = 0.0f;

    switch (cfg->type) {
        case (INPUTTILT_PPM):
            data->throttle = VESC_IF->get_ppm();
            data->connected = VESC_IF->get_ppm_age() < 1;
            break;
        case (INPUTTILT_UART): {
            remote_state remote = VESC_IF->get_remote_state();
            data->throttle = remote.js_y;
            data->connected = remote.age_s < 1;
            break;
        }
        case (INPUTTILT_NONE):
            break;
    }

    // TODO wind down when not connected?

    if (cfg->invert_throttle) {
        data->throttle *= -1.0f;
    }
}
