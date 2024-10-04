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
    data->traction_soft_release = 0.0f;
}

void traction_configure(Traction *data, const CfgTraction *cfg, float dt) {
    data->winddown_alpha = half_time_to_alpha(cfg->soft_release_winddown, dt);
}

void traction_update(Traction *data, const CfgTraction *cfg, const IMUData *imu, const MotorData *mot) {
    // SLIP
    const float accel_diff = mot->wheel_accel - imu->board_accel;
    const float slip_thr = cfg->wheelslip_threshold;
    data->slip_factor = remap_to_01(fabsf(accel_diff), slip_thr, 2.0f * slip_thr);
    const float slip_mult = 1.0f - (1.0f - cfg->wheelslip_strength) * data->slip_factor;

    // DROP
    // data->accel_mag = sqrtf(ax_sq + ay_sq + az_clamped_sq);
    const float drop_thr = cfg->drop_threshold;
    data->drop_factor = 1.0f - remap_to_01(imu->accel_mag, 0.5f * drop_thr, 1.5f * drop_thr);
    const float drop_mult = 1.0f - (1.0f - cfg->drop_strength) * data->drop_factor * data->slip_factor;
    // times wheelslip factor to get rid of false positives on steep dh

    // AMP MULTIPLIER
    data->multiplier = slip_mult * drop_mult;
    // data->multiplier = fminf(slip_mult, drop_mult);

    // MODIFIER WINDDOWN
    if (data->slip_factor > data->traction_soft_release) {
        data->traction_soft_release = data->slip_factor;
        // rate_limitf(&data->traction_soft_release, data->slip_factor, data->);
    } else {
        filter_ema(&data->traction_soft_release, data->slip_factor, data->winddown_alpha);
    }
}
