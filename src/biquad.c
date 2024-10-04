// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#include "biquad.h"

#include <math.h>

void biquad_configure(Biquad *biquad, BiquadType type, float frequency) {
    float K = tanf(M_PI * frequency);
    float Q = 0.707f;  // maximum sharpness (0.5 = maximum smoothness)
    float norm = 1.0f / (1.0f + K / Q + K * K);
    if (type == BQ_LOWPASS) {
        biquad->a0 = K * K * norm;
        biquad->a1 = 2.0f * biquad->a0;
        biquad->a2 = biquad->a0;
    } else if (type == BQ_HIGHPASS) {
        biquad->a0 = 1.0f * norm;
        biquad->a1 = -2.0f * biquad->a0;
        biquad->a2 = biquad->a0;
    }
    biquad->b1 = 2.0f * (K * K - 1.0f) * norm;
    biquad->b2 = (1.0f - K / Q + K * K) * norm;
}

// void biquad_configure(Biquad *biquad, float half_time, float dt) {
//     // TODO check half time calculation
//     // float K = tanf(M_PI * dt * cutoff_freq);
//     float K = tanf(0.5f * dt / half_time);
//     float Q = 0.707f;  // critically damped
//     float norm = 1.0f / (1.0f + K / Q + K * K);

//     biquad->a0 = K * K * norm;
//     biquad->a1 = 2.0f * biquad->a0;
//     biquad->a2 = biquad->a0;

//     biquad->b1 = 2.0f * (K * K - 1.0f) * norm;
//     biquad->b2 = (1.0f - K / Q + K * K) * norm;
// }

void biquad_reset(Biquad *biquad) {
    biquad->z1 = 0.0f;
    biquad->z2 = 0.0f;
}

float biquad_process(Biquad *biquad, float in) {
    float out = in * biquad->a0 + biquad->z1;
    biquad->z1 = in * biquad->a1 + biquad->z2 - biquad->b1 * out;
    biquad->z2 = in * biquad->a2 - biquad->b2 * out;
    return out;
}
