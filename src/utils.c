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

#include "utils.h"

#include <math.h>

uint32_t rnd(uint32_t seed) {
    return seed * 1664525u + 1013904223u;
}


float asin_approx(float x) {
    return x + 0.18f * x * x * x;
}

float sec_approx(float x) {
    return 1.0f + 0.52f * x * x;
}

float bell_curve(float x) {
    // gaussian approximation
    // 1 at 0 and 0.5 at ± 0.5

    // sharper center, fatter tails
    // return 1.0f / (1.0f + x * x);

    // better, still not perfect
    float x2 = x * x;
    float x4 = x2 * x2;
    return 1.0f / (1.0f + 0.5 * x2 + 0.5 * x4);
    
    // very close fit
    // float x2 = x * x;
    // float x4 = x2 * x2;
    // float x8 = x4 * x4;
    // return 1.0f / (1.0f + 0.66f * x2 + 0.31f * x4 + 0.03f * x8);
}

float sin_scaled(float t) {
    // based on Bhaskara I's sine approximation
    // valid for 0 <= t <= 1, outputs -1 to 1
    const bool second_half = t > 0.5f;
    const float x = (second_half) ? t - 0.5f : t;
    float sin = 20.0f / (16.0f * x * x - 8.0f * x + 5.0f) - 4.0f;
    if (second_half) {
        sin *= -1.0f;
    }
    return sin;
}

float magnitude_3d(float x, float y, float z) {
    return sqrtf(x * x + y * y + z * z);
}

float magnitude_4d(float w, float x, float y, float z) {
    return sqrtf(w * w + x * x + y * y + z * z);
}

float g_to_mps2(float x) {
    return 9.806650f * x;
}

float mps2_to_g(float x) {
    return 0.101972f * x;
}


float clamp(float value, float min, float max) {
    const float m = value < min ? min : value;
    return m > max ? max : m;
}

float clamp_01(float value) {
    return clamp(value, 0.0f, 1.0f);
}

float clamp_sym(float value, float limit) {
    return clamp(value, -limit, limit);
}

float remap_to_01(float value, float a, float b) {
    return clamp_01((value - a) / (b - a));
}


float dead_zone(float value, float threshold) {
    return fmaxf(fabsf(value) - threshold, 0.0f) * sign(value);
}

void rate_limitf(float *value, float target, float step) {
    // if (fabsf(target - *value) < step) {
    //     *value = target;
    // } else if (target - *value > 0) {
    //     *value += step;
    // } else {
    //     *value -= step;
    // }
    *value += clamp_sym(target - *value, step);
}

float half_time_to_alpha(float half_time, float dt) {
    if (half_time < 0.001f) {
        return 1.0f;
    }
    return 1.0f - exp2f(-dt / half_time);
}

float half_time_to_alpha_fast(float half_time, float dt) {
    // approximation only valid for small dt
    if (half_time < 0.69f * dt) {
        return 1.0f;
    }
    return 0.69f * dt / half_time;
}

float half_time_to_alpha_iir2(float half_time, float dt) {
    return half_time_to_alpha(0.414f * half_time, dt);
}

float half_time_to_alpha_iir3(float half_time, float dt) {
    return half_time_to_alpha(0.261f * half_time, dt);
}

void filter_ema(float *out, float target, float alpha) {
    *out += alpha * (target - *out);
}

void filter_ema_clamp(float *out, float target, float alpha, float step) {
    *out += clamp_sym(alpha * (target - *out), step);
}

void filter_iir2(float *out, float *tmp, float target, float alpha) {
    *tmp += alpha * (target - *tmp);
    *out += alpha * (*tmp - *out);
}

void filter_iir2_clamp(float *out, float *tmp, float target, float alpha, float step) {
    *tmp += alpha * (target - *tmp);
    *out += clamp_sym(alpha * (*tmp - *out), step);
}

void filter_iir3(float *out, float *tmp1, float *tmp2, float target, float alpha) {
    *tmp1 += alpha * (target - *tmp1);
    *tmp2 += alpha * (*tmp1 - *tmp2);
    *out += alpha * (*tmp2 - *out);
}




void gaussian_configure(GaussianFilter *filter, float half_time) {
    // TODO div by 0
    filter->k = 0.693f / half_time;
}

void gaussian_reset(GaussianFilter *filter, float value, float speed) {
    filter->value = value;
    filter->speed = speed;
    filter->accel = 0.0f;
}

void gaussian_update(GaussianFilter *filter, float target, float dt) {
    // Coefficients chosen to approximate Gaussian filter and preserve half time

    float speed = 1.34f * filter->k * (target - filter->value);
    float accel = 3.62f * filter->k * (speed - filter->speed);
    float jerk = 9.77f * filter->k * (accel - filter->accel);

    filter->accel += dt * jerk;
    filter->speed += dt * filter->accel;
    filter->value += dt * filter->speed;
}




// float smoothstep(float x) {
//     return x * x * (3.0f - 2.0f * x);
// }

// float sigmoid(float x, float radius) {
//     return tanhf(x / radius);
// }
