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

#pragma once

#include "vesc_c_if.h"

#include <stdint.h>

#define TAU 6.283185307179586f
// #define G 9.80665f

#define ERPM_MOVING_THRESHOLD 5.0f

#define unused(x) (void) (x)

#if defined(__GNUC__) && __GNUC__ < 9

#define log_msg(fmt, ...)                                                                          \
    do {                                                                                           \
        if (!VESC_IF->app_is_output_disabled()) {                                                  \
            float t = VESC_IF->system_time();                                                      \
            uint32_t decimals = (uint32_t) ((t - (uint32_t) t) * 1000000);                         \
            VESC_IF->printf("%d.%.6d [stig] " fmt, (uint32_t) t, decimals, ##__VA_ARGS__);      \
        }                                                                                          \
    } while (0)

#define log_error(fmt, ...) log_msg("Error: " fmt, ##__VA_ARGS__)

#else

#define log_msg(fmt, ...)                                                                          \
    do {                                                                                           \
        if (!VESC_IF->app_is_output_disabled()) {                                                  \
            float t = VESC_IF->system_time();                                                      \
            uint32_t decimals = (uint32_t) ((t - (uint32_t) t) * 1000000);                         \
            VESC_IF->printf(                                                                       \
                "%d.%.6d [stig] " fmt, (uint32_t) t, decimals __VA_OPT__(, ) __VA_ARGS__        \
            );                                                                                     \
        }                                                                                          \
    } while (0)

#define log_error(fmt, ...) log_msg("Error: " fmt __VA_OPT__(, ) __VA_ARGS__)

#endif

// Declaration for the SEMD_APP_DATA macro, definition needs to be in main.c.
void send_app_data_overflow_terminate();

/**
 * DRY macro to check the buffer didn't overflow and send the app data.
 */
#define SEND_APP_DATA(buffer, buf_size, ind)                                                       \
    do {                                                                                           \
        if (ind > buf_size) {                                                                      \
            log_error("%s: App data buffer overflow, terminating.", __func__);                     \
            /* terminate the main thread, the memory has just been corrupted by buffer overflow */ \
            send_app_data_overflow_terminate();                                                    \
        }                                                                                          \
        VESC_IF->send_app_data(buffer, ind);                                                       \
    } while (0)

#define sign(x) (((x) < 0) ? -1 : 1)

#define deg2rad(deg) ((deg) * (M_PI / 180.0f))
#define rad2deg(rad) ((rad) * (180.0f / M_PI))

#define min(a, b)                                                                                  \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _a < _b ? _a : _b;                                                                         \
    })

#define max(a, b)                                                                                  \
    ({                                                                                             \
        __typeof__(a) _a = (a);                                                                    \
        __typeof__(b) _b = (b);                                                                    \
        _a > _b ? _a : _b;                                                                         \
    })

// See @rate_limitf
#define rate_limit(value, target, step)                                                            \
    do {                                                                                           \
        if (abs(target - *value) < step) {                                                         \
            *value = target;                                                                       \
        } else if (target - *value > 0) {                                                          \
            *value += step;                                                                        \
        } else {                                                                                   \
            *value -= step;                                                                        \
        }                                                                                          \
    } while (0)

uint32_t rnd(uint32_t seed);

float asin_approx(float x);
float sec_approx(float x);
float bell_curve(float x);
float sin_scaled(float t);

float g_to_mps2(float x);
float mps2_to_g(float x);

float clamp(float value, float min, float max);
float clamp_01(float value);
float clamp_sym(float value, float limit);
float remap_to_01(float value, float a, float b);

float dead_zone(float value, float threshold);
void rate_limitf(float *value, float target, float step);

float half_time_to_alpha(float half_time, float dt);
float half_time_to_alpha_fast(float half_time, float dt);
float half_time_to_alpha_iir2(float half_time, float dt);
float half_time_to_alpha_iir3(float half_time, float dt);

void filter_ema(float *out, float target, float alpha);
void filter_ema_clamp(float *out, float target, float alpha, float step);

void filter_iir2(float *out, float *tmp, float target, float alpha);
void filter_iir2_clamp(float *out, float *tmp, float target, float alpha, float step);

void filter_iir3(float *out, float *tmp1, float *tmp2, float target, float alpha);

typedef struct {
    float value;
    float speed;
    float accel;
    float k;
} GaussianFilter;

void gaussian_configure(GaussianFilter *filter, float half_time);
void gaussian_reset(GaussianFilter *filter, float value, float speed);
void gaussian_update(GaussianFilter *filter, float target, float dt);
