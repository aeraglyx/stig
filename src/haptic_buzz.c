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

#include "haptic_buzz.h"
#include "utils.h"

#include "vesc_c_if.h"

#include <math.h>

static const float BEEPER_TONE_LENGTH = 0.0625f;

void haptic_buzz_init(HapticBuzz *data) {
    data->is_playing = false;
    data->is_beeping = false;
    data->beeps_left = 0;
    data->beep_timer = 0.0f;
}

// void haptic_buzz_configure(HapticBuzz *data, const CfgWarnings *cfg) {
// }

static BuzzType get_buzz_type(WarningType warning_type, RunState run_state) {
    if (run_state != STATE_RUNNING) {
        return BUZZ_NONE;
    }

    switch (warning_type) {
        case WARNING_DUTY:
        case WARNING_GHOST:
            return BUZZ_FAST;
        case WARNING_LV:
        case WARNING_HV:
        case WARNING_TEMP_FET:
        case WARNING_TEMP_MOT:
            return BUZZ_SLOW;
        case WARNING_DEBUG:
        case WARNING_SENSORS:
            return BUZZ_FULL;
        default:
            return BUZZ_NONE;
    }
}

static bool get_beep_target(BuzzType buzz_type, float speed) {
    if (buzz_type == BUZZ_NONE) {
        return false;
    }
    if (buzz_type == BUZZ_FULL) {
        return true;
    }

    if (buzz_type == BUZZ_SLOW) {
        speed *= 0.25f;
    }
    bool beep;
    float current_time = VESC_IF->system_time();
    float time = current_time * speed;
    float x = time - (long) time;
    beep = (x < 0.5f);
    return beep;
}

void beep_alert(HapticBuzz *data, uint8_t num_beeps) {
    data->beeps_left = 2 * num_beeps - 1;
    data->beep_timer = VESC_IF->system_time() + BEEPER_TONE_LENGTH;
}

static bool get_beeper_target(HapticBuzz *data) {
    if (data->beeps_left == 0) {
        return false;
    }

    bool res = data->beeps_left & 0x1;

    if (VESC_IF->system_time() >= data->beep_timer) {
        data->beeps_left--;
        if (data->beeps_left > 0) {
            data->beep_timer = VESC_IF->system_time() + BEEPER_TONE_LENGTH;
        }
    }

    return res;
}

static float get_amplitude(const CfgHaptics *cfg, float fast_boi) {
    float strength_diff = cfg->strength_at_speed - cfg->strength;
    return cfg->strength + fast_boi * strength_diff;
}

void haptic_buzz_update(
    HapticBuzz *data,
    const CfgHaptics *cfg,
    const MotorData *mot,
    MotorTone *tone,
    WarningType warning_type,
    RunState run_state
) {
    BuzzType buzz_type = get_buzz_type(warning_type, run_state);
    bool beep_target_low = get_beep_target(buzz_type, cfg->speed);

    if (data->is_playing && !beep_target_low) {
        motor_control_stop_tone(tone);
        data->is_playing = false;
    } else if (!data->is_playing && beep_target_low) {
        float amplitude = get_amplitude(cfg, mot->fast_boi);
        motor_control_play_tone(tone, cfg->frequency, amplitude);
        data->is_playing = true;
    }

    bool beep_target_high = get_beeper_target(data);

    if (data->is_beeping && !beep_target_high) {
        VESC_IF->foc_play_tone(0, 1, 0.0f);
        data->is_beeping = false;
    } else if (!data->is_beeping && beep_target_high) {
        VESC_IF->foc_play_tone(0, 523.25f, cfg->beeper_strength);
        data->is_beeping = true;
    }
}
