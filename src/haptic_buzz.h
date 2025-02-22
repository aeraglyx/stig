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

#pragma once

#include "motor_data.h"
#include "motor_control.h"
#include "state.h"
#include "warnings.h"

#include "conf/datatypes.h"

typedef struct {
    bool is_playing;
    bool is_beeping;
    uint8_t beeps_left;
    float beep_timer;
} HapticBuzz;

typedef enum {
    BUZZ_NONE = 0,
    BUZZ_FAST = 1,
    BUZZ_SLOW = 2,
    BUZZ_FULL = 3,
} BuzzType;

void haptic_buzz_init(HapticBuzz *data);

// void haptic_buzz_configure(HapticBuzz *data, const CfgWarnings *cfg);

void beep_alert(HapticBuzz *data, uint8_t num_beeps);

void haptic_buzz_update(
    HapticBuzz *data,
    const CfgHaptics *cfg,
    const MotorData *mot,
    MotorTone *tone,
    WarningType warning_type,
    RunState run_state
);
