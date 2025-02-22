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

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    STATE_DISABLED = 0,
    STATE_STARTUP = 1,
    STATE_READY = 2,
    STATE_RUNNING = 3
} RunState;

typedef enum {
    MODE_NORMAL = 0,
    MODE_HANDTEST = 1
} Mode;

typedef enum {
    STOP_NONE = 0,
    STOP_PITCH = 1,
    STOP_ROLL = 2,
    STOP_SENSOR = 3,
    STOP_GHOST = 4,
    STOP_REVERSE_STOP = 5,
    STOP_QUICKSTOP = 6
} StopCondition;

// leaving gaps for more states inbetween the different "classes" of the types
// (normal / warning / error)
typedef enum {
    SAT_NONE = 0,
    SAT_CENTERING = 1,
    SAT_REVERSESTOP = 2
} SetpointAdjustmentType;

typedef struct {
    RunState state;
    Mode mode;
    SetpointAdjustmentType sat;
    StopCondition stop_condition;
    bool charging;
    bool wheelslip;
} State;

void state_init(State *state, bool disable);

void state_stop(State *state, StopCondition stop_condition);

void state_engage(State *state);

/**
 * Compatibility function for the Float State enum for the app data commands.
 */
uint8_t state_compat(const State *state);
