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

#include "state.h"

void state_init(State *state, bool disable) {
    state->state = disable ? STATE_DISABLED : STATE_STARTUP;
    state->mode = MODE_NORMAL;
    state->sat = SAT_NONE;
    state->stop_condition = STOP_NONE;
    state->charging = false;
    state->wheelslip = false;
}

void state_stop(State *state, StopCondition stop_condition) {
    state->state = STATE_READY;
    state->stop_condition = stop_condition;
    state->wheelslip = false;
}

void state_engage(State *state) {
    state->state = STATE_RUNNING;
    state->sat = SAT_NONE;
    state->stop_condition = STOP_NONE;
}

uint8_t state_compat(const State *state) {
    if (state->charging) {
        return 14;  // CHARGING
    }

    switch (state->state) {
    case STATE_DISABLED:
        return 15;  // DISABLED
    case STATE_STARTUP:
        return 0;  // STARTUP
    case STATE_READY:
        switch (state->stop_condition) {
        case STOP_NONE:
            return 11;  // FAULT_STARTUP
        case STOP_PITCH:
            return 6;  // FAULT_ANGLE_PITCH
        case STOP_ROLL:
            return 7;  // FAULT_ANGLE_ROLL
        // XXX STOP_SWITCH_HALF used to return 8
        case STOP_SENSOR:
            return 9;  // FAULT_SWITCH_FULL
        case STOP_GHOST:
            return 12;  // FAULT_REVERSE
        case STOP_REVERSE_STOP:
            return 12;  // FAULT_REVERSE
        case STOP_QUICKSTOP:
            return 13;  // FAULT_QUICKSTOP
        }
        return 11;  // FAULT_STARTUP
    case STATE_RUNNING:
        // TODO if a warning is active, return 2
        if (state->wheelslip) {
            return 3;  // RUNNING_WHEELSLIP
        }
        return 1;  // RUNNING
    }
    return 0;  // STARTUP
}
