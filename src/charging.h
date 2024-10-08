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

#include "state.h"

#include <stddef.h>

typedef enum {
    COMMAND_CHARGING_STATE = 28,  // to be called by ADV LCM while charging
} ChargingCommands;

typedef struct {
    float timer;
    float voltage;
    float current;
} Charging;

void charging_init(Charging *charging);

void charging_timeout(Charging *charging, State *state);

/**
 * Command to be called by ADV/LCM to announce that the board is charging.
 */
void charging_state_request(Charging *charging, uint8_t *buffer, size_t len, State *state);
