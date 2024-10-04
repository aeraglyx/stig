
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

#include "conf/datatypes.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool connected;
    float throttle;
} RemoteData;

// void remote_data_init(RemoteData *rem);

// void remote_data_configure(RemoteData *remote, const CfgInputTilt *cfg, float dt);

void remote_data_update(RemoteData *remote, const CfgHwRemote *cfg);
