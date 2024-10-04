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

#include "conf/datatypes.h"
#include "motor_data.h"
#include "remote_data.h"

typedef struct {
    float interpolated;
    float tmp;
    float tilt_alpha;
    float tilt_step;
} InputTilt;

void input_tilt_reset(InputTilt *data);

void input_tilt_configure(InputTilt *data, const CfgInputTilt *cfg, float dt);

void input_tilt_update(InputTilt *data, const CfgInputTilt *cfg, const RemoteData *remote);
