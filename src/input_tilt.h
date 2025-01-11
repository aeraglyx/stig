// Copyright 2022 Dado Mista
// Copyright 2024 Lukas Hrazky
// Copyright 2024 Vladislav Macicek
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

#include "remote_data.h"
#include "utils.h"

#include "conf/datatypes.h"

typedef struct {
    GaussianFilter filter;
} InputTilt;

void input_tilt_configure(InputTilt *data, const CfgInputTilt *cfg);

void input_tilt_reset(InputTilt *data);

void input_tilt_update(
    InputTilt *data,
    const CfgInputTilt *cfg,
    const RemoteData *remote,
    float dt
);
