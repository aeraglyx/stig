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

#include "conf/datatypes.h"
#include "motor_data.h"
#include "imu_data.h"

typedef struct {
    float multiplier;
    float slip_factor;
    float drop_factor;
    float traction_soft_release;

    float winddown_alpha;
} Traction;

void traction_init(Traction *data);

void traction_configure(Traction *data, const CfgTraction *cfg, float dt);

void traction_update(Traction *data, const CfgTraction *cfg, const IMUData *imu, const MotorData *mot);
