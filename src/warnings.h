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
#include "footpad_sensor.h"

typedef enum {
    WARNING_NONE = 0,
    WARNING_LV = 1,
    WARNING_HV = 2,
    WARNING_TEMP_FET = 3,
    WARNING_TEMP_MOT = 4,
    WARNING_CURRENT = 5,
    WARNING_DUTY = 6,
    WARNING_SENSORS = 7,
    WARNING_LOWBATT = 8,
    WARNING_IDLE = 9,
    WARNING_DEBUG = 10,
    WARNING_ERROR = 11,
    WARNING_GHOST = 12
} WarningType;

typedef struct {
    WarningType type;
    WarningType type_last;

    float cell_to_pack_ratio;
    float mc_max_temp_fet;
    float mc_max_temp_mot;
} Warnings;

void warnings_init(Warnings *warnings);

void warnings_configure(Warnings *warnings, const CfgWarnings *cfg);

void warnings_update(
    Warnings *warnings,
    const CfgWarnings *cfg,
    const MotorData *mot,
    const FootpadSensor *sensor,
    bool ghost,
    bool debug
);
