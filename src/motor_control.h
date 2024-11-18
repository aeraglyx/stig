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

#include <stdbool.h>
#include <stdint.h>

#include "conf/datatypes.h"
#include "state.h"

typedef struct {
    bool is_torque_requested;
    float requested_torque;

    float brake_timeout;
    bool use_strong_brake;

    float brake_current;
    // ParkingBrakeMode parking_brake_mode;
} MotorControl;

void motor_control_init(MotorControl *mc);

void motor_control_configure(MotorControl *mc, const StigConfig *cfg);

void motor_control_request_torque(MotorControl *mc, float torque);

void motor_control_apply(MotorControl *mc, const MotorData *motor, float time);
