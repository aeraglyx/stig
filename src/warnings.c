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

#include "vesc_c_if.h"

#include "warnings.h"
#include "state.h"
#include "utils.h"

#include <math.h>

void warnings_init(Warnings *warnings) {
    warnings->type = WARNING_NONE;
    warnings->type_last = WARNING_NONE;
}

void warnings_configure(Warnings *warnings, const CfgWarnings *cfg) {
    unused(cfg);

    warnings->cell_to_pack_ratio = 1.0f / VESC_IF->get_cfg_int(CFG_PARAM_si_battery_cells);

    warnings->mc_max_temp_fet = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_fet_start) - 3.0f;
    warnings->mc_max_temp_mot = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_motor_start) - 3.0f;
}

void warnings_update(
    Warnings *warnings,
    const CfgWarnings *cfg,
    const MotorData *mot,
    const FootpadSensor *sensor,
    bool ghost,
    bool debug
) {
    // Duty Cycle
    if (mot->duty_cycle > cfg->duty_threshold) {
        warnings->type = WARNING_DUTY;
        return;
    }

    // Ghost Safeguard
    if (ghost) {
        warnings->type = WARNING_GHOST;
        return;
    }

    float voltage = VESC_IF->mc_get_input_voltage_filtered() * warnings->cell_to_pack_ratio;
    float allowed_sag = 0.0025f * fabsf(mot->current_smooth);
    // TODO use battery current?

    // Low Voltage
    if (voltage + allowed_sag < cfg->lv_threshold) {
        warnings->type = WARNING_LV;
        return;
    }

    // High Voltage
    if (voltage > cfg->hv_threshold) {
        warnings->type = WARNING_HV;
        return;
    }

    // FET Temp
    float temp_fet = VESC_IF->mc_temp_fet_filtered();
    if (temp_fet > warnings->mc_max_temp_fet) {
        warnings->type = WARNING_TEMP_FET;
        return;
    }

    // Motor Temp
    float temp_mot = VESC_IF->mc_temp_motor_filtered();
    if (temp_mot > warnings->mc_max_temp_mot) {
        warnings->type = WARNING_TEMP_MOT;
        return;
    }

    // Footpad Sensor
    bool sensor_disengaged = sensor->state == FS_NONE;
    bool riding = fabsf(mot->board_speed) > 1.0f;
    if (sensor_disengaged && riding) {
        warnings->type = WARNING_SENSORS;
        return;
    }

    // Debug
    if (debug) {
        warnings->type = WARNING_DEBUG;
        return;
    }

    warnings->type = WARNING_NONE;
}
