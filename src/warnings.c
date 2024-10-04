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
#include "utils.h"

#include <math.h>

void warnings_configure(Warnings *warnings, const CfgWarnings *cfg, float dt) {
    warnings->lv_threshold = cfg->lv_threshold * cfg->battery_cells;
    warnings->hv_threshold = cfg->hv_threshold * cfg->battery_cells;

    warnings->mc_max_temp_fet = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_fet_start) - 3.0f;
    warnings->mc_max_temp_mot = VESC_IF->get_cfg_float(CFG_PARAM_l_temp_motor_start) - 3.0f;

    warnings->duty_step = cfg->duty_speed * dt;
    warnings->lv_step = 0.25f * dt;
    warnings->hv_step = 0.5f * dt;
    warnings->temp_step = 0.25f * dt;

    warnings->duty_ramp_mult = 1.0f / cfg->duty_ramp;
}

void warnings_reset(Warnings *warnings, float alpha) {
    filter_ema(&warnings->factor, 0.0f, alpha);

    filter_ema(&warnings->duty, 0.0f, alpha);
    filter_ema(&warnings->lv, 0.0f, alpha);
    filter_ema(&warnings->hv, 0.0f, alpha);
    filter_ema(&warnings->temp_fet, 0.0f, alpha);
    filter_ema(&warnings->temp_mot, 0.0f, alpha);

    warnings->sensor = 0.0f;
    warnings->debug = 0.0f;
}

static void strongest_warning(Warnings *warnings, float warning, WarningReason reason) {
    if (warning > warnings->factor) {
        warnings->factor = warning;
        warnings->reason = reason;
    }
}

void warnings_update(
    Warnings *warnings,
    const CfgWarnings *cfg,
    const MotorData *mot,
    const FootpadSensor *sensor,
    float debug
) {
    warnings->factor = 0.0f;

    // DUTY CYCLE
    const float duty_pct = mot->duty_cycle / mot->duty_max;
    float duty_target = (duty_pct - cfg->duty_threshold) * warnings->duty_ramp_mult;
    duty_target = clamp(duty_target, 0.0f, 1.0f);
    // duty_target = remap_to_01(mot->duty_cycle, cfg->duty_threshold, cfg->duty_threshold + cfg->duty_ramp);
    rate_limitf(&warnings->duty, duty_target, warnings->duty_step);
    strongest_warning(warnings, warnings->duty, WARNING_DUTY);
    // TODO only at speed?

    // MOTOR CURRENT
    // float current_pct = mot->current / mot->current_max;
    // current_target = remap_to_01(current_pct, cfg->current_thr, cfg->current_thr + cfg->current_ramp);
    // rate_limitf(&warnings->duty, duty_target, warnings->duty_step);
    // strongest_warning(warnings, warnings->duty, WARNING_DUTY);

    // BATTERY CURRENT
    // const float bat_amps_pct = mot->bat_amps / mot->bat_amps_max;
    // bat_amps_target = remap_to_01(bat_amps_pct, cfg->bat_amps_thr, cfg->bat_amps_thr + cfg->bat_amps_ramp);
    // rate_limitf(&warnings->bat_amps, bat_amps_target, warnings->duty_step);
    // strongest_warning(warnings, warnings->duty, WARNING_DUTY);


    // TODO use voltage per cell
    const float voltage = VESC_IF->mc_get_input_voltage_filtered();
    // TODO use battery current
    const float sag = 0.05f * fabsf(mot->current_smooth);
    const bool is_duty_non_zero = mot->duty_cycle > 0.05f;

    // LOW VOLTAGE
    const bool lv_active = voltage + sag < warnings->lv_threshold && is_duty_non_zero;
    const float lv_target = lv_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->lv, lv_target, warnings->lv_step);
    strongest_warning(warnings, warnings->lv, WARNING_LV);

    // HIGH VOLTAGE
    // const bool hv_active = voltage > warnings->hv_threshold && is_duty_non_zero;
    const bool hv_active = voltage > warnings->hv_threshold;
    const float hv_target = hv_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->hv, hv_target, warnings->hv_step);
    strongest_warning(warnings, warnings->hv, WARNING_HV);


    // FET TEMP
    const float temp_fet = VESC_IF->mc_temp_fet_filtered();
    const bool temp_fet_active = temp_fet > warnings->mc_max_temp_fet;
    const float temp_fet_target = temp_fet_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->temp_fet, temp_fet_target, warnings->temp_step);
    strongest_warning(warnings, warnings->temp_fet, WARNING_TEMP_FET);

    // MOTOR TEMP
    const float temp_mot = VESC_IF->mc_temp_motor_filtered();
    const bool temp_mot_active = temp_mot > warnings->mc_max_temp_mot;
    const float temp_mot_target = temp_mot_active ? 1.0f : 0.0f;
    rate_limitf(&warnings->temp_mot, temp_mot_target, warnings->temp_step);
    strongest_warning(warnings, warnings->temp_mot, WARNING_TEMP_MOT);


    // FOOTPAD SENSOR
    const bool sensor_disengaged = sensor->state == FS_NONE;
    if (sensor_disengaged) {
        const float speed_factor = (fabsf(mot->board_speed) - 1.0f) * 2.0f;
        warnings->sensor = clamp(speed_factor, 0.0f, 1.0f);
    } else {
        warnings->sensor = 0.0f;
    }
    strongest_warning(warnings, warnings->sensor, WARNING_SENSORS);
    // if (warnings->sensor > warnings->factor) {
    //     warnings->factor = warnings->sensor;
    // }


    // DEBUG
    warnings->debug = debug;
    strongest_warning(warnings, warnings->debug, WARNING_DEBUG);
    // if (warnings->debug > warnings->factor) {
    //     warnings->factor = warnings->sensor;
    // }
    // TODO SPEED
}
