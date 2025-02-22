// Copyright 2019 - 2022 Mitch Lustig
// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#include "vesc_c_if.h"

#include "balance_filter.h"
#include "imu_data.h"
#include "motor_data.h"
#include "remote_data.h"

#include "warnings.h"

#include "modifiers.h"
#include "input_tilt.h"

#include "pid.h"
#include "haptic_buzz.h"
#include "motor_control.h"

#include "charging.h"
#include "footpad_sensor.h"
#include "lcm.h"
#include "leds.h"
#include "state.h"
#include "utils.h"

#include "conf/buffer.h"
#include "conf/conf_general.h"
#include "conf/confparser.h"
#include "conf/confxml.h"
#include "conf/datatypes.h"

#include <math.h>
#include <string.h>

HEADER

// This is all persistent state of the application, which will be allocated in init. It
// is put here because variables can only be read-only when this program is loaded
// in flash without virtual memory in RAM (as all RAM already is dedicated to the
// main firmware and managed from there). This is probably the main limitation of
// loading applications in runtime, but it is not too bad to work around.
typedef struct {
    lib_thread main_thread;
    lib_thread led_thread;

    StigConfig config;

    // Firmware version, passed in from Lisp
    int fw_version_major, fw_version_minor, fw_version_beta;

    State state;
    Charging charging;

    // IMU data for the balancing filter
    BalanceFilterData balance_filter;

    // Board data
    IMUData imu;
    MotorData motor;
    RemoteData remote;
    FootpadSensor footpad_sensor;

    Leds leds;
    LcmData lcm;

    Modifiers modifiers;
    InputTilt input_tilt;

    PID pid;
    Warnings warnings;
    HapticBuzz haptic_buzz;
    MotorControl motor_control;

    // Config values
    float dt;

    float current_time;
    float last_time;
    float loop_overshoot_filtered;
    // float computation_time;

    float startup_pitch_trickmargin;
    float startup_pitch_tolerance;

    GaussianFilter setpoint_filter;
    float setpoint;

    float disengage_timer;
    float nag_timer;
    float idle_voltage;

    float fault_angle_pitch_timer;
    float fault_angle_roll_timer;
    float fault_sensor_timer;
    float fault_ghost_timer;

    float wheelslip_timer;

    // Feature: Reverse Stop
    // TODO refactor reverse stop
    float reverse_stop_step_size;
    float reverse_tolerance;
    float reverse_total_distance;
    float reverse_timer;

    // Odometer
    // float odo_timer;
    // int odometer_dirty;
    // uint64_t odometer;

} data;

static void configure(data *d) {
    // TODO not the whole state_init
    state_init(&d->state, d->config.disabled);

    d->dt = 1.0f / d->config.hardware.esc.frequency;

    balance_filter_configure(&d->balance_filter, &d->config.tune.balance_filter);

    imu_data_configure(&d->imu, &d->config.tune.traction, d->config.hardware.esc.imu_x_offset, d->dt);
    motor_data_configure(&d->motor, &d->config.tune, &d->config.hardware, &d->config.rider, d->dt);
    motor_control_configure(&d->motor_control, &d->config);
    // remote_data_configure(&d->remote, &d->config.tune.input_tilt, d->dt);
    lcm_configure(&d->lcm, &d->config.leds);

    modifiers_configure(&d->modifiers, &d->config.tune, d->dt);
    input_tilt_configure(&d->input_tilt, &d->config.tune.input_tilt);
    gaussian_configure(&d->setpoint_filter, d->config.startup.filter);

    pid_configure(&d->pid, &d->config.tune.pid, d->dt);
    warnings_configure(&d->warnings, &d->config.warnings);

    d->disengage_timer = d->current_time;

    // Feature: Dirty Landings
    d->startup_pitch_trickmargin = d->config.startup.dirtylandings_enabled ? 10 : 0;

    // Backwards compatibility hack:
    // If mahony kp from the firmware internal filter is higher than 1, it's
    // the old setup with it being the main balancing filter. In that case, set
    // the kp and acc confidence decay to hardcoded defaults of the former true
    // pitch filter, to preserve the behavior of the old setup in the new one.
    // (Though Mahony KP 0.4 instead of 0.2 is used, as it seems to work better)
    if (VESC_IF->get_cfg_float(CFG_PARAM_IMU_mahony_kp) > 1) {
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_kp, 0.4);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_mahony_ki, 0);
        VESC_IF->set_cfg_float(CFG_PARAM_IMU_accel_confidence_decay, 0.1);
    }

    // TODO set AppCfg accel Z filter to 0

    // Feature: Reverse Stop
    d->reverse_tolerance = 0.06f;  // in meters
    d->reverse_stop_step_size = 100.0 * d->dt;

    if (d->state.state == STATE_DISABLED) {
        beep_alert(&d->haptic_buzz, 3);
    }
}

static void reset_vars(data *d) {
    float time_disengaged = d->current_time - d->disengage_timer;
    float alpha = clamp(time_disengaged, 0.0f, 1.0f);

    modifiers_reset(&d->modifiers, alpha);
    input_tilt_reset(&d->input_tilt);
    pid_reset(&d->pid, &d->imu, alpha);
    gaussian_reset(&d->setpoint_filter, d->imu.pitch_balance, clamp_sym(d->imu.gyro[1], 50.0f));
    
    d->startup_pitch_tolerance = d->config.startup.pitch_tolerance;
}

static void engage(data *d) {
    reset_vars(d);
    state_engage(&d->state);
    // TODO click
}

/**
 * check_odometer: see if we need to write back the odometer during fault state
 */
// static void check_odometer(data *d) {
//     // Make odometer persistent if we've gone 200m or more
//     if (d->odometer_dirty > 0) {
//         float stored_odo = VESC_IF->mc_get_odometer();
//         if ((stored_odo > d->odometer + 200) || (stored_odo < d->odometer - 10000)) {
//             if (d->odometer_dirty == 1) {
//                 // Wait 10 seconds before writing to avoid writing if immediately continuing to ride
//                 d->odo_timer = d->current_time;
//                 d->odometer_dirty++;
//             } else if ((d->current_time - d->odo_timer) > 10) {
//                 VESC_IF->store_backup_data();
//                 d->odometer = VESC_IF->mc_get_odometer();
//                 d->odometer_dirty = 0;
//             }
//         }
//     }
// }

static bool board_may_have_ghosted(StopCondition condition) {
    return condition == STOP_GHOST || condition == STOP_ROLL || condition == STOP_PITCH;
}

// TODO no need to pass full data
static bool is_sensor_engaged(const data *d) {
    if (d->footpad_sensor.state == FS_BOTH) {
        return true;
    }

    if (d->footpad_sensor.state == FS_NONE) {
        return false;
    }

    // Half Engaged:
    if (board_may_have_ghosted(d->state.stop_condition)) {
        return false;
    }

    // Posi always enabled
    return true;
}

static bool startup_conditions_met(data *d) {
    if (!is_sensor_engaged(d)) {
        return false;
    }

    if (d->state.charging) {
        return false;
    }

    // bool is_pitch_valid = fabsf(d->imu.pitch_balance) < d->startup_pitch_tolerance;
    bool is_pitch_valid = fabsf(d->imu.pitch) < d->startup_pitch_tolerance;
    bool is_roll_valid = fabsf(d->imu.roll) < d->config.startup.roll_tolerance;

    if (is_pitch_valid && is_roll_valid) {
        return true;
    }

    bool is_push_start = d->config.startup.pushstart_enabled && d->motor.speed > 1.0f;
    if (is_push_start && (fabsf(d->imu.pitch_balance) < 45) && is_roll_valid) {
        return true;
    }

    return false;
}

static bool check_faults(data *d) {
    bool disable_sensor_faults =
        d->config.faults.moving_fault_disabled &&
        d->motor.board_speed > 0.5f &&
        fabsf(d->imu.pitch) < 40 && fabsf(d->imu.roll) < 75;

    // Switch fully open
    if (d->footpad_sensor.state == FS_NONE) {
        if (!disable_sensor_faults) {
            float sensor_time = d->current_time - d->fault_sensor_timer;
            float sensor_delay = 0.001f * d->config.faults.sensor_delay;

            if (sensor_time > sensor_delay) {
                state_stop(&d->state, STOP_SENSOR);
                return true;
            }
        }

        // Feature: Quick Stop
        // TODO only under load?
        if (d->motor.speed_abs < 0.2f && d->imu.pitch > 14.0f &&
            sign(d->imu.pitch) == d->motor.speed_sign) {
            state_stop(&d->state, STOP_QUICKSTOP);
            return true;
        }
    } else {
        d->fault_sensor_timer = d->current_time;
    }

    // TODO
    // Feature: Reverse-Stop
    if (d->state.sat == SAT_REVERSESTOP) {
        //  Taking your foot off entirely while reversing? Ignore delays
        if (d->footpad_sensor.state == FS_NONE) {
            state_stop(&d->state, STOP_SENSOR);
            return true;
        }
        if (fabsf(d->imu.pitch) > 15) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        // Above 10 degrees for a half a second? Switch it off
        if (fabsf(d->imu.pitch) > 10 && d->current_time - d->reverse_timer > .5) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        // Above 5 degrees for a full second? Switch it off
        if (fabsf(d->imu.pitch) > 5 && d->current_time - d->reverse_timer > 1) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        if (d->reverse_total_distance > d->reverse_tolerance * 3) {
            state_stop(&d->state, STOP_REVERSE_STOP);
            return true;
        }
        if (fabsf(d->imu.pitch) <= 5) {
            d->reverse_timer = d->current_time;
        }
    }

    // Feature: Ghost Buster
    // TODO exclude wheelslip - use confidence
    bool riding_backwards = d->motor.board_speed < - d->config.faults.ghost_speed;
    bool sensor_half_active = d->footpad_sensor.state == FS_LEFT || d->footpad_sensor.state == FS_RIGHT;
    if (riding_backwards && sensor_half_active) {
        float ghost_time = d->current_time - d->fault_ghost_timer;
        if (ghost_time > d->config.faults.ghost_delay) {
            state_stop(&d->state, STOP_GHOST);
            return true;
        }
    } else {
        d->fault_ghost_timer = d->current_time;
    }

    // Check roll angle
    if (fabsf(d->imu.roll) > d->config.faults.roll_threshold) {
        float roll_time = d->current_time - d->fault_angle_roll_timer;
        float roll_delay = 0.001f * d->config.faults.roll_delay;

        if (roll_time > roll_delay) {
            state_stop(&d->state, STOP_ROLL);
            return true;
        }
    } else {
        d->fault_angle_roll_timer = d->current_time;
    }

    // Check pitch angle
    if (fabsf(d->imu.pitch - d->input_tilt.filter.value) > d->config.faults.pitch_threshold) {
        float pitch_time = d->current_time - d->fault_angle_pitch_timer;
        float pitch_delay = d->config.faults.pitch_delay;

        if (pitch_time > pitch_delay) {
            state_stop(&d->state, STOP_PITCH);
            return true;
        }
    } else {
        d->fault_angle_pitch_timer = d->current_time;
    }

    return false;
}

static void imu_ref_callback(float *acc, float *gyro, float *mag, float dt) {
    unused(mag);

    data *d = (data *) ARG;
    balance_filter_update(&d->balance_filter, gyro, acc, dt);
}

static void time_vars_update(data *d) {
    d->current_time = VESC_IF->system_time();
    float dt_measured = d->current_time - d->last_time;
    d->last_time = d->current_time;

    // TODO measure loop overshoot only when running
    float loop_overshoot = dt_measured - d->dt;

    // TODO use 2nd order IIR
    filter_ema(&d->loop_overshoot_filtered, loop_overshoot, 0.002f);
}

static void stig_thd(void *arg) {
    data *d = (data *) arg;

    configure(d);

    d->last_time = VESC_IF->system_time() - d->dt;

    // VESC_IF->plot_init("Time", "Y");
    // VESC_IF->plot_add_graph("something");

    while (!VESC_IF->should_terminate()) {
        time_vars_update(d);
        
        charging_timeout(&d->charging, &d->state);

        imu_data_update(&d->imu, &d->balance_filter);
        motor_data_update(&d->motor, d->config.hardware.esc.frequency, &d->imu);
        remote_data_update(&d->remote, &d->config.hardware.remote);
        footpad_sensor_update(&d->footpad_sensor, &d->config.faults);

        // Control Loop State Logic
        switch (d->state.state) {
        case (STATE_STARTUP):
            if (VESC_IF->imu_startup_done()) {
                // TODO check if ghosting
                d->state.state = STATE_READY;
                beep_alert(&d->haptic_buzz, 1);
            }
            break;

        case (STATE_RUNNING):
            // Check for faults
            // TODO put switching into STATE_READY outside check_faults() ?
            if (check_faults(d)) {
                if (d->state.stop_condition == STOP_SENSOR) {
                    // dirty landings: add extra margin
                    d->startup_pitch_tolerance =
                        d->config.startup.pitch_tolerance + d->startup_pitch_trickmargin;
                    d->fault_angle_pitch_timer = d->current_time;
                }
                break;
            }
            // d->odometer_dirty = 1;

            d->disengage_timer = d->current_time;

            gaussian_update(&d->setpoint_filter, 0.0f, d->dt);

            modifiers_update(
                &d->modifiers,
                &d->config.tune,
                &d->motor,
                &d->imu,
                d->dt
            );

            input_tilt_update(
                &d->input_tilt,
                &d->config.tune.input_tilt,
                &d->remote,
                d->dt
            );

            d->setpoint = d->setpoint_filter.value;
            d->setpoint += d->modifiers.filter.value;
            d->setpoint += d->input_tilt.filter.value;

            // TODO move to the bf module
            CfgBalanceFilter *bf_cfg = &d->config.tune.balance_filter;
            float kp_mult_src_pitch = d->setpoint_filter.value - d->imu.pitch;
            float kp_mult_src_modif = d->modifiers.filter.value + d->input_tilt.filter.value;
            float kp_mult_src = kp_mult_src_pitch * bf_cfg->boost_pitch + kp_mult_src_modif * bf_cfg->boost_modif;

            d->balance_filter.kp_mult = 1.0f / (1.0f + fabsf(kp_mult_src));

            pid_update(
                &d->pid,
                &d->imu,
                &d->motor,
                &d->config.tune.pid,
                d->setpoint,
                d->modifiers.filter.speed
            );

            bool warning_ghost = d->current_time - d->fault_ghost_timer > 0.1f;
            bool warning_debug = false;
            warnings_update(
                &d->warnings,
                &d->config.warnings,
                &d->motor,
                &d->footpad_sensor,
                warning_ghost,
                warning_debug
            );
            if (d->warnings.type != WARNING_NONE) {
                d->warnings.type_last = d->warnings.type;
            }

            float torque_requested = d->pid.pid_value * d->motor.traction.multiplier;
            motor_control_request_torque(&d->motor_control, torque_requested);

            break;

        case (STATE_READY):
            if ((d->current_time - d->fault_angle_pitch_timer) > 1.0f) {
                // 1 second after disengaging - set startup tolerance back to normal (aka tighter)
                d->startup_pitch_tolerance = d->config.startup.pitch_tolerance;
            }

            // check_odometer(d);

            if (startup_conditions_met(d)) {
                engage(d);
                break;
            }

            break;

        case (STATE_DISABLED):
            break;
        }

        haptic_buzz_update(
            &d->haptic_buzz,
            &d->config.haptics,
            &d->motor,
            &d->motor_control.tone,
            d->warnings.type,
            d->state.state
        );

        if (d->state.state != STATE_DISABLED) {
            motor_control_apply(&d->motor_control, &d->motor, d->current_time);
        }

        // d->computation_time = VESC_IF->system_time() - d->current_time;
        // const float dt_correction = d->computation_time + d->loop_overshoot_filtered;
        // VESC_IF->sleep_us(1e6 * max(d->dt - dt_correction, 0));

        VESC_IF->sleep_us(1e6 * fmaxf(d->dt - d->loop_overshoot_filtered, 0.0f));
        // VESC_IF->sleep_us(1e6 * d->dt);
    }
}

static void write_cfg_to_eeprom(data *d) {
    uint32_t ints = sizeof(StigConfig) / 4 + 1;
    uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
    if (!buffer) {
        log_error("Failed to write config to EEPROM: Out of memory.");
        return;
    }

    bool write_ok = true;
    memcpy(buffer, &(d->config), sizeof(StigConfig));
    for (uint32_t i = 0; i < ints; i++) {
        eeprom_var v;
        v.as_u32 = buffer[i];
        if (!VESC_IF->store_eeprom_var(&v, i + 1)) {
            write_ok = false;
            break;
        }
    }

    VESC_IF->free(buffer);

    if (write_ok) {
        eeprom_var v;
        v.as_u32 = STIGCONFIG_SIGNATURE;
        VESC_IF->store_eeprom_var(&v, 0);
    } else {
        log_error("Failed to write config to EEPROM.");
    }

    // beep_alert(d, 1, 0);
}

static void led_thd(void *arg) {
    data *d = (data *) arg;

    while (!VESC_IF->should_terminate()) {
        leds_update(&d->leds, &d->state, d->footpad_sensor.state);
        VESC_IF->sleep_us(1e6 / LEDS_REFRESH_RATE);
    }
}

static void read_cfg_from_eeprom(StigConfig *config) {
    uint32_t ints = sizeof(StigConfig) / 4 + 1;
    uint32_t *buffer = VESC_IF->malloc(ints * sizeof(uint32_t));
    if (!buffer) {
        log_error("Failed to read config from EEPROM: Out of memory.");
        return;
    }

    eeprom_var v;
    bool read_ok = VESC_IF->read_eeprom_var(&v, 0);
    if (read_ok) {
        if (v.as_u32 == STIGCONFIG_SIGNATURE) {
            for (uint32_t i = 0; i < ints; i++) {
                if (!VESC_IF->read_eeprom_var(&v, i + 1)) {
                    read_ok = false;
                    break;
                }
                buffer[i] = v.as_u32;
            }
        } else {
            log_error("Failed signature check while reading config from EEPROM, using defaults.");
            confparser_set_defaults_stigconfig(config);
            return;
        }
    }

    if (read_ok) {
        memcpy(config, buffer, sizeof(StigConfig));
    } else {
        confparser_set_defaults_stigconfig(config);
        log_error("Failed to read config from EEPROM, using defaults.");
    }

    VESC_IF->free(buffer);
}

static void data_init(data *d) {
    memset(d, 0, sizeof(data));

    read_cfg_from_eeprom(&d->config);

    // d->odometer = VESC_IF->mc_get_odometer();

    motor_control_init(&d->motor_control);
    lcm_init(&d->lcm, &d->config.hardware.leds);
    charging_init(&d->charging);

    imu_data_init(&d->imu);
    motor_data_init(&d->motor);
    warnings_init(&d->warnings);
    haptic_buzz_init(&d->haptic_buzz);
}

// See also:
// LcmCommands in lcm.h
// ChargingCommands in charging.h
enum {
    COMMAND_GET_INFO = 0,  // get version / package info
    COMMAND_GET_RTDATA = 1,  // get rt data
    // COMMAND_RT_TUNE = 2,  // runtime tuning (don't write to eeprom)
    // COMMAND_TUNE_DEFAULTS = 3,  // set tune to defaults (no eeprom)
    COMMAND_CFG_SAVE = 4,  // save config to eeprom
    COMMAND_CFG_RESTORE = 5,  // restore config from eeprom
    // COMMAND_TUNE_OTHER = 6,  // make runtime changes to startup/etc
    // COMMAND_RC_MOVE = 7,  // move motor while board is idle
    // COMMAND_BOOSTER = 8,  // change booster settings
    COMMAND_PRINT_INFO = 9,  // print verbose info
    // COMMAND_GET_ALLDATA = 10,  // send all data, compact
    // COMMAND_EXPERIMENT = 11,  // generic cmd for sending data, used for testing/tuning new features
    COMMAND_LOCK = 12,
    COMMAND_HANDTEST = 13,
    // COMMAND_TUNE_TILT = 14,

    // commands above 200 are unstable and can change protocol at any time
    COMMAND_GET_RTDATA_2 = 201,
    COMMAND_LIGHTS_CONTROL = 202,
} Commands;

static void cmd_print_info(data *d) {
    unused(d);
}

static void cmd_lock(data *d, unsigned char *cfg) {
    if (d->state.state < STATE_RUNNING) {
        // restore config before locking to avoid accidentally writing temporary changes
        // read_cfg_from_eeprom(&d->config);
        d->config.disabled = cfg[0] ? true : false;
        d->state.state = cfg[0] ? STATE_DISABLED : STATE_STARTUP;
        write_cfg_to_eeprom(d);
        beep_alert(&d->haptic_buzz, 3);
    }
}

static void cmd_handtest(data *d, unsigned char *cfg) {
    if (d->state.state != STATE_READY) {
        return;
    }

    if (d->state.mode != MODE_NORMAL && d->state.mode != MODE_HANDTEST) {
        return;
    }

    d->state.mode = cfg[0] ? MODE_HANDTEST : MODE_NORMAL;
    if (d->state.mode == MODE_HANDTEST) {
        d->motor.current_max = d->motor.current_min = 7;
        // TODO set expo gains to 0
        d->config.tune.pid.ki = 0.0f;
        d->config.tune.pid.kp_brake = 1.0f;
        d->config.tune.pid.kd_brake = 1.0f;
        d->config.tune.torque_tilt.enabled = false;
        d->config.tune.atr.enabled = false;
        d->config.tune.turn_tilt.strength = 0.0f;
        d->config.tune.speed_tilt.constant = 0.0f;
        d->config.tune.speed_tilt.variable = 0.0f;
        d->config.faults.pitch_threshold = 30;
        d->config.faults.roll_threshold = 30;
        beep_alert(&d->haptic_buzz, 2);
    } else {
        read_cfg_from_eeprom(&d->config);
        configure(d);
    }
}

static void send_realtime_data(data *d) {
    static const int bufsize = 87 + 4;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_GET_RTDATA_2;

    // mask indicates what groups of data are sent, to prevent sending data
    // that are not useful in a given state
    uint8_t mask = 0;
    if (d->state.state == STATE_RUNNING) {
        mask |= 0x1;
    }

    if (d->state.charging) {
        mask |= 0x2;
    }

    buffer[ind++] = mask;

    buffer[ind++] = d->state.mode << 4 | d->state.state;

    uint8_t flags = d->state.charging << 5 | false << 1 | d->state.wheelslip;
    buffer[ind++] = d->footpad_sensor.state << 6 | flags;

    buffer[ind++] = d->state.sat << 4 | d->state.stop_condition;

    buffer[ind++] = d->warnings.type_last;  // previously beep_reason

    buffer_append_float32_auto(buffer, d->imu.pitch, &ind);
    buffer_append_float32_auto(buffer, d->imu.pitch_balance, &ind);
    buffer_append_float32_auto(buffer, d->imu.roll, &ind);

    buffer_append_float32_auto(buffer, d->footpad_sensor.adc1, &ind);
    buffer_append_float32_auto(buffer, d->footpad_sensor.adc2, &ind);

    buffer_append_float32_auto(buffer, 0.0f, &ind);

    if (d->state.state == STATE_RUNNING) {
        // Setpoints
        buffer_append_float32_auto(buffer, d->setpoint, &ind);

        // Modifiers
        buffer_append_float32_auto(buffer, d->modifiers.target, &ind);
        buffer_append_float32_auto(buffer, d->modifiers.filter.value, &ind);
        buffer_append_float32_auto(buffer, d->modifiers.filter.speed, &ind);

        // PID
        buffer_append_float32_auto(buffer, d->pid.proportional, &ind);
        buffer_append_float32_auto(buffer, d->pid.integral, &ind);
        buffer_append_float32_auto(buffer, d->pid.derivative, &ind);
        buffer_append_float32_auto(buffer, d->pid.feed_forward, &ind);

        // Debug
        buffer_append_float32_auto(buffer, d->motor.traction.confidence, &ind);
        buffer_append_float32_auto(buffer, d->motor.traction.multiplier, &ind);
        buffer_append_float32_auto(buffer, d->motor.slope_data.slope, &ind);

        // Speed & Accel.
        buffer_append_float32_auto(buffer, d->motor.speed, &ind);
        buffer_append_float32_auto(buffer, d->motor.board_speed, &ind);
        buffer_append_float32_auto(buffer, d->motor.wheel_accel, &ind);
        buffer_append_float32_auto(buffer, d->motor.board_accel, &ind);
    }

    if (d->state.charging) {
        buffer_append_float32_auto(buffer, d->charging.current, &ind);
        buffer_append_float32_auto(buffer, d->charging.voltage, &ind);
    }

    SEND_APP_DATA(buffer, bufsize, ind);
}

static void lights_control_request(CfgLeds *leds, uint8_t *buffer, size_t len, LcmData *lcm) {
    if (len < 2) {
        return;
    }

    uint8_t mask = buffer[0];
    uint8_t value = buffer[1];

    if (mask != 0) {
        if (mask & 0x1) {
            leds->on = value & 0x1;
        }

        if (mask & 0x2) {
            leds->headlights_on = value & 0x2;
        }

        if (lcm->enabled) {
            lcm_configure(lcm, leds);
        }
    }
}

static void lights_control_response(const CfgLeds *leds) {
    static const int bufsize = 3;
    uint8_t buffer[bufsize];
    int32_t ind = 0;

    buffer[ind++] = 101;  // Package ID
    buffer[ind++] = COMMAND_LIGHTS_CONTROL;
    buffer[ind++] = leds->headlights_on << 1 | leds->on;

    SEND_APP_DATA(buffer, bufsize, ind);
}

// Handler for incoming app commands
static void on_command_received(unsigned char *buffer, unsigned int len) {
    data *d = (data *) ARG;
    uint8_t magicnr = buffer[0];
    uint8_t command = buffer[1];

    if (len < 2) {
        log_error("Received command data too short.");
        return;
    }
    if (magicnr != 101) {
        log_error("Invalid Package ID: %u", magicnr);
        return;
    }

    switch (command) {
        case COMMAND_GET_INFO: {
            int32_t ind = 0;
            uint8_t send_buffer[10];
            send_buffer[ind++] = 101;  // magic nr.
            send_buffer[ind++] = 0x0;  // command ID
            send_buffer[ind++] = (uint8_t) (10 * PACKAGE_MAJOR_MINOR_VERSION);
            send_buffer[ind++] = 1;  // build number
            // Send the full type here. This is redundant with cmd_light_info. It
            // likely shouldn't be here, as the type can be reconfigured and the
            // app would need to reconnect to pick up the change from this command.
            send_buffer[ind++] = d->config.hardware.leds.type;
            VESC_IF->send_app_data(send_buffer, ind);
            return;
        }
        case COMMAND_GET_RTDATA_2: {
            send_realtime_data(d);
            return;
        }
        // case COMMAND_TUNE_OTHER: {
        //     if (len >= 14) {
        //         cmd_runtime_tune_other(d, &buffer[2], len - 2);
        //     } else {
        //         log_error("Command data length incorrect: %u", len);
        //     }
        //     return;
        // }
        // case COMMAND_TUNE_TILT: {
        //     if (len >= 10) {
        //         cmd_runtime_tune_tilt(d, &buffer[2], len - 2);
        //     } else {
        //         log_error("Command data length incorrect: %u", len);
        //     }
        //     return;
        // }
        case COMMAND_CFG_RESTORE: {
            read_cfg_from_eeprom(&d->config);
            return;
        }
        case COMMAND_CFG_SAVE: {
            write_cfg_to_eeprom(d);
            return;
        }
        // case COMMAND_TUNE_DEFAULTS: {
        //     cmd_tune_defaults(d);
        //     return;
        // }
        case COMMAND_PRINT_INFO: {
            cmd_print_info(d);
            return;
        }
        // case COMMAND_GET_ALLDATA: {
        //     if (len == 3) {
        //         cmd_send_all_data(d, buffer[2]);
        //     } else {
        //         log_error("Command data length incorrect: %u", len);
        //     }
        //     return;
        // }
        // case COMMAND_EXPERIMENT: {
        //     cmd_experiment(d, &buffer[2]);
        //     return;
        // }
        case COMMAND_LOCK: {
            cmd_lock(d, &buffer[2]);
            return;
        }
        case COMMAND_HANDTEST: {
            cmd_handtest(d, &buffer[2]);
            return;
        }
        case COMMAND_LCM_POLL: {
            lcm_poll_request(&d->lcm, &buffer[2], len - 2);
            lcm_poll_response(&d->lcm, &d->state, d->footpad_sensor.state, &d->motor, d->imu.pitch);
            return;
        }
        case COMMAND_LCM_LIGHT_INFO: {
            lcm_light_info_response(&d->lcm);
            return;
        }
        case COMMAND_LCM_LIGHT_CTRL: {
            lcm_light_ctrl_request(&d->lcm, &buffer[2], len - 2);
            return;
        }
        case COMMAND_LCM_DEVICE_INFO: {
            lcm_device_info_response(&d->lcm);
            return;
        }
        case COMMAND_LCM_GET_BATTERY: {
            lcm_get_battery_response(&d->lcm);
            return;
        }
        case COMMAND_CHARGING_STATE: {
            charging_state_request(&d->charging, &buffer[2], len - 2, &d->state);
            return;
        }
        case COMMAND_LIGHTS_CONTROL: {
            lights_control_request(&d->config.leds, &buffer[2], len - 2, &d->lcm);
            lights_control_response(&d->config.leds);
            return;
        }
        default: {
            if (!VESC_IF->app_is_output_disabled()) {
                log_error("Unknown command received: %u", command);
            }
        }
    }
}

// Called from Lisp on init to pass in the version info of the firmware
static lbm_value ext_set_fw_version(lbm_value *args, lbm_uint argn) {
    data *d = (data *) ARG;
    if (argn > 2) {
        d->fw_version_major = VESC_IF->lbm_dec_as_i32(args[0]);
        d->fw_version_minor = VESC_IF->lbm_dec_as_i32(args[1]);
        d->fw_version_beta = VESC_IF->lbm_dec_as_i32(args[2]);
    }
    return VESC_IF->lbm_enc_sym_true;
}

// Used to send the current or default configuration to VESC Tool.
static int get_cfg(uint8_t *buffer, bool is_default) {
    data *d = (data *) ARG;

    StigConfig *cfg;
    if (is_default) {
        cfg = VESC_IF->malloc(sizeof(StigConfig));
        if (!cfg) {
            log_error("Failed to send default config to VESC tool: Out of memory.");
            return 0;
        }
        confparser_set_defaults_stigconfig(cfg);
    } else {
        cfg = &d->config;
    }

    int res = confparser_serialize_stigconfig(buffer, cfg);

    if (is_default) {
        VESC_IF->free(cfg);
    }

    return res;
}

// Used to set and write configuration from VESC Tool.
static bool set_cfg(uint8_t *buffer) {
    data *d = (data *) ARG;

    // don't let users use the Stig Cfg "write" button in special modes
    if (d->state.mode != MODE_NORMAL) {
        return false;
    }

    bool res = confparser_deserialize_stigconfig(buffer, &d->config);

    // Store to EEPROM
    if (res) {
        write_cfg_to_eeprom(d);
        configure(d);
        leds_configure(&d->leds, &d->config.leds);
    }

    return res;
}

static int get_cfg_xml(uint8_t **buffer) {
    // Note: As the address of data_stigconfig_ is not known
    // at compile time it will be relative to where it is in the
    // linked binary. Therefore we add PROG_ADDR to it so that it
    // points to where it ends up on the STM32.
    *buffer = data_stigconfig_ + PROG_ADDR;
    return DATA_STIGCONFIG__SIZE;
}

// Called when code is stopped
static void stop(void *arg) {
    data *d = (data *) arg;
    VESC_IF->imu_set_read_callback(NULL);
    VESC_IF->set_app_data_handler(NULL);
    VESC_IF->conf_custom_clear_configs();
    if (d->led_thread) {
        VESC_IF->request_terminate(d->led_thread);
    }
    if (d->main_thread) {
        VESC_IF->request_terminate(d->main_thread);
    }
    log_msg("Terminating.");
    leds_destroy(&d->leds);
    VESC_IF->free(d);
}

INIT_FUN(lib_info *info) {
    INIT_START
    log_msg("Initializing Stig v" PACKAGE_VERSION);

    data *d = VESC_IF->malloc(sizeof(data));
    if (!d) {
        log_error("Out of memory, startup failed.");
        return false;
    }
    data_init(d);

    info->stop_fun = stop;
    info->arg = d;

    VESC_IF->conf_custom_add_config(get_cfg, set_cfg, get_cfg_xml);

    balance_filter_init(&d->balance_filter);
    VESC_IF->imu_set_read_callback(imu_ref_callback);

    footpad_sensor_update(&d->footpad_sensor, &d->config.faults);

    d->main_thread = VESC_IF->spawn(stig_thd, 1024, "Stig Main", d);
    if (!d->main_thread) {
        log_error("Failed to spawn Stig Main thread.");
        return false;
    }

    bool have_leds = leds_init(
        &d->leds, &d->config.hardware.leds, &d->config.leds, d->footpad_sensor.state
    );

    if (have_leds) {
        d->led_thread = VESC_IF->spawn(led_thd, 1024, "Stig LEDs", d);
        if (!d->led_thread) {
            log_error("Failed to spawn Stig LEDs thread.");
            leds_destroy(&d->leds);
        }
    }

    VESC_IF->set_app_data_handler(on_command_received);
    // VESC_IF->lbm_add_extension("ext-dbg", ext_dbg);
    VESC_IF->lbm_add_extension("ext-set-fw-version", ext_set_fw_version);

    return true;
}

void send_app_data_overflow_terminate() {
    VESC_IF->request_terminate(((data *) ARG)->main_thread);
}
