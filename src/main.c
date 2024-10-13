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
#include "traction.h"

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
    Traction traction;
    Warnings warnings;
    HapticBuzz haptic_buzz;

    // Beeper
    int beep_num_left;
    int beep_duration;
    int beep_countdown;
    int beep_reason;
    bool beeper_enabled;

    // Config values
    float loop_time;

    float current_time;
    float last_time;
    float loop_overshoot_filtered;
    // float computation_time;

    float startup_pitch_trickmargin;
    float startup_pitch_tolerance;

    float startup_step_size;
    float tiltback_return_step_size;

    float setpoint;
    float setpoint_target;
    float setpoint_target_interpolated;

    float disengage_timer;
    float nag_timer;
    float idle_voltage;

    float fault_angle_pitch_timer;
    float fault_angle_roll_timer;
    float fault_switch_full_timer;
    float fault_switch_half_timer;
    float fault_ghost_timer;

    float brake_timeout;
    float wheelslip_timer;

    // Feature: Reverse Stop
    float reverse_stop_step_size;
    float reverse_tolerance;
    float reverse_total_distance;
    float reverse_timer;

    bool use_strong_brake;

    // float current_requested;

    // Odometer
    // float odo_timer;
    // int odometer_dirty;
    // uint64_t odometer;

} data;

static void brake(data *d);
// static void set_current(float current);

const VESC_PIN beeper_pin = VESC_PIN_PPM;

#define EXT_BEEPER_ON() VESC_IF->io_write(beeper_pin, 1)
#define EXT_BEEPER_OFF() VESC_IF->io_write(beeper_pin, 0)

void beeper_init() {
    VESC_IF->io_set_mode(beeper_pin, VESC_PIN_MODE_OUTPUT);
}

void beeper_update(data *d) {
    if (d->beeper_enabled && (d->beep_num_left > 0)) {
        d->beep_countdown--;
        if (d->beep_countdown <= 0) {
            d->beep_countdown = d->beep_duration;
            d->beep_num_left--;
            if (d->beep_num_left & 0x1) {
                EXT_BEEPER_ON();
            } else {
                EXT_BEEPER_OFF();
            }
        }
    }
}

void beeper_enable(data *d, bool enable) {
    d->beeper_enabled = enable;
    if (!enable) {
        EXT_BEEPER_OFF();
    }
}

void beep_alert(data *d, int num_beeps, bool longbeep) {
    if (!d->beeper_enabled) {
        return;
    }
    if (d->beep_num_left == 0) {
        d->beep_num_left = num_beeps * 2 + 1;
        d->beep_duration = longbeep ? 300 : 80;
        d->beep_countdown = d->beep_duration;
    }
}

// void beep_alert_idle(data *d) {
//     // alert user after 30 minutes
//     if (d->current_time - d->disengage_timer > 1800) {
//         // beep every 60 seconds
//         if (d->current_time - d->nag_timer > 60) {
//             d->nag_timer = d->current_time;
//             const float input_voltage = VESC_IF->mc_get_input_voltage_filtered();
//             if (input_voltage > d->idle_voltage) {
//                 // don't beep if the voltage keeps increasing (board is charging)
//                 d->idle_voltage = input_voltage;
//             } else {
//                 d->beep_reason = BEEP_IDLE;
//                 beep_alert(d, 2, 1);
//             }
//         }
//     } else {
//         d->nag_timer = d->current_time;
//         d->idle_voltage = 0;
//     }
// }

void beep_off(data *d, bool force) {
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_OFF();
    }
}

void beep_on(data *d, bool force) {
    if (!d->beeper_enabled) {
        return;
    }
    // don't mess with the beeper if we're in the process of doing a multi-beep
    if (force || (d->beep_num_left == 0)) {
        EXT_BEEPER_ON();
    }
}

static void configure(data *d) {
    state_init(&d->state, d->config.disabled);

    d->loop_time = 1.0f / d->config.hardware.esc.frequency;

    balance_filter_configure(&d->balance_filter, &d->config.tune.balance_filter);

    imu_data_configure(&d->imu, &d->config.tune.traction, d->config.hardware.esc.imu_x_offset, d->loop_time);
    motor_data_configure(&d->motor, &d->config.tune, &d->config.hardware, &d->config.rider, d->loop_time);
    // remote_data_configure(&d->remote, &d->config.tune.input_tilt, d->loop_time);
    lcm_configure(&d->lcm, &d->config.leds);

    // Tune modifiers
    modifiers_configure(&d->modifiers, &d->config.tune, d->loop_time);
    input_tilt_configure(&d->input_tilt, &d->config.tune.input_tilt, d->loop_time);

    pid_configure(&d->pid, &d->config.tune.pid, d->loop_time);
    traction_configure(&d->traction, &d->config.tune.traction, d->loop_time);
    warnings_configure(&d->warnings, &d->config.warnings);
    // haptic_buzz_configure(&d->haptic_buzz, &d->config.warnings);

    d->disengage_timer = d->current_time;

    d->startup_step_size = d->config.startup.speed * d->loop_time;
    d->tiltback_return_step_size = d->config.warnings.tiltback_return_speed * d->loop_time;

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
    d->reverse_stop_step_size = 100.0 * d->loop_time;

    d->beeper_enabled = d->config.hardware.esc.is_beeper_enabled;

    if (d->state.state == STATE_DISABLED) {
        beep_alert(d, 3, false);
    } else {
        beep_alert(d, 1, false);
    }
}

static void init_vars(data *d) {
    imu_data_init(&d->imu);
    motor_data_init(&d->motor);
    // remote_data_init(&d->remote);
    traction_init(&d->traction);
    // d->use_strong_brake = false;

    warnings_init(&d->warnings);
    haptic_buzz_init(&d->haptic_buzz);

    d->setpoint = d->imu.pitch_balance;
    d->setpoint_target_interpolated = d->imu.pitch_balance;
    d->setpoint_target = 0.0f;

    d->brake_timeout = 0.0f;

    d->startup_pitch_tolerance = d->config.startup.pitch_tolerance;

    // d->current_requested = 0.0f;
}

static void reset_vars(data *d) {
    float time_disengaged = d->current_time - d->disengage_timer;
    // const float alpha = half_time_to_alpha(0.1f, time_disengaged);
    float alpha = clamp(time_disengaged, 0.0f, 1.0f);

    modifiers_reset(&d->modifiers, alpha);
    input_tilt_reset(&d->input_tilt);
    pid_reset(&d->pid, &d->config.tune.pid, alpha);

    filter_ema(&d->setpoint, d->imu.pitch_balance, alpha);
    filter_ema(&d->setpoint_target_interpolated, d->imu.pitch_balance, alpha);
    filter_ema(&d->setpoint_target, 0.0f, alpha);
    
    d->brake_timeout = 0.0f;

    d->startup_pitch_tolerance = d->config.startup.pitch_tolerance;

    // state_engage(&d->state);
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

static float get_setpoint_adjustment_step_size(const data *d) {
    switch (d->state.sat) {
    case (SAT_NONE):
        return d->tiltback_return_step_size;
    case (SAT_CENTERING):
        return d->startup_step_size;
    case (SAT_REVERSESTOP):
        return d->reverse_stop_step_size;
    default:
        return 0.0f;
    }
}

static bool is_sensor_engaged(const data *d) {
    if (d->footpad_sensor.state == FS_BOTH) {
        return true;
    }

    if (d->footpad_sensor.state == FS_NONE) {
        return false;
    }

    // Half Engaged:

    bool board_ghosted = d->state.stop_condition == STOP_GHOST;
    bool after_disengage = d->current_time - d->disengage_timer < 2.0f;
    bool after_ghost = board_ghosted && after_disengage;
    bool riding_backwards = d->motor.board_speed < - d->config.faults.ghost_speed;

    if (after_ghost || riding_backwards) {
        return false;
    }

    return true;

    // if (d->config.faults.is_posi_enabled) {
    //     return true;
    // }

    // if (d->config.startup.simplestart_enabled) {
    //     // simple start is disabled for a few seconds after disengaging
    //     if (d->current_time - d->disengage_timer > 2.0f) {
    //         return true;
    //     }
    // }
}

static bool check_faults(data *d) {
    // CfgFaults cfg = d->config.faults;

    bool disable_switch_faults =
        d->config.faults.moving_fault_disabled &&
        d->motor.board_speed > 0.5f &&
        fabsf(d->imu.pitch) < 40 && fabsf(d->imu.roll) < 75;

    // Switch fully open
    if (d->footpad_sensor.state == FS_NONE) {
        if (!disable_switch_faults) {
            float switch_full_time = d->current_time - d->fault_switch_full_timer;
            float switch_full_delay = 0.001f * d->config.faults.switch_full_delay;

            if (switch_full_time > switch_full_delay) {
                state_stop(&d->state, STOP_SWITCH_FULL);
                return true;
            }

            // float switch_half_delay = 0.001f * d->config.faults.switch_half_delay;
            // bool is_slow = d->motor.speed_abs < d->config.faults.switch_half_speed * 6;

            // } else if (switch_full_time > switch_half_delay && is_slow) {
            //     state_stop(&d->state, STOP_SWITCH_FULL);
            //     return true;
            // }
        }

        // Feature: Quick Stop
        // TODO only under load?
        if (d->motor.speed_abs < 0.2f && fabsf(d->imu.pitch) > 14.0f &&
            sign(d->imu.pitch) == d->motor.speed_sign) {
            state_stop(&d->state, STOP_QUICKSTOP);
            return true;
        }
    } else {
        d->fault_switch_full_timer = d->current_time;
    }

    // Feature: Reverse-Stop
    if (d->state.sat == SAT_REVERSESTOP) {
        //  Taking your foot off entirely while reversing? Ignore delays
        if (d->footpad_sensor.state == FS_NONE) {
            state_stop(&d->state, STOP_SWITCH_FULL);
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

    // Switch partially open and stopped
    // if (!d->config.faults.is_posi_enabled) {
    //     bool is_below_switch_half_speed = d->motor.speed_abs < d->config.faults.switch_half_speed;

    //     if (!is_sensor_engaged(d) && is_below_switch_half_speed) {
    //         float switch_half_time = d->current_time - d->fault_switch_half_timer;
    //         float switch_half_delay = 0.001f * d->config.faults.switch_half_delay;

    //         if (switch_half_time > switch_half_delay) {
    //             state_stop(&d->state, STOP_SWITCH_HALF);
    //             return true;
    //         }
    //     } else {
    //         d->fault_switch_half_timer = d->current_time;
    //     }
    // }

    // Feature: Ghost Safeguard
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
    if (fabsf(d->imu.pitch - d->input_tilt.interpolated) > d->config.faults.pitch_threshold) {
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

static void calculate_setpoint_target(data *d) {
    if (d->state.sat == SAT_REVERSESTOP) {
        // accumalete distance:
        d->reverse_total_distance += d->motor.speed * d->loop_time;
        if (fabsf(d->reverse_total_distance) > d->reverse_tolerance) {
            // tilt down by 10 degrees after 0.06 m
            d->setpoint_target = 10.0f * (fabsf(d->reverse_total_distance) - d->reverse_tolerance) / 0.06f;
        } else {
            if (fabsf(d->reverse_total_distance) <= d->reverse_tolerance / 2.0f) {
                if (d->motor.speed >= 0.0f) {
                    d->state.sat = SAT_NONE;
                    d->reverse_total_distance = 0.0f;
                    d->setpoint_target = 0.0f;
                    d->pid.integral = 0.0f;
                }
            }
        }
    } else if (
        fabsf(d->motor.wheel_accel) > 1.0f &&  // not normal, either wheelslip or wheel getting stuck
        sign(d->motor.wheel_accel) == d->motor.speed_sign &&
        d->motor.duty_cycle > 0.3f &&
        d->motor.speed_abs > 2.0f)  // acceleration can jump a lot at very low speeds
    {
        d->state.wheelslip = true;
        d->state.sat = SAT_NONE;
        d->wheelslip_timer = d->current_time;
    } else if (d->state.wheelslip) {
        // Remain in wheelslip state for at least 500ms to avoid any overreactions
        if (d->motor.duty_cycle > d->motor.duty_max - 0.1f) {
            d->wheelslip_timer = d->current_time;
        } else if (d->current_time - d->wheelslip_timer > 0.2f) {
            if (d->motor.duty_cycle < 0.7f) {
                // Leave wheelslip state only if duty < 70%
                d->state.wheelslip = false;
            }
        }
        if (d->config.faults.is_reversestop_enabled && (d->motor.speed < 0)) {
            // the 500ms wheelslip time can cause us to blow past the reverse stop condition!
            d->state.sat = SAT_REVERSESTOP;
            d->reverse_timer = d->current_time;
            d->reverse_total_distance = 0;
        }
    } else if (d->state.sat != SAT_CENTERING || d->setpoint_target_interpolated == d->setpoint_target) {
        // Normal running
        if (d->config.faults.is_reversestop_enabled && d->motor.speed < -0.2f) {
            d->state.sat = SAT_REVERSESTOP;
            d->reverse_timer = d->current_time;
            d->reverse_total_distance = 0;
        } else {
            d->state.sat = SAT_NONE;
        }
        d->setpoint_target = 0;
    }

    if (d->state.wheelslip && d->motor.duty_cycle > d->motor.duty_max - 0.1f) {
        d->setpoint_target = 0;
    }
}

static void calculate_setpoint_interpolated(data *d) {
    if (d->setpoint_target_interpolated != d->setpoint_target) {
        rate_limitf(
            &d->setpoint_target_interpolated,
            d->setpoint_target,
            get_setpoint_adjustment_step_size(d)
        );
    }
}

static bool startup_conditions_met(data *d) {
    if (!is_sensor_engaged(d)) {
        return false;
    }

    bool is_pitch_valid = fabsf(d->imu.pitch_balance) < d->startup_pitch_tolerance;
    bool is_roll_valid = fabsf(d->imu.roll) < d->config.startup.roll_tolerance;

    if (is_pitch_valid && is_roll_valid) {
        return true;
    }

    // TODO only positive speed when ghost safeguard is enabled
    // bool is_push_start = d->config.startup.pushstart_enabled && d->motor.speed_abs > 1.0f;
    bool is_push_start = d->config.startup.pushstart_enabled && d->motor.speed > 1.0f;
    if (is_push_start && (fabsf(d->imu.pitch_balance) < 45) && is_roll_valid) {
        return true;
    }

    return false;
}

static void brake(data *d) {
    float brake_timeout_length = 1.0f;  // Brake Timeout hard-coded to 1s
    if (fabsf(d->motor.erpm) > ERPM_MOVING_THRESHOLD || d->brake_timeout == 0.0f) {
        d->brake_timeout = d->current_time + brake_timeout_length;
    }

    if (d->brake_timeout != 0.0f && d->current_time > d->brake_timeout) {
        return;
    }

    VESC_IF->timeout_reset();

    if (d->motor.speed_abs > 0.25f) {
        d->use_strong_brake = false;
    } else if (d->motor.speed_abs < 0.05f) {
        d->use_strong_brake = true;
    }

    if (d->use_strong_brake) {
        VESC_IF->mc_set_duty(0);
    } else {
        VESC_IF->mc_set_brake_current(d->config.brake_current);
    }
}

static void set_current(float current, const MotorData *mot) {
    float current_limit = mot->braking ? mot->current_min : mot->current_max;
    float current_limited = clamp_sym(current, current_limit);

    VESC_IF->timeout_reset();
    VESC_IF->mc_set_current_off_delay(0.025f);
    VESC_IF->mc_set_current(current_limited);
}

// static void limit_current(float *current, const MotorData *mot) {
//     float current_limit = mot->braking ? mot->current_min : mot->current_max;
//     *current = clamp_sym(*current, current_limit);
//     // TODO soft max?
// }

static void imu_ref_callback(float *acc, float *gyro, float *mag, float dt) {
    unused(mag);

    data *d = (data *) ARG;
    balance_filter_update(&d->balance_filter, gyro, acc, dt);
}

static void time_vars_update(data *d) {
    d->current_time = VESC_IF->system_time();
    float loop_time_measured = d->current_time - d->last_time;
    d->last_time = d->current_time;

    // TODO measure loop overshoot only when running
    float loop_overshoot = loop_time_measured - d->loop_time;

    // TODO use 2nd order IIR
    filter_ema(&d->loop_overshoot_filtered, loop_overshoot, 0.002f);
}

static void stig_thd(void *arg) {
    data *d = (data *) arg;

    configure(d);  // XXX configure before init
    init_vars(d);
    d->last_time = VESC_IF->system_time() - d->loop_time;

    // VESC_IF->plot_init("Time", "Y");
    // VESC_IF->plot_add_graph("something");

    while (!VESC_IF->should_terminate()) {
        time_vars_update(d);
        
        beeper_update(d);
        charging_timeout(&d->charging, &d->state);

        imu_data_update(&d->imu, &d->balance_filter);
        motor_data_update(&d->motor, d->config.hardware.esc.frequency, &d->imu);
        remote_data_update(&d->remote, &d->config.hardware.remote);
        footpad_sensor_update(&d->footpad_sensor, &d->config.faults);
        traction_update(&d->traction, &d->config.tune.traction, &d->imu, &d->motor);

        // motor_update_post_traction(&d->motor, &d->traction);

        // Control Loop State Logic
        switch (d->state.state) {
        case (STATE_STARTUP):
            brake(d);

            if (VESC_IF->imu_startup_done()) {
                state_engage(&d->state);
                d->state.state = STATE_READY;

                // TODO
                // // if within 5V of LV tiltback threshold, issue 1 beep for each volt below that
                // float bat_volts = VESC_IF->mc_get_input_voltage_filtered();
                // float threshold = d->config.warnings.lv.threshold + 5.0f;
                // if (bat_volts < threshold) {
                //     int beeps = (int) fminf(6.0f, threshold - bat_volts);
                //     beep_alert(d, beeps + 1, true);
                //     d->beep_reason = BEEP_LOWBATT;
                // } else {
                //     // Let the rider know that the board is ready (one long beep)
                //     beep_alert(d, 1, true);
                // }
            }
            break;

        case (STATE_RUNNING):
            // Check for faults
            // TODO put switching into STATE_READY outside check_faults() ?
            if (check_faults(d)) {
                if (d->state.stop_condition == STOP_SWITCH_FULL) {
                    // dirty landings: add extra margin
                    d->startup_pitch_tolerance =
                        d->config.startup.pitch_tolerance + d->startup_pitch_trickmargin;
                    d->fault_angle_pitch_timer = d->current_time;
                }
                break;
            }
            // d->odometer_dirty = 1;

            d->disengage_timer = d->current_time;

            // Calculate setpoint and interpolation
            calculate_setpoint_target(d);
            calculate_setpoint_interpolated(d);
            d->setpoint = d->setpoint_target_interpolated;

            float confidence = 1.0f - d->traction.traction_soft_release;

            modifiers_update(
                &d->modifiers,
                &d->config.tune,
                &d->motor,
                &d->imu,
                confidence
            );

            input_tilt_update(
                &d->input_tilt,
                &d->config.tune.input_tilt,
                &d->remote
            );

            d->setpoint += d->modifiers.interpolated;
            d->setpoint += d->input_tilt.interpolated;

            // d->balance_filter.kp_mult = 1.0f - fabsf(d->modifiers.atr.interpolated * d->config.tune.atr.tightness);
            d->balance_filter.kp_mult = 1.0f;

            pid_update(&d->pid, &d->imu, &d->motor, &d->config.tune.pid, d->setpoint, confidence);

            bool warning_ghost = d->current_time - d->fault_ghost_timer > 0.0f;
            bool warning_debug = d->traction.slip_factor > 0.5f;
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

            float torque_requested = d->pid.pid_value * d->traction.multiplier;
            // torque_requested = clamp_sym(torque_requested, d->config.tune.torque_limit)  // XXX
            // torque_requested += d->haptic_buzz.buzz_output;

            float current_requested = torque_requested / d->motor.c_torque;
            set_current(current_requested, &d->motor);
            break;

        case (STATE_READY):
            // beep_alert_idle(d);

            if ((d->current_time - d->fault_angle_pitch_timer) > 1.0f) {
                // 1 second after disengaging - set startup tolerance back to normal (aka tighter)
                d->startup_pitch_tolerance = d->config.startup.pitch_tolerance;
            }

            // check_odometer(d);

            if (startup_conditions_met(d)) {
                reset_vars(d);
                state_engage(&d->state);
                break;
            }

            brake(d);
            break;

        case (STATE_DISABLED):
            break;
        }

        haptic_buzz_update(
            &d->haptic_buzz,
            &d->config.haptics,
            &d->motor,
            d->warnings.type,
            d->state.state
        );

        // d->computation_time = VESC_IF->system_time() - d->current_time;
        // const float loop_time_correction = d->computation_time + d->loop_overshoot_filtered;
        // VESC_IF->sleep_us(1e6 * max(d->loop_time - loop_time_correction, 0));

        VESC_IF->sleep_us(1e6 * fmaxf(d->loop_time - d->loop_overshoot_filtered, 0.0f));
        // VESC_IF->sleep_us(1e6 * d->loop_time);
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

    beep_alert(d, 1, 0);
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

    lcm_init(&d->lcm, &d->config.hardware.leds);
    charging_init(&d->charging);
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
        d->config.tune.pid.ki = 0.0f;
        d->config.tune.pid.kp_brake = 1.0f;
        d->config.tune.pid.kd_brake = 1.0f;
        d->config.tune.torque_tilt.strength = 0.0f;
        d->config.tune.torque_tilt.strength_regen = 0.0f;
        d->config.tune.atr.strength_up = 0.0f;
        d->config.tune.atr.strength_down = 0.0f;
        d->config.tune.turn_tilt.strength = 0.0f;
        d->config.tune.speed_tilt.constant = 0.0f;
        d->config.tune.speed_tilt.variable = 0.0f;
        d->config.faults.pitch_threshold = 30;
        d->config.faults.roll_threshold = 30;
    } else {
        read_cfg_from_eeprom(&d->config);
        configure(d);
    }
}

static void send_realtime_data(data *d) {
    static const int bufsize = 79 + 8;
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
    buffer_append_float32_auto(buffer, d->remote.throttle, &ind);

    buffer_append_float32_auto(buffer, d->imu.board_accel, &ind);
    buffer_append_float32_auto(buffer, d->motor.wheel_accel, &ind);

    if (d->state.state == STATE_RUNNING) {
        // Setpoints
        buffer_append_float32_auto(buffer, d->setpoint, &ind);

        buffer_append_float32_auto(buffer, d->modifiers.target, &ind);
        buffer_append_float32_auto(buffer, d->modifiers.interpolated, &ind);
        buffer_append_float32_auto(buffer, d->modifiers.speed, &ind);
        buffer_append_float32_auto(buffer, d->traction.traction_soft_release, &ind);
        buffer_append_float32_auto(buffer, d->motor.slope, &ind);

        // PID
        buffer_append_float32_auto(buffer, d->pid.proportional, &ind);
        buffer_append_float32_auto(buffer, d->pid.integral, &ind);
        buffer_append_float32_auto(buffer, d->pid.derivative, &ind);

        // Debug
        buffer_append_float32_auto(buffer, d->motor.speed, &ind);
        buffer_append_float32_auto(buffer, d->motor.board_speed, &ind);
        buffer_append_float32_auto(buffer, d->motor.debug, &ind);  // free
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

    if ((d->config.hardware.esc.is_beeper_enabled) ||
        (d->config.hardware.remote.type != INPUTTILT_PPM)) {
        beeper_init();
    }

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
