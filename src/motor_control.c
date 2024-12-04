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

#include "motor_control.h"

#include "conf/datatypes.h"
#include "utils.h"
#include "vesc_c_if.h"

#include <math.h>

void motor_control_init(MotorControl *mc) {
    mc->is_torque_requested = false;
    mc->requested_torque = 0.0f;

    mc->brake_timeout = 0.0f;
    mc->use_strong_brake = false;

    mc->tone.amplitude = 0.0f;
    mc->tone.frequency = 0;
    mc->tone.t = 0.0f;
}

void motor_control_configure(MotorControl *mc, const StigConfig *cfg) {
    mc->brake_current = cfg->brake_current;
    // mc->parking_brake_mode = cfg->parking_brake_mode;
}

void motor_control_request_torque(MotorControl *mc, float torque) {
    mc->is_torque_requested = true;
    mc->requested_torque = torque;
}

void motor_control_play_tone(MotorTone *tone, uint16_t frequency, float amplitude) {
    tone->frequency = frequency;
    tone->amplitude = amplitude;
}

void motor_control_stop_tone(MotorTone *tone) {
    tone->amplitude = 0.0f;
    tone->t = 0.0f;
}

static void set_current(float torque, const MotorData *motor) {
    float current = torque / motor->c_torque;

    float current_limit = motor->braking ? motor->current_min : motor->current_max;
    float current_limited = clamp_sym(current, current_limit);

    VESC_IF->mc_set_current_off_delay(0.025f);
    VESC_IF->mc_set_current(current_limited);
}

static void set_brake(MotorControl *mc, const MotorData *motor, float time) {
    if (fabsf(motor->erpm) > ERPM_MOVING_THRESHOLD) {
        mc->brake_timeout = time + 1.0f;
    }

    if (time > mc->brake_timeout) {
        VESC_IF->mc_set_current(0.0f);
        return;
    }

    if (mc->use_strong_brake) {
        VESC_IF->mc_set_duty(0);
    } else {
        VESC_IF->mc_set_brake_current(mc->brake_current);
    }
}

static float tone_output(MotorTone *tone, float dt) {
    if (tone->amplitude == 0.0f) {
        return 0.0f;
    }

    tone->t += tone->frequency * dt;
    tone->t = tone->t - (int) tone->t;
    float wave = sin_scaled(tone->t);
    return wave * tone->amplitude;
}

void motor_control_apply(MotorControl *mc, const MotorData *motor, float time) {
    // TODO use strong brake after a ghost
    // TODO parking brake mode
    if (motor->speed_abs > 0.25f) {
        mc->use_strong_brake = false;
    } else if (motor->speed_abs < 0.05f) {
        mc->use_strong_brake = true;
    }

    // Reset VESC Firmware safety timeout
    VESC_IF->timeout_reset();

    if (mc->is_torque_requested) {
        mc->requested_torque += tone_output(&mc->tone, motor->dt);
        set_current(mc->requested_torque, motor);
    } else {
        set_brake(mc, motor, time);
    }

    mc->is_torque_requested = false;
    mc->requested_torque = 0.0f;
}
