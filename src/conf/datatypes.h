// Copyright 2022 Benjamin Vedder <benjamin@vedder.se>
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

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    INPUTTILT_NONE = 0,
    INPUTTILT_UART,
    INPUTTILT_PPM
} FLOAT_INPUTTILT_REMOTE_TYPE;

typedef enum {
    LED_TYPE_NONE = 0,
    LED_TYPE_RGB,
    LED_TYPE_RGBW,
    LED_TYPE_EXTERNAL,
} LedType;

typedef enum {
    LED_PIN_B6 = 0,
    LED_PIN_B7
} LedPin;

typedef enum {
    COLOR_BLACK = 0,
    COLOR_WHITE_FULL,
    COLOR_WHITE_RGB,
    COLOR_WHITE_SINGLE,
    COLOR_RED,
    COLOR_FERRARI,
    COLOR_FLAME,
    COLOR_CORAL,
    COLOR_SUNSET,
    COLOR_SUNRISE,
    COLOR_GOLD,
    COLOR_ORANGE,
    COLOR_YELLOW,
    COLOR_BANANA,
    COLOR_LIME,
    COLOR_ACID,
    COLOR_SAGE,
    COLOR_GREEN,
    COLOR_MINT,
    COLOR_TIFFANY,
    COLOR_CYAN,
    COLOR_STEEL,
    COLOR_SKY,
    COLOR_AZURE,
    COLOR_SAPPHIRE,
    COLOR_BLUE,
    COLOR_VIOLET,
    COLOR_AMETHYST,
    COLOR_MAGENTA,
    COLOR_PINK,
    COLOR_FUCHSIA,
    COLOR_LAVENDER,
} LedColor;

typedef enum {
    LED_MODE_SOLID = 0,
    LED_MODE_FADE,
    LED_MODE_PULSE,
    LED_MODE_STROBE,
    LED_MODE_KNIGHT_RIDER
} LedMode;

typedef enum {
    LED_TRANS_FADE = 0,
    LED_TRANS_FADE_OUT_IN,
    LED_TRANS_CIPHER,
    LED_TRANS_MONO_CIPHER,
} LedTransition;

typedef struct {
    float brightness;
    LedColor color1;
    LedColor color2;
    LedMode mode;
    float speed;
} LedBar;

typedef struct {
    uint16_t idle_timeout;
    float duty_threshold;
    float red_bar_percentage;
    bool show_sensors_while_running;
    float brightness_headlights_on;
    float brightness_headlights_off;
} StatusBar;

typedef struct {
    bool on;
    bool headlights_on;

    LedTransition headlights_transition;
    LedTransition direction_transition;

    bool lights_off_when_lifted;
    bool status_on_front_when_lifted;

    LedBar headlights;
    LedBar taillights;
    LedBar front;
    LedBar rear;
    StatusBar status;
    LedBar status_idle;
} CfgLeds;

typedef struct {
    uint8_t count;
    bool reverse;
} CfgLedStrip;

typedef struct {
    LedType type;
    LedPin pin;
    CfgLedStrip status;
    CfgLedStrip front;
    CfgLedStrip rear;
} CfgHwLeds;

typedef struct {
    FLOAT_INPUTTILT_REMOTE_TYPE type;
    bool invert_throttle;
} CfgHwRemote;

typedef struct {
    uint16_t frequency;
    bool is_beeper_enabled;
    float imu_x_offset;
} CfgHwEsc;

typedef struct {
    float torque_constant;
    float rolling_resistance;
    float acceleration_resistance;
} CfgHwMotor;

typedef struct {
    CfgHwMotor motor;
    CfgHwEsc esc;
    CfgHwLeds leds;
    CfgHwRemote remote;
    // TODO sensor?
    float mass;
} CfgHardware;

typedef struct {
    float mass;
    float height;
    float drag_coefficient;
} CfgRider;

typedef struct {
    float tiltback_return_speed;

    float duty_threshold;
    float duty_ramp;
    float duty_speed;

    uint8_t battery_cells;
    float lv_threshold;
    float hv_threshold;

    // TODO buzz struct?
    float buzz_strength;
    float buzz_strength_variable;  // TODO rename?
    float buzz_frequency;
    float buzz_speed;

    bool is_dutybeep_enabled;
    // bool is_footbeep_enabled;
    bool is_surgebeep_enabled;
} CfgWarnings;

typedef struct {
    float pitch_tolerance;
    float roll_tolerance;
    float speed;
    bool simplestart_enabled;
    bool pushstart_enabled;
    bool dirtylandings_enabled;
} CfgStartup;

typedef struct {
    float pitch_threshold;
    float roll_threshold;
    float adc1_threshold;
    float adc2_threshold;
    uint16_t pitch_delay;
    uint16_t roll_delay;
    uint16_t switch_half_delay;
    uint16_t switch_full_delay;
    float switch_half_speed;
    bool is_posi_enabled;
    bool moving_fault_disabled;
    bool is_reversestop_enabled;
} CfgFaults;

typedef struct {
    float mahony_kp;
    float mahony_kp_roll;
    float accel_confidence_decay;
    float az_filter;
    // bool use_bf_fix;
} CfgBalanceFilter;

typedef struct {
    float kp;
    float kd;
    float kd_filter;
    float ki;
    float ki_limit;
    float kp_brake;
    float kd_brake;
    float soft_start;
} CfgPid;

typedef struct {
    float drop_strength;
    float drop_threshold;
    float filter;
    float wheelslip_strength;
    float wheelslip_threshold;
    float soft_release_winddown;
    float mod_winddown;
} CfgTraction;

typedef struct {
    float strength_up;
    float strength_down;
    float strength_boost;
    float tightness;
    float threshold;
    float angle_limit;
    // float speed;
    // float speed_max;
    // float speed_boost;
} CfgAtr;

typedef struct {
    float strength;
    float strength_regen;
    float strength_boost;
    float threshold;
    float angle_limit;
    // float speed;
    // float speed_max;
    float turn_boost;
    float method;
    float filter;
} CfgTorqueTilt;

typedef struct {
    float strength;
    float strength_boost;
    float angle_limit;
    // float speed;
    // float speed_max;
    float start_angle;
    uint16_t start_erpm;
    float filter;
} CfgTurnTilt;

typedef struct {
    float constant;
    float variable;
    // float variable_max;
    // float speed;
} CfgSpeedTilt;

typedef enum {
    ACCELERATION_SOURCE_ERPM = 0,
    ACCELERATION_SOURCE_IMU,
} AccelerationSource;

// TODO move modifiers here
typedef struct {
    float angle_limit;
    float speed_limit;
    float filter;
    float board_speed_filter;
    AccelerationSource acceleration_source;
} CfgModifiers;

typedef struct {
    float speed_limit;
    float angle_limit;
    float filter;
    float threshold;
} CfgInputTilt;

typedef struct {
    CfgBalanceFilter balance_filter;
    CfgPid pid;
    CfgTraction traction;
    CfgModifiers modifiers;
    CfgAtr atr;
    CfgTorqueTilt torque_tilt;
    CfgTurnTilt turn_tilt;
    CfgSpeedTilt speed_tilt;
    CfgInputTilt input_tilt;
} CfgTune;

typedef struct {
    float version;
    bool disabled;

    CfgHardware hardware;
    CfgRider rider;

    CfgTune tune;
    CfgStartup startup;
    CfgFaults faults;
    CfgWarnings warnings;
    CfgLeds leds;

    float brake_current;

    float surge_duty_start;
    float surge_angle;
} StigConfig;

// DATATYPES_H_
#endif
