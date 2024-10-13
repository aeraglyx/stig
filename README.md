# Stig
Experimental self-balancing skateboard package based on the [Refloat](https://github.com/lukash/refloat) package by Mitch Lustig, Dado Mista, Nico Aleman and Lukáš Hrázký.

This is my personal, stripped down version of Refloat. I do not care about backwards compatibility or 3rd party app support. There may be bugs. Please read the [warnings](#warnings) before riding.

## Features
- Improved ATR
    - Rewritten to estimate true road slope using longitudinal vehicle dynamics.
    - Independent of rider's weight and motor's characteristics
    - Strength of 1 should be completely parallel to the ground.
    - To learn more, see [ATR docs](docs/atr.md).
- Improved Torque Tilt
    - TT now takes true acceleration into account, which makes it a lot more "symmetric".
    - Option to boost TT based on Yaw Rate. Similar to Turn Tilt, but for keeping corners off the ground regardless of whether you're accelerating or braking.

- Modifier Smoothing
    - Using a 2nd order IIR filter for modifiers and input tilt smoothing.
- Traction Control
    - Option to reduce torque when a drop or wheelslip is detected.
- Rudimentary ghost safeguard
- Miscellaneous
    - Use torque [Nm] instead of current [A] for almost everything.
    - Use m/s instead of ERPM for most speed-based things. As a rule of thumb, 1000 ERPM ≈ 1 m/s.
    - Filters use half time [s] instead of frequency [Hz].
    - Frequency independency
    - More user friendly increments in Cfg.

## Fresh Installation
Make sure you have a backup of your configuration for reverting back. After installing this package, configure your **Specs** in **Stig Cfg**, namely:

- Cells in Series (e.g. ADV has a 20s2p battery, so use 20)
- Double check ADC Voltages
- Rider Mass and Board Mass
- Motor Torque Constant (see [ATR docs](docs/atr.md))
- LED Type is set to External Module by default (changes require reboot)

## Migrating from Refloat
Many parameters are not compatible with Refloat, so it is best to start with the defaults. Here are some notable parameter changes to keep in mind:

- Battery warnings are based on a single cell's voltage, so the thresholds should be around 3.0 and 4.3 and generally don't need to be changed. Instead, you must specify the number of Cells in Series (Cfg > Specs).
- P and D terms are based on torque instead of current and in general the numbers will be lower than in Refloat.
- I Term is frequency independent, the typical values are now 0-15 (instead of ~0.005).
- Higher tune modifier tiltback speeds are now possible in combination with setpoint filtering.
- Filters are now expressed in seconds (half time) instead of Hz.
- ATR strength of 1 now means parallel to the ground and you should need roughly 4x lower numbers (if you had 1.2, try 0.3).
- Torque Tilt also has a different base unit, 0.1 should feel pretty decent.
- Speeds are in [m/s] instead of ERPMs (1 m/s ≈ 2.2 mph ≈ 3.6 km/h ≈ 1000 ERPM).

## Warnings

⚠️ **Requires 6.05+ firmware!**

⚠️ **There are no Safety Tiltbacks!** Instead the only warning system in place is a version of "Haptic Feedback", so make sure it's configured properly.

Heel lift is no longer possible, posi FTW.

Although some parameter names remain the same, many have been changed and can't use Refloat's equivalent values. See the section above for more details.

Just like Refloat, Mahony KP is configured in the package, so App Cfg > IMU > Mahony KP should be closer to 0.4, not 2. Similarly, acceleration Z filtering has been moved from App Cfg to the Package Cfg. So AppCfg > IMU > Accel Z Filter should be 0.

Read the descriptions of any settings you are unsure of (some might be outdated, sorry).

## Known Issues
- 3rd party apps (Floaty, Float Control) not supported
- Tune Archive and Quicksaves not working
- Odometer disabled

## Code Gotchas
- Speeds are in [m/s]
- Accelerations are in [g]

## Disclaimer
Use at your own risk. Electric vehicles are inherently dangerous, authors of this package shall not be liable for any damage or harm caused by errors in the software. Not endorsed by the VESC project.

## Credits
Author: Vladislav Macíček

Original Float/Refloat package authors: Mitch Lustig, Dado Mista, Nico Aleman, Lukáš Hrázký

## Download (TODO)
https://github.com/aeraglyx/stig/releases
