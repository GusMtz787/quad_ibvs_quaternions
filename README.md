# QUAV start-up guide

Maintainer: Gustavo Olivas

As to July 2024 the quad-rotor manages a **Raspberry Pi 4** computer paired with a **NAVIO2 autopilot hat** device. What was done up until the mentioned date was a design of a low-level **PID control algorithm** that allows the quad-rotor (QUAV) to stabilize itself using the incoming data from a **VICON Valkyrie** camera system.

## Powering up

**Recommended:** if this is your first time working with this type of drone-computer system, the propellers should be removed from the motors before powering up the drone as a safety measure.

To start the quad-rotor first connect the Li-Po battery to the NAVIO2's voltage divider. Consider that the Li-PO battery should have an optimal complete charge of 12.56 V for maximum flight duration, charge it if it is below 11.3 V.
Afterwards, connect the smaller voltage divider's cable to the input of the NAVIO2's power module (which will power the main Raspberry computer too) and connect the larger cable of the voltage divider's to the motor's distribution board. You will hear beeps from the motors while the NAVIO2's autopilot (Ardupilot) starts. As to the date mentioned, the autopilot was set to start automatically when powering the Raspberry. Once the autopilot is automatically engaged, the beeps will stop and this means that the drone is ready to fly.

## Flying, the fun part

**Be specially CAREFUL** at this step since the QUAV is able to fly now if enabled. The upper left-side gauge of the Radioshack controller will start the motors. Flip it downwards if you want the motors to start. Otherwise, mantain it on the upper position.

