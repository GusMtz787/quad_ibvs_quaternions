# QUAV start-up guide

Maintainer: Gustavo Olivas

As to July 2024 the quad-rotor manages a **Raspberry Pi 4** computer paired with a **NAVIO2 autopilot hat** device. What was done up until the mentioned date was a design of a low-level **PID control algorithm** that allows the quad-rotor (QUAV) to stabilize itself using the incoming data from a **VICON Valkyrie** camera system.

It should be noted that this intended goal was not achieved to its entirety, _i.e._ some of the tests showed succesfull results, but other tests did not. There is a hypothesis to this and this will be explained at the end of this document.

All the software development regarding the interaction with the NAVIO2 Ardupilot computer was done based on the official documentation and can be found in the following link [https://docs.emlid.com/navio2/ardupilot/installation-and-running/](https://docs.emlid.com/navio2/ardupilot/installation-and-running/).

## Powering up

**Recommended:** if this is your first time working with this type of drone-computer system, the propellers should be **removed** from the motors before powering up the drone as a safety measure.

To start the quad-rotor first connect the Li-Po battery to the NAVIO2's voltage divider. Consider that the Li-PO battery should have an optimal complete charge of 12.56 V for maximum flight duration, charge it if it is below 11.3 V.
Afterwards, connect the smaller voltage divider's cable to the input of the NAVIO2's power module (which will power the main Raspberry computer too) and connect the larger cable of the voltage divider's to the motor's distribution board. You will hear beeps from the motors while the NAVIO2's autopilot (Ardupilot) starts. As to the date mentioned, the autopilot was set to start automatically when powering the Raspberry. Once the autopilot is automatically engaged, the beeps will stop and this means that the drone is ready to fly.

## Manual flying, the fun part

**Be specially CAREFUL** at this step since the QUAV is able to fly now if enabled. The upper left-side gauge of the Radioshack controller will start the motors. Flip it downwards if you want the motors to start. Otherwise, mantain it in the upper position.

## Automatic flying

The automatic flying is achieved by disabling the Ardupilot autopilot. This is a very simple operation 