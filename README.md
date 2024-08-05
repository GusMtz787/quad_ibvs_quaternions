# QUAV start-up guide

Maintainer: Gustavo Olivas

This document provides a **guide to fly a QUAV** built in the Multi-Robot Systems Laboratory at Tecnológico de Monterrey, Monterrey Campus. The drone is able to fly in two modes: **manual and automatic**. As to July 2024 the quad-rotor manages a **Raspberry Pi 4** computer paired with a **NAVIO2 autopilot hat** device. What was done up until the mentioned date was a design of a low-level **PID control algorithm** that allows the quad-rotor (QUAV) to stabilize itself using the incoming data from a **VICON Valkyrie** camera system.

It should be noted that this intended goal was not achieved to its entirety, _i.e._ some of the tests showed succesfull results, but other tests did not. There is a hypothesis to this and this will be explained at the end of this document.

All the software development regarding the interaction with the NAVIO2-Ardupilot computer was done based on the official documentation of the NAVIO2 and can be found in the following link [https://docs.emlid.com/navio2/ardupilot/installation-and-running/](https://docs.emlid.com/navio2/ardupilot/installation-and-running/).

## Powering up

**Safety measure recommended:** if this is your first time working with this type of drone-computer system, the propellers should be **removed** from the motors before powering up the drone as a safety measure.

1. To start the quad-rotor first connect the Li-Po battery to the NAVIO2's voltage divider. Consider that the Li-PO battery should have an optimal complete charge of 12.56 V for maximum flight duration, charge it if it is below 11.3 V.
1. Afterwards, connect the smaller voltage divider's cable to the input of the NAVIO2's power module (which will power the main Raspberry computer too) and connect the larger cable of the voltage divider's to the motor's distribution board. You will hear beeps from the motors while the NAVIO2's autopilot (Ardupilot) starts. As to the date mentioned, the autopilot was set to start automatically when powering the Raspberry.
1. Once the autopilot is automatically engaged, the beeps will stop and this means that the drone is ready to fly. You can also verify this with the upper LED from the NAVIO2, whenever it is ready to fly, this should be as a steady blue. If it is uncalibrated or starting, this will be yellow.

## Manual flying, the fun part

**Be specially CAREFUL** at this step since the QUAV is able to fly now if enabled. The upper left-side gauge of the Radioshack controller will start the motors. Flip it downwards if you want the motors to start, now the drone can be flown. Otherwise, mantain it in the upper position.

## Automatic flying

The automatic flying mode is achieved by disabling the Ardupilot autopilot. This is a very simple operation and can be found in the official documentation. To disable the Ardupilot autopilot run the following command in a terminal from the QUAV's Raspberry computer:

`pi@navio: ~ $ sudo systemctl start arducopter`