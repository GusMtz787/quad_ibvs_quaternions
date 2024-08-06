# QUAV start-up guide

This document provides a **guide to fly a QUAV** built in the Multi-Robot Systems Laboratory at Tecnológico de Monterrey, Monterrey Campus. The drone is able to fly in two modes: **manual and autonomous**. As to July 2024 the quad-rotor manages a **Raspberry Pi 4** computer paired with a **NAVIO2 autopilot hat** device. What was done up until the mentioned date was a design of a low-level **PID control algorithm** that allows the quad-rotor (QUAV) to stabilize itself using the incoming data from a **VICON Valkyrie** camera system. The Robot Operating System (ROS) framework, C++ and Python programming languages were used for the complete system to work.

It should be noted that this intended goal was not achieved to its entirety, _i.e._ some of the tests showed succesfull results, but other tests did not. There is a hypothesis to this and this will be explained at the end of this document.

All the software development regarding the interaction with the NAVIO2-Ardupilot computer was done based on the official documentation of the NAVIO2 and can be found in the following link [https://docs.emlid.com/navio2/ardupilot/installation-and-running/](https://docs.emlid.com/navio2/ardupilot/installation-and-running/).

## Powering up

> **Safety measure recommended:** if this is your first time working with this type of drone-computer system, the propellers should be **removed** from the motors before powering up the drone as a safety measure.

1. To start the quad-rotor first connect the Li-Po battery to the NAVIO2's voltage divider. Consider that the Li-PO battery should have an optimal complete charge of 12.56 V for maximum flight duration, charge it if it is below 11.3 V.

1. Afterwards, connect the smaller voltage divider's cable to the input of the NAVIO2's power module (which will power the main Raspberry computer too) and connect the larger cable of the voltage divider's to the motor's distribution board. You will hear beeps from the motors while the NAVIO2's autopilot (Ardupilot) starts. As to the date mentioned, the autopilot was set to start automatically when powering the Raspberry.

1. Once the autopilot is automatically engaged, the beeps will stop and this means that the drone is ready to fly. You can also verify this with the upper LED from the NAVIO2, whenever it is ready to fly, this should be as a steady blue. If it is uncalibrated or starting, this will be yellow.

## Manual flying, the fun part

> **Safety measure recommended:** Be **specially CAREFUL** at this step since the QUAV is able to fly now if enabled.

The **activation and deactivation** of the motors works with the **upper left-side gauge** of the Radioshack controller, this will start the motors. Flip it downwards if you want the motors to start, now the drone can be flown. Otherwise, mantain it in the upper position.

## Autonomous flying

The autonomous flying mode is achieved by **disabling the Ardupilot autopilot**. This is a very simple operation and can be found in the official documentation. To **disable** the Ardupilot autopilot run the following command in a terminal from the QUAV's Raspberry computer:

`pi@navio: ~ $ sudo systemctl stop arducopter`

You should start hearing the motors making some noises, this happens everytime Ardupilot's service is correctly stopped.

> **Tip:** in case you want to **start** the Ardupilot's service once again, just run the following command:

`pi@navio: ~ $ sudo systemctl start arducopter`

Now that the Arudpilot's service is disabled, the QUAV can be flown autonomously and the command to do this will be explained after the following safety recommendations.

> **Safety measure recommended:** Be very **CAREFUL** since in this step the QUAV will now be able to **fly by its own**. It is recommended in this stage, even if you are experienced with QUAVs, to tie the quadrotor to a rope within the experimental area. As of July 2024, there is a specific hook in the laboratory to connect a rope to the quadrotor. In case you need to abort the autonomous flying, you can control that the drone does not fall using the rope.

**WARNING:** before launching the drone, read the following safety tip to know how to stop the QUAV during tests.

> **Safety tip:** if for any reason you want to terminate the quadcopter's routine, **press Ctrl-C** on the same terminal were you will run the roslaunch command. The **drone should stop** once the Ctrl-C command is pressed. Consider there may be a little time delay and that the drone will **fall** if the command was executed while flying and the drone was not secured with a rope.

Now that you know the safety measures to stop the drone for an emergency, **open a terminal** and run the following roslaunch file:

`pi@navio: ~ $ roslaunch ibvs quad_VICON.launch`

If you disabled the Ardupilot's service, you should now see the quadrotor power its motors by itself. If you have worked with ROS before, you know that the roslaunch has launched all the necessary nodes needed for the proper working of the drone. In the following section, this nodes will be properly discussed.

## Code

All the code that **enables a proper functioning** of the QUAV will be explained in this section. For the whole system to work, **several nodes** need to be running within ROS. As seen in the _quad_VICON.launch_ file, the needed nodes for the system to fly are the following:

- **Position node:** controls the position of the QUAV.
- **Attitude node:** controls the attitude of the QUAV.
- **Radio control node:** establishes communication between the radio controller and the main computer.
- **PWM publisher node:** calculates the PWM signals needed to control the QUAV based on the information of the position and attitude nodes.
- **PWM sender node:** sends the PWM signals previously calculated to the ESCs.
- **Estimator node:** this node is in charge of estimating the velocities and accelerations of the QUAV based on the data extracted from the VICON camera system.
- **VICON node:** this node was adopted from an ETH university's repository that can be found in the following link [https://github.com/ethz-asl/vicon_bridge](https://github.com/ethz-asl/vicon_bridge). This node acquires the information from the VICON camera system using a socket and the information is then used for the position and attitude calculations.

Consider that, for the PID controller algorithm, only the files ending with the VICON termination were used. If in doubt of which files are relevant to fly the QUAV with the PID controller, check the _quad_VICON.launch_ and verify the scripts used for each of the nodes.

As such, the **desired position**, attitude and velocity of the drone to which the user would like the drone to follow is set within the **position node**, more specifically, the _position_control_VICON.cpp_ file and these can be set within lines 258 to 260:

```c++
attitude_desired << 0.0, 0.0, 0.0;
quad_desired_pos << 0.0, 0.0, 0.5;
quad_desired_vel << 0.0, 0.0, 0.0;
```

For the previous code line, the position is set to (0,0,0.5) for the (X,Y,Z) positions, all desired angles are set to 0° and a 0 m/s for the speed in all directions.

Within the same script, an error between the actual and desired position is calculated as shown in lines 316 and 317.

```c++
error = quad_desired_pos - quad_pos;
error_dot = quad_desired_vel - quad_vel_BF;
```

Using this information, the **thrust** (line 322),

```c++
float thrust_before_saturation = (quad_mass / (cos(quad_att(0))*cos(quad_att(1)))) * (accelerations_desired(2) + gravity + Kp(2)*error(2) + Ki(2)*error_integrated(2) + Kd(2)*error_dot(2));
```

and **desired angles** (lines 352, 363, and 382) are also calculated.

```c++
attitude_desired(2) = 0.0; // For now, yaw is fixed     
```

```c++
roll_des_arg = (quad_mass / thrust) * (sin(attitude_desired(2))*(Kp(0)*error(0) + Ki(0)*error_integrated(0) + Kd(0)*error_dot(0)) - cos(attitude_desired(2))*(Kp(1)*error(1) + Ki(1)*error_integrated(1) + Kd(1)*error_dot(1)));
    ...
    ...
    ...
attitude_desired(0) = asin(roll_des_arg); //Roll desired
```

```c++
pitch_des_arg = ((quad_mass / thrust) * (Kp(0)*error(0) + Ki(0)*error_integrated(0) + Kd(0)*error_dot(0)) - sin(attitude_desired(2))*sin(attitude_desired(0))) / (cos(attitude_desired(2))*cos(attitude_desired(0)));
    ...
    ...
    ...
attitude_desired(1) = asin(pitch_des_arg); //Pitch desired    
```

