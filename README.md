# QUAV start-up guide

Last updated: August 2024 - Gustavo Olivas

This document provides a **guide to fly a QUAV** built in the Multi-Robot Systems Laboratory at Tecnológico de Monterrey, Monterrey Campus. The drone is able to fly in two modes: **manual and autonomous**. As of July 2024 the quad-rotor integrates a **Raspberry Pi 4** computer paired with a **NAVIO2 autopilot hat** device. What was done up until the mentioned date was a design of a low-level **PID control algorithm** that allows the quad-rotor (QUAV) to stabilize itself using the incoming data from a **VICON Valkyrie** camera system. The Robot Operating System (ROS) framework, C++, and Python programming languages were used for the complete system to work.

It should be noted that this intended goal was not achieved to its entirety, _i.e._ some of the tests showed succesfull results, but other tests did not. There is a hypothesis to this and this will be explained at the end of this document.

All the software development regarding the interaction with the NAVIO2-Ardupilot computer was done based on the official documentation of the NAVIO2 and can be found in the following link [https://docs.emlid.com/navio2/ardupilot/installation-and-running/](https://docs.emlid.com/navio2/ardupilot/installation-and-running/).

## Powering up

> **Safety measure recommended:** if this is your first time working with this type of drone-computer system, the propellers should be **removed** from the motors before powering up the drone as a safety measure.

1. To start the quad-rotor first connect the Li-Po battery to the NAVIO2's voltage divider. Consider that the Li-PO battery should have an optimal complete charge of 12.56 V for maximum flight duration, charge it if it is below 11.3 V.

1. Afterwards, connect the smaller voltage divider's cable to the input of the NAVIO2's power module (which will power the main Raspberry computer too) and connect the larger cable of the voltage divider's to the motor's distribution board. You will hear beeps from the motors while the NAVIO2's autopilot (Ardupilot) starts. As to the date mentioned, the autopilot was set to start automatically when powering the Raspberry.

1. Once the autopilot is automatically engaged, the beeps will stop and this means that the drone is ready to fly. You can also verify this with the upper LED from the NAVIO2, whenever it is ready to fly, this should be as a steady blue. If it is uncalibrated or starting, this will be a blinking yellow.

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
- **PWM sender node:** sends the PWM signals previously calculated to the ESCs (Electronic Speed Controller).
- **Estimator node:** this node is in charge of estimating the velocities and accelerations of the QUAV based on the data extracted from the VICON camera system.
- **VICON node:** this node was adopted from an ETH university's repository that can be found in the following link [https://github.com/ethz-asl/vicon_bridge](https://github.com/ethz-asl/vicon_bridge). This node acquires the information from the VICON camera system using a socket and the information is then used for the position and attitude calculations.

Consider that, for the PID controller algorithm, only the files ending with the VICON termination were used. If in doubt of which files are relevant to fly the QUAV with the PID controller, check the _quad_VICON.launch_ and verify the scripts used for each of the nodes.

### Position node

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

and **desired angles** (lines 352, 363, and 382) are also calculated according to the **PID controller**.

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

### Attitude node

Now that the thrust and desired angles are known, the only pending variable to calculate are the torques. For this, the **attitude** node is in charge, the _attitude_control_VICON.cpp_ handles this calculation by first specifying the **desired velocities and accelerations** (lines 252 and 253):

```c++
attitude_vel_des << 0.0, 0.0, 0.0;
attitude_acc_des << 0.0, 0.0, 0.0;
```

remembering that the **desired angles** were previously set in the **position node**, and properly **passed to the attitude node**, the next step is calculating an **error** between the desired and real angles (lines 289 and 290):

```c++
error = attitude_des - attitude;
error_dot = attitude_vel_des - attitude_vel;
```

knowing the error between the desired and actual angular position allows the calculation of the needed **torques** following the **PID formulation** (lines 311, 312, and 313):

```c++
tau(0) = Kp(0)*error(0) + Ki(0)*error_integrated(0) + Kd(0)*error_dot(0);

tau(1) = Kp(1)*error(1) + Ki(1)*error_integrated(1) + Kd(1)*error_dot(1);

tau(2) = Kp(2)*error(2) + Ki(2)*error_integrated(2) + Kd(2)*error_dot(2);
```

> **NOTE:** Due to testing phases some of the variables were saturated and others were not, when tuning the drone all variables should be saturated. These saturation limits may be experimental or can be previously calculated according to the available electronics and equipment. Therefore the **limits should be adjusted according** to the system and enabled for all variables.

Now that the control inputs of the system have been calculated (thrust and torques) a **mapping** from thrust/torques to PWM is performed. This is achieved thanks to what is known as the allocation matrix. This is a matrix that relates the torque/thrust variable as a vector with the needed PWM vector based on several factors that will be further explained. As such, the node **PWM calculation node** is the next one to be discussed.

### PWM calculation node

The _pwms.cpp_ file contains the essential elements to relate the **control inputs** to the **PWM signals**. First of all, there is a correlation that can be drawn between the control inputs $\boldsymbol{y} \in \mathbb{R}^{4} $ and the velocities of the motors squared $\boldsymbol{x} \in \mathbb{R}^{4}$ which is given by the allocation matrix $\boldsymbol{A} \in \mathbb{R}^{4 \times 4}$ as expressed in the following equation:

$$\boldsymbol{y} = \boldsymbol{A} \boldsymbol{x}$$

the allocation matrix contains information about the QUAV's model, this matrix is properly presented:

$$
\boldsymbol{A} =
\begin{bmatrix}
    C_{T}   &   C_{T}   &   C_{T}   &   C_{T} \\
    \sin(\theta) L C_{T}  &   \sin(\theta) L C_{T}  &   -\sin(\theta) L C_{T} &   -\sin(\theta) L C_{T} \\
    -\sin(\theta) L C_{T}  &   \sin(\theta) L C_{T} &   \sin(\theta) L C_{T} &   -\sin(\theta) L C_{T} \\
    C_D    &   -C_D &   C_D    &   -C_D
\end{bmatrix}
$$

where $C_{T}$ describes the coefficient of thrust of the motors, $C_{D}$ stands for the motor's torque coefficient, $L$ represents the length of the arm from the center of mass to the rotor, and $\theta$ represents the angle from each of arms of the drone with respect to its centerline in radians. In this case, since a quad-rotor is being studied, the angle is $\theta = 45° = \pi/4$. Plus, the signs are considered based on the body frame used. In this case the NED (North-East-Down) frame was chosen.

> **NOTE:** for matrix $\boldsymbol{A}$, the order of the motors 1,2,3,4 was considered as 1 being the upper-left motor, 2 the downward-left, 3 the downward-right, and 4 the upper-right.

For simplicity by replacing the $\sin(\theta)$ with the respective angle for this drone _i.e._ $\pi/4$, the allocation matrix can be represented as:

$$
\boldsymbol{A} =
\begin{bmatrix}
    C_{T}   &   C_{T}   &   C_{T}   &   C_{T} \\
    0.7071 L C_{T}  &   0.7071 L C_{T}  &   -0.7071 L C_{T} &   -0.7071 L C_{T} \\
    -0.7071 L C_{T}  &   0.7071 L C_{T} &   0.7071 L C_{T} &   -0.7071 L C_{T} \\
    C_D    &   -C_D &   C_D    &   -C_D
\end{bmatrix}
$$

This same matrix was declared in the _pwms.cpp_ file in line 29 with the funky-looking C++ Eigen library sintax:

```c++
static Eigen::Matrix4f A = (Eigen::Matrix4f() << thrust_coefficient, thrust_coefficient, thrust_coefficient, thrust_coefficient,
L * sin_pi_4 * thrust_coefficient, L * sin_pi_4 * thrust_coefficient, -L * sin_pi_4 * thrust_coefficient, -L * sin_pi_4 * thrust_coefficient,
-L * sin_pi_4 * thrust_coefficient, L * sin_pi_4 * thrust_coefficient, L * sin_pi_4 * thrust_coefficient, -L * sin_pi_4 * thrust_coefficient,
torque_coefficient, -torque_coefficient, torque_coefficient, -torque_coefficient).finished();
```

Finally, the whole equation can be expanded using the appropriate terms for clarity:

$$
\begin{equation}
    \begin{bmatrix}
        T_h \\
        \tau_\phi \\
        \tau_\theta \\
        \tau_\psi \\
    \end{bmatrix}
    =
    \begin{bmatrix}
        C_{T}   &   C_{T}   &   C_{T}   &   C_{T} \\
        0.7071 L C_{T}  &   0.7071 L C_{T}  &   -0.7071 L C_{T} &   -0.7071 L C_{T} \\
        -0.7071 L C_{T}  &   0.7071 L C_{T} &   0.7071 L C_{T} &   -0.7071 L C_{T} \\
        C_D    &   -C_D &   C_D    &   -C_D
    \end{bmatrix}
    \begin{bmatrix}
        \Omega_{1}^{2} \\
        \Omega_{2}^{2} \\
        \Omega_{3}^{2} \\
        \Omega_{4}^{2} \\
    \end{bmatrix}
    \notag
\end{equation}
$$

where the control inputs vector $\boldsymbol{x}$ is already known and the speeds are the ones that need to be solved for, therefore:

$$
\boldsymbol{x} = \boldsymbol{A}^{-1} \boldsymbol{y}
$$

as such:

$$
\begin{equation}
    \begin{bmatrix}
        \Omega_{1}^{2} \\
        \Omega_{2}^{2} \\
        \Omega_{3}^{2} \\
        \Omega_{4}^{2} \\
    \end{bmatrix}
    =
    \begin{bmatrix}
        C_{T}   &   C_{T}   &   C_{T}   &   C_{T} \\
        0.7071 L C_{T}  &   0.7071 L C_{T}  &   -0.7071 L C_{T} &   -0.7071 L C_{T} \\
        -0.7071 L C_{T}  &   0.7071 L C_{T} &   0.7071 L C_{T} &   -0.7071 L C_{T} \\
        C_D    &   -C_D &   C_D    &   -C_D
    \end{bmatrix}^{-1}
    \begin{bmatrix}
        T_h \\
        \tau_\phi \\
        \tau_\theta \\
        \tau_\psi \\
    \end{bmatrix}
    \notag
\end{equation}
$$

This same procedure is found in line 73:

```c++
omega << A.inverse() * control_inputs; 
```

Now the velocity squared is known, remember that the vector $\boldsymbol{x}$ has its elements squared by the own nature of the equation. Ideally it would be needed to have the velocities without the square. However, they are left squared for a specific reason that is going to be discussed now. To obtain the **value of a PWM** based on the **velocity of the motor**, experimental tests can be performed on a **dynamometer** using a motor from the quadrotor. As of July 2024, for a EMAX 900KV MT2212 motor (see the specifications here: [https://emaxmodel.com/products/emax-mt2212-900kv-multirotor-motor-cooling-series-with-prop1045-combo#](https://emaxmodel.com/products/emax-mt2212-900kv-multirotor-motor-cooling-series-with-prop1045-combo#)), the relationship that was found is expressed in the following equation:

$$ \text{PWM} = -0.000000001149 \Omega^{4} + 0.00226 \Omega^{2} + 1118$$

As it can be seen, within this 4th degree polynomial regression, the first $\Omega$ is raised to the fourth power and the second one is squared. As such, there is no need to perform a square root operation to the squared velocities that were found.

> **NOTE:** as additional information, consider that ESCs commonly work between PWM signals of 1000 to 2000 mili-seconds.

Therefore, the **PWM values** for all 4 motors are calculated in lines 75 to 78:

```c++
pwm_signal(0) = -0.000000001149 * powf(omega(0), 2) + 0.00226 * omega(0) + 1118;

pwm_signal(1) = -0.000000001149 * powf(omega(1), 2) + 0.00226 * omega(1) + 1118;

pwm_signal(2) = -0.000000001149 * powf(omega(2), 2) + 0.00226 * omega(2) + 1118;

pwm_signal(3) = -0.000000001149 * powf(omega(3), 2) + 0.00226 * omega(3) + 1118;
```

Before closing the topic regarding the PWM calculation, it must be noted that at this moment the PWMs are represented in micro-seconds. However, the hardware requires the value in mili-seconds, as such a last operation is performed to transform them from micro- to mili-seconds:

```c++
pwm_values.w = pwm_signal(3) * 0.001;
pwm_values.x = pwm_signal(1) * 0.001;
pwm_values.y = pwm_signal(0) * 0.001;
pwm_values.z = pwm_signal(2) * 0.001;
```

> **NOTE:** the order of the PWMs were changed. First, remember that the **order** of the motors from the **allocation matrix** was specified as 1 being the upper-left motor, 2 the downward-left, 3 the downward-right, and 4 the upper-right. Now, the variable _pwm_values_ contains the **correct order** in which the **output pins of the NAVIO2** are **conected** to the motors, which is **different from the one presented for matrix** $\boldsymbol{A}$. Therefore, considering the w,x,y,z subsets of the variable _pwm_values_ as motors 1,2,3,4 respectively, then motor 1 is the upper-right one, 2 the downward-left, 3 upper-left, and finally 4 the downward-right one. This explains the **change of order** undergone in the previous code of block, apart from the multiplication operation.

### PWM sender node

Now that the PWM values are known, the software will interact with the hardware of the NAVIO2. To achieve this, another **ROS package** named _pwm_package_ was created with the purpose of isolating the scripts that just perform the calculations and those that interact with the hardware. The Python script _set_pwm.py_ **arms the ESCs** and **sends the PWM values** needed to control the QUAV throughout the whole experiment.

> **NOTE:** At this stage, much of the code to interact with the NAVIO2 hardware was extracted from their **Github page**, which can be found here: [https://github.com/emlid/Navio2](https://github.com/emlid/Navio2).

#### Accessing PWM available pins

First of all, to interact with the PWM pins we need to have **access** to them. As seen in the offcial NAVIO2's Github documentation, we achieve this using the **with()** operator from Python. In line 76, all four PWM pins are accessed in this way:

```c++
with navio2.pwm.PWM(PWM_OUTPUT_MOTOR_1) as pwm1, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_2) as pwm2, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_3) as pwm3, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_4) as pwm4:

    time.sleep(1.0) # MEGA IMPORTANT...

    # Got the idea from here: ...
```

> **NOTE:** It was experimentally found that it is important to **wait for 1 second** for the system to properly access the PWM pins before executing other commands, otherwise the computer may report a **permission error**.

#### Setting operating frequencies and enabling pins

The NAVIO2 requests to set a frequency at which the PWM pins will work, as well as explicitly enable the PWM pins using the _enable()_ function. In this project, the operating frequency was set as 50 Hz, the NAVIO2 supports as much as 400 Hz. This was all programmed in lines 80 through 87:

```c++
pwm1.set_period(50)
pwm2.set_period(50)
pwm3.set_period(50)
pwm4.set_period(50)
pwm1.enable()
pwm2.enable()
pwm3.enable()
pwm4.enable()
```

> **NOTE:** there are certain doubts regarding the operating frequency with the NAVIO2. It was initially believed that **increasing** the frequency would **help** with the **stabilization** of the drone. As such, 400 Hz was also tested but the results were the same. **More experiments** changing this frequency are **recommended**.

#### Arming the ESCs

Before sending the PWMs that will control the system, it is necessary to first **arm the ESCs**. This procedure **may vary between ESCs**, but usually this is achieved by sending a **high signal** for a brief period of time.
As of July 2024, the drone is equipped with 4 Hobbywing Platinum 30A Brushless ESCs, for this specific model of ESCs the arming was achieved by sending a high signal and waiting 1 mili-second. This was written in lines 91 to 95:

```Python
# Check ESC arm
print("Enabling ESCs")
pwm1.set_duty_cycle(SERVO_MAX)
pwm2.set_duty_cycle(SERVO_MAX)
pwm3.set_duty_cycle(SERVO_MAX)
pwm4.set_duty_cycle(SERVO_MAX)
time.sleep(0.001)
print("Finished enabling")
```

#### Sending the PWM

Now that all the steps prior to the normal operation of the ESCs were covered, the PWM needed to control the system can be sent. The PWM previously calculated in the prior node transmits the value through a ROS topic. This PWMs are then limited to a certain threshold and they can be finally sent as seen in lines 110 to 121:

```Python
if (enable == 1 and not maxAngle):
    pwm1.set_duty_cycle(pwm_signals[0])
    pwm2.set_duty_cycle(pwm_signals[1])
    pwm3.set_duty_cycle(pwm_signals[2])
    pwm4.set_duty_cycle(pwm_signals[3])

else:
    #print("Disabling motors")
    pwm1.set_duty_cycle(SERVO_ENABLE)
    pwm2.set_duty_cycle(SERVO_ENABLE)
    pwm3.set_duty_cycle(SERVO_ENABLE)
    pwm4.set_duty_cycle(SERVO_ENABLE)
```

> **NOTE:** for the signals to actually be sent, **two conditions** must be met. First, the **upper-left switch** from the Radioshack controller needs to be **enabled** _i.e._ in the **downward position**. Second, the **roll** and **pitch** angles must be **less** than **0.75 radians** (~ 43°). This is a **safety** measure given that the drone may start a routine and if it gets out of control, the angles will probably exceed this limiting angles. If this happens, then the computer will send the minimum PWM possible, which **doesn't spin** the rotors.

> **NOTE:** The buttons that **enable or disable** certain behaviours **can be modified** within the _RadioControl.py_ file in the _pwm_package_ ROS package.

#### Estimator node

As mentioned before, the QUAV uses the data coming from the **VICON Valkyrie cameras** to **determine** its **current position**. The NAVIO2 incorporates several IMUs to estimate its position, velocity and accelerations. However, this project focused on extracting this information from the cameras and as such, this will be discussed.

The first thing that needs to be considered is that the VICON Valkyrie cameras can only **stream the position and attitude** of any object (last update: August 2024). This means that **velocities and accelerations need to be estimated** in order to have this information. As such, **Dr. Armando Miranda** provided the _estimator.cpp_ file that includes help a novel **fixed-time extended state observer (FxTESO)** to **estimate the velocities and accelerations**. The only thing to do in this file is to make sure that the position and attitude information from the camera is being **accessed** through the **correct ROS topics** within this file. The theory behind this estimator leaves the scope of this comprehensive guide, however the implementation is simple.

First, after making sure that the position and attitude information is being correctly accessed in this file. An **error** can be calculated between the real positio and the estimated position (similar to a controller) as is shown in line 238:

```c++
estimation_error_linear(i) = position(i) - pos_est(i);
```

This operation is performed for **all x-y-z axes**. Then, the FxTESO operation is performed (lines 238 through 245):

```c++
// Proceding with the estimator
x2_dot(i) = G2(i) * sign(estimation_error_linear(i)) * ( powf(std::abs(estimation_error_linear(i)),((lambda(i) + 1)/2))  +  powf(std::abs(estimation_error_linear(i)),(varphi(i) + 1)/2) );
vel_est(i) = vel_est(i) + x2_dot(i) * step;

x1_dot(i) = vel_est(i) + G1(i) * sign(estimation_error_linear(i)) * ( powf(std::abs(estimation_error_linear(i)),(lambda(i) + 2)/3)  +  powf(std::abs(estimation_error_linear(i)),(varphi(i) + 2)/3) );
pos_est(i) = pos_est(i) + x1_dot(i) * step;
```

The same procedure is then repeated for the **attitude** (lines 254 through 261):

```c++
estimation_error_angular(i) = attitude(i) - att_est(i);

// Proceding with the estimator
x2_dot(3+i) = G2(3+i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),((lambda(3+i) + 1)/2))  +  powf(std::abs(estimation_error_angular(i)),(varphi(3+i) + 1)/2) );
attvel_est(i) = attvel_est(i) + x2_dot(3+i) * step;

x1_dot(3+i) = attvel_est(i) + G1(3+i) * sign(estimation_error_angular(i)) * ( powf(std::abs(estimation_error_angular(i)),(lambda(3+i) + 2)/3)  +  powf(std::abs(estimation_error_angular(i)),(varphi(3+i) + 2)/3) );
att_est(i) = att_est(i) + x1_dot(3+i) * step;
```

Consider that the variables _x1_dot_ and _x2_dot_ are the **velocity** and **acceleration** for each axis, respectively. This information is **now available** and it is **streamed** through **ROS topics** for the other nodes to access it when needed.

## Discussion

As was mentioned in the introduction of this documentation, a full autonomous PID-controlled QUAV system hasn't been achieved for all kinds of tests (ignoring the Ardupilot autopilot). **Attitude control** was achieved in many tests, however the **position control** hasn't been finished. Nevertheless, specific problems were detected where potential problems are believed to cause special difficulty to achieve a full attitude and position control:

- It appears that with the current hardware setup and without graphing the telemetry obtained, the maximum frequency at which the control loop ran was of ~170 Hz. It has been found in the literature that higher frequencies are needed for a proper control. This implies that calculating the control law and therefore the control inputs, may potentially be **too slow** if based **completely in ROS**. To **solve** this, a **Teensy 4.1** board was proposed to replace the low-level microcontroller that calculates the control inputs. In other words, the **Raspberry Pi** would be in charge of **receiving the data** from the cameras through WIFI and then this information is **transferred to the Teensy**, which is capabable of running at **higher frequencies** than 170 Hz.

- Another potential problem that was not thoroughly tested is the **update frequency** of the QUAV's **position**. The VICON Valkyrie system is in charge of this, and the ROS topic that enables this update was set to **run at 100 Hz**. It would be beneficial to test if this topic can be set at **higher frequencies** and compare if that **affects the overall performance of the QUAV**. Alternatively, the NAVIO2 contains several IMUs, these can be accessed using the code provided by the NAVIO2's developer on their Github account (showed earlier in this documentation). Processing incoming IMU data may (almost surely) be faster than acquiring the information from the cameras via WIFI.

## Conclusion

With this comprehensive guide, the intention was to **describe** the main **calculations** and **features** that were developed for the QUAV. With it the user is able to fly the quadrotor either **manually**, using Ardupilot's service, or **autonomously**, using a self-programmed PID controller for the latter. For any more questions regarding the development of this project, please contact Dr. Herman Castañeda from Tecnológico de Monterrey.
