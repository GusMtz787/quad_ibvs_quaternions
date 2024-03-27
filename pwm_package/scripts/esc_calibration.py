#!/usr/bin/env python3

import rospy
import time
import numpy as np
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32
import math

import navio2.pwm
import navio2.util

PWM_OUTPUT_MOTOR_1 = 0
PWM_OUTPUT_MOTOR_2 = 1
PWM_OUTPUT_MOTOR_3 = 2
PWM_OUTPUT_MOTOR_4 = 3
#SERVO_ENABLE = 1.00 #ms
SERVO_MIN = 1.0 #mS
SERVO_MAX = 2.0 #

pwm_signals = np.array([0.0, 0.0, 0.0, 0.0])

def sender():

    # Enable PWM signals
    with navio2.pwm.PWM(PWM_OUTPUT_MOTOR_1) as pwm1, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_2) as pwm2, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_3) as pwm3, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_4) as pwm4:
        time.sleep(1.0) # MEGA IMPORTANT, without a delay the code will give permission errors.
                        # Got the idea from here: https://github.com/vsergeev/python-periphery/issues/35
        
        pwm1.set_period(400)
        # pwm2.set_period(400)
        # pwm3.set_period(400)
        # pwm4.set_period(400)
        pwm1.enable()
        # pwm2.enable()
        # pwm3.enable()
        # pwm4.enable()

        # Calibrate ESC
        print("Calibrating ESCs")
        print("HIGH signal")
        pwm1.set_duty_cycle(SERVO_MAX)
        # pwm2.set_duty_cycle(SERVO_MAX)
        # pwm3.set_duty_cycle(SERVO_MAX)
        # pwm4.set_duty_cycle(SERVO_MAX)
        time.sleep(2)

        print("LOW signal")
        pwm1.set_duty_cycle(SERVO_MIN)
        # pwm2.set_duty_cycle(SERVO_MIN)
        # pwm3.set_duty_cycle(SERVO_MIN)
        # pwm4.set_duty_cycle(SERVO_MIN)
        time.sleep(2)
        print("Finished calibration")

if __name__ == '__main__':
    
    navio2.util.check_apm() # Check ardupilot is NOT running

    sender() # Energize motors

