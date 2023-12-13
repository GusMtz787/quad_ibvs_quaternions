#!/usr/bin/env python3

import rospy
import sys
import time

import navio2.pwm
import navio2.util

rospy.init_node('pwm_subscriber', anonymous=True)

navio2.util.check_apm()

PWM_OUTPUT = 0
SERVO_MIN = 1.250 #ms
SERVO_MAX = 2.000 #ms

with navio2.pwm.PWM(PWM_OUTPUT) as pwm:
    time.sleep(1.0) # MEGA IMPORTANT, without a delay the code will give permission errors.
                    # Got the idea from here: https://github.com/vsergeev/python-periphery/issues/35
    pwm.set_period(50)
    pwm.enable()

    print("Enabling ESC")
    pwm.set_duty_cycle(SERVO_MIN)
    time.sleep(2)
    
    while not rospy.is_shutdown():
        print("Minimums")
        pwm.set_duty_cycle(SERVO_MIN)
        time.sleep(2)
        print("Maximum")
        pwm.set_duty_cycle(SERVO_MAX)
        time.sleep(2)