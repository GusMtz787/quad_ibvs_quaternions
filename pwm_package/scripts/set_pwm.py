#!/usr/bin/env python3

import rospy
import time
import numpy as np
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

import navio2.pwm
import navio2.util

PWM_OUTPUT_MOTOR_1 = 0
PWM_OUTPUT_MOTOR_2 = 1
PWM_OUTPUT_MOTOR_3 = 2
PWM_OUTPUT_MOTOR_4 = 3
SERVO_ENABLE = 1.00 #ms
SERVO_MIN = SERVO_ENABLE + 0.3 #mS
SERVO_MAX = 2.000 #

pwm_signals = np.array([0.0,0.0,0.0,0.0])
stop = 0 # 0 means DON'T stop, 1 means DO STOP

def fix_below_threshold(arr, threshold, replacement_value):
    # Find the indices of elements below the threshold
    below_threshold_indices = arr < threshold

    # Replace elements below the threshold with the replacement value
    arr[below_threshold_indices] = replacement_value

def callback_pwm(pwms):
    pwm_signals[0] = pwms.w * 0.001 
    pwm_signals[1] = pwms.x * 0.001
    pwm_signals[2] = pwms.y * 0.001
    pwm_signals[3] = pwms.z * 0.001

def cmd_vel_callback(msg):
    global stop
    stop = msg.angular.z

def sender():

    # Set ROS node parameters
    rospy.init_node('pwm_subscriber', anonymous=True)
    rospy.Subscriber("pwm_values", Quaternion, callback_pwm)
    rospy.Subscriber("cmd_vel", Twist, cmd_vel_callback)

    # Enable PWM signals
    with navio2.pwm.PWM(PWM_OUTPUT_MOTOR_1) as pwm1, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_2) as pwm2, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_3) as pwm3, navio2.pwm.PWM(PWM_OUTPUT_MOTOR_4) as pwm4:
        time.sleep(1.0) # MEGA IMPORTANT, without a delay the code will give permission errors.
                        # Got the idea from here: https://github.com/vsergeev/python-periphery/issues/35
        
        pwm1.set_period(400)
        pwm2.set_period(400)
        pwm3.set_period(400)
        pwm4.set_period(400)
        pwm1.enable()
        pwm2.enable()
        pwm3.enable()
        pwm4.enable()

        # Check ESC arm
        print("Enabling ESCs")
        pwm1.set_duty_cycle(SERVO_MAX)
        pwm2.set_duty_cycle(SERVO_MAX)
        pwm3.set_duty_cycle(SERVO_MAX)
        pwm4.set_duty_cycle(SERVO_MAX)
        time.sleep(5)
        
        # Enter PWM signal loop
        while not rospy.is_shutdown():

            fix_below_threshold(pwm_signals, SERVO_MIN, SERVO_MIN)

            # FAIL-SAFE FEATURE: 
            # stop: this variable will be 1 if the "J" keyboard is pressed within the commanding laptop.
            # if the stop is 1, then the motors will rotate at its slowest velocity.
            print(stop)

            if (stop == 1):
                print("Disabling motors")
                pwm1.set_duty_cycle(SERVO_ENABLE)
                pwm2.set_duty_cycle(SERVO_ENABLE)
                pwm3.set_duty_cycle(SERVO_ENABLE)
                pwm4.set_duty_cycle(SERVO_ENABLE)

            else:
                pwm1.set_duty_cycle(pwm_signals[0])
                pwm2.set_duty_cycle(pwm_signals[1])
                pwm3.set_duty_cycle(pwm_signals[2])
                pwm4.set_duty_cycle(pwm_signals[3])

            # count = 0
            # while (count < 2000 and not rospy.is_shutdown()):
            #     pwm1.set_duty_cycle(SERVO_MAX)
            #     pwm2.set_duty_cycle(SERVO_MAX)
            #     pwm3.set_duty_cycle(SERVO_MAX)
            #     pwm4.set_duty_cycle(SERVO_MAX)
            #     count = count + 1
            # count = 0
            # while (count < 2000 and not rospy.is_shutdown()):
            #     pwm1.set_duty_cycle(SERVO_MIN)
            #     pwm2.set_duty_cycle(SERVO_MIN)
            #     pwm3.set_duty_cycle(SERVO_MIN)
            #     pwm4.set_duty_cycle(SERVO_MIN)
            #     count = count + 1

if __name__ == '__main__':
    
    navio2.util.check_apm() # Check ardupilot is NOT running

    sender() # Energize motors
