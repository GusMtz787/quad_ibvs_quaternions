#!/usr/bin/env python3

import rospy
import time
import numpy as np
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Int32

import navio2.pwm
import navio2.util

PWM_OUTPUT_MOTOR_1 = 0
PWM_OUTPUT_MOTOR_2 = 1
PWM_OUTPUT_MOTOR_3 = 2
PWM_OUTPUT_MOTOR_4 = 3
SERVO_ENABLE = 1.00 #ms
SERVO_MIN = SERVO_ENABLE + 0.4 #mS
SERVO_MAX = 2.000 #

pwm_signals = np.array([0.0,0.0,0.0,0.0])
enable = 0 # 0 means DON'T start, 1 means START

def fix_below_threshold(arr, threshold):
    # # Find the indices of elements below the threshold
    # below_threshold_indices = arr < threshold

    # # Replace elements below the threshold with the replacement value
    # arr[below_threshold_indices] = replacement_value
    for i in range(len(arr)):
        if arr[i] < threshold:
            arr[i] = threshold


def callback_pwm(pwms):
    pwm_signals[0] = pwms.w * 0.001 
    pwm_signals[1] = pwms.x * 0.001
    pwm_signals[2] = pwms.y * 0.001
    pwm_signals[3] = pwms.z * 0.001

    fix_below_threshold(pwm_signals, SERVO_MIN)

def callback_rcArm(msg):
    global enable
    enable = msg.data

def sender():

    # Set ROS node parameters
    rospy.init_node('pwm_subscriber', anonymous=True)
    rospy.Subscriber("pwm_values", Quaternion, callback_pwm)
    rospy.Subscriber("rcArm", Int32, callback_rcArm)
    pwm_values_final = rospy.Publisher('pwm_values_final', Quaternion, queue_size=10)

    rate = rospy.Rate(300)  # 100 Hz

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

        # Initialize pwm variable to publish of type
        # quaternion just because it has 4 elements already.
        pwm_final = Quaternion()
        
        # Enter PWM signal loop
        while not rospy.is_shutdown():

            # FAIL-SAFE FEATURE: 
            # enable variable controls if the motors will spin or not.
            # if enable is 1, then the motors are set to spin, else they will stop.

            if (enable == 1):
                print(pwm_signals[0])
                pwm1.set_duty_cycle(pwm_signals[0])
                pwm2.set_duty_cycle(pwm_signals[1])
                pwm3.set_duty_cycle(pwm_signals[2])
                pwm4.set_duty_cycle(pwm_signals[3])

            else:
                print("Disabling motors")
                pwm1.set_duty_cycle(SERVO_ENABLE)
                pwm2.set_duty_cycle(SERVO_ENABLE)
                pwm3.set_duty_cycle(SERVO_ENABLE)
                pwm4.set_duty_cycle(SERVO_ENABLE)

            pwm_final.w = pwm_signals[0]
            pwm_final.x = pwm_signals[1]
            pwm_final.y = pwm_signals[2]
            pwm_final.z = pwm_signals[3]

            pwm_values_final.publish(pwm_final)

            rate.sleep()

if __name__ == '__main__':
    
    navio2.util.check_apm() # Check ardupilot is NOT running

    sender() # Energize motors
