#!/usr/bin/env python3

import rospy
import sys
import time
import threading

import navio2.pwm
import navio2.util

PWM_OUTPUT_MOTOR_1 = 0
PWM_OUTPUT_MOTOR_2 = 1
PWM_OUTPUT_MOTOR_3 = 2
PWM_OUTPUT_MOTOR_4 = 3
SERVO_MIN = 1.250 #ms
SERVO_MAX = 2.000 #ms

# def sender1():

#     #rospy.init_node('pwm_subscriber', anonymous=True)

#     with navio2.pwm.PWM(PWM_OUTPUT_MOTOR_1) as pwm1:
#         time.sleep(1.0) # MEGA IMPORTANT, without a delay the code will give permission errors.
#                         # Got the idea from here: https://github.com/vsergeev/python-periphery/issues/35
#         pwm1.set_period(400)
#         pwm1.enable()

#         print("Enabling ESC 1")
#         pwm1.set_duty_cycle(SERVO_MIN)
#         time.sleep(2)
        
#         while not rospy.is_shutdown():
#             print("Minimums 1")
#             pwm1.set_duty_cycle(SERVO_MIN)
#             time.sleep(2)
#             print("Maximum 1")
#             pwm1.set_duty_cycle(SERVO_MAX)
#             time.sleep(2)

# def sender2():

#     #rospy.init_node('pwm_subscriber', anonymous=True)

#     with navio2.pwm.PWM(PWM_OUTPUT_MOTOR_2) as pwm2:
#         time.sleep(1.0) # MEGA IMPORTANT, without a delay the code will give permission errors.
#                         # Got the idea from here: https://github.com/vsergeev/python-periphery/issues/35
#         #pwm2.set_period(50)
#         #pwm2.enable()

#         print("Enabling ESC 2")
#         pwm2.set_duty_cycle(SERVO_MIN)
#         time.sleep(2)
        
#         while not rospy.is_shutdown():
#             print("Minimums 2")
#             pwm2.set_duty_cycle(SERVO_MIN)
#             time.sleep(2)
#             print("Maximum 2")
#             pwm2.set_duty_cycle(SERVO_MAX)
#             time.sleep(2)

# def sender3():

#     #rospy.init_node('pwm_subscriber', anonymous=True)

#     with navio2.pwm.PWM(PWM_OUTPUT_MOTOR_3) as pwm3:
#         time.sleep(1.0) # MEGA IMPORTANT, without a delay the code will give permission errors.
#                         # Got the idea from here: https://github.com/vsergeev/python-periphery/issues/35
#         #pwm2.set_period(50)
#         #pwm2.enable()

#         print("Enabling ESC 3")
#         pwm3.set_duty_cycle(SERVO_MIN)
#         time.sleep(2)
        
#         while not rospy.is_shutdown():
#             print("Minimums 3")
#             pwm3.set_duty_cycle(SERVO_MIN)
#             time.sleep(2)
#             print("Maximum 3")
#             pwm3.set_duty_cycle(SERVO_MAX)
#             time.sleep(2)

# def sender4():

#     #rospy.init_node('pwm_subscriber', anonymous=True)

#     with navio2.pwm.PWM(PWM_OUTPUT_MOTOR_4) as pwm4:
#         time.sleep(1.0) # MEGA IMPORTANT, without a delay the code will give permission errors.
#                         # Got the idea from here: https://github.com/vsergeev/python-periphery/issues/35
#         #pwm2.set_period(50)
#         #pwm2.enable()

#         print("Enabling ESC 4")
#         pwm4.set_duty_cycle(SERVO_MIN)
#         time.sleep(2)
        
#         while not rospy.is_shutdown():
#             print("Minimums 4")
#             pwm4.set_duty_cycle(SERVO_MIN)
#             time.sleep(2)
#             print("Maximum 4")
#             pwm4.set_duty_cycle(SERVO_MAX)
#             time.sleep(2)

def sender():

    #rospy.init_node('pwm_subscriber', anonymous=True)

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

        print("Enabling ESCs")
        pwm1.set_duty_cycle(SERVO_MIN)
        pwm2.set_duty_cycle(SERVO_MIN)
        pwm3.set_duty_cycle(SERVO_MIN)
        pwm4.set_duty_cycle(SERVO_MIN)
        time.sleep(2)
        
        while not rospy.is_shutdown():
            print("Minimums 4")
            count = 0
            while (count < 2000 and not rospy.is_shutdown()):
                pwm1.set_duty_cycle(SERVO_MAX)
                pwm2.set_duty_cycle(SERVO_MAX)
                pwm3.set_duty_cycle(SERVO_MAX)
                pwm4.set_duty_cycle(SERVO_MAX)
                count = count + 1
            count = 0
            while (count < 2000 and not rospy.is_shutdown()):
                pwm1.set_duty_cycle(SERVO_MIN+0.1)
                pwm2.set_duty_cycle(SERVO_MIN+0.3)
                pwm3.set_duty_cycle(SERVO_MIN+0.1)
                pwm4.set_duty_cycle(SERVO_MIN+0.3)
                count = count + 1

if __name__ == '__main__':
    navio2.util.check_apm()

    sender()

    # t1 = threading.Thread(target=sender1)
    # t2 = threading.Thread(target=sender2)
    # t3 = threading.Thread(target=sender3)
    # t4 = threading.Thread(target=sender4)

    # t1.start()
    # t2.start()
    # t3.start()
    # t4.start()
 
    # t1.join()
    # t2.join()
    # t3.join()
    # t4.join()