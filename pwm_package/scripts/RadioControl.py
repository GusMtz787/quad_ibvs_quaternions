#!/usr/bin/env python3

import rospy, time
from std_msgs.msg import Int32
import navio2.rcinput
import navio2.util

rcin = navio2.rcinput.RCInput()

def rc_reader():

    # Initialize the ROS node with a unique name
    rospy.init_node('rc_input_publisher', anonymous=True)

    # Create a publisher for the topics
    pubArm = rospy.Publisher('rcArm', Int32, queue_size=10)
    pubMode = rospy.Publisher('rcMode', Int32, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():

        arm = rcin.read(9)
        mode = rcin.read(8)
        #print(mode)

        # Consider: if arm is 1932 then ARMED (1), else DISARM (0)
        if int(arm) == 1932:
            arm = 1
        else:
            arm = 0

        # Consider: if mode 1933 then execute VISUAL SERVOING (1), else VICON control (0)
        if int(mode) == 1933:
            mode = 1
        else:
            mode = 0
        
        # Publish the messages on the topics
        pubArm.publish(arm)
        pubMode.publish(mode)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':

    navio2.util.check_apm()

    rc_reader()
