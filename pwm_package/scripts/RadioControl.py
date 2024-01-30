#!/usr/bin/env python3

import rospy, time
from std_msgs.msg import Int32
import navio2.rcinput
import navio2.util

rcin = navio2.rcinput.RCInput()

def rc_reader():

    # Initialize the ROS node with a unique name
    rospy.init_node('rc_input_publisher', anonymous=True)

    # Create a publisher for the 'chatter' topic, publishing messages of type String
    pub = rospy.Publisher('rcArm', Int32, queue_size=10)

    # Set the loop rate (in Hz)
    rate = rospy.Rate(10)  # 1 Hz

    while not rospy.is_shutdown():

        arm = rcin.read(9)
        print(arm)

        if int(arm) == 1932:
            arm = 1
        else:
            arm = 0
        
        # Publish the message on the 'chatter' topic
        pub.publish(arm)

        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':

    navio2.util.check_apm()

    rc_reader()
