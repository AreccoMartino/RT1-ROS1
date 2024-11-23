#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose

# Callback function for receiving the turtle's position
def turtle_callback(msg):
    rospy.loginfo("Turtle subscriber@[%f, %f, %f]", msg.x, msg.y, msg.theta)

def main():
    # Initialize the ROS node
    rospy.init_node('turtlebot_subscriber', anonymous=True)
    
    # Define the subscriber to the turtle's position
    rospy.Subscriber('turtle1/pose', Pose, turtle_callback)
    
    # Keep the node running and listening for incoming messages
    rospy.spin()

if __name__ == '__main__':
    main()

