#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

turtle1_pos = None
turtle2_pos = None

pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1)




def turtle_callback1(msg):
	global turtle1_pos, pub1
	turtle1_pos = msg

    
    
def turtle_callback2(msg):
	global turtle2_pos, pub2
	turtle2_pos = msg




def check_distance_and_control(event):
	global turtle1_pos, turtle2_pos, pub1, pub2
	if turtle1_pos is not None and turtle2_pos is not None:
        
		value = (turtle1_pos.x - turtle2_pos.x)**2 + (turtle1_pos.y - turtle2_pos.y)**2
		distance = value**0.5
		
		#distance = min(turtle1_pos.x - turtle2_pos.x,turtle1_pos.y - turtle2_pos.y)
		 
		#print(distance) 
        
		if distance < 1.0:
			stop_vel = Twist()
			stop_vel.linear.x = 0
			stop_vel.linear.y = 0
			pub1.publish(stop_vel)
			pub2.publish(stop_vel)
			

			

			
			

def check_margin_and_control(event):

	global turtle1_pos, turtle2_pos, pub1, pub2
	
	if turtle1_pos is None or turtle2_pos is None:
		return
	
	stop_vel = Twist()
	vel1 = Twist()
	vel2 = Twist()
	
	
	if turtle1_pos.x > 10 or turtle1_pos.x < 1 or turtle1_pos.y > 10 or turtle1_pos.y < 1:
		pub1.publish(stop_vel)
		 
	if turtle2_pos.x > 10 or turtle2_pos.x < 1 or turtle2_pos.y > 10 or turtle2_pos.y < 1:
		pub2.publish(stop_vel)

	


def main():
	global turtle1_pos, turtle2_pos, pub1, pub2
	rospy.init_node('Distance', anonymous=True)
    
	rospy.Subscriber('/turtle1/pose', Pose, turtle_callback1)
	rospy.Subscriber('/turtle2/pose', Pose, turtle_callback2)
    
	rospy.Timer(rospy.Duration(0.0005), check_distance_and_control)
	rospy.Timer(rospy.Duration(0.0005), check_margin_and_control)
    
	rospy.spin()

if __name__ == '__main__':
	main()
