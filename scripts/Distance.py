#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time
from turtlesim.srv import TeleportAbsolute

##################### GLOB VAR #####################
turtle1_pos = None
turtle2_pos = None

turtle1_vel = Twist()
turtle2_vel = Twist()

temp_vel_1 = Twist()
temp_vel_2 = Twist()
distance_flag = False

pub1 = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=1)
pub2 = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=1)

turtle1_vel = Twist()
turtle2_vel = Twist()

##################### CALLBACK #####################
def turtle1_vel_callback(msg):
    global turtle1_vel
    turtle1_vel = msg
    
def turtle2_vel_callback(msg):
    global turtle2_vel
    turtle2_vel = msg

def turtle_callback1(msg):
    global turtle1_pos, pub1
    turtle1_pos = msg

def turtle_callback2(msg):
    global turtle2_pos, pub2
    turtle2_pos = msg

##################### SCALE VEL #####################
def scale_velocities(vel):

	scaled_vel = Twist()

	scaled_vel.linear.x  =  -(vel.linear.x   * 0.01)
	scaled_vel.linear.y  =  -(vel.linear.y   * 0.01)
	scaled_vel.angular.z =  -(vel.angular.z  * 0.01)
	#if scaled_vel.linear.x != 0:
		#print(scaled_vel)
	
	return scaled_vel

##################### DISTANCE BETWEEN TURTLES #####################
def check_distance_and_control(event):
	global turtle1_pos, turtle2_pos, pub1, pub2, turtle1_vel, turtle2_vel, temp_vel_1, temp_vel_2, distance_flag
	if turtle1_pos is not None and turtle2_pos is not None:
		value = (turtle1_pos.x - turtle2_pos.x)**2 + (turtle1_pos.y - turtle2_pos.y)**2
		distance = value**0.5
		print(distance)
		
		if distance_flag:
			pub1.publish(temp_vel_1)
			pub2.publish(temp_vel_2)
			if distance > 1.0:
				distance_flag = False
		
		if distance < 1.0 and not distance_flag:
			# Preparation of the velocities to distance the turtles after the stop (to put them again in safe zone)
			# The turtle that was moved go back in the same direction (-vel*0.1)
			temp_vel_1 = scale_velocities(turtle1_vel)
			temp_vel_2 = scale_velocities(turtle2_vel)
		
			# Stop the turtles
			stop_vel = Twist()
			pub1.publish(stop_vel)
			pub2.publish(stop_vel)
			
			# the funcion know that we are unsafe 
			distance_flag = True
			
		
			
##################### IS STOPPED? #####################
def is_turtle_stopped(vel):
    return vel.linear.x == 0 and vel.linear.y == 0 and vel.angular.z == 0

##################### MARGIN CONTROL #####################
def check_margin_and_control(event):
    global turtle1_pos, turtle2_pos, pub1, pub2, turtle1_vel, turtle2_vel

    stop_vel = Twist()

    # Turtle1 control [ Publishing stop_vel the turtle don't go over the wall and with teleport we bring it back in the safe zone ] 
    if turtle1_pos is not None:
        if turtle1_pos.x > 10 or turtle1_pos.x < 1 or turtle1_pos.y > 10 or turtle1_pos.y < 1:
            pub1.publish(stop_vel)
            rospy.logwarn("Turtle1 fuori dai limiti. Fermata.")
            
            rospy.wait_for_service("/turtle1/teleport_absolute")
            client1 = rospy.ServiceProxy("/turtle1/teleport_absolute", TeleportAbsolute)
            if turtle1_pos.x > 10:  # Limite destro
                client1(9.999, turtle1_pos.y, turtle1_pos.theta)
            elif turtle1_pos.x < 1:  # Limite sinistro
                client1(1.001, turtle1_pos.y, turtle1_pos.theta)
            elif turtle1_pos.y > 10:  # Limite superiore
                client1(turtle1_pos.x, 9.999, turtle1_pos.theta)
            elif turtle1_pos.y < 1:  # Limite inferiore
                client1(turtle1_pos.x, 1.001, turtle1_pos.theta)

    # Turtle2 control [ Same logic as before ]
    if turtle2_pos is not None:
        if turtle2_pos.x > 10 or turtle2_pos.x < 1 or turtle2_pos.y > 10 or turtle2_pos.y < 1:
            pub2.publish(stop_vel)
            rospy.logwarn("Turtle2 fuori dai limiti. Fermata.")

            rospy.wait_for_service("/turtle2/teleport_absolute")
            client2 = rospy.ServiceProxy("/turtle2/teleport_absolute", TeleportAbsolute)
            if turtle2_pos.x > 10:  # Limite destro
                client2(9.999, turtle2_pos.y, turtle2_pos.theta)
            elif turtle2_pos.x < 1:  # Limite sinistro
                client2(1.001, turtle2_pos.y, turtle2_pos.theta)
            elif turtle2_pos.y > 10:  # Limite superiore
                client2(turtle2_pos.x, 9.999, turtle2_pos.theta)
            elif turtle2_pos.y < 1:  # Limite inferiore
                client2(turtle2_pos.x, 1.001, turtle2_pos.theta)

##################### MAIN #####################
def main():
	global turtle1_pos, turtle2_pos, pub1, pub2
	rospy.init_node('Distance', anonymous=True)
    
	rospy.Subscriber('/turtle1/pose', Pose, turtle_callback1)
	rospy.Subscriber('/turtle2/pose', Pose, turtle_callback2)
    
	rospy.Subscriber('/turtle1/cmd_vel', Twist, turtle1_vel_callback)
	rospy.Subscriber('/turtle2/cmd_vel', Twist, turtle2_vel_callback)

	rospy.Timer(rospy.Duration(0.0005), check_distance_and_control)
	rospy.Timer(rospy.Duration(0.0005), check_margin_and_control)
    
	rospy.spin()

##################### #### #####################
if __name__ == '__main__':
    main()

