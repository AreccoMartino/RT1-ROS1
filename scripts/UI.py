#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from threading import Timer


##################### SPAWN #####################
def spawn_turtle():
	# Spawn turtle2
    rospy.wait_for_service('/spawn')
    spawn_client = rospy.ServiceProxy('/spawn', Spawn)
    try:
        spawn_response = spawn_client(1.5, 1.0, 0.0, 'turtle2')
        rospy.loginfo(f"Spawned turtle named {spawn_response.name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return  # Exit if the service call fails

##################### TURTLE #####################
def turtle_choice():
	while True:
		# Select the turtle
		turtle_id = input("\n\nSelect 1 to move turtle1 or select 2 to move turtle2: ")
        
		# Verify the input
		if turtle_id == '1':
			pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
			return pub
		elif turtle_id == '2':
			pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
			return pub
		else:
			rospy.logerr("Invalid turtle: Please select either 1 or 2.")
			# continue  # If the selection is invalid, go back to the beginning of the loop
	
	

##################### VEL #####################
def vel_choice():
	# Set up velocities for the turtle
	vel = Twist()

	while True:
		print("\nSet up for the velocities of the turtle\n")
		try:
			vel.linear.x = float(input("Vx = "))
			vel.linear.y = float(input("Vy = "))
			vel.angular.z = float(input("Wz = "))
			break  # Esce dal loop se tutti gli input sono corretti
		except ValueError:
			print("Error: Please select a valid velocity value.")
	
	return vel
	
##################### MOVE #####################
def move_turtle(pub,vel):
	# Publish the velocity for 1 second
	
	pub.publish(vel)
	stop_vel = Twist()  
	# After one second the turtle is stopped 
	t = Timer(1.0, lambda: pub.publish(stop_vel))
	t.start()

	rospy.loginfo("Velocities published for 1 second, then stopped.")
	print("\nReturning to selection...\n")

##################### MAIN #####################
def main():
	rospy.init_node('UI', anonymous=True)

	spawn_turtle()

	while not rospy.is_shutdown():
        
		pub = turtle_choice()
        
		vel = vel_choice()

		move_turtle(pub,vel)

##################### #### #####################
if __name__ == '__main__':
    main()
