#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn

def main():
    rospy.init_node('UI', anonymous=True)

    # Spawn turtle2
    rospy.wait_for_service('/spawn')
    spawn_client = rospy.ServiceProxy('/spawn', Spawn)
    try:
        spawn_response = spawn_client(1.5, 1.0, 0.0, 'turtle2')
        rospy.loginfo(f"Spawned turtle named {spawn_response.name}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return  # Exit if the service call fails

    while not rospy.is_shutdown():
        # Select the turtle
        turtle_id = input("\n\nSelect 1 to move turtle1 or select 2 to move turtle2: ")
        
        # Verify the input
        if turtle_id == '1':
            pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        elif turtle_id == '2':
            pub = rospy.Publisher('/turtle2/cmd_vel', Twist, queue_size=10)
        else:
            rospy.logerr("Invalid turtle: Please select either 1 or 2.")
            continue  # If the selection is invalid, go back to the beginning of the loop

        # Set up velocities for the turtle
        vel = Twist()
        print("\nSet up for the velocities of the turtle\n")
        vel.linear.x = float(input("Linear velocity Vx = "))
        vel.linear.y = float(input("Vy = "))
        vel.linear.z = float(input("Vz = "))

        vel.angular.x = float(input("Angular velocity Vx = "))
        vel.angular.y = float(input("Vy = "))
        vel.angular.z = float(input("Vz = "))

        # Publish the velocity for 1 second
        t0 = rospy.Time.now()
        while (rospy.Time.now() - t0).to_sec() < 1.0:
            pub.publish(vel)

        # Stop the turtle by setting velocities to zero
        stop_vel = Twist()  
        pub.publish(stop_vel)  

        rospy.loginfo("Velocities published for 1 second, then stopped.")
        print("\nReturning to selection...\n")

if __name__ == '__main__':
    main()

