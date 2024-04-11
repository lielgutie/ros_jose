#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CircleTurtle:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('circle_turtle', anonymous=True)
        
        # Publisher to publish velocity commands
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Set the loop rate
        self.rate = rospy.Rate(1) # 1 Hz
        
    def move_in_circle(self):
        # Create a Twist message
        velocity_msg = Twist()
        
        # Set linear velocity
        velocity_msg.linear.x = 1.0  # m/s
        velocity_msg.linear.y = 0.0
        velocity_msg.linear.z = 0.0
        
        # Set angular velocity
        velocity_msg.angular.x = 0.0
        velocity_msg.angular.y = 0.0
        velocity_msg.angular.z = 1.0  # rad/s
        
        # Publish the Twist message
        self.velocity_publisher.publish(velocity_msg)
        
    def run(self):
        # Run the node until shutdown
        while not rospy.is_shutdown():
            # Move in circle
            self.move_in_circle()
            
            # Sleep according to the loop rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        circle_turtle = CircleTurtle()
        circle_turtle.run()
    except rospy.ROSInterruptException:
        pass
