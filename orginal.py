#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

class GoToInitialCoordinate:
    def __init__(self):
        rospy.init_node('go_to_initial_coordinate', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.initial_position = Point(0, 0, 0)
        self.current_position = Point()
        self.current_yaw = 0
        self.distance_tolerance = 0.1

    def odom_callback(self, msg):
        # Extract current position from odometry message
        self.current_position = msg.pose.pose.position
        # Convert quaternion to Euler angles to get yaw
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.current_yaw) = euler_from_quaternion(orientation_list)

    def move_to_initial_coordinate(self):
        # Calculate the distance to the initial position
        distance = math.sqrt((self.initial_position.x - self.current_position.x) ** 2 +
                             (self.initial_position.y - self.current_position.y) ** 2)

        # Calculate the angle to the initial position
        desired_angle = math.atan2(self.initial_position.y - self.current_position.y,
                                   self.initial_position.x - self.current_position.x)
        
        # Calculate the angle difference
        angle_difference = desired_angle - self.current_yaw

        # Adjust the angle to be within -pi to pi
        if angle_difference > math.pi:
            angle_difference -= 2 * math.pi
        if angle_difference < -math.pi:
            angle_difference += 2 * math.pi

        # Create Twist message to control the robot's movement
        cmd_vel = Twist()

        # If the robot is far from the initial position, move towards it
        if distance > self.distance_tolerance:
            cmd_vel.linear.x = 0.2  # Linear velocity
            # Adjust angular velocity based on the angle difference
            cmd_vel.angular.z = 0.5 * angle_difference
        else:
            # Stop the robot if it's close enough to the initial position
            cmd_vel.linear.x = 0
            cmd_vel.angular.z = 0

        # Publish the Twist message
        self.pub.publish(cmd_vel)

    def run(self):
        while not rospy.is_shutdown():
            self.move_to_initial_coordinate()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        gotoinitial = GoToInitialCoordinate()
        gotoinitial.run()
    except rospy.ROSInterruptException:
        pass
