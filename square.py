#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import sqrt

class MoveAndStop:
    def __init__(self):
        rospy.init_node('move_and_stop_node', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.initial_x = None
        self.goal_reached = False
        self.rate = rospy.Rate(10)  # 10 Hz

    def odom_callback(self, msg):
        if not self.goal_reached:
            if self.initial_x is None:
                self.initial_x = msg.pose.pose.position.x
            current_x = msg.pose.pose.position.x
            distance_moved = abs(current_x - self.initial_x)
            if distance_moved >= 1.0:
                self.stop_robot()
                self.goal_reached = True
                rospy.loginfo("Goal reached!")

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = 0.2  # Adjust speed as needed
        self.cmd_vel_pub.publish(twist)

    def run(self):
        rospy.loginfo("Moving the robot forward...")
        while not rospy.is_shutdown() and not self.goal_reached:
            self.move_forward()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        mover = MoveAndStop()
        mover.run()
    except rospy.ROSInterruptException:
        pass
