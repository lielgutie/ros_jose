#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

class CircleRobot:
    def __init__(self):
        rospy.init_node('circle_robot', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz
        self.twist = Twist()

    def move(self):
        # Set linear velocity in x direction
        self.twist.linear.x = 0.2  # You can adjust the linear velocity as needed

        # Set angular velocity in z direction to make the robot turn
        self.twist.angular.z = 0.5  # You can adjust the angular velocity as needed

        # Publish the Twist message
        self.pub.publish(self.twist)

    def run(self):
        while not rospy.is_shutdown():
            self.move()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        robot = CircleRobot()
        robot.run()
    except rospy.ROSInterruptException:
        pass
