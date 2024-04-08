#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtlePIDController:
    def __init__(self):
        rospy.init_node('turtle_pid_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

        # PID constants
        self.kp = 1.0  # Proportional constant
        self.ki = 0.1  # Integral constant
        self.kd = 0.01  # Derivative constant

        # Control variables
        self.error = 0.0
        self.error_integral = 0.0
        self.error_derivative = 0.0
        self.last_error = 0.0

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def calculate_error(self, goal_pose):
        self.error = math.sqrt((goal_pose.x - self.pose.x) ** 2 + (goal_pose.y - self.pose.y) ** 2)
        self.error_derivative = self.error - self.last_error
        self.error_integral += self.error
        self.last_error = self.error

    def control_pid(self, goal_pose):
        while self.error > 0.1:
            self.calculate_error(goal_pose)

            # PID control formula
            angular_speed = self.kp * self.error + self.ki * self.error_integral + self.kd * self.error_derivative

            # Linear velocity (constant)
            vel_msg = Twist()
            vel_msg.linear.x = 1.0
            vel_msg.angular.z = angular_speed

            self.velocity_publisher.publish(vel_msg)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = TurtlePIDController()

        goal_pose = Pose()
        goal_pose.x = float(input("Introduce la coordenada x del objetivo: "))
        goal_pose.y = float(input("Introduce la coordenada y del objetivo: "))
        theta_degrees = float(input("Introduce el ángulo theta del objetivo (en grados): "))

        # Convertir ángulo de grados a radianes
        theta_radians = math.radians(theta_degrees)
        goal_pose.theta = theta_radians

        controller.control_pid(goal_pose)

    except rospy.ROSInterruptException:
        pass
