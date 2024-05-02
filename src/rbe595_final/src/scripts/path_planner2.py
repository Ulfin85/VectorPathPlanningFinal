#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Pose
import math
import numpy as np

class VFHPathPlanner:
    def __init__(self):
        rospy.init_node('vfh_path_planner', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize other variables
        self.robot_pose = None
        self.laser_scan = None
        self.grid_map = None
        self.goal = Pose()  # Set your goal pose here
        self.goal.position.x = 1.8
        self.goal.position.y = -0.5

    def odom_callback(self, msg):
        # Update robot pose
        self.robot_pose = msg.pose.pose

    def scan_callback(self, msg):
        # Update laser scan data
        self.laser_scan = msg

    def map_callback(self, msg):
        # Update occupancy grid map
        self.grid_map = msg

    def calculate_vector_field(self):
        if self.robot_pose is not None and self.laser_scan is not None and self.grid_map is not None:
            # Extract robot position
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y

            # Calculate attractive force towards the goal
            attractive_force_x = self.goal.position.x - robot_x
            attractive_force_y = self.goal.position.y - robot_y

            # Calculate repulsive forces from obstacles using laser scan data
            repulsive_force_x = 0.0
            repulsive_force_y = 0.0
            for i, scan_range in enumerate(self.laser_scan.ranges):
                if not math.isnan(scan_range) and scan_range < 0.5:  # Consider obstacles within 1 meter
                    angle = self.laser_scan.angle_min + i * self.laser_scan.angle_increment
                    repulsive_force_x += math.cos(angle)
                    repulsive_force_y += math.sin(angle)

            # Combine attractive and repulsive forces to generate resultant vector
            resultant_force_x = attractive_force_x + repulsive_force_x
            resultant_force_y = attractive_force_y + repulsive_force_y

            return np.array([resultant_force_x, resultant_force_y])
        else:
            return None

    def select_best_direction(self):
        # Calculate the angle of the resultant vector
        resultant_force = self.calculate_vector_field()
        if resultant_force is not None:
            angle = math.atan2(resultant_force[1], resultant_force[0])
            return angle
        else:
            return None

    def navigate(self):
        # Select the best direction and publish velocity commands
        angle = self.select_best_direction()
        if angle is not None:
            # Calculate angular velocity proportional to the angle
            angular_vel = angle * 0.056  # Proportional control
            linear_vel = 0.1  # Constant linear velocity

            # Publish velocity commands
            twist_msg = Twist()
            twist_msg.linear.x = linear_vel
            twist_msg.angular.z = angular_vel
            self.cmd_pub.publish(twist_msg)

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.navigate()
            rate.sleep()

if __name__ == '__main__':
    try:
        path_planner = VFHPathPlanner()
        path_planner.run()
    except rospy.ROSInterruptException:
        pass
