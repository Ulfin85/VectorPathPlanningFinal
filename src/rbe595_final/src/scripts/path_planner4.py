#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist, Point, PoseStamped
from tf.transformations import euler_from_quaternion
import numpy as np
from math import atan2, sqrt

class SquarePathPlanner:
    def __init__(self):
        rospy.init_node('square_path_planner', anonymous=True)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.pose_sub = rospy.Subscriber('/pose', PoseStamped, self.pose_callback)
        self.velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.map = None
        self.map_resolution = None
        self.square_size = None
        self.robot_pose = None
        self.goal_pose = None
        self.vector_field_resolution = 0.1  # Adjust resolution as needed
        self.vector_field = None
        self.waypoints = []

    def map_callback(self, msg):
        self.map = msg.data
        self.map_resolution = msg.info.resolution
        self.square_size = 0.2  # Set square size based on map resolution

    def pose_callback(self, msg):
        orientation_q = msg.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose = (msg.pose.position.x, msg.pose.position.y, yaw)

    def split_map_into_squares(self):
        # Split the map into squares and identify obstacle squares
        # Assuming the map is centered at (0,0) with x increasing to the right and y increasing upwards
        if self.map is not None:
            map_width = self.map.info.width
            map_height = self.map.info.height

            for x in range(0, map_width, int(self.square_size / self.map_resolution)):
                for y in range(0, map_height, int(self.square_size / self.map_resolution)):
                    square = self.extract_square((x, y))
                    is_obstacle = self.is_square_obstacle(square)
                    if not is_obstacle:
                        self.waypoints.append((x, y))

    def extract_square(self, center):
        # Extract a square from the map centered at the given position
        square_size_cells = int(self.square_size / self.map_resolution)
        half_size = square_size_cells // 2
        map_data = np.reshape(self.map, (self.map.info.height, self.map.info.width))
        x, y = center
        square = map_data[y - half_size:y + half_size, x - half_size:x + half_size]
        return square

    def is_square_obstacle(self, square):
        # Check if the square contains any obstacle cells
        return np.any(square)

    def plan_path(self):
        # Simplified path planning: Directly navigate from current pose to goal pose
        self.waypoints.append((self.goal_pose[0], self.goal_pose[1]))

    def calculate_vector_field(self):
        if self.waypoints:
            waypoint = self.waypoints[0]
            dx = waypoint[0] - self.robot_pose[0]
            dy = waypoint[1] - self.robot_pose[1]
            distance = sqrt(dx**2 + dy**2)
            desired_direction = atan2(dy, dx)
            linear_velocity = 0.5  # Adjust linear velocity as needed
            angular_velocity = 0.5 * (desired_direction - self.robot_pose[2])  # P controller for angular velocity

            # Create a vector field that points towards the next waypoint
            if self.vector_field is None:
                self.vector_field = np.zeros((int(1 / self.vector_field_resolution), int(1 / self.vector_field_resolution), 2))

            # Calculate the grid cell corresponding to the robot's position
            x_idx = int(self.robot_pose[0] / self.vector_field_resolution)
            y_idx = int(self.robot_pose[1] / self.vector_field_resolution)

            # Update the vector field
            self.vector_field[x_idx, y_idx, 0] = linear_velocity * np.cos(angular_velocity)
            self.vector_field[x_idx, y_idx, 1] = linear_velocity * np.sin(angular_velocity)

            return linear_velocity, angular_velocity
        else:
            return 0, 0

    def control_robot(self):
        # Control the robot's motion towards the next waypoint
        linear_velocity, angular_velocity = self.calculate_vector_field()
        self.publish_velocity(linear_velocity, angular_velocity)

    def publish_velocity(self, linear_velocity, angular_velocity):
        twist_msg = Twist()
        twist_msg.linear.x = linear_velocity
        twist_msg.angular.z = angular_velocity
        self.velocity_pub.publish(twist_msg)

    def execute(self):
        rate = rospy.Rate(10)  # Adjust rate as needed
        while not rospy.is_shutdown():
            if self.map is not None:
                # Split map into squares, plan path, calculate vector field, and control robot
                self.split_map_into_squares()
                self.plan_path()
                self.control_robot()
            rate.sleep()

if __name__ == '__main__':
    try:
        planner = SquarePathPlanner()
        planner.execute()
    except rospy.ROSInterruptException:
        pass
