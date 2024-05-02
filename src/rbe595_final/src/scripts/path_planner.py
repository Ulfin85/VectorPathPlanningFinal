#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist, Point
import numpy as np

class VectorFieldPathPlanning:
    def __init__(self):
        rospy.init_node('vector_field_path_planning')

        # Initialize subscribers
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Initialize publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize variables
        self.map_data = None
        self.robot_pose = None
        self.goal_position = None

        # Set maximum speeds
        self.max_linear_speed = 0.1  # m/s
        self.max_angular_speed = 0.7  # rad/s

    def map_callback(self, map_msg):
        # Update map data
        self.map_data = map_msg

    def odom_callback(self, odom_msg):
        # Update robot's pose
        self.robot_pose = odom_msg.pose.pose.position

    def set_goal(self, goal_position):
        # Set the goal position
        self.goal_position = goal_position

    def calculate_repulsive_force(self, robot_x, robot_y):
        if self.map_data is None:
            return 0, 0

        # Convert occupancy grid to numpy array
        map_array = np.array(self.map_data.data).reshape((self.map_data.info.height, self.map_data.info.width))

        # Constants for repulsion behavior
        k_rep = 40.0  # Repulsion coefficient
        min_dist = 0.01  # Minimum distance to consider repulsion
        

        # Initialize repulsive force components
        F_rep_x = 0
        F_rep_y = 0

        # Iterate through the neighborhood of the robot's position
        for y in range(max(0, robot_y - 5), min(self.map_data.info.height, robot_y + 6)):
            for x in range(max(0, robot_x - 5), min(self.map_data.info.width, robot_x + 6)):
                # Calculate distance to obstacle
                if map_array[y, x] > 0:  # Obstacle detected
                    dist_x = (robot_x - x) * self.map_data.info.resolution
                    dist_y = (robot_y - y) * self.map_data.info.resolution
                    dist = np.sqrt(dist_x**2 + dist_y**2)

                    if dist < min_dist:
                        dist = min_dist

                    # Calculate repulsive force components
                    F_rep_x += k_rep / dist**2 * (dist_x / dist)
                    F_rep_y += k_rep / dist**2 * (dist_y / dist)

        return F_rep_x, F_rep_y

    def calculate_vector_field(self):
        if self.map_data is None or self.robot_pose is None or self.goal_position is None:
            return None
        
        # Calculate robot's grid position
        robot_x = int((self.robot_pose.x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        robot_y = int((self.robot_pose.y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # Calculate repulsive force
        F_rep_x, F_rep_y = self.calculate_repulsive_force(robot_x, robot_y)

        # Calculate attractive force towards the goal
        F_attr_x = self.goal_position.x - self.robot_pose.x
        F_attr_y = self.goal_position.y - self.robot_pose.y

        # Normalize the attractive force
        attr_magnitude = np.sqrt(F_attr_x**2 + F_attr_y**2)
        if attr_magnitude != 0:
            F_attr_x /= attr_magnitude
            F_attr_y /= attr_magnitude

        #Scaling value to make attractive force towards goal more powerful
        attK = 2000
        # Combine attractive and repulsive forces
        F_total_x = F_attr_x * attK + F_rep_x
        F_total_y = F_attr_y * attK + F_rep_y
        
        # Normalize the total force
        total_magnitude = np.sqrt(F_total_x**2 + F_total_y**2)
        if total_magnitude != 0:
            F_total_x /= total_magnitude
            F_total_y /= total_magnitude
            print(F_total_x,F_total_y)

        return F_total_x, F_total_y

    def navigate_towards_goal(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.map_data is not None and self.robot_pose is not None and self.goal_position is not None:
                # Calculate vector field
                vector_field_x, vector_field_y = self.calculate_vector_field()

                # Calculate angular.z (rotation)
                angle_to_goal = np.arctan2(vector_field_y, vector_field_x)
                angular_z = np.clip(angle_to_goal, -self.max_angular_speed, self.max_angular_speed)

                # Set linear.x to move forward
                linear_x = np.clip(self.max_linear_speed, -self.max_linear_speed, self.max_linear_speed)

                # Create Twist message
                twist_msg = Twist()
                twist_msg.linear.x = linear_x
                twist_msg.angular.z = angular_z

                # Publish velocity commands
                self.velocity_publisher.publish(twist_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        path_planner = VectorFieldPathPlanning()

        # Set a sample goal position
        goal_position = Point()
        goal_position.x = 1.5
        goal_position.y = -0.5
        path_planner.set_goal(goal_position)

        # Start navigating towards the goal
        path_planner.navigate_towards_goal()

    except rospy.ROSInterruptException:
        pass

