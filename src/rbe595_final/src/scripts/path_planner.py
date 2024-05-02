#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math

class VectorFieldPathPlanning:
    def __init__(self):
        rospy.init_node('vector_field_path_planning')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.robot_pose = None
        self.obstacle_ranges = None
        self.goal = (1.5, -0.5)  # Example goal position
        self.map_data = None

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def laser_callback(self, msg):
        self.obstacle_ranges = msg.ranges

    def map_callback(self, msg):
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))

    def calculate_vector_field(self):
        if self.robot_pose is None or self.obstacle_ranges is None or self.map_data is None:
            return 0, 0

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        attractive_force_x = self.goal[0] - robot_x
        attractive_force_y = self.goal[1] - robot_y

        for i, distance in enumerate(self.obstacle_ranges):
            if distance < 0.03:  # Threshold distance to obstacles
                angle = self.laser_to_angle(i, len(self.obstacle_ranges))
                repulsive_force_x = -math.cos(angle)
                repulsive_force_y = -math.sin(angle)
                attractive_force_x += repulsive_force_x
                attractive_force_y += repulsive_force_y

        # Apply repulsive forces based on map data
        map_resolution = 0.05  # meters per cell
        map_origin_x = -10.0
        map_origin_y = -10.0
        robot_grid_x = int((robot_x - map_origin_x) / map_resolution)
        robot_grid_y = int((robot_y - map_origin_y) / map_resolution)

        repulsive_force_x_map = 0
        repulsive_force_y_map = 0

        for i in range(-5, 6):  # Consider neighboring cells
            for j in range(-5, 6):
                x = robot_grid_x + i
                y = robot_grid_y + j
                if 0 <= x < self.map_data.shape[1] and 0 <= y < self.map_data.shape[0]:
                    if self.map_data[y, x] > 0:  # Cell is occupied
                        dx = robot_x - (x * map_resolution + map_origin_x)
                        dy = robot_y - (y * map_resolution + map_origin_y)
                        distance = math.sqrt(dx ** 2 + dy ** 2)
                        if distance > 0:
                            repulsive_force_x_map += dx / distance ** 3
                            repulsive_force_y_map += dy / distance ** 3

        attractive_force_x += repulsive_force_x_map
        attractive_force_y += repulsive_force_y_map

        return attractive_force_x, attractive_force_y

    def laser_to_angle(self, index, num_readings):
        angle_min = -math.pi / 2
        angle_max = math.pi / 2
        angle_increment = (angle_max - angle_min) / (num_readings - 1)
        return angle_min + index * angle_increment

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            vector_field_x, vector_field_y = self.calculate_vector_field()

            # Normalize the vector field
            magnitude = math.sqrt(vector_field_x ** 2 + vector_field_y ** 2)
            if magnitude > 0.0:
                vector_field_x /= magnitude
                vector_field_y /= magnitude
                if vector_field_x > 0.12:
                    vector_field_x = 0.12
                if vector_field_y > 0.12:
                    vector_field_y = 0.12
                if vector_field_x < -0.12:
                    vector_field_x = -0.12
                if vector_field_y < -0.12:
                    vector_field_y = -0.12 

            cmd_vel_msg = Twist()
            cmd_vel_msg.linear.x = vector_field_x
            cmd_vel_msg.linear.y = vector_field_y
            self.cmd_vel_pub.publish(cmd_vel_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        vfp = VectorFieldPathPlanning()
        vfp.run()
    except rospy.ROSInterruptException:
        pass

