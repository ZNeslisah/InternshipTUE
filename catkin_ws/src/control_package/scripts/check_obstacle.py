#!/usr/bin/env python3

import rospy
import math
import time
from geometry_msgs.msg import Twist, PoseStamped, Point
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path 

class ObstacleChecker:
    def __init__(self):
        rospy.init_node("check_obstacle")

        # Initialize the variables
        self.obstacle_coordinates = []  
        self.is_in_safe = False
        self.obstacle_detected = False  
        self.last_linear_velocity = 0.0
        self.last_angular_velocity = 0.0
        
        self.sub = rospy.Subscriber("myturtlebot/scan", LaserScan, self.laser_callback)  
        self.sub_vel = rospy.Subscriber("myturtlebot/cmd_vel_key", Twist, self.teleop_callback)
        self.sub_predictor = rospy.Subscriber("myturtlebot/predicted_path", Path, self.predicted_path_callback)
        self.pub = rospy.Publisher("myturtlebot/check_vel_obstacle", Twist, queue_size=10)  
        self.pub_bool = rospy.Publisher('myturtlebot/obstacle_detected', Bool, queue_size=10)

    def filter_invalid_readings(self,readings):
        return [r for r in readings if not math.isnan(r) and not math.isinf(r)] #filter readings which are not numbers or infinity
    
    def calculate_distance(self, point1, point2):
        return math.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)

    def laser_callback(self, scan):
        #rospy.loginfo("laser_callback working")
        ranges = scan.ranges
        angle_increment = scan.angle_increment
        min_angle = scan.angle_min

        try:
            self.front = min(self.filter_invalid_readings(scan.ranges[-len(scan.ranges) // 8:] + scan.ranges[:len(scan.ranges) // 8]))
            self.left  = min(self.filter_invalid_readings(scan.ranges[len(scan.ranges) // 8 : 3 * len(scan.ranges) // 8]))
            self.rear  = min(self.filter_invalid_readings(scan.ranges[3 * len(scan.ranges) // 8 : 5 * len(scan.ranges) // 8]))
            self.right = min(self.filter_invalid_readings(scan.ranges[5 * len(scan.ranges) // 8 : 7 * len(scan.ranges) // 8]))
        
        except ValueError:
            front = float('inf')

        self.obstacle_coordinates.clear()  # Clear the old values

        for i, obstacle_distance in enumerate(ranges):
            angle = min_angle + i * angle_increment
            x = obstacle_distance * math.cos(angle)
            y = obstacle_distance * math.sin(angle)
            obstacle_point = Point()
            obstacle_point.x = x
            obstacle_point.y = y
            self.obstacle_coordinates.append(obstacle_point)
    
    def teleop_callback(self, data):
        self.last_linear_velocity = data.linear.x
        self.last_angular_velocity = data.angular.z
      
    def reset_safe_state(self, event):
        self.is_in_safe = False

    def predicted_path_callback(self, msg):
        #rospy.loginfo("predicted_path_callback working")
        safety_margin = 0.1881 #Robot size:0.266*0.266*0.094, furthest point from center: 0.133*sqrt(2)
        predicted_points = msg.poses

        for idx, point in enumerate(predicted_points):
            #distances_to_obstacles = []

            if self.is_in_safe:
                    return
            
            for obstacle in self.obstacle_coordinates:

                distance = self.calculate_distance(point.pose.position, obstacle) - safety_margin
                
                if idx < 7:
                    if distance < 0.01: 
                        rospy.logwarn(f"Point {idx} at coordinates ({point.pose.position.x}, {point.pose.position.y}) is too close to an obstacle, turtlebot is stopped!")
                        self.pub_bool.publish(True)  
                        self.stop_and_turn_to_safe_position()
                        self.obstacle_detected = True #Flag
                        self.is_in_safe = True #Flag

                        rospy.Timer(rospy.Duration(5), self.reset_safe_state, oneshot=True)
                        return

                    else:
                        self.obstacle_detected = False
                        
                elif 7 <= idx < 13:
                    if distance < 0.01:
                        rospy.logwarn(f"Point {idx} at coordinates ({point.pose.position.x}, {point.pose.position.y}) is close to an obstacle, be careful!")

                #distances_to_obstacles.append(distance)

        if not self.obstacle_detected:
            self.is_in_safe =False

        self.pub_bool.publish(self.obstacle_detected)   

    def find_min_distance_vector(self):
        min_distance = float('inf')
        min_distance_vector = None

        for point in self.obstacle_coordinates:
            distance = math.sqrt(point.x**2 + point.y**2)
        
            if distance < min_distance:
                min_distance = distance
                min_distance_vector = (point.x, point.y)
    
        return min_distance_vector
            
    def find_angle_with_forward(self, min_distance_vector):
        forward_vector = [1, 0]  # This represents the robot's forward direction

        dot_product = min_distance_vector[0]*forward_vector[0] + min_distance_vector[1]*forward_vector[1]
        magnitude_forward = math.sqrt(forward_vector[0]**2 + forward_vector[1]**2)
        magnitude_min_distance = math.sqrt(min_distance_vector[0]**2 + min_distance_vector[1]**2)

        angle = math.acos(dot_product / (magnitude_forward * magnitude_min_distance))
        return math.degrees(angle)
    
    def find_orientation_with_forward(self, min_distance_vector):
        forward_vector = [1, 0]  # This represents the robot's forward direction
    
        cross_product_z = forward_vector[0] * min_distance_vector[1] - forward_vector[1] * min_distance_vector[0]

        #min_distance_vector position wrt forward vector
        if cross_product_z > 0:
            return 'left'
        elif cross_product_z < 0:
            return 'right'
        else:
            return 'parallel'

    def turn_by_angle(self, angle, angular_velocity):
        rate = rospy.Rate(10)  

        move_cmd = Twist()
        move_cmd.angular.z = angular_velocity  

        # Convert angle to radians
        angle_in_radians = angle * (3.141592653589793 / 180.0)

        # Calculate time to turn
        time_to_turn = abs(angle_in_radians / angular_velocity)
        end_time = rospy.Time.now().to_sec() + time_to_turn

        while rospy.Time.now().to_sec() < end_time:
            self.pub.publish(move_cmd)
            rate.sleep()

        move_cmd.angular.z = 0
        self.pub.publish(move_cmd)

    def stop_and_turn_to_safe_position(self):

        min_distance_vector = self.find_min_distance_vector()
        orientation = self.find_orientation_with_forward(min_distance_vector)
        angle = self.find_angle_with_forward(min_distance_vector)

        rospy.loginfo(f"angle:{angle}, oriantation: {orientation}")

        reversed_linear_velocity = -self.last_linear_velocity
        reversed_angular_velocity = -self.last_angular_velocity

        move_cmd = Twist()
        move_cmd.linear.x = reversed_linear_velocity
        move_cmd.angular.z = reversed_angular_velocity
        self.pub.publish(move_cmd)
        time.sleep(2.0)

        if orientation == "left":
            self.turn_by_angle(90-angle, -1)
        elif orientation == "right":
            self.turn_by_angle(90-angle, 1)
        else:
            self.turn_by_angle(90-angle, 1)


if __name__ == '__main__':
    checker = ObstacleChecker()
    rospy.spin()
