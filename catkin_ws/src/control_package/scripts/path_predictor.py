#!/usr/bin/env python3

import rospy
import math
import numpy as np
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Twist


class PathPredictor: 
    def __init__(self):
        rospy.init_node('path_predictor')
        self.path_publisher = rospy.Publisher('myturtlebot/predicted_path', Path, queue_size=10)
        rospy.Subscriber('myturtlebot/cmd_vel', Twist, self.cmd_vel_callback)

        self.v = 0.0
        self.w = 0.0

    def cmd_vel_callback(self, cmd_vel_msg):
        # Callback to update linear and angular velocities from /cmd_vel topic
        self.v = cmd_vel_msg.linear.x
        self.w = cmd_vel_msg.angular.z
        
    @staticmethod
    def calculate_curvature(v,w):
        return w / v if v != 0 else 0
    
    def predicted_path(self, v, w, prediction_time=2.0, dt=0.1):
        predicted_points = []

        x,y,theta = 0,0,0

        #Calculate the steps in prediction time
        steps = int(prediction_time / dt)
        
        for _ in range(steps):
            kappa = self.calculate_curvature(v,w)
            #rospy.loginfo(_)
            #rospy.loginfo(f"The value of the kappa is: {kappa}")

            if kappa != 0: #Curved motion
                R = 1 / kappa
                theta += w * dt # Change in orientation
                x += R * (math.sin(theta)-math.sin(theta - w * dt))
                y -= R * (math.cos(theta)-math.cos(theta - w * dt))

            else: #Straight line motion
                x += v * dt * math.cos(theta)
                y += v * dt * math.sin(theta)

            predicted_points.append((x,y))

        return predicted_points
    
  
    def convert_to_ros_path(self,predicted_points):
        path = Path()
        path.header.frame_id = 'myturtlebot/base_link'
        path.header.stamp = rospy.Time.now()

        for point in predicted_points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]

            path.poses.append(pose)
        
        return path
    
    def publish_path(self, v, w, prediction_time=5.0):
        predicted_points = self.predicted_path(v, w, prediction_time)    
     
        ros_path = self.convert_to_ros_path(predicted_points)
        self.path_publisher.publish(ros_path)
    
if __name__ == '__main__':
    predictor = PathPredictor()
    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        predictor.publish_path(predictor.v, predictor.w)
        rate.sleep()