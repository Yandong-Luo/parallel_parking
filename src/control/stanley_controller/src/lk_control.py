#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@Description:       :
@Date     :2022/11/27 13:18:23
@Author      :Yandong Luo
@version      :1.0
'''

# Python Headers
import os 
import csv
import math
import numpy as np
from numpy import linalg as la
import scipy.signal as signal

# ROS Headers
import rospy
# import alvinxy.alvinxy as axy 

from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import Header
from pacmod_msgs.msg import PacmodCmd, PositionWithSpeed, VehicleSpeedRpt
from darknet_ros_msgs.msg import BoundingBoxes
from rgb_depth.msg import detector_info
from lane_line_detector.msg import line_info, lane_info

vehicle_width = 1.41
vehicle_length = 2.62

###################################### 用于control ######################################
class Onlinct_errorilter(object):

    def __init__(self, cutoff, fs, order):
        
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq

        # Get the filter coct_errorficients 
        self.b, self.a = signal.butter(order, normal_cutoff, btype='low', analog=False)

        # Initialize
        self.z = signal.lfilter_zi(self.b, self.a)
    
    def get_data(self, data):

        filted, self.z = signal.lfilter(self.b, self.a, [data], zi=self.z)
        return filted

class PID():
    def __init__(self, kp, ki, kd, wg=None):
        # 初始化参数
        self.iterm  = 0
        self.last_t = None
        self.last_e = 0
        self.kp     = kp
        self.ki     = ki
        self.kd     = kd
        self.wg     = wg
        self.derror = 0
    
    def reset(self):
        self.iterm = 0
        self.last_e = 0
        self.derror = None
    
    def get_control(self,t,e,fwd=0):
        if self.last_t is None:
            self.last_t = t
            de = 0
        else:
            de = (e - self.last_e) / (t - self.last_t)

        if abs(e - self.last_e) > 0.5:
            de = 0
        
        self.iterm += e*(t-self.last_t)

        # take care of integral winding-up
        if self.wg is not None:
            if self.iterm > self.wg:
                self.iterm = self.wg
            elif self.iterm < -self.wg:
                self.iterm = -self.wg

        self.last_e = e
        self.last_t = t
        self.derror = de

        return fwd + self.kp * e + self.ki * self.iterm + self.kd * de




class Lane_Keep():
    def __init__(self) -> None:
        
        self.rate = rospy.Rate(30)
        
        vehicle_speed_topic = rospy.get_param('vehicle_speed_topic','/pacmod/parsed_tx/vehicle_speed_rpt')
        lane_info_topic = rospy.get_param('lane_info_topic','/lane_info')
        
    
        rospy.Subscriber(vehicle_speed_topic, VehicleSpeedRpt, self.speed_callback)
        rospy.Subscriber(lane_info_topic, line_info, self.lane_callback)
        
        
        
        self.cmd_pub = rospy.Publisher('/gem/steer_cmd', AckermannDrive, queue_size=1)
        
        
        self.ackermann_msg                         = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0
        self.ackermann_msg.steering_angle          = 0.0

        self.safe_distance = 10.0
        self.desired_speed = 0.6  # m/s
        self.max_accel     = 0.48 # % of acceleration
        self.pid_speed     = PID(0.5, 0.0, 0.1, wg=20)
        self.speed_filter  = Onlinct_errorilter(1.2, 30, 4)

        self.speed      = 0.0
        
    # Get vehicle speed
    def speed_callback(self, msg):
        self.speed = round(msg.vehicle_speed, 3) # forward velocity in m/s
        
    # 基于PID控制对车辆直线进行控制
    def pid_control(self, desired_speed):
        filt_vel = np.squeeze(self.speed_filter.get_data(self.speed))
        # 获取期望的加速度
        a_expected = self.pid_speed.get_control(rospy.get_time(), desired_speed - filt_vel)

        if a_expected > 0.64 :
            throttle_percent = 0.5

        if a_expected < 0.0 :
            throttle_percent = 0.0
        
        throttle_percent = (a_expected+2.3501) / 7.3454
        
        self.ackermann_msg.acceleration   = throttle_percent

        
        
    def lane_callback(self, data):
        slope_l = data.left_fit[1]
        slope_r = data.right_fit[1]
        
        if abs(slope_l) < 1 and abs(slope_r) < 1:
            self.ackermann_msg.steering_angle = 0.0
        elif slope_l > 1 and slope_r > 1:
            self.ackermann_msg.steering_angle = 1.0
        elif slope_l > 1 and slope_r < -1:
            self.ackermann_msg.steering_angle = -1.0
            
        
            
    def run(self):
        while not rospy.is_shutdown():
            self.pid_control(0.6)
            self.cmd_pub.publish(self.ackermann_msg)
            self.rate.sleep() 


if __name__ == '__main__':
    rospy.init_node('lane_keep', anonymous=True)
    node = Lane_Keep()
    try:
        node.run()
        # rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
