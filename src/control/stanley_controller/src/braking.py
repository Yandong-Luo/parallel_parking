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
# from darknet_ros_msgs.msg import BoundingBoxes
from rgb_depth.msg import detector_info

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


class Brake():
    def __init__(self) -> None:
        # 订阅的话题
        self.rate = rospy.Rate(1)

        # 获取话题名字
        # darknet_result_topic = rospy.get_param('darknet_result_topic','/darknet_ros/bounding_boxes')
        objects_info_topic = rospy.get_param('objects_info_topic','/detector_info')
        vehicle_speed_topic = rospy.get_param('vehicle_speed_topic','/pacmod/parsed_tx/vehicle_speed_rpt')

        # 订阅darknet识别后的结果，其实可以不用获得这个，而是直接获得人的位置
        # self.result_sub = rospy.Subscriber(darknet_result_topic,BoundingBoxes)

        # 订阅检测到的人和stop sign的距离和位置信息
        rospy.Subscriber(objects_info_topic, detector_info, self.objects_info_callback)
        rospy.Subscriber(vehicle_speed_topic, VehicleSpeedRpt, self.speed_callback)


        # 发布车辆控制
        self.cmd_pub = rospy.Publisher('/gem/braking_cmd', AckermannDrive, queue_size=1)
        self.ackermann_msg                         = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0
        self.ackermann_msg.steering_angle          = 0.0

        self.safe_distance = 6.0   # m
        self.slow_distance = 15.0   # m
        self.desired_speed = 0.6  # m/s
        self.max_accel     = 0.48 # % of acceleration
        self.pid_speed     = PID(0.5, 0.0, 0.1, wg=20)
        self.speed_filter  = Onlinct_errorilter(1.2, 30, 4)

        self.person_x = np.inf
        self.person_y = np.inf

        self.speed      = 0.0

        # GEM vehilce brake control
        self.brake_pub = rospy.Publisher('/pacmod/as_rx/brake_cmd', PacmodCmd, queue_size=1)
        self.brake_cmd = PacmodCmd()
        self.brake_cmd.enable = False
        self.brake_cmd.clear  = True
        self.brake_cmd.ignore = True

    def objects_info_callback(self, objects_msg):
        print("class:",objects_msg.objects[0].type,"point x:",objects_msg.objects[0].point.point.x, "point y:",objects_msg.objects[0].point.point.y,\
            "point z:",objects_msg.objects[0].point.point.z,"dist:",objects_msg.objects[0].distance)

        # 迭代计算出现的人是否会与车辆运行的轨迹相碰撞
        for object in objects_msg.objects:
            self.person_x = object.point.point.x
            self.person_y = object.point.point.y

            # 五米开始刹停, 增大车辆的宽度0.3m使更安全
            # abs(self.person_y) < vehicle_width/2 + 1.5 and
            if abs(self.person_x) - vehicle_length/2 < self.safe_distance:
                # 刹车
                # 直接刹死
                print("Stop it")
                # ackermann_msg 去强制刹车
                # self.ackermann_msg.steering_angle_velocity = 0.0
                # self.ackermann_msg.acceleration            = 0.0
                # self.ackermann_msg.jerk                    = 0.0
                # self.ackermann_msg.speed                   = 0.0
                # self.ackermann_msg.steering_angle          = 0.0
                # self.cmd_pub.publish(self.ackermann_msg)

                # pacmod刹车
                self.brake_cmd.enable  = True
                self.brake_cmd.clear   = False
                self.brake_cmd.ignore  = False
                self.brake_cmd.f64_cmd = 0.0
                self.brake_pub.publish(self.brake_cmd)

                # 用PID去刹车
                # self.pid_control(0)
            elif self.person_x < self.slow_distance:
                print("SLOW DOWN")
                self.pid_control(0.6)
            else:
                self.person_x = np.inf
                self.person_y = np.inf
                print("Desired Speed")
                # 以期望速度运行
                self.pid_control(self.desired_speed)
    
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
        self.ackermann_msg.steering_angle = 0
        self.cmd_pub.publish(self.ackermann_msg)

    def run(self):
        while not rospy.is_shutdown():
            # self.person_x = np.inf
            # self.person_y = np.inf
            self.rate.sleep()

            if self.person_x is np.inf or self.person_y is np.inf:
                print("Desired Speed")
                self.pid_control(self.desired_speed)
            

if __name__ == '__main__':
    rospy.init_node('braking_node', anonymous=True)
    node = Brake()
    try:
        node.run()
        # rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")