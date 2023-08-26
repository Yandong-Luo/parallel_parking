#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@Description:       :
@Date     :2022/11/28 11:43:15
@Author      :Yandong Luo
@version      :1.0
'''

import rospy
import numpy as np
import message_filters
from ackermann_msgs.msg import AckermannDrive
# GEM PACMod Headers
from pacmod_msgs.msg import PositionWithSpeed, PacmodCmd, SystemRptFloat, VehicleSpeedRpt, SystemRptFloat

class State_subscriber():
    def __init__(self) -> None:
        # 订阅速度,速度可以不要
        # self.speed_sub  = rospy.Subscriber("/pacmod/parsed_tx/vehicle_speed_rpt", VehicleSpeedRpt, self.speed_callback)
        # 订阅加速度
        # self.accel_sub = rospy.Subscriber("/pacmod/parsed_tx/accel_rpt", SystemRptFloat, self.accel_callback)
        # 订阅车辆转角
        # self.steer = 0.0 # degrees
        # self.steer_sub = rospy.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat, self.steer_callback)
        
        # <remap from="/gem/track_lane_cmd" to="/gem/stanley_gnss_cmd" /> 
        # <remap from="/gem/track_lane_cmd" to="/gem/stanley_gnss_cmd" /> 
        # <remap from="/gem/track_lane_cmd" to="/gem/stanley_gnss_cmd" /> 
        # <remap from="/gem/track_lane_cmd" to="/gem/stanley_gnss_cmd" /> 

        # 同步订阅
        self.accel_sub = message_filters.Subscriber("/pacmod/parsed_tx/accel_rpt", SystemRptFloat)
        self.steer_sub = message_filters.Subscriber("/pacmod/parsed_tx/steer_rpt", SystemRptFloat)

        self.ts = message_filters.ApproximateTimeSynchronizer([self.accel_sub, self.steer_sub], queue_size=5, slop=0.5)

        self.ts.registerCallback(self.state_callback)

        # 发布
        self.control_pub = rospy.Publisher('/gem/track_lane_cmd', AckermannDrive, queue_size=1)

        self.ackermann_msg                         = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0
        self.ackermann_msg.steering_angle          = 0.0
    
    def state_callback(self, accel_msg, steer_msg):
        self.ackermann_msg.acceleration = accel_msg.output
        # 转向角
        # steer = round(np.degrees(steer_msg.output),1)
        self.ackermann_msg.steering_angle_velocity = steer_msg.output

        self.control_pub.publish(self.ackermann_msg)

        
    # # Get vehicle speed
    # def speed_callback(self, msg):
    #     self.speed = round(msg.vehicle_speed, 3) # forward velocity in m/s

    # # Get value of steering wheel
    # def steer_callback(self, msg):
    #     self.steer = round(np.degrees(msg.output),1)

if __name__ == '__main__':
    rospy.init_node('state_listener_node', anonymous=True)
    node = State_subscriber()
    try:
        # node.run()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")