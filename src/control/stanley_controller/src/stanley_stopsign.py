#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@Description:       :
@Date           :2022/11/24 18:01:45
@Author         :Yandong Luo
@version        :1.0
@Description    : 用于pointPillars的parallel parking
'''

import rospy
import tf
import math
import csv
import pandas as pd
import numpy as np
import scipy.signal as signal
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PointStamped, PoseStamped
from gazebo_msgs.srv import GetModelState,GetModelStateResponse
from visualization_msgs.msg import MarkerArray

# 根据官网，车辆最高时速为40km/h
vehicle_max_speed = 40000/(60*60)   # m/s

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

class PID(object):
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
    
    # PID控制
    def update(self,currentPose,referencePose):
        """Compute PID control value based on feedback_val.
        """

        # current state
        current_x = currentPose[0]
        current_y = currentPose[1]
        current_theta = currentPose[2]
        current_v = currentPose[3]

        # reference state
        reference_x = referencePose[0]
        reference_y = referencePose[1]
        reference_orientation = referencePose[2]
        reference_v = referencePose[3]

        # The along-track error: (x ∗ (t) − x B (t)) cos θ B (t) + (y ∗ (t) − y B (t)) sin θ B (t)
        delta_s = (reference_x - current_x)*np.cos(current_theta) + (reference_y - current_y)*np.sin(current_theta)

        # The cross-track error: −(x ∗ (t) − x B (t)) sin θ B (t) + (y ∗ (t) − y B (t)) cos θ B (t).
        delta_n = -(reference_x - current_x)*np.sin(current_theta) +  (reference_y - current_y)*np.cos(current_theta)

        # The heading error
        delta_theta = reference_orientation - current_theta

        # The speed error
        delta_v = reference_v - current_v

        # using matrix to describe the error part
        error_u = np.array([[delta_s], [delta_n], [delta_theta], [delta_v]])

        # K matrix
        # k_x = 0.1
        # k_y = 0.5
        # k_v = 0.5
        # k_theta = 0.8
        # K = np.array([[k_x, 0, 0, k_v],[0,k_y,k_theta,0]])

        K = np.array([[self.k_x, 0, 0, self.k_v],[0,self.k_y,self.k_theta,0]])
        

        # U matrix
        U = np.array([[current_v],[currentPose.twist.angular.z]])
        
        result = np.dot(K,error_u)

        return result


class Stanley(object):
    def __init__(self) -> None:
        self.rate = rospy.Rate(30)
        
        # /sPath不包含转向,换一个
        path_topic = rospy.get_param("planning_path","/sPath")
        # vehicle_pose_topic = rospy.get_param("vehicle_pose_topic","/sPathVehicle")
        odom_topic = rospy.get_param("odom","/odom")
        
        # 该位置是相对于map坐标系的
        self.vehicle_x = 0.0
        self.vehicle_y = 0.0
        self.vehicle_heading = 0.0

        # 车辆线速度
        self.speed = 0.0
        # 车辆角速度
        self.angular_speed = 0.0

        # 用于存储路径，里面是相对于map坐标系的
        self.path_points_x = []
        self.path_points_y = []
        self.path_points_heading = []

        self.ackermann_msg                         = AckermannDrive()
        self.ackermann_msg.steering_angle_velocity = 0.0
        self.ackermann_msg.acceleration            = 0.0
        self.ackermann_msg.jerk                    = 0.0
        self.ackermann_msg.speed                   = 0.0
        self.ackermann_msg.steering_angle          = 0.0

        # PID for longitudinal control
        self.desired_speed = 0.6  # m/s
        self.max_accel_val     = 0.48 * 7.3454 # 0.48是百分比 % of acceleration
        self.max_accel     = 0.48 # % of acceleration
        self.pid_speed     = PID(0.5, 0.0, 0.1, wg=20)
        self.speed_filter  = Onlinct_errorilter(1.2, 30, 4)

        

        # 订阅规划后平滑处理后的path
        rospy.Subscriber(path_topic, Path, self.path_callback)
        # rospy.Subscriber(vehicle_pose_topic,MarkerArray,self.path_callback)
        # 订阅odometry的消息，包含了车辆相对世界坐标系的内容
        rospy.Subscriber(odom_topic, Odometry, self.odom_callback)
        # 车辆控制的发布
        self.ackermann_pub = rospy.Publisher(
            "/ackermann_cmd", AckermannDrive, queue_size=1
        )
    
    # 读取平滑的路径信息，记录在list中
    def path_callback(self, path_msg):
        self.path_points_x = []
        self.path_points_y = []
        self.path_points_heading = []

        # 注意：path中的节点顺序是从目标点到起始点，需要逆序，从起始点到目标点
        for path_pose in path_msg.poses:
            self.path_points_x.insert(0,path_pose.pose.position.x)
            self.path_points_y.insert(0,path_pose.pose.position.y)

            # 四元素转欧拉角
            orientation_q = path_pose.pose.orientation
            orientation_list = [
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w,
            ]
            (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
            self.path_points_heading.insert(0,yaw)
        
        # print(self.path_points_heading)
        # 写入csv文件用于检查
        dataframe = pd.DataFrame({'x':self.path_points_x,'y':self.path_points_y,'heading':self.path_points_heading})
        
        #将DataFrame存储为csv,index表示是否显示行名，default=True
        dataframe.to_csv("path.csv",index=False,sep=',')

    #####################################################
    ##################### 用于仿真 #######################
    #####################################################
    def odom_callback(self, odom_msg):
        # currentPose = getModelState()
        odom_rob_pose = PoseStamped()
        odom_rob_pose.header.frame_id = "/odom"
        odom_rob_pose.header.stamp =rospy.Time(0)
        odom_rob_pose.pose = odom_msg.pose.pose

        # 将odom坐标系的车辆坐标转换到map frame下
        map_vehicle_pos = self.transform_to_map_frame(init_pose=odom_rob_pose,target_frame='map',inital_frame='odom')

        self.vehicle_x = map_vehicle_pos.pose.position.x
        self.vehicle_y = map_vehicle_pos.pose.position.y

        # 四元素转欧拉角
        orientation_q = map_vehicle_pos.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.vehicle_heading = yaw
        # x轴方向的速度分量，这里的x轴应该是世界坐标系的，但是没有关系
        current_vx = odom_msg.twist.twist.linear.x
        current_vy = odom_msg.twist.twist.linear.y  # y方向速度分量
        self.speed = math.sqrt(pow(current_vx, 2) + pow(current_vy, 2))
        
        # 角速度暂时不用吧
        # self.angular_speed = odom_msg.twist.twist.angular.z
    
    # Conversion of front wheel to steering wheel
    def front2steer(self, f_angle):
        if(f_angle > 35):
            f_angle = 35
        if (f_angle < -35):
            f_angle = -35
        if (f_angle > 0):
            steer_angle = round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        elif (f_angle < 0):
            f_angle = -f_angle
            steer_angle = -round(-0.1084*f_angle**2 + 21.775*f_angle, 2)
        else:
            steer_angle = 0.0
        return steer_angle
    
    # Find close yaw in waypoint path
    def find_close_yaw(self, arr, val):
        diff_arr = np.array( np.abs( np.abs(arr) - np.abs(val) ) )
        idx = np.where(diff_arr < 0.5)
        return idx
    
    # Conversion to -pi to pi
    # 这样将不允许倒车
    def pi_2_pi(self, angle):

        if angle > np.pi:
            return angle - 2.0 * np.pi

        if angle < -np.pi:
            return angle + 2.0 * np.pi

        return angle
    
    # get the difference between yaw angle of vehicle and tangent line of waypoint
    def getDiffYaw(self, targetYaw, vehicleYaw):
        if targetYaw >= 0:
            alpha = targetYaw - vehicleYaw
        else:
            if targetYaw * vehicleYaw < 0:
                if targetYaw < 0 and vehicleYaw > 0:
                    alpha = (math.pi - vehicleYaw) + (math.pi + targetYaw)
                else:
                    alpha = -((math.pi + vehicleYaw) + (math.pi - targetYaw))
            else:
                alpha = targetYaw - vehicleYaw
        return alpha
    
    # transform the pose from initial frame to target frame
    def transform_to_map_frame(self,init_pose,target_frame, inital_frame):
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(inital_frame,target_frame,rospy.Time(),rospy.Duration(4.0))

        got_tf_transform = False
        while got_tf_transform == False:
            try:
                now = rospy.Time.now()

                (trans,rot) = self.listener.lookupTransform(inital_frame, target_frame, now)
            
                got_tf_transform = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                got_tf_transform = False
        # have got the transform matrix, and then just calculate the position of odom at the frame of map
        if got_tf_transform == True:
            rob_in_map_frame = self.listener.transformPose(target_frame,init_pose)
            return rob_in_map_frame
        else:
            return
    
    def start_pid(self):
        while not rospy.is_shutdown():
            if len(self.path_points_x)==0 or len(self.path_points_y) == 0 or len(self.path_points_heading) == 0:
                self.rate.sleep()
                continue

            self.path_points_x = np.array(self.path_points_x)
            self.path_points_y = np.array(self.path_points_y)
            self.path_points_yaw = np.array(self.path_points_heading)

            # 找到最靠近的waypoint序号
            target_idx = self.find_close_yaw(self.path_points_yaw, self.vehicle_heading)

            self.target_path_points_x = self.path_points_x[target_idx]
            self.target_path_points_y = self.path_points_y[target_idx]
            self.target_path_points_yaw = self.path_points_yaw[target_idx]

            # find the closest point
            dx = [self.vehicle_x - x for x in self.target_path_points_x]
            dy = [self.vehicle_y - y for y in self.target_path_points_y]

            # find the index of closest point
            target_point_idx = int(np.argmin(np.hypot(dx,dy)))

            # 选择下一目标点
            if (target_point_idx != len(self.target_path_points_x)-1):
                target_point_idx = target_point_idx +1
            
            result = self.pid_speed.update(currentPose=[self.vehicle_x,self.vehicle_y,self.vehicle_heading,self.speed],\
            referencePose=[self.target_path_points_x[target_point_idx],self.target_path_points_y[target_point_idx],self.target_path_points_yaw[target_point_idx],target_speed])

    # Start Stanley controller
    def start_stanley(self):
        while not rospy.is_shutdown():
            if len(self.path_points_x)==0 or len(self.path_points_y) == 0 or len(self.path_points_heading) == 0:
                self.rate.sleep()
                continue

            self.path_points_x = np.array(self.path_points_x)
            self.path_points_y = np.array(self.path_points_y)
            self.path_points_yaw = np.array(self.path_points_heading)

            # 找到最靠近的waypoint序号
            target_idx = self.find_close_yaw(self.path_points_yaw, self.vehicle_heading)

            self.target_path_points_x = self.path_points_x[target_idx]
            self.target_path_points_y = self.path_points_y[target_idx]
            self.target_path_points_yaw = self.path_points_yaw[target_idx]

            # find the closest point
            dx = [self.vehicle_x - x for x in self.target_path_points_x]
            dy = [self.vehicle_y - y for y in self.target_path_points_y]

            # find the index of closest point
            target_point_idx = int(np.argmin(np.hypot(dx,dy)))

            # 选择下一目标点
            if (target_point_idx != len(self.target_path_points_x)-1):
                target_point_idx = target_point_idx +1
            
            vec_target_2_front = np.array([[dx[target_point_idx]], [dy[target_point_idx]]])
            front_axle_vec_rot_90 = np.array([[np.cos(self.vehicle_heading - np.pi / 2.0)], [np.sin(self.vehicle_heading - np.pi / 2.0)]])

            # crosstrack error
            ct_error = np.dot(vec_target_2_front.T, front_axle_vec_rot_90)
            ct_error = float(np.squeeze(ct_error))

            # heading error
            # 计算两个yaw的偏差
            # diffYaw = self.getDiffYaw(self.target_path_points_yaw[target_point_idx], self.vehicle_heading)

            # # get the error between target position and current position
            # if diffYaw >= 0:
            #     theta_e = abs(math.sqrt(math.pow(self.vehicle_x - self.target_path_points_x[target_point_idx],2) + math.pow(self.vehicle_y - self.target_path_points_y[target_point_idx],2)))
            # else:
            #     theta_e = -abs(math.sqrt(math.pow(self.vehicle_x - self.target_path_points_x[target_point_idx],2) + math.pow(self.vehicle_y - self.target_path_points_y[target_point_idx],2)))

            # 崔航写的部分，不包含倒车
            theta_e = self.pi_2_pi(self.target_path_points_yaw[target_point_idx]-self.vehicle_heading) 

            theta_e_deg = round(np.degrees(theta_e), 1)
            print("Crosstrack Error: " + str(round(ct_error,3)) + ", Heading Error: " + str(theta_e_deg))

            # --------------------------- Longitudinal control using PD controller ---------------------------
            filt_vel = np.squeeze(self.speed_filter.get_data(self.speed))

            print("filt_vel:",filt_vel)

            a_expected = self.pid_speed.get_control(rospy.get_time(), self.desired_speed - filt_vel)
            print("期望的加速度:",a_expected)
            if a_expected > 0.64 :
                throttle_percent = 0.5

            if a_expected < 0.0 :
                throttle_percent = 0.0

            throttle_percent = (a_expected+2.3501) / 7.3454

            throttle_val =  (a_expected+2.3501)

            # 加速度百分比限制
            if throttle_percent > self.max_accel:
                throttle_percent = self.max_accel

            if throttle_percent < 0.3:
                throttle_percent = 0.37

            # 加速度限制
            if throttle_val > self.max_accel_val:
                throttle_val = self.max_accel_val

            if throttle_val < 0.3:
                throttle_val = 0.37
            
            # -------------------------------------- Stanley controller --------------------------------------
            f_delta        = round(theta_e + np.arctan2(ct_error*0.4, filt_vel), 3)
            f_delta        = round(np.clip(f_delta, -0.61, 0.61), 3)
            f_delta_deg    = np.degrees(f_delta)
            steering_angle = self.front2steer(f_delta_deg)

            if (filt_vel < 0.2):
                self.ackermann_msg.acceleration   = throttle_val
                self.ackermann_msg.steering_angle = 0
                self.ackermann_msg.speed = self.desired_speed
                print(self.ackermann_msg.steering_angle)
            else:
                self.ackermann_msg.acceleration   = throttle_val
                self.ackermann_msg.steering_angle = round(steering_angle,1)
                self.ackermann_msg.speed = self.desired_speed
                print(self.ackermann_msg.steering_angle)

            # ------------------------------------------------------------------------------------------------ 
            
            self.ackermann_pub.publish(self.ackermann_msg)
            self.rate.sleep()


def stanley_run():
    rospy.init_node('stanley_tracker_node',anonymous=True)
    stanley = Stanley()
    
    try:
        stanley.start_stanley()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    stanley_run()