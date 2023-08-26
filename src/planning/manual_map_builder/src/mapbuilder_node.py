#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@Description:       :
@Date           :2022/11/26 09:57:26
@Author         :Yandong Luo
@version        :1.0
@Description    : 用于pointPillars的parallel parking
'''

import rospy
import numpy as np
from autoware_msgs.msg import DetectedObjectArray, DetectedObject
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from gazebo_msgs.srv import GetModelState,GetModelStateResponse
from gazebo_msgs.msg import ModelState
import tf
import tf2_ros
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool

class map_Builder():
    def __init__(self) -> None:
        rospy.Subscriber("/detection/lidar_detector/objects", DetectedObjectArray, self.callback3)
        rospy.Subscriber("/replanning",Bool,self.replan_callback)
        self.pub_map = rospy.Publisher('/map', OccupancyGrid, queue_size=1)
        self.pub_goal = rospy.Publisher('/goal_pose',PoseStamped,queue_size=1)
        self.pub_start = rospy.Publisher('/start_pose',PoseWithCovarianceStamped,queue_size=1)
        self.replan = True
    
    def replan_callback(self, replan_msg):
        self.replan = replan_msg.data

    # 检验识别后的数据是否有效
    def IsObjectValid(self, object_data):
        if object_data.valid == False or\
        np.isnan(object_data.pose.orientation.x) or\
        np.isnan(object_data.pose.orientation.y) or\
        np.isnan(object_data.pose.orientation.z) or\
        np.isnan(object_data.pose.orientation.w) or\
        np.isnan(object_data.pose.position.x) or\
        np.isnan(object_data.pose.position.y) or\
        np.isnan(object_data.pose.position.z) or\
        (object_data.pose.position.x == 0.) or\
        (object_data.pose.position.y == 0.) or\
        (object_data.dimensions.x <= 0.) or\
        (object_data.dimensions.y <= 0.) or\
        (object_data.dimensions.z <= 0.):
            return False
        else:
            return True
    
    def get_r_gator_pose(self):

        rospy.wait_for_service("/gazebo/get_model_state")

        try:
            service_response = rospy.ServiceProxy(
                "/gazebo/get_model_state", GetModelState
            )
            model_state = service_response(model_name="r_gator")
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q = model_state.pose.orientation
        orientation_list = [
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w,
        ]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        return round(x, 4), round(y, 4), round(yaw, 4)
    
    # 方案一创建一个map，包含了车辆所有方向的map，但里面需要做很多的变换，比较复杂
    # 弃用了
    def callback(self, objects_data):
        
        height = 200
        width = 200
        factor = 4      # 4个grid=1m

        # initial map data
        map_data = OccupancyGrid()
        map_data.header.frame_id = "map"
        # 设定map的尺寸，尺寸适合能够优化计算，目前初始估计为200x200的栅格，每栅格0.25m
        map_data.info.height = height
        map_data.info.width = width
        map_data.info.resolution = 0.25     # The map resolution [m/cell] 一个cell里边长为多少米，越小规划地越好
        map_data.info.origin.position.x = height/2/factor
        map_data.info.origin.position.y = width/2/factor
        map_data.info.origin.position.z = 0
        map_data.info.origin.orientation.x = 0
        map_data.info.origin.orientation.y = 0
        map_data.info.origin.orientation.z = 1
        map_data.info.origin.orientation.w = 0

        # 初始化map内容全为0，用一个二维数组来表达
        map = np.zeros((width, height))

        # 储存投影在map上的四个点
        objects_list = list()
        for object in objects_data.objects:
            if(self.IsObjectValid(object)):
                # object的中心位置
                object_x = object.pose.position.x
                object_y = object.pose.position.y
                object_x_size = object.dimensions.x
                object_y_size = object.dimensions.y

                objects_list.append([object_x,object_y])

                if object_x >=0 and object_y >=0:
                    col = round(height/2)-factor*round(object_x)
                    row = round(width/2) - factor*round(object_y)
                    
                elif object_x>0 and object_y<0:
                    col = round(height/2)-factor*round(object_x)
                    row = round(factor*abs(object_y))+round(width/2)
                elif object_x<0 and object_y>0:
                    col = factor*round(abs(object_x))+round(height/2)
                    row = round(width/2) - factor*round(object_y)
                elif object_x<0 and object_y<0:
                    col = factor*round(abs(object_x))+round(height/2)
                    row = factor*round(abs(object_y))+round(width/2)
                
                low_row = row-round(factor*object_x_size/2) 
                high_row = row+round(factor*object_x_size/2)
                low_col = col-round(factor*object_y_size/2)
                high_col = col+round(factor*object_y_size/2)

                if max(high_col,high_row)>height or max(low_col,low_row)<0:
                    print("object的尺寸超出了栅格化地图的边界了")
                    return

                map[low_row:high_row,low_col:high_col] = 100

        convert_data = np.reshape(map.astype(int), (1, height*width))
        map_data.data = convert_data[0]
        self.pub_map.publish(map_data)

        self.pubGoalPose(objects_list)
        # self.pubStartPose()

    # 方案二，创建一个map，map的坐标系采用的是以车右边方向为x，以前进为y，parking位置位于右边
    def callback2(self, objects_data):
        height = 200
        width = 200
        factor = 4      # 4个grid=1m

        # initial map data
        map_data = OccupancyGrid()
        map_data.header.frame_id = "map"
        # 设定map的尺寸，尺寸适合能够优化计算，目前初始估计为200x200的栅格，每栅格0.25m
        map_data.info.height = height
        map_data.info.width = width
        map_data.info.resolution = 0.25     # The map resolution [m/cell] 一个cell里边长为多少米，越小规划地越好
        map_data.info.origin.position.x = -width/2/factor
        map_data.info.origin.position.y = -height/2/factor
        map_data.info.origin.position.z = 0
        map_data.info.origin.orientation.x = 0
        map_data.info.origin.orientation.y = 0
        map_data.info.origin.orientation.z = 0
        map_data.info.origin.orientation.w = 1

        # 初始化map内容全为0，用一个二维数组来表达
        map = np.zeros((width, height))

        # 储存投影在map上的四个点
        objects_list = list()
        for object in objects_data.objects:
            if(self.IsObjectValid(object)):
                # object的中心位置
                object_x = object.pose.position.x
                object_y = object.pose.position.y
                object_x_size = object.dimensions.x
                object_y_size = object.dimensions.y

                objects_list.append([object_x,object_y])

                # 在robot frame下所检测到的目标位置变换到map坐标系中
                rob_object_pose = PointStamped()
                rob_object_pose.header.frame_id = "/base_footprint"
                rob_object_pose.header.stamp =rospy.Time(0)
                rob_object_pose.point.x = object_x
                rob_object_pose.point.y = object_y
                rob_object_pose.point.z = 0

                # 将车辆坐标系所检测到的物体的坐标转换到map frame下
                map_object_pos = self.transform_to_map_frame(init_pose=rob_object_pose,target_frame='map',inital_frame='base_footprint')
                map_object_pos_x = map_object_pos.point.x
                map_object_pos_y = map_object_pos.point.y

                col = round(width/2) + factor*round(map_object_pos_x)
                row = round(height/2) + factor*round(map_object_pos_y)

                low_row = row-round(factor*object_y_size/2) 
                high_row = row+round(factor*object_y_size/2)
                low_col = col-round(factor*object_x_size/2)
                high_col = col+round(factor*object_x_size/2)

                if max(high_col,high_row)>height or max(low_col,low_row)<0:
                    print("object的尺寸超出了栅格化地图的边界了")
                    return

                map[int(low_row):int(high_row),int(low_col):int(high_col)] = 100

        convert_data = np.reshape(map.astype(int), (1, height*width))
        map_data.data = convert_data[0]
        self.pub_map.publish(map_data)

        self.pubGoalPose(objects_list)
        self.pubStartPose()
    
        # 方案三，创建一个map，map的坐标系采用的是以车右边方向为x，以前进为y，parking位置位于右边，resolution设为1米
    def callback3(self, objects_data):
        if self.replan == False:
            return
        
        self.replan = False

        height = 40
        width = 40
        factor = 1      # 4个grid=1m

        # initial map data
        map_data = OccupancyGrid()
        map_data.header.frame_id = "map"
        # 设定map的尺寸，尺寸适合能够优化计算，目前初始估计为20x20的栅格，每栅格1m
        map_data.info.height = height
        map_data.info.width = width
        map_data.info.resolution = 1     # The map resolution [m/cell] 一个cell里边长为多少米，默认为一米
        map_data.info.origin.position.x = -width/2/factor
        map_data.info.origin.position.y = -height/2/factor
        map_data.info.origin.position.z = 0
        map_data.info.origin.orientation.x = 0
        map_data.info.origin.orientation.y = 0
        map_data.info.origin.orientation.z = 0
        map_data.info.origin.orientation.w = 1

        # 初始化map内容全为0，用一个二维数组来表达
        map = np.zeros((width, height))

        # 储存投影在map上的四个点
        objects_list = list()
        for object in objects_data.objects:
            if(self.IsObjectValid(object)):
                # object的中心位置
                object_x = object.pose.position.x
                object_y = object.pose.position.y
                object_x_size = object.dimensions.x
                object_y_size = object.dimensions.y

                objects_list.append([object_x,object_y])

                # 在robot frame下所检测到的目标位置变换到map坐标系中
                rob_object_pose = PointStamped()
                rob_object_pose.header.frame_id = "/base_footprint"
                rob_object_pose.header.stamp =rospy.Time(0)
                rob_object_pose.point.x = object_x
                rob_object_pose.point.y = object_y
                rob_object_pose.point.z = 0

                # 将车辆坐标系所检测到的物体的坐标转换到map frame下
                map_object_pos = self.transform_to_map_frame(init_pose=rob_object_pose,target_frame='map',inital_frame='base_footprint')
                map_object_pos_x = map_object_pos.point.x
                map_object_pos_y = map_object_pos.point.y

                col = round(width/2) + factor*round(map_object_pos_x)
                row = round(height/2) + factor*round(map_object_pos_y)

                low_row = row-round(factor*object_y_size/2) 
                high_row = row+round(factor*object_y_size/2)
                low_col = col-round(factor*object_x_size/2)
                high_col = col+round(factor*object_x_size/2)

                if max(high_col,high_row)>height or max(low_col,low_row)>width:
                    print("object的尺寸超出了栅格化地图的边界了")
                    map = np.zeros((width, height))
                    # return
                else:
                    map[int(low_row):int(high_row),int(low_col):int(high_col)] = 100

        convert_data = np.reshape(map.astype(int), (1, height*width))
        map_data.data = convert_data[0]
        self.pub_map.publish(map_data)

        self.pubGoalPose(objects_list)
        self.pubStartPose()
    
    def pubGoalPose(self,objects_list):
        if len(objects_list) == 2:
            goal_x = (objects_list[0][0]+objects_list[1][0])/2
            goal_y = (objects_list[0][1]+objects_list[1][1])/2
            # print("goal_x:",goal_x,"goal_y:",goal_y)
            base_goal_pose = PointStamped()
            base_goal_pose.header.frame_id = "/base_footprint"
            base_goal_pose.header.stamp = rospy.Time(0)
            base_goal_pose.point.x = goal_x
            base_goal_pose.point.y = goal_y
            base_goal_pose.point.z = 0

            # 将处于base_footprint坐标系的goal位置变换到map坐标系中
            map_goal_pos = self.transform_to_map_frame(init_pose=base_goal_pose,target_frame='map',inital_frame='base_footprint')

            goal_data = PoseStamped()
            goal_data.pose.position.x = map_goal_pos.point.x 
            goal_data.pose.position.y = map_goal_pos.point.y 
            goal_data.pose.position.z = 0
            goal_data.pose.orientation.x = 0
            goal_data.pose.orientation.y = 0
            goal_data.pose.orientation.z = 0.706825181105366
            goal_data.pose.orientation.w = 0.7073882691671998

            self.pub_goal.publish(goal_data)
    
    # Publish the start pose
    def pubStartPose(self):
        # get current position and orientation in the world frame
        cur_x, cur_y, cur_yaw = self.get_r_gator_pose()

        odom_rob_pose = PointStamped()
        odom_rob_pose.header.frame_id = "/odom"
        odom_rob_pose.header.stamp =rospy.Time(0)
        odom_rob_pose.point.x = cur_x
        odom_rob_pose.point.y = cur_y
        odom_rob_pose.point.z = 0

        # 将odom坐标系的车辆坐标转换到map frame下
        map_rob_pos = self.transform_to_map_frame(init_pose=odom_rob_pose,target_frame='map',inital_frame='odom')

        init_data = PoseWithCovarianceStamped()
        init_data.pose.pose.position.x = map_rob_pos.point.x - 0.3
        init_data.pose.pose.position.y = map_rob_pos.point.y - 0.6
        init_data.pose.pose.position.z = 0

        init_data.pose.pose.orientation.x = 0
        init_data.pose.pose.orientation.y = 0
        init_data.pose.pose.orientation.z = 0.706825181105366
        init_data.pose.pose.orientation.w = 0.7073882691671998

        self.pub_start.publish(init_data)
    
    # transform the point from initial frame to target frame
    def transform_to_map_frame(self,init_pose,target_frame, inital_frame):
        self.listener = tf.TransformListener()
        self.listener.waitForTransform(inital_frame,target_frame,rospy.Time(),rospy.Duration(4.0))

        got_tf_transform = False
        while got_tf_transform == False:
            try:
            #   rospy.loginfo('Waiting for the robot transform')
                now = rospy.Time.now()

                #   self.listener.waitForTransform("/odom","/map",now,rospy.Duration(4.0))  # 过时了这方法

                (trans,rot) = self.listener.lookupTransform(inital_frame, target_frame, now)
            
                got_tf_transform = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                got_tf_transform = False
        # have got the transform matrix, and then just calculate the position of odom at the frame of map
        if got_tf_transform == True:
            rob_in_map_frame = self.listener.transformPoint(target_frame,init_pose)
            return rob_in_map_frame
        else:
            return



if __name__ == "__main__":
    # init args
    rospy.init_node('mapbuilder_node', anonymous=True)
    map_builder = map_Builder()
    rospy.spin()