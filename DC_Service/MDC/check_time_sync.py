#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import sys
import os 
import subprocess
from std_msgs.msg import String
from std_msgs.msg import Header
from proto_msgs.msg import Camera_Image_Info
from struct_msgs.msg import LaneLineSet
from struct_msgs.msg import FusionObjectsInfo       
from struct_msgs.msg import CameraPerceptionObjectsInfo 
from struct_msgs.msg import VehicleServiceOutputInfo 
from struct_msgs.msg import RoadInfo
from struct_msgs.msg import LocalizationEstimate


class Check():
    def __init__(self):
        self.local_ros = None
        
        self.camera_lane_timestamps = None
        self.msg_camera_lane_timestamps = None
        
        self.msg_lane_topo_timestamps = None 
        self.msg_lane_topo_isp_timestamps = None
        
        self.msg_lane_debug_header_timestamps = None
        self.lane_debug_front_timestamps = None
        
        self.lane_topo_debug_info_header_timestamps = None
        self.lane_topo_debug_info_isp_timestamps = None
        
        self.camera_objects_isp_timestamps = None
        self.msg_camera_objects_timestamps = None
        
        self.fusion_objects_timestamps = None
        self.fusion_objects_isptimestamp = None
        
        self.msg_road_fusion_timestamps = None
        self.road_fuison_isptimestamps = None 
        
        self.vehicle_timestamp = None
        
        self.msg_localization_timestamp = None
        
        self.frame_count_lane = 0
        self.last_time_lane = None
        self.frame_rate_lane = 0.0
        
        self.frame_count_topo = 0
        self.last_time_topo = None
        self.frame_rate_topo = 0.0
        
        self.frame_count_object = 0
        self.last_time_object = None
        self.frame_rate_object = 0.0
        
        self.window_size = 2.0
        
    def Ros_init(self):
        try:
            cmd = "echo $ROS_PACKAGE_PATH"
            self.local_ros = os.popen(cmd).read().strip()
            
            # if self.local_ros == "/opt/ros/melodic/share":
            if self.local_ros == "/home/ros/dev_ws/src:/opt/ros/noetic/share":
                print(self.local_ros)
                print("Ros env init is True !")
                return True
            else:
                print("Ros env is False ! && please source bash | zsh env")
                print(self.local_ros)
                return False
            
        except subprocess.CalledProcessError as e:
            print("执行命令时出错:", e)
            return False
    
    def lanes(self):
        
        topic_names = [
            "/iflytek/camera_perception/lane_lines",
            "/iflytek/camera_perception/lane_topo"
        ]
        
        published_topics = rospy.get_published_topics()
        for topic, _ in published_topics:
            if topic in topic_names:
                return topic
        return None

    def check_topics_for_lanes(self):
        topic = self.lanes()
        if topic == "/iflytek/camera_perception/lane_lines":
            print("Lane lines topic exists")
            return True
        elif topic == "/iflytek/camera_perception/lane_topo":
            print("Lane topo topic exists")
            return False
        else:
            print("Lane topic does not exist")
            return None

    def callback_lane(self, data):
        
        if isinstance(data, LaneLineSet):
            self.msg_lane_debug_header_timestamps = data.msg_header.timestamp
            self.lane_debug_front_timestamps = data.isp_timestamp
            
            if self.msg_lane_debug_header_timestamps != 0 and self.lane_debug_front_timestamps != 0:
                debug = int((self.msg_lane_debug_header_timestamps) / 1000 ) - int((self.lane_debug_front_timestamps) / 1000 )
                if int(debug) <= 250:
                    print("\n")
                    rospy.loginfo("lane_debug_header_timestamps - lane_front_isp_times is : %s ms -- 未超阈值" , (int(debug)))
                else:
                    rospy.loginfo("lane_debug_header_timestamps - lane_front_isp_times is : %s ms -- 超出阈值" , (int(debug)))
            else:
                print(f"Errors debug_header_timestamps : {self.msg_lane_debug_header_timestamps} , front_isp_times : {self.lane_debug_front_timestamps} ")
            
            self.frame_count_lane += 1
            current_time = rospy.get_time()
            if self.last_time_lane is not None:
                elapsed_time = current_time - self.last_time_lane
                if elapsed_time >= self.window_size:
                    self.frame_rate_lane = self.frame_count_lane / elapsed_time
                    rospy.loginfo("lane_lines_debug_info rate: %.2f HZ", self.frame_rate_lane)
                    self.frame_count_lane = 0
                    self.last_time_lane = current_time
            else:
                self.last_time_lane = current_time
        
    def callback_topo(self, data):
        if isinstance(data, LaneLineSet):
            self.lane_topo_debug_info_header_timestamps = data.msg_header.timestamp
            self.lane_topo_debug_info_isp_timestamps = data.isp_timestamp
            
            if self.lane_topo_debug_info_header_timestamps != 0 and self.lane_topo_debug_info_isp_timestamps != 0:
                topo_debug = int((self.lane_topo_debug_info_header_timestamps) / 1000 ) - int((self.lane_topo_debug_info_isp_timestamps) / 1000 )
                
                if int(topo_debug) <= 250:
                    print("\n")
                    rospy.loginfo("lane_topo_debug_info_header_timestamps - lane_topo_debug_info_isp_timestamps is : %s ms -- 未超阈值" , (int(topo_debug)))
                else:
                    rospy.loginfo("lane_topo_debug_info_header_timestamps - lane_topo_debug_info_isp_timestamps is : %s ms -- 超出阈值" , (int(topo_debug)))
            else:
                print(f"Errors lane_topo_debug_info_header_timestamps : {self.lane_topo_debug_info_header_timestamps} , lane_topo_debug_info_isp_timestamps : {self.lane_topo_debug_info_isp_timestamps} ") 
            
            self.frame_count_topo += 1
            current_time = rospy.get_time()
            if self.last_time_topo is not None:
                elapsed_time = current_time - self.last_time_topo
                if elapsed_time >= self.window_size:
                    self.frame_rate_topo = self.frame_count_topo / elapsed_time
                    rospy.loginfo("lane_topo_debug_info rate: %.2f HZ", self.frame_rate_topo)
                    self.frame_count_topo = 0
                    self.last_time_topo = current_time
            else:
                self.last_time_topo = current_time
    
    def callback_object(self, data):
        if isinstance(data, CameraPerceptionObjectsInfo):
            self.camera_objects_isp_timestamps = data.isp_timestamp
            self.msg_camera_objects_header_timestamps = data.msg_header.timestamp
            
            if self.msg_camera_objects_header_timestamps != 0 and self.camera_objects_isp_timestamps != 0:
                objects = int((self.msg_camera_objects_header_timestamps / 1000 ) - (self.camera_objects_isp_timestamps / 1000 ) )
                if int(objects) <= 250:
                    print("\n")
                    rospy.loginfo("msg_camera_objects_header_timestamps - camera_objects_isp_timestamps is : %s ms -- 未超阈值" , (int(objects)))
                else:
                    rospy.loginfo("msg_camera_objects_header_timestamps - camera_objects_isp_timestamps is : %s ms -- 超出阈值" , (int(objects)))
            else:
                print(f"Errors msg_camera_objects_header_timestamps : {self.msg_camera_objects_header_timestamps} , camera_objects_isp_timestamps : {self.camera_objects_isp_timestamps} ") 
            
            self.frame_count_object += 1
            current_time = rospy.get_time()
            if self.last_time_object is not None:
                elapsed_time = current_time - self.last_time_object
                if elapsed_time >= self.window_size:
                    self.frame_rate_object = self.frame_count_object / elapsed_time
                    rospy.loginfo("camera_object rate: %.2f HZ", self.frame_rate_object)
                    self.frame_count_object = 0
                    self.last_time_object = current_time
            else:
                self.last_time_object = current_time
            
def rosRun(topics_list):
    ck = Check()
        
    if ck.Ros_init() == True: 
        rospy.init_node('topics', anonymous=True)
        rospy.Subscriber(topics_list[0] , CameraPerceptionObjectsInfo, ck.callback_object)
        # rospy.Subscriber(topics_list[1] , LaneLineSet, ck.callback)
        # rospy.Subscriber(topics_list[6] , LaneLineSet, ck.callback)
        # rospy.Subscriber(topics_list[2] , FusionObjectsInfo, ck.callback)
        # rospy.Subscriber(topics_list[3] , VehicleServiceOutputInfo, ck.callback)
        # rospy.Subscriber(topics_list[7] , LocalizationEstimate, ck.callback)
        rospy.Subscriber(topics_list[8] , LaneLineSet, ck.callback_lane)
        rospy.Subscriber(topics_list[9] , LaneLineSet, ck.callback_topo)
        # rospy.Subscriber(topics_list[2], FusionObjectsInfo, ck.diffback)
        # rospy.Subscriber(topics_list[0], CameraPerceptionObjectsInfo, ck.diffback)
        # rospy.Subscriber(topics_list[1], LaneLineSet, ck.diffback)
        rospy.spin()
    else:
        print("Ros env is False ! && please source env")
        sys.exit(1)
        
    
if __name__ == '__main__':
    
    topics_list = [
        
            "/iflytek/camera_perception/objects",
            "/iflytek/camera_perception/lane_lines",
            "/iflytek/fusion/objects",
            "/iflytek/vehicle_service",
            "/iflytek/fusion/road_fusion",
            "/iflytek/localization/ego_pose",
            "/iflytek/camera_perception/lane_topo",
            "/iflytek/localization/ego_pose",
            "/iflytek/camera_perception/lane_lines_debug_info",
            "/iflytek/camera_perception/lane_topo_debug_info"
        ]
    
    rosRun(topics_list)