#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import sys
import os 
import subprocess
from std_msgs.msg import String
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
        
        self.camera_objects_isp_timestamps = None
        self.msg_camera_objects_timestamps = None
        
        self.fusion_objects_timestamps = None
        self.fusion_objects_isptimestamp = None
        
        self.msg_road_fusion_timestamps = None
        self.road_fuison_isptimestamps = None 
        
        self.vehicle_timestamp = None
        
        self.msg_localization_timestamp = None
        
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

    def callback(self, data):
        # if isinstance(data, LaneLineSet):
        #     topic_check_result = self.check_topics_for_lanes()
            
        #     if topic_check_result is True:
        #         self.camera_lane_timestamps = data.isp_timestamp
        #         self.msg_camera_lane_timestamps = data.msg_header.timestamp
        #         self.msg_lane_debug_header_timestamps = data.msg_header.timestamp
        #         self.lane_debug_front_timestamps = data.isp_timestamp
                
        #         # print(f"is lane lines")
        #         camera_lane_diff = int(self.msg_camera_lane_timestamps - self.camera_lane_timestamps) / 1000
        #         rospy.loginfo("msg_camera_lane_timestamps - camera_lane_timestamps : %s ms\n",(int(camera_lane_diff)))
        #         # self.msg_lane_debug_header_timestamps = data.msg_header.timestamp
        #         # self.lane_debug_front_timestamps = data.camera_perception_input_timestamp.front_isp_timestamp
        #         if self.msg_lane_debug_header_timestamps != 0 and self.lane_debug_front_timestamps != 0:
        #             debug = int((self.msg_lane_debug_header_timestamps) / 1000 ) - int((self.lane_debug_front_timestamps) / 1000 )
        #             rospy.loginfo("debug_header_timestamps - front_isp_times is : %s ms" , (int(debug)))
        #         else:
        #             print(f"Errors debug_header_timestamps : {self.msg_lane_debug_header_timestamps} , front_isp_times : {self.lane_debug_front_timestamps} ")
                
        #     elif topic_check_result is False:
        #         self.msg_lane_topo_timestamps = data.msg_header.timestamp
        #         self.msg_lane_topo_isp_timestamps = data.isp_timestamp
        #         # rospy.loginfo("Topo isp timestamp : %s ms\n",int(self.msg_lane_topo_isp_timestamps))
        #         # print(f"is topo line")

        #     else:
        #         rospy.logerr("not get lane")
        
        if isinstance(data, LaneLineSet):
            self.msg_lane_debug_header_timestamps = data.msg_header.timestamp
            self.lane_debug_front_timestamps = data.isp_timestamp
            
            if self.msg_lane_debug_header_timestamps != 0 and self.lane_debug_front_timestamps != 0:
                debug = int((self.msg_lane_debug_header_timestamps) / 1000 ) - int((self.lane_debug_front_timestamps) / 1000 )
                if int(debug) <= 250:
                    rospy.loginfo("debug_header_timestamps - front_isp_times is : %s ms -- 未超阈值" , (int(debug)))
                else:
                    rospy.loginfo("debug_header_timestamps - front_isp_times is : %s ms -- 超出阈值" , (int(debug)))
            else:
                print(f"Errors debug_header_timestamps : {self.msg_lane_debug_header_timestamps} , front_isp_times : {self.lane_debug_front_timestamps} ")
                
            
        # elif isinstance(data ,LaneLineSet):
            # rospy.loginfo("camera_lane isp_timestamp is : %s", self.camera_lane_timestamps)
        

            # rospy.loginfo("topo: %sms\n",(int(self.msg_lane_topo_isp_timestamps) / 1000))
            
        # elif isinstance(data, CameraPerceptionObjectsInfo):
        #     self.camera_objects_isp_timestamps = data.isp_timestamp
        #     self.msg_camera_objects_timestamps = data.msg_header.timestamp
            
        #     camera_object_diff = int(self.fusion_objects_timestamps - self.msg_camera_objects_timestamps) / 1000
        #     rospy.loginfo("fusion_objects_timestamps - msg_camera_objects_timestamps : %sms\n",(int(camera_object_diff)))
        #     # rospy.loginfo("camera_object isp_timestamp is : %s", self.camera_objects_timestamps)
        
        # elif isinstance(data ,LaneLineSet):
        #     self.msg_lane_debug_header_timestamps = data.msg_header.timestamp
        #     self.lane_debug_front_timestamps = data.camera_perception_input_timestamp.front_isp_timestamp
        #     debug = int((self.msg_lane_debug_header_timestamps)) - int((self.lane_debug_front_timestamps))
        #     rospy.loginfo("debug_header_timestamps - front_isp_times is : %s" , (int(debug)))
            
        # elif isinstance(data, LocalizationEstimate):
        #     self.msg_localization_timestamp = data.msg_header.timestamp
        
        # elif isinstance(data, FusionObjectsInfo):
        #     self.fusion_objects_timestamps = data.msg_header.timestamp
            
        #     diff = int((self.fusion_objects_timestamps/1000) - (self.msg_camera_objects_timestamps / 1000))
        #     if abs(diff) < 250:
        #         rospy.loginfo("fusin_object - camera_object timestamp is : %s ms\n",(int(diff)))
        #     else:
        #         print(f"fusin_object:{self.fusion_objects_timestamps}\n camera_object:{self.msg_camera_objects_timestamps}\n diff:{diff}\n")
        #     # rospy.loginfo("fusion_object isp_timestamp is : %s", self.fusion_objects_timestamps)
        
        # elif isinstance(data, CameraPerceptionObjectsInfo):
        #     self.camera_objects_isp_timestamps = data.isp_timestamp
        #     self.msg_camera_objects_timestamps = data.msg_header.timestamp
            
        #     camera_object_diff = int((self.fusion_objects_timestamps / 1000) - (self.msg_camera_objects_timestamps / 1000))
        #     rospy.loginfo("fusion_objects_timestamps - msg_camera_objects_timestamps : %sms\n",(int(camera_object_diff)))
        #     # rospy.loginfo("camera_object isp_timestamp is : %s", self.camera_objects_timestamps)
        
            
        # elif isinstance(data, VehicleServiceOutputInfo):
        #     self.vehicle_timestamp = data.msg_header.timestamp
            
        #     # print(f"VS:{self.vehicle_timestamp}")
        #     if self.vehicle_timestamp is not None and self.msg_camera_lane_timestamps is not None:
        #         lane_diff = int((self.vehicle_timestamp / 1000) - (self.msg_camera_lane_timestamps / 1000))
        #         rospy.loginfo("vehicle - camera_lane isp_timestamp is : %sms\n", lane_diff)
        #     elif self.msg_lane_topo_isp_timestamps is not None and self.vehicle_timestamp is not None:
        #         topo_lane_diff = int((self.vehicle_timestamp / 1000) - (self.msg_lane_topo_isp_timestamps / 1000))
        #         rospy.loginfo("vehicle - topo_lane isp_timestamp is : %sms\n", topo_lane_diff)
        #     else:
        #         rospy.logerr("One or both of the timestamps are None. Cannot perform subtraction.")
        
        # elif isinstance(data, RoadInfo):
        #     self.msg_road_fusion_timestamps = data.msg_header.timestamp
        #     road_fusion_diff = int((self.msg_localization_timestamp / 1000) - (self.msg_road_fusion_timestamps / 1000))
        #     rospy.loginfo("msg_localization_timestamp - msg_road_fusion is : %s ms\n",road_fusion_diff)

        
    def diffback(self,data):
        
        ### fusion_object - camera_object
        if self.fusion_objects_timestamps is not None and self.msg_camera_objects_timestamps is not None:
            diff = int(self.fusion_objects_timestamps - self.msg_camera_objects_timestamps)
            rospy.loginfo("fusin_object - camera_object timestamp is : %s", int(diff))
        
        ### msg.camera_object - isp_timestamps.camera_object
        elif self.msg_camera_objects_timestamps is not None and self.msg_camera_objects_timestamps is not None:
            camera_object_diff = int(self.msg_camera_objects_timestamps - self.msg_camera_objects_timestamps)
            rospy.loginfo("msg_camera_objects_timestamps - camera_objects_timestampsis : %s", int(camera_object_diff))
        
        ### msg.camera_lane - isp_timestamps.camera_lane
        elif self.camera_lane_timestamps is not None and self.msg_camera_lane_timestamps is not None:
            camera_lane_diff = int(self.msg_camera_lane_timestamps - self.camera_lane_timestamps)
            rospy.loginfo("msg_camera_lane_timestamps - camera_lane_timestamps : %s", int(camera_lane_diff))
        
        else:
            rospy.logwarn("timestamps is None. Waiting for ros data...")
            sys.exit(1)

def rosRun(topics_list):
    ck = Check()
    
    # try:
    #     cmd = "echo $ROS_PACKAGE_PATH"
    #     local_ros = os.popen(cmd).read().strip()
    #     if local_ros == "/home/ros/dev_ws/src:/opt/ros/noetic/share":
    #         print("Ros env init is True !")
            
    #         rospy.init_node('topics', anonymous=True)
    #         rospy.Subscriber(topics_list[1] , LaneLineSet, ck.callback)
    #         rospy.Subscriber("/iflytek/fusion/objects", FusionObjectsInfo ,ck.diffback)
    #         rospy.spin()
            
    #     else:
    #         print(local_ros)
    #         print("Ros env is False ! && please source env")
    #         sys.exit(1)
    # except subprocess.CalledProcessError as e:
    #     print("执行命令时出错:", e)     
        
    if ck.Ros_init() == True: 
        rospy.init_node('topics', anonymous=True)
        # rospy.Subscriber(topics_list[0] , CameraPerceptionObjectsInfo, ck.callback)
        # rospy.Subscriber(topics_list[1] , LaneLineSet, ck.callback)
        # rospy.Subscriber(topics_list[6] , LaneLineSet, ck.callback)
        # rospy.Subscriber(topics_list[2] , FusionObjectsInfo, ck.callback)
        # rospy.Subscriber(topics_list[3] , VehicleServiceOutputInfo, ck.callback)
        # rospy.Subscriber(topics_list[7] , LocalizationEstimate, ck.callback)
        rospy.Subscriber(topics_list[8] , LaneLineSet, ck.callback)
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
            "/iflytek/camera_perception/lane_lines_debug_info"
        ]
    
    rosRun(topics_list)