#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

#@author:shuaima6
import rospy
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_interface.msg import NaviFusion
from sensor_interface.msg import CompressedVideo
from proto_msgs.msg import VehicleServiceOutputInfo 
from proto_msgs.msg import PboxDataIMU
from message_filters import ApproximateTimeSynchronizer, Subscriber
from proto_msgs.msg import Camera_Image_Info


topicss = ['front_120_8M','front_left_100_2M','front_right_100_2M','rear_100_2M','rear_left_100_2M','rear_right_100_2M']

global isp_arrival_utc_rear_righ

def callback(msg):
    
    exposure_time = [msg.exposure_starting_utc_time] #相机曝光开始UTC时间(微妙)
    isp_arrival_utc = [msg.isp_arrival_utc_timestamp] #从ISP收到数据的到达UTC时间(微秒)
    exposure_duration_time = [msg.exposure_duration] #相机的曝光时长(微秒)
    
    offsets = [str(msg.exposure_starting_utc_time)]
    offset_str = ",".join(offsets)
    offset_list = offset_str.split(",")
    
    camera_offsets = [offset[11:13] for offset in offset_list]
    camera_clocks = [offset[10:13] for offset in offset_list]
    rospy.loginfo("camera exposure_time: {}  ".format(exposure_time) + "camera isp_arrival_utc : {}  ".format(isp_arrival_utc) + "camera exposure_duration_time : {} ".format(exposure_duration_time))
    # rospy.loginfo("camera isp_arrival_utc : {}  ".format(isp_arrival_utc))
    # rospy.loginfo("camera exposure_duration_time : {}  ".format(exposure_duration_time))
    # rospy.loginfo("camera offset: {}  ".format(offsets))
    
    
def front_camera_callback(*msgs):
    timestamps = [msg.exposure_starting_utc_time for msg in msgs]
    offsets = [str(msg.exposure_starting_utc_time) for msg in msgs]
    offset_str = ",".join(offsets)
    offset_list = offset_str.split(",")
    
    camera_offsets = [offset[11:13] for offset in offset_list]
    camera_clocks = [offset[10:13] for offset in offset_list]
    rospy.loginfo("Front_camera offset: {}  ".format(camera_offsets) + "clock_camera_nus is: {}  ".format(camera_clocks) + "camera_timestmaps: {}  ".format(timestamps))

def front_left_camera_callback(*msgs):
    timestamps = [msg.exposure_starting_utc_time for msg in msgs]
    offsets = [str(msg.exposure_starting_utc_time) for msg in msgs]
    offset_str = ",".join(offsets)
    offset_list = offset_str.split(",")
    
    camera_offsets = [offset[11:13] for offset in offset_list]
    camera_clocks = [offset[10:13] for offset in offset_list]
    rospy.loginfo("Front_left_camera offset: {}  ".format(camera_offsets) + "clock_camera_nus is: {}  ".format(camera_clocks) + "camera_timestmaps: {}  ".format(timestamps) )  

def front_right_camera_callback(*msgs):
    timestamps = [msg.exposure_starting_utc_time for msg in msgs]
    offsets = [str(msg.exposure_starting_utc_time) for msg in msgs]
    offset_str = ",".join(offsets)
    offset_list = offset_str.split(",")
    
    camera_offsets = [offset[11:13] for offset in offset_list]
    camera_clocks = [offset[10:13] for offset in offset_list]
    rospy.loginfo("Front_right_camera offset: {}  ".format(camera_offsets) + "clock_camera_nus is: {}  ".format(camera_clocks) + "camera_timestmaps: {}  ".format(timestamps) )  
    
def main():
    rospy.init_node('timestamp_sync_example')
    # 创建相机
    camera_topics = ['/sensor/fdc/camera/front_left_100_2M', '/sensor/fdc/camera/rear_right_100_2M']
    # for topic in camera_topics:
    #     sub = Subscriber(topic, Camera_Image_Info)
    #     camera_subs.append(sub)
    subscribers = []
    for topic in camera_topics:
        subscriber = rospy.Subscriber(topic,Camera_Image_Info, callback)
        subscribers.append(subscriber)
    rospy.spin()
    # 创建时间同步器
    # front_camera_topic = '/sensor/fdc/camera/front_120_8M'
    # front_left_camera_topic = '/sensor/fdc/camera/front_left_100_2M'
    # front_right_camera_topic = '/sensor/fdc/camera/front_right_100_2M'
    # rear_camera_topic = '/sensor/fdc/camera/rear_100_2M'
    # rear_camera_left = '/sensor/fdc/camera/rear_left_100_2M'
    # rear_camera_right = '/sensor/fdc/camera/rear_right_100_2M'
    # # ts = ApproximateTimeSynchronizer(camera_subs, queue_size=20, slop=0.1)
    # # ts.registerCallback(camera_callback)
    # rospy.Subscriber(front_camera_topic,Camera_Image_Info,front_camera_callback)

    # rospy.Subscriber(front_left_camera_topic,Camera_Image_Info,front_left_camera_callback)
    
    # rospy.Subscriber(front_right_camera_topic,Camera_Image_Info,front_right_camera_callback)
    
    
    
    # rospy.spin()
    
if __name__ == '__main__':
    
    main()