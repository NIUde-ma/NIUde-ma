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
import message_filters

topicss = ['front_120_8M','front_left_100_2M','front_right_100_2M','rear_100_2M','rear_left_100_2M','rear_right_100_2M']


class CamTs():
    def __init__(self):
        self.ts = {}
        self.frame_count = 0
    
    def getKeyByTopicName(self, topic):
        pass
    
    def setTs(self, topic, tss):
        if topic not in self.ts:
            self.ts[topic] = []  # 创建每个topic的列表
        self.ts[topic].append(tss)
    
    def printTs(self):
        for topic, tss_list in self.ts.items():
            if len(tss_list) >= frame_threshold:  # 只处理达到数量的列表
                printStr = ""
                for tss in tss_list:
                    for k, v in tss.items():
                        printStr += k + ": " + v + "\n"
                print(printStr)
                tss_list.clear()  # 清空该topic的列表
camTs = CamTs()
frame_threshold = 6  # 指定每一帧的消息数量

def callback(msg, topicName):
    global frame_count
    exposure_time = str(msg.exposure_starting_utc_time)
    isp_arrival_utc = str(msg.isp_arrival_utc_timestamp)
    my_isp_time = str(isp_arrival_utc)
    offset = isp_arrival_utc[11:13]
    one = my_isp_time[10:13]
    exposure_duration_time = str(msg.exposure_duration)
    
    tss = {
        "exposure_time": format((exposure_time)),
        "isp_arrival_utc": format((one)),
        "exposure_duration_time": format((exposure_duration_time)),
    }
    
    camTs.setTs(topicName, tss)
    if camTs.frame_count % frame_threshold == frame_threshold - 1:
        camTs.printTs()
    camTs.frame_count += 1
    
def main():
    rospy.init_node('timestamp_sync_example')
    # rate = rospy.Rate(20)  # 设置循环频率为20Hz
    
    # 创建相机
    camera_topics = ['/sensor/fdc/camera/front_120_8M','/sensor/fdc/camera/front_left_100_2M','/sensor/fdc/camera/front_right_100_2M',
                     '/sensor/fdc/camera/rear_100_2M','/sensor/fdc/camera/rear_left_100_2M','/sensor/fdc/camera/rear_right_100_2M']
    subscribers = []
    for topic in camera_topics:
        subscriber = rospy.Subscriber(topic, Camera_Image_Info, callback, topic)
        subscribers.append(subscriber)
        
    # while not rospy.is_shutdown():
    rate = rospy.Rate(20)
    rospy.spin()
    
if __name__ == '__main__':
    main()
