#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#@author:shuaima6
import rospy
import time
from std_msgs.msg import String
from std_msgs.msg import Header
from message_filters import ApproximateTimeSynchronizer, Subscriber
from proto_msgs.msg import Camera_Image_Info
from threading import Lock
import sys


camTopics=[
    "/sensor/fdc/camera/front_120_8M/compressed",
    "/sensor/fdc/camera/front_left_100_2M/compressed",
    "/sensor/fdc/camera/front_right_100_2M/compressed",
    "/sensor/fdc/camera/rear_100_2M/compressed",
    "/sensor/fdc/camera/rear_left_100_2M/compressed",
    "/sensor/fdc/camera/rear_right_100_2M/compressed"
]

fishTopics=[
    "/sensor/fdc/camera/surround_front_190_1M",
    "/sensor/fdc/camera/surround_left_190_1M",
    "/sensor/fdc/camera/surround_rear_190_1M",
    "/sensor/fdc/camera/surround_right_190_1M"
]

# def Camera_topic():
#     rospy.init_node('camera_topic_read', anonymous=True)
#     published_topics = rospy.get_published_topics()
#     if camTopics in published_topics:
#         print("True")
#     else:
#         print("Not find")
        
frame_threshold = len(camTopics)

class CamTs():
    def __init__(self):
        self.ts = {}
        self.frame_count = 0
        self.lock = Lock()
        self.errCount = 0
        self.historyMaxDelta = 0
        
    def setTs(self,topic,tss):
        self.ts[topic] = tss
        self.frame_count+=1

    def checkerArrivalTs(self,arrivalTss,threshold=10):
        arrivalTss.sort()
        if arrivalTss[-1]-arrivalTss[0] <= threshold:
            # print("dada")
            return True
        else:
            # print("faf")
            return False
    
    
    def checkerExposureTs(self,exposureTss,threshold=10):
        exposureTss.sort()
        diff = exposureTss[-1]-exposureTss[0]
        if diff > self.historyMaxDelta:
            self.historyMaxDelta = diff
        if diff <= threshold:
            print("相机曝光同步正常 曝光时长最大差值为{}ms 程序启动以来接受到的最大差值 {}ms".format(diff,self.historyMaxDelta))
        else:
            print("相机曝光同步异常 曝光时长最大差值为{}ms 程序启动以来接受到的最大差值 {}ms".format(diff.self.historyMaxDelta))
            self.errCount+=1
            if self.errCount>5:
                errmsg= "检测到{}次相机同步异常,程序自动终止".format(self.errCount)
                print(errmsg)
                rospy.signal_shutdown(errmsg)

    def printTs(self):
        # global camera_offset
        printStr = ""
        keys = list(self.ts.keys())
        if len(keys)!=frame_threshold:
            return
        keys.sort()
        try:
            arrTss = []
            arrTss2 = []
            for k in keys:
                s = "{:<45}".format(k)
                arrTss.append(self.ts[k]["isp_arrival_utc_timestamp"])
                arrTss2.append(self.ts[k]["exposure_starting_utc_time"])
                camera_offset = arrTss[11:12]
                s2 = " "
                for k2,v2 in self.ts[k].items():
                    s2+= "{} {} ".format(k2,v2)
                printStr += s+s2+"\n"
            if self.checkerArrivalTs(arrTss):
                print(printStr)
                self.checkerExposureTs(arrTss2)
            else:
                pass
        except Exception as e:
            print(e)
            pass

camTs = CamTs()

def callback(msg,topicName):

    isp_arrival_utc = str(msg.isp_arrival_utc_timestamp)
    camera_offest = isp_arrival_utc[11:13]
    camera_clock = isp_arrival_utc[10:13]
    # print(format(camera_clock))
    # print(topicName ,"exposure_starting_utc_time" , int(msg.exposure_starting_utc_time/1000),"isp_arrival_utc_timestamp" , int(msg.isp_arrival_utc_timestamp/1000),"camera_offset:" , format(camera_offest))
    
    tss = {
        "clock" : format(camera_clock),
        "exposure_starting_utc_time": int(msg.exposure_starting_utc_time/1000) , #相机曝光开始UTC时间(微妙)
        "isp_arrival_utc_timestamp": int(msg.isp_arrival_utc_timestamp/1000), #从ISP收到数据的到达UTC时间(微秒)
        "exposure_duration": int(msg.exposure_duration/1000) , #相机的曝光时长(微秒)
        "camera_offset" : format(camera_offest),
        # "clock_camera" : format(camera_clock)
    }
    # print(topicName, tss)
    camTs.setTs(topicName, tss)
    camTs.printTs()

def call_fish(msg,topicName):
    isp_arrival_utc = str(msg.isp_arrival_utc_timestamp)
    camera_offest = isp_arrival_utc[11:13]
    camera_clock = isp_arrival_utc[10:13]
    # print(format(camera_clock))
    # print(topicName ,"exposure_starting_utc_time" , int(msg.exposure_starting_utc_time/1000),"isp_arrival_utc_timestamp" , int(msg.isp_arrival_utc_timestamp/1000),"camera_offset:" , format(camera_offest))
    
    all_clock = format(camera_clock)
    # print(all_clock)
    tss = {
        "clock" : all_clock,
        "exposure_starting_utc_time": int(msg.exposure_starting_utc_time/1000) , #相机曝光开始UTC时间(微妙)
        "isp_arrival_utc_timestamp": int(msg.isp_arrival_utc_timestamp/1000), #从ISP收到数据的到达UTC时间(微秒)
        "exposure_duration": int(msg.exposure_duration/1000) , #相机的曝光时长(微秒)
        "camera_offset" : format(camera_offest),
        # "clock_camera" : format(camera_clock)
    }
    # diff = [all_clock[i] - all_clock[i-1] for i in range(1, len(all_clock))]
    
    # average_difference = sum(differences) / len(differences)
    # print(average_difference)
    # if format(camera_clock)
    print(topicName,tss)
    
def main():
    rospy.init_node('timestamp_sync_checker')
    # rate = rospy.Rate(20)  # 设置循环频率为20Hz
    topics = rospy.get_published_topics()
    
    # topic_list = [topic[0] for topic in topics]
    # # print(topic_list)
    # # fish_list = [topic_1[0] for topic_1 in topics]
    
    # missing_topics = [topic for topic in camTopics if topic not in topic_list]

    # missing_topics_fish = [topic for topic in fishTopics if topic not in topic_list]
    
    # if not missing_topics:
    #     print("All camTopics found in topics")
    #     camera_topics = camTopics
    #     subscribers = []

    #     for topic1 in camera_topics:
    #         subscriber = rospy.Subscriber(topic, Camera_Image_Info, callback, topic1)
    #         subscribers.append(subscriber)
    #     rospy.spin()
    
    
    # elif not missing_topics_fish:
    #     print("All fishTopics found in topics")
    #     fish_topic = fishTopics
    #     subscribers = []    
       
    #     for topic2 in fish_topic:

    #         subscriberss = rospy.Subscriber(topic2, Camera_Image_Info, call_fish, topic2)
    #         subscribers.append(subscriberss)
    #     rospy.spin()
    
        
    # else:
    #     print("Missing camTopics in topics:")
    #     for topic in missing_topics:
    #         print(topic)
    #     for topic in missing_topics_fish:
    #         print(topic)
            
    # print(published_topics)
    # print(camTopics)
    # # for camera in camTopics:
    # if any(topic in published_topics for topic in camTopics):
    #     print("True")
    # else:
    #     print("Not find camera panorama_camera")
        
    # 创建相机

    camera_topics = camTopics
    subscribers = []
    
    for topic1 in camera_topics:
        subscriber = rospy.Subscriber(topic1, Camera_Image_Info, callback, topic1)
        subscribers.append(subscriber)
        
      
    # while not rospy.is_shutdown():
    # rate = rospy.Rate(19.99)
    # # rospy.spin()
    # rate.sleep()  # 控制循环的频率
    rospy.spin()  
      
if __name__ == '__main__':
    # Camera_topic()

    main()
    
    # while not rospy.is_shutdown():
    #     camTs.printTs()
    #     # rate = rospy.Rate(20)
    #     # rate.sleep(0.1)
    #     rospy.spin()