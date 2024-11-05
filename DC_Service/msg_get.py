#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
import time
import sys
from std_msgs.msg import String
from threading import Lock
from proto_msgs.msg import Camera_Image_Info

camera1 = [
                    '/sensor/fdc/camera/front_120_8M',
                    '/sensor/fdc/camera/front_left_100_2M',
                    '/sensor/fdc/camera/front_right_100_2M',
                    '/sensor/fdc/camera/rear_100_2M',
                    '/sensor/fdc/camera/rear_left_100_2M',
                    '/sensor/fdc/camera/rear_right_100_2M'
            ]

def get_topics():
    camera = [
                    '/sensor/fdc/camera/front_120_8M',
                    '/sensor/fdc/camera/front_left_100_2M',
                    '/sensor/fdc/camera/front_right_100_2M',
                    '/sensor/fdc/camera/rear_100_2M',
                    '/sensor/fdc/camera/rear_left_100_2M',
                    '/sensor/fdc/camera/rear_right_100_2M'
                ]

    # my_camera = ' '.join(camera)
    rospy.init_node('camera_init', anonymous=True)

    topics = rospy.get_published_topics()
    for topic in topics:
        if topic[0] in camera:
            print(f"{topic[0]}")

class cammsg:
    def __init__(self):
        self.offset = 0
        self.camera = []
        self.lock = 0
        self.isp = []
        self.result = [[]]
        self.result_s = [[]]
        
    def camera_input(self, data):
            if data.exposure_starting_utc_time is not None:
                isp = int(data.isp_arrival_utc_timestamp / 1000)
                self.isp.append(isp)
                self.group_timestamps()
                self.offset = isp % 100
                # print(self.isp)
                # rospy.loginfo(f"同一帧的时间戳为：{self.result[current_group]}")
                # rospy.loginfo(f"\n---({topic1}相机曝光):--- %s\n", int(data.exposure_starting_utc_time / 1000))
                # rospy.loginfo(f"\n---({topic1}ISP到达时间):--- %s\n", int(data.isp_arrival_utc_timestamp  / 1000))
                # if all(isp[i] - isp[i-1] <= isp for i in range(1, len(isp))):
                # rospy.loginfo(f"\n---({topic1}---offset): %s\n", self.offset)
                            
                            
            else:
                print(f"---未有输出---")
                exit.sys(0)

        
    def group_timestamps(self):
        self.isp.sort() 
        self.result = [[]]  
        # self.result_s = [[]]
        current_group = 0 
        self.result[current_group].append(self.isp[0])
        
        for i in range(1, len(self.isp)):
            diff = self.isp[i] - self.isp[i - 1]
            
            if diff <= 10 :  
                self.result[current_group].append(self.isp[i])
            else:  
                self.result.append([self.isp[i]])
                current_group += 1
            diffs = self.result[current_group][-1] - self.result[current_group][0]
            diffss = abs(diffs)  #强绑定，去负数
            # print(diffs)
            if diffss <= 6 and len(self.result[current_group]) == 6:
                # print(self.result[current_group])
                rospy.loginfo(f"Isp到达时间的同一帧的时间戳为：{self.result[current_group]},时间同步为：{diffs}")
            # else:  
            #     print("chao zhi")
            
    def Rear_camera_input(self, data):
        if data.exposure_starting_utc_time is not None:
            isp = int(data.isp_arrival_utc_timestamp / 1000)
            self.offset = isp % 100
            rospy.loginfo("\nRear_100---(相机曝光):--- %s\n", int(data.exposure_starting_utc_time / 1000))
            rospy.loginfo("\nRear_100---(ISP到达时间):--- %s\n", int(data.isp_arrival_utc_timestamp  / 1000))
            rospy.loginfo("\nRear_100---(offset):--- %s\n", self.offset)
        else:
            print(f"/sensor/fdc/camera/rear_100_2M---未有输出")
        
def camera_topics_output():
    global topic
    rospy.init_node('camera_init', anonymous=True)
    cam = cammsg()
    # get_camera_topic = get_topics()
    # if get_camera_topic is not None:
    # rospy.Subscriber('/sensor/fdc/camera/rear_100_2M', Camera_Image_Info, cam.Front_camera_input)
    # # rospy.Subscriber('/sensor/fdc/camera/rear_100_2M', Camera_Image_Info, cam.Rear_camera_input)
    # rospy.spin()
    subscribers = []
    for topic in camera1:
        subscriber = rospy.Subscriber(topic,Camera_Image_Info, cam.camera_input,)
        subscribers.append(subscriber)
    rospy.spin()


            
if __name__ == '__main__':
    camera_topics_output()
