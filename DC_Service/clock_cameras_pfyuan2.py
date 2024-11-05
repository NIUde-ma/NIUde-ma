#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#@author:shuaima6
import rospy
import time
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_interface.msg import NaviFusion
from sensor_interface.msg import CompressedVideo
from proto_msgs.msg import VehicleServiceOutputInfo 
from proto_msgs.msg import PboxDataIMU
from message_filters import ApproximateTimeSynchronizer, Subscriber
from proto_msgs.msg import Camera_Image_Info
from threading import Lock


# topicss = ['front_120_8M','front_left_100_2M','front_right_100_2M','rear_100_2M','rear_left_100_2M','rear_right_100_2M']
camTopics = ['/sensor/fdc/camera/front_120_8M','/sensor/fdc/camera/front_left_100_2M','/sensor/fdc/camera/front_right_100_2M',
                     '/sensor/fdc/camera/rear_100_2M','/sensor/fdc/camera/rear_left_100_2M','/sensor/fdc/camera/rear_right_100_2M']
frame_threshold = len(camTopics)

class CamTs():
    def __init__(self):
        self.ts = {}
        self.frame_count = 0
        self.lock = Lock()
        
    def getKeyByTopicName(self,topic):
        pass
        

    def setTs(self,topic,tss):
        self.ts[topic] = tss
        self.frame_count+=1
        
    

    def printTs(self):
        printStr = ""
        keys = list(self.ts.keys())
        keys.sort()
        try:
            arrTss = []
            arrTss2 = []
            for k in keys:
                s = "{:<38}".format(k)
                arrTss.append(self.ts[k]["isp_arrival_utc"])
                arrTss2.append(self.ts[k]["exposure_duration_time"])
                arrTss.sort()
                arrTss2.sort()
                s2 = " "
                for k2,v2 in self.ts[k].items():
                    s2+= "{} {} ".format(k2,v2)
                printStr += s+s2+"\n"
            if np.mean(np.abs(np.diff(arrTss))) < 11 and np.mean(np.abs(np.diff(arrTss2))) < 5:
                print(np.mean(np.abs(np.diff(arrTss))))
                print(printStr+"\n")
                # print(np.mean(np.abs(np.diff(arrTss))))    
            else:
                print("errors")
                pass
            
        except Exception as e:
            print("获取异常")
            pass



camTs = CamTs()

def callback(msg,topicName):
    

    exposure_time = msg.exposure_starting_utc_time #相机曝光开始UTC时间(微妙)
    # now_time = rospy.Time.now().to_sec()
    # date_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now_time))
    # rospy.loginfo(f'当前处理的时间段为：{date_time}')
    isp_arrival_utc = msg.isp_arrival_utc_timestamp #从ISP收到数据的到达UTC时间(微秒)

    # offset = isp_arrival_utc[11:13]
    # one = isp_arrival_utc[10:13]
    exposure_duration_time = msg.exposure_duration #相机的曝光时长(微秒)
    
    tss = {
        "exposure_time": int(exposure_time/1000) ,
        "isp_arrival_utc": int(isp_arrival_utc/1000),
        "exposure_duration_time":int(exposure_duration_time/1000) ,
    }

    camTs.lock.acquire()
    camTs.setTs(topicName, tss)
    frameCount = camTs.frame_count
    v = frameCount % frame_threshold
    if v == 0 and frameCount != 0:
        camTs.printTs()
    camTs.lock.release()
    
def main():
    global camera_topics
    rospy.init_node('timestamp_sync_example')
    # rate = rospy.Rate(20)  # 设置循环频率为20Hz
    
    # 创建相机
    camera_topics = camTopics
    subscribers = []
    
    for topic1 in camera_topics:
        subscriber = rospy.Subscriber(topic1, Camera_Image_Info, callback, topic1)
        subscribers.append(subscriber)
        
      
    # while not rospy.is_shutdown():
    rate = rospy.Rate(19.99)
    # # rospy.spin()
    # rate.sleep()  # 控制循环的频率
    rospy.spin()  
      
if __name__ == '__main__':
    main()
    
    # while not rospy.is_shutdown():
    #     camTs.printTs()
    #     # rate = rospy.Rate(20)
    #     # rate.sleep(0.1)
    #     rospy.spin()