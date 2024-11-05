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

def camera_callback(*msgs):
    # 从不同相机的消息中获取时间戳
    timestamps = [msg.header.stamp.to_sec() for msg in msgs]
    rospy.loginfo("Timestamps: {}".format(timestamps))
def main():
    rospy.init_node('timestamp_sync_example')
    # 创建相机消息的订阅者
    camera_subs = []
    camera_topics = ['/sensor/fdc/camera/front_120_8M', '/sensor/fdc/camera/front_left_100_2M', '/sensor/fdc/camera/front_right_100_2M', '/sensor/fdc/camera/rear_100_2M', ' /sensor/fdc/camera/rear_left_100_2M', '/sensor/fdc/camera/rear_right_100_2M']
    for topic in camera_topics:
        sub = Subscriber(topic, CompressedImage)
        camera_subs.append(sub)
    # 创建时间同步器
    ts = ApproximateTimeSynchronizer(camera_subs, queue_size=10, slop=0.1)
    ts.registerCallback(camera_callback)
    rospy.spin()
if __name__ == '__main__':
    main()