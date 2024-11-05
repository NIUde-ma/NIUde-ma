#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#@author:shuaima6
import rospy
from std_msgs.msg import String
from sensor_interface.msg import  NaviFusion
from std_msgs.msg import Header



def callback(data):
    rospy.loginfo("Received data with timestamp: %s", data.header.stamp.secs)
    

    new_data = String()
    #new_data.data = data.data
    #new_data.header = Header()  
    new_data.data = str(data.header.stamp.secs)
    
    pub.publish(new_data)  
    print(new_data)
    
if __name__ == '__main__':
    
    rospy.init_node('sensor_node')
    print("start pub")
    pub = rospy.Publisher('/sensor/niude', String, queue_size=10)
    print("start subscribe")
    rospy.Subscriber('/sensor/navi/navifusion', NaviFusion, callback)
    print("spin")
    rospy.spin()
