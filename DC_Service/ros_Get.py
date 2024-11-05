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

def Navi_Get(Navi):
    rospy.loginfo("niude get Navi_timestamp: %s", Navi.meta.timestamp_us)
    rospy.loginfo("niude get Gnss_status %s", Navi.gps_status)

    navi = String()
    navi.data = str(Navi.meta.timestamp_us)
    navi.data = str(Navi.gps_status)
    pub.publish(navi)

def Camera_Get_front(data):
    rospy.loginfo("niude get front camera_timestamp: %s", data.header.stamp)
    
    camera = String()
    camera.data = str(data.header.stamp)
    timestamp_str = str(data.header.stamp)
    two_digits = timestamp_str[11:13]
    print(f"front 120 camera offset {two_digits}")
    camera.data = two_digits
    pub.publish(camera)

def Camera_Get_rear(data):
    rospy.loginfo("niude get rear camera_timestamp: %s", data.header.stamp)
    
    camera = String()
    camera.data = str(data.header.stamp)
    timestamp_str = str(data.header.stamp)
    two_digits = timestamp_str[11:13]
    print(f"rear 100 camera offset {two_digits}")
    camera.data = two_digits
    pub.publish(camera)

def Camera_Get_front_left(data):
    rospy.loginfo("niude get front_left camera_timestamp: %s", data.header.stamp)
    
    camera = String()
    camera.data = str(data.header.stamp)
    timestamp_str = str(data.header.stamp)
    two_digits = timestamp_str[11:13]
    print(f"rear 100 camera offset {two_digits}")
    camera.data = two_digits
    pub.publish(camera)
    
def Camera_Get_front_right(data):
    rospy.loginfo("niude get front_right camera_timestamp: %s", data.header.stamp)
    
    camera = String()
    camera.data = str(data.header.stamp)
    timestamp_str = str(data.header.stamp)
    two_digits = timestamp_str[11:13]
    print(f"front_right camera offset {two_digits}")
    camera.data = two_digits
    pub.publish(camera)    
    
def Camera_Get_rear_left(data):
    rospy.loginfo("niude get rear_left camera_timestamp: %s", data.header.stamp)
    
    camera = String()
    camera.data = str(data.header.stamp)
    timestamp_str = str(data.header.stamp)
    two_digits = timestamp_str[11:13]
    print(f"rear_left camera offset {two_digits}")
    camera.data = two_digits
    pub.publish(camera)  

def Camera_Get_rear_right(data):
    rospy.loginfo("niude get rear_right camera_timestamp: %s", data.header.stamp)
    
    camera = String()
    camera.data = str(data.header.stamp)
    timestamp_str = str(data.header.stamp)
    two_digits = timestamp_str[11:13]
    print(f"rear_right camera offset {two_digits}")
    camera.data = two_digits
    pub.publish(camera)  
    
def Vehicle_Get(data):
    rospy.loginfo("niude get Vehicle_timestamp: %s", data.header.timestamp)
    global  vehicle_times
    vehicle = String()
    vehicle.data = str(data.header.timestamp)
    vehicle_times = vehicle.data
    pub.publish(vehicle)

def Imu_Get(Imu):
    rospy.loginfo("niude get Imu_timestamp: %s", Imu.header.timestamp)
    # rospy.loginfo("niude111 get Imu_timestamp: %s", Imu.header.timestamp )
    global vehicle_times
    imu = String()
    imu.data = str(Imu.header.timestamp)
    imu_times = imu.data
    mutimes = float(imu_times) - float(vehicle_times)
    pub.publish(imu)
    print(f"imu - vehicle timestamps is :{mutimes} us")

# def OK():
#     global imu_times, vehicle_times
#     imu_timess = str(imu_times)
#     vehicle_timess = str(vehicle_times)
#     mytime = imu_timess - vehicle_timess
#     print(f"imu - vehicle timestamps is :{mytime}")
        
if __name__ == '__main__':
    
    rospy.init_node('sensor_node', anonymous=True)
    print("start pub")
    pub = rospy.Publisher('/sensor/niude', String, queue_size=5)
    print("---Get Navi data---")
    #rospy.Subscriber('/sensor/navi/navifusion', NaviFusion, Navi_Get)
    print("---Get Camera data---")
    rospy.Subscriber('/sensor/fdc/camera/front_120_8M', CompressedVideo, Camera_Get_front)
    #
    rospy.Subscriber('/sensor/fdc/camera/front_left_100_2M', CompressedVideo, Camera_Get_front_left)
    #
    rospy.Subscriber('/sensor/fdc/camera/front_right_100_2M', CompressedVideo, Camera_Get_front_right)
    #
    rospy.Subscriber('/sensor/fdc/camera/rear_100_2M', CompressedVideo, Camera_Get_rear)
    #
    rospy.Subscriber('/sensor/fdc/camera/rear_left_100_2M', CompressedVideo, Camera_Get_rear_left)
    #
    rospy.Subscriber('/sensor/fdc/camera/rear_right_100_2M', CompressedVideo, Camera_Get_rear_right)
    #
    print("---Get Vehicle data---")
    # rospy.Subscriber('/iflytek/vehicle_service', VehicleServiceOutputInfo, Vehicle_Get)
    print("---Get Imu data---")
    # rospy.Subscriber('/iflytek/sensor/pbox/imu', PboxDataIMU, Imu_Get)
    print("spin")
    rospy.spin()
