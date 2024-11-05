#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from sensor_interface.msg import NaviFusion ### local_navi
from struct_msgs.msg import IFLYGnss ### ifly_gnss
from struct_msgs.msg import PlanningHMIOutputInfoStr ### planning_hmi
from struct_msgs.msg import FuncStateMachine ### soc_state


class Check:
    def __init__(self):
        self.navi_status = None
        self.gnss = None
        self.gnss_latitude = None
        self.gnss_longitude = None
        self.gnss_altitude = None


        self.planning_hmi_status = None
        self.planning_hmi_sdmaproad = None
        self.planning_hmi_road_type = None

        #fsm status list:
        self.fms_status = None

        #Acc
        self.ACC_STANDBY = 0
        self.ACC_ACTIVATE = 0
        self.ACC_OVERRID = 0

        #Scc
        self.SCC_STANDBY = 0
        self.SCC_ACTIVATE = 0
        self.SCC_OVERRIDE = 0

        #Noa
        self.NOA_STANDBY = 0
        self.NOA_ACTIVATE = 0
        self.NOA_OVERRIDE = 0



    def callback(self):
        navi_data = rospy.wait_for_message("/sensor/navi/navifusion", NaviFusion , timeout=1.0)
        gnss_data = rospy.wait_for_message("/iflytek/sensor/gnss", IFLYGnss , timeout=1.0)
        planning_hmi_data = rospy.wait_for_message("/iflytek/planning/hmi", PlanningHMIOutputInfoStr , timeout=1.0)
        fsm_status_data = rospy.wait_for_message("/iflytek/fsm/soc_state", FuncStateMachine , timeout=1.0)

        if isinstance(navi_data, NaviFusion):
            self.navi_status = navi_data.gps_status
            self.gnss_latitude = navi_data.latitude
            self.gnss_longitude = navi_data.longitude
            self.gnss_altitude = navi_data.altitude

            if self.navi_status is not None and self.navi_status == 42:
                rospy.loginfo("Navi_fusion: %s status is True\n", int(self.navi_status))
                if self.gnss_latitude is not None and self.gnss_longitude is not None and self.gnss_altitude is not None:
                    rospy.loginfo("经纬度 : %s - %s - %s \n" , self.gnss_latitude , self.gnss_longitude , self.gnss_altitude)
                else:
                    rospy.loginfo("经纬度获取异常，请检查Navifusion的状态 : %s %s %s \n" , self.gnss_latitude , self.gnss_longitude , self.gnss_altitude)
            else:
                rospy.loginfo("Navi_fusion: %s status is Warning\n", self.navi_status)

        if isinstance(gnss_data, IFLYGnss):
            self.gnss = gnss_data.gnss_msg.gnss_quality

            if self.gnss is not None and self.gnss == 4:
                rospy.loginfo("mdc_gnss: %s status is True\n", int(self.gnss))

            else:
                rospy.loginfo("mdc_gnss: %s status is Warning\n", self.gnss)

        if isinstance(planning_hmi_data, PlanningHMIOutputInfoStr):
            # self.planning_hmi_status = planning_hmi_data.ihc_output_info.ihc_request_status
            self.planning_hmi_sdmaproad = planning_hmi_data.ad_info.is_in_sdmaproad
            self.planning_hmi_road_type = planning_hmi_data.ad_info.road_type

            if self.planning_hmi_status is not None and self.planning_hmi_sdmaproad == "True" and (self.planning_hmi_road_type == 1 or self.planning_hmi_road_type == 2):
                rospy.loginfo("Noa status is: %s , Noa road_type is %s\n", self.planning_hmi_sdmaproad , self.planning_hmi_road_type)
            else:
                rospy.loginfo("Noa status is: %s , Noa road_type is %s\n", self.planning_hmi_sdmaproad , self.planning_hmi_road_type)

        if isinstance(fsm_status_data , FuncStateMachine):
            self.fms_status = fsm_status_data.current_state

            if self.fms_status is not None and self.fms_status == 3:
                rospy.loginfo("fsm status: %s , 当前状态为:行车抑制\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 4:
                rospy.loginfo("fsm status: %s , 当前状态为:ACC_STANDBY\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 5:
                rospy.loginfo("fsm status: %s , 当前状态为:ACC_ACTIVATE\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 6:
                rospy.loginfo("fsm status: %s , 当前状态为:ACC_OVERRIDE\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 7:
                rospy.loginfo("fsm status: %s , 当前状态为:SCC_STANDBY\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 8:
                rospy.loginfo("fsm status: %s , 当前状态为:SCC_ACTIVATE\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 9:
                rospy.loginfo("fsm status: %s , 当前状态为:SCC_OVERRIDE\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 10:
                rospy.loginfo("fsm status: %s , 当前状态为:NOA_STANDBY\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 11:
                rospy.loginfo("fsm status: %s , 当前状态为:NOA_ACTIVATE\n" , self.fms_status)
            elif self.fms_status is not None and self.fms_status == 12:
                rospy.loginfo("fsm status: %s , 当前状态为:NOA_OVERRIDE\n" , self.fms_status)
            else:
                rospy.loginfo("fsm status: %s , 当前状态为None\n", self.fms_status)


def rosRun():
    CK = Check()
    rospy.init_node('topics', anonymous=True)

    while not rospy.is_shutdown():
        CK.callback()

if __name__ == '__main__':
    try:
        rosRun()
    except KeyboardInterrupt:
        print("code is exit!")