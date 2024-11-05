#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import argparse
import rospy
import json
import os
from std_msgs.msg import String
import rostopic
import roslib
import sys
import paramiko
import signal
import subprocess
import threading

def check_version():
    try:
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        client.connect('192.168.1.10', username='root', password='', allow_agent=False, look_for_keys=False)

        stdin, stdout, stderr = client.exec_command('cat /asw/control/VERSION')
        control_version = stdout.read().decode('utf-8')

        stdin, stdout, stderr = client.exec_command('cat /asw/planning/VERSION')
        planning_version = stdout.read().decode('utf-8')

        stdin, stdout, stderr = client.exec_command('head -n 1 /asw/CHANGELOG.MD')
        changelog_version = stdout.read().decode('utf-8')

        print(f"Control Version: {control_version}")
        print(f"Planning Version: {planning_version}")
        print(f"Changelog Version: {changelog_version}")

        client.close()
    except Exception as e:
        print(f"Error: {str(e)}")

def interrupt_handler(signum, frame):
    global soc_timestamp,vehicle_timestamp,socformcu_timestamp,position_datas,imu_timestamp,soc_state

    print("键盘中断，程序退出")
    sys.exit(0)

def check_topics_and_timestamps():
    roscore_cmd = 'source /home/ros/dew_ws/devel/setep.zsh'

    roscore_process = subprocess.Popen(roscore_cmd, shell=True, executable='/bin/bash', preexec_fn=os.setsid)

    rostopic_cmd = 'rostopic echo /iflytek/vehicle_service'

    grep_cmd = 'grep -E "pilot_long_control_actuator_status:|pilot_lat_control_actuator_status:"'

    rostopic_echo = 'rostopic echo /iflytek/system_state/soc_state'

    grep_cmd2 = 'grep timestamp'

    rostopic_process = subprocess.Popen(rostopic_cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    grep_process = subprocess.Popen(grep_cmd, shell=True, executable='/bin/bash', stdin=rostopic_process.stdout, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

    mcu_for_soc_cmd = 'rostopic echo /iflytek/adas_function_debug'
    
    grep_cmd3 = 'grep lat_ctrl_trq_req_value:'
    
    get_debug_cmd4 = 'rostopic echo /iflytek/planning/debug_info'
   
    grep_cmd4 = 'grep '
    
    get_debug_cmd5 = 'rostopic echo /iflytek/ehr/position'
    
    grep_cmd5 = 'grep -E "failsafe_loc_status:|failsafe_hdmap_status:|failsafe_gnss_status:|failsafe_camera_status:|failsafe_imu_status:|failsafe_vehicle_status:"'
    
    get_imu_cmd = 'rostopic echo /iflytek/sensor/pbox/imu'
    
    grep_imu_cmd = 'grep timestamp'
# rostopic echo /iflytek/ehr/position |grep -E "failsafe_loc_status:|failsafe_hdmap_status:|failsafe_gnss_status:|failsafe_camera_status:|failsafe_imu_status:|failsafe_vehicle_status:"     
    try:
        for line in grep_process.stdout:
            print(line.strip())

        rostopic_process.stdout.close()
        grep_process.stdout.close()

        grep_process.wait(timeout=60)

        if grep_process.returncode != 0:
            print("命令执行出错")

    except subprocess.TimeoutExpired:
        grep_process.terminate()
        print("命令执行超时")

    except KeyboardInterrupt:
        grep_process.terminate()
        rostopic_process.terminate()
        print("键盘中断，程序退出")
        sys.exit(0)

    echo_process = subprocess.Popen(rostopic_echo, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    grep_cmd2_process = subprocess.Popen(grep_cmd2, shell=True, executable='/bin/bash', stdin=echo_process.stdout,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

    try:
        for line2 in grep_cmd2_process.stdout:
            print(line2.strip())

        echo_process.stdout.close()
        grep_cmd2_process.stdout.close()

        grep_cmd2_process.wait(timeout=60)

        if grep_cmd2_process.returncode != 0:
            print("命令执行出错")

    except subprocess.TimeoutExpired:
        grep_cmd2_process.terminate()
        print("命令执行超时")

    except KeyboardInterrupt:
        grep_cmd2_process.terminate()
        echo_process.terminate()
        print("键盘中断，程序退出")

        
    mcu_for_soc_process = subprocess.Popen(mcu_for_soc_cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
    grep3_process = subprocess.Popen(grep_cmd3, shell=True, executable='/bin/bash', stdin=echo_process.stdout,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)

    try:
        for line3 in grep3_process.stdout:
            
            print(line3.strip())

        mcu_for_soc_process.stdout.close()
        grep3_process.stdout.close()

        grep3_process.wait(timeout=60)

        if grep3_process.returncode != 0:
            print("命令执行出错")

    except subprocess.TimeoutExpired:
        grep3_process.terminate()
        print("命令执行超时")

    except KeyboardInterrupt:
        grep3_process.terminate()
        mcu_for_soc_process.terminate()
        print("键盘中断，程序退出")
    
    position_process = subprocess.Popen(get_debug_cmd5, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    grep5_process = subprocess.Popen(grep_cmd5, shell=True, executable='/bin/bash', stdin=echo_process.stdout,
                                        stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    try:
        for line3 in grep3_process.stdout:
            
            print(line3.strip())

        mcu_for_soc_process.stdout.close()
        grep3_process.stdout.close()

        grep3_process.wait(timeout=60)

        if grep3_process.returncode != 0:
            print("命令执行出错")

    except subprocess.TimeoutExpired:
        grep3_process.terminate()
        print("命令执行超时")

    except KeyboardInterrupt:
        grep3_process.terminate()
        mcu_for_soc_process.terminate()
        print("键盘中断，程序退出")    
    
    
    imu_process = subprocess.Popen(get_imu_cmd, shell=True, executable='/bin/bash', stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    grep_imu_process = subprocess.Popen(grep_imu_cmd, shell=True, executable='/bin/bash', stdin=echo_process.stdout,
                                    stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
    try:
        for line3 in grep3_process.stdout:
            
            print(line3.strip())

        mcu_for_soc_process.stdout.close()
        grep3_process.stdout.close()

        grep3_process.wait(timeout=60)

        if grep3_process.returncode != 0:
            print("命令执行出错")

    except subprocess.TimeoutExpired:
        grep3_process.terminate()
        print("命令执行超时")

    except KeyboardInterrupt:
        grep3_process.terminate()
        mcu_for_soc_process.terminate()
        print("键盘中断，程序退出")
        
# def pprint_timestamp():
#     soc_timestamp_cmd = 'rostopic echo /iflytek/system_state/soc_state'
#     vehicle_timestamp_cmd = 'rostopic echo /iflytek/vehicle_service'
#     grep_cmd = 'grep timestamp'
    
#     soc_process = subprocess.Popen(soc_timestamp_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
#     vehicle_process = subprocess.Popen(vehicle_timestamp_cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    
#     # 获取 /iflytek/system_state/soc_state 时间戳
#     grep_cmd1_process = subprocess.Popen(grep_cmd, shell=True, stdin=soc_process.stdout,
#                                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
#     soc_timestamp_regex = r'timestamp: (\d+)'  
#     soc_timestamp_list = []
    
#     try:
#         for line in grep_cmd1_process.stdout:
#             soc_matches = re.findall(soc_timestamp_regex, line) 
#             for match1 in soc_matches:
#                 timestamp = match1
#                 soc_timestamp_list.append(float(timestamp))
        
#             for timestamp in soc_timestamp_list:
#                 print(f"Soc_timestamps:{timestamp}")

#     except subprocess.TimeoutExpired:
#         grep_cmd1_process.terminate()
#         print("命令执行超时")

#     except KeyboardInterrupt:
#         grep_cmd1_process.terminate()
#         print("键盘中断，程序退出")
        
#     # 获取 /iflytek/vehicle_service 时间戳
#     grep_cmd2_process = subprocess.Popen(grep_cmd, shell=True, stdin=vehicle_process.stdout,
#                                          stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
#     vehicle_timestamp_regex = r'timestamp: (\d+)'  
#     vehicle_timestamp_list = []

#     try:
#         for line2 in grep_cmd2_process.stdout:
#             vehicle_matches = re.findall(vehicle_timestamp_regex, line2)
#             for match2 in vehicle_matches:
#                 timestamp = match2
#                 vehicle_timestamp_list.append(float(timestamp))

#         for timestamp in vehicle_timestamp_list:
#             print(f"Vehicle_timestamps:{timestamp}")

#     except subprocess.TimeoutExpired:
#         grep_cmd2_process.terminate()
#         print("命令执行超时")

#     except KeyboardInterrupt:
#         grep_cmd2_process.terminate()
#         print("键盘中断，程序退出")

msgDefinitions = []
socTopic = "/iflytek/system_state/soc_state"
socDataClass = rostopic.get_topic_class(socTopic, blocking=False)[0]
msgDefinitions.append((socTopic, socDataClass))

vehicleTopic = "/iflytek/vehicle_service"
vehicleDataClass = rostopic.get_topic_class(vehicleTopic, blocking=False)[0]
msgDefinitions.append((vehicleTopic, vehicleDataClass))

socformcu = "/iflytek/adas_function_debug"
socformcuClass = rostopic.get_topic_class(socformcu, blocking=False)[0]
msgDefinitions.append((socformcu, socformcuClass))

position = "/iflytek/ehr/position"
positionClass = rostopic.get_topic_class(position, blocking=False)[0]
# print(f"1111{positionClass}")
msgDefinitions.append((position, positionClass))

imuTopic = "/iflytek/sensor/pbox/imu"
imuClass = rostopic.get_topic_class(imuTopic, blocking=False)[0]
msgDefinitions.append((imuTopic, imuClass))

soc_timestamp = 0
vehicle_timestamp = 0
socformcu_timestamp = 0
position_datas = 0
imu_timestamp = 0

for i in msgDefinitions:
    if i[1] is None:
        print("Failed to get [ {} ]'s msg type, have you sourced the setup.bash file?".format(i[0]))
        sys.exit(0)

def printSocTS(msg,):
    global soc_timestamp
    soc_timestamp = msg.header.timestamp
    soc_state = msg.current_state
    
    if soc_state == 19:
        print(f"当前的功能为：== NOA_ACTIVATE {soc_state} \n")
    elif soc_state == 12:
        print(f"当前的功能为：== SCC_ACTIVATE {soc_state} \n")
    elif soc_state == 5:
        print(f"当前的功能为：== ACC_ACTIVATE {soc_state} \n")
    elif soc_state == 23:
        print(f"当前的功能为：== PARK_IN_APA_IN {soc_state} \n")
    elif soc_state == 1:
        print(f"当前的功能为：== STANDBY {soc_state} \n")
    elif soc_state == 27:
        print(f"当前的功能为： == PARK_IN_NO_READY {soc_state} \n")
    elif soc_state == 28:
        print(f"当前的功能为： == PARK_IN_ACTIVATE {soc_state} \n")
    elif soc_state == 29:
        print(f"当前的功能为： == PARK_IN_ACTIVATE_WAIT {soc_state} \n")
    elif soc_state == 30:
        print(f"当前的功能为： == PARK_IN_ACTIVATE_CONTROL {soc_state} \n")
    else:
        print(f"当前的功能为： === ？ {soc_state} 异常请联系shuaima6")
    
def printVehicleTS(msg,):
    global vehicle_timestamp
    vehicle_timestamp = msg.header.timestamp

    if soc_timestamp != 0 and vehicle_timestamp != 0:
        time_diff = soc_timestamp - vehicle_timestamp
        clock_time = format(time_diff)
        print(f"Soc-Mcu timestamp is: {clock_time} μs \n")
    else:
        print("---Errors 未获取到Soc的时间戳、Vehicle的时间戳---")
        
def printMcuforSoc(msg,):
    global  socformcu_timestamp
    # global soc_state
    # soc_state = msg.header.current_state
    print(soc_state)
    socformcu_timestamp = msg.control_adaptor_debug_output_info.lat_ctrl_trq_req_value
    #zuizhong = format(socformcu_timestamp)
    print(f"Mcu is get Soc data time is: {socformcu_timestamp} \n")
    
def printposition(msg,):
    global position_datas
    position_datas = msg.fail_safe
    print(f"position_datas is: \n {position_datas} \n")

def printimu_timestamps(msg,):
    global imu_timestamp
    imu_timestamp = msg.header.timestamp
    if soc_timestamp != 0 and imu_timestamp != 0:
        clocks_time = imu_timestamp - soc_timestamp
        times = format(clocks_time)
        print(f"imu_timestamp - soc_timestamp is : {clocks_time} μs\n")
    else:
        print("---Errors 未获取到Imu的时间戳、Soc的时间戳---")
        
if __name__ == '__main__':
    #check_version()
    parser = argparse.ArgumentParser(description='注意要执行的功能！')
    parser.add_argument('module', metavar='MODULE', type=str, choices=["SCC","NOA","ACC"], help='注意使用功能如：SCC、NOA、ACC')
    args = parser.parse_args()
    signal.signal(signal.SIGINT, interrupt_handler)
    threading.Thread(target=check_topics_and_timestamps).start()
    
    if args.module == "SCC" or args.module == "ACC":
        rospy.init_node('niude_monitor', anonymous=True)
        
        socTopic = "/iflytek/system_state/soc_state"
        socDataClass = rostopic.get_topic_class(socTopic, blocking=False)[0]
        socstateClass = rostopic.get_topic_class(socTopic, blocking=False)[0]
        rospy.Subscriber(socTopic, socDataClass, printSocTS, queue_size=1)
          
        vehicleTopic = "/iflytek/vehicle_service"
        vehicleDataClass = rostopic.get_topic_class(vehicleTopic, blocking=False)[0]
        rospy.Subscriber(vehicleTopic, vehicleDataClass, printVehicleTS, queue_size=1)
        
        socformcu = "/iflytek/adas_function_debug"
        socformcuClass = rostopic.get_topic_class(socformcu, blocking=False)[0]
        rospy.Subscriber(socformcu, socformcuClass, printMcuforSoc, queue_size=1)
        
        imuTopic = "/iflytek/sensor/pbox/imu"
        imuClass = rostopic.get_topic_class(imuTopic, blocking=False)[0]
        rospy.Subscriber(imuTopic, imuClass, printimu_timestamps, queue_size=1)
        
    elif args.module == "NOA":
        rospy.init_node('niude_monitor', anonymous=True)
        
        socTopic = "/iflytek/system_state/soc_state"
        socDataClass = rostopic.get_topic_class(socTopic, blocking=False)[0]
        socstateClass = rostopic.get_topic_class(socTopic, blocking=False)[0]
        rospy.Subscriber(socTopic, socDataClass, printSocTS, queue_size=1)
        
        vehicleTopic = "/iflytek/vehicle_service"
        vehicleDataClass = rostopic.get_topic_class(vehicleTopic, blocking=False)[0]
        rospy.Subscriber(vehicleTopic, vehicleDataClass, printVehicleTS, queue_size=1)
        
        socformcu = "/iflytek/adas_function_debug"
        socformcuClass = rostopic.get_topic_class(socformcu, blocking=False)[0]
        rospy.Subscriber(socformcu, socformcuClass, printMcuforSoc, queue_size=1)
        
        position = "/iflytek/ehr/position"
        positionClass = rostopic.get_topic_class(position, blocking=False)[0]
        rospy.Subscriber(position, positionClass, printposition, queue_size=1)
        
        imuTopic = "/iflytek/sensor/pbox/imu"
        imuClass = rostopic.get_topic_class(imuTopic, blocking=False)[0]
        rospy.Subscriber(imuTopic, imuClass, printimu_timestamps, queue_size=1)
        
    else:
        print(f"---请检查你输入的功能是正确---")
        exit()
        
    # elif args.module == "NOA":
    #     position = "/iflytek/ehr/position"
    #     positionClass = rostopic.get_topic_class(position, blocking=False)[0]
    #     rospy.Subscriber(position, positionClass, printposition, queue_size=1)
    # else:
    #     print(f"---请检查你输入的功能是正确---")
    #     exit()
        
    rospy.spin()

# if __name__ == '__main__':
#     check_version()
#     signal.signal(signal.SIGINT, interrupt_handler)
#     check_topics_and_timestamps()
#     rospy.init_node('niude_monitor', anonymous=True)
#     rospy.Subscriber(socTopic, socDataClass, printSocTS, queue_size=1)
#     rospy.Subscriber(vehicleTopic, vehicleDataClass, printVehicleTS, queue_size=1)
#     rospy.spin()