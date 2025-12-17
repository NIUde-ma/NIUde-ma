#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import datetime
from datetime import datetime, timedelta , timezone
import subprocess
import command
import common

collect = command.host_env()

def output_beiyun_size():
    get_beiyun_size_cmd = "timeout 3 nc 192.168.5.111 4444"

    try:
        result = subprocess.run(
            get_beiyun_size_cmd,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=5
        )

        if result.stdout is not None:
            output = result.stdout.decode("utf-8").strip()
            return output

    except subprocess.TimeoutExpired:
        print("cmd command is timeout")
    except Exception as e:
        print(f"cmd command is errors: {e}")

def parse_gps_time(gps_week, gps_sow):

    gps_epoch = datetime(1980, 1, 6)
    total_seconds = gps_week * 7 * 24 * 3600 + gps_sow
    utc_time = gps_epoch + timedelta(seconds=total_seconds)

    beijing_tz = timezone(timedelta(hours=8))
    beijing_time = utc_time.astimezone(beijing_tz)
    beijing_time_plus8 = beijing_time + timedelta(hours=8)
    final_time = beijing_time_plus8.replace(microsecond=0, tzinfo=None)
    return final_time

# def parse_inspvaxa():
#     outputs = output_beiyun_size()
    
#     if outputs:
#         if outputs.startswith('#'):
#             outputs = outputs[1:]
        
#         fields = outputs.split(',')
        
#         if len(fields) < 7:
#             return None
        
#         try:
#             gps_week = int(fields[5])
#             gps_sow = float(fields[6])
            
#             utc_time_str = parse_gps_time(gps_week, gps_sow)  # 获取字符串
#             print(f"原始时间字符串: {utc_time_str}")
            
#             if utc_time_str is not None:
#                 # 将字符串转换为 datetime 对象
#                 try:
#                     # 假设格式是 "2025-12-08 18:03:22"
#                     utc_datetime = datetime.datetime.strptime(utc_time_str, "%Y-%m-%d %H:%M:%S")
#                     return [0, utc_datetime]  # 返回 datetime 对象
#                 except ValueError as e:
#                     print(f"时间格式转换错误: {e}")
#                     return [1, None]
#             else:
#                 return [1, None]

#         except (ValueError, IndexError) as e:
#             print(f"解析错误: {e}")
#             return [1, None]

def parse_inspvaxa():
    outputs = output_beiyun_size()
    
    if outputs:
        if outputs.startswith('#'):
            outputs = outputs[1:]
        
        fields = outputs.split(',')
        
        if len(fields) < 7:
            return None
        
        try:
            gps_week = int(fields[5])
            gps_sow = float(fields[6])
            
            utc_time = parse_gps_time(gps_week, gps_sow)
            # print(utc_time)
            # print(str(utc_time.split(" ")))
            if utc_time is not None:
                # date_part = utc_time.split(" ")[0] 
                return [ 0 , utc_time]
            else:
                return [ 1 , utc_time ]

        except (ValueError, IndexError) as e:
            print(f"解析错误: {e}")

def diff_main():
    orin_time = collect.get_orin_time()  # (0, '2025-12-08')
    GPS_data = parse_inspvaxa()  # [0, datetime.datetime(2025, 12, 8, 18, 6)]
    
    if orin_time is not None and GPS_data is not None:
        # 提取日期字符串进行比较
        orin_date = orin_time[1]  # '2025-12-08'
        gps_datetime = GPS_data[1]  # datetime对象
        gps_date = gps_datetime.strftime("%Y-%m-%d")  # 转换为日期字符串
        
        if orin_date == gps_date:
            return 0, orin_time
        else:
            return 1, orin_time


# if __name__ == "__main__"
    # parse_inspvaxa()