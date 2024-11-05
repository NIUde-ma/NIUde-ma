#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import canmatrix
from subprocess import check_output
import can
import cantools 

def can():
    # matrix = canmatrix.formats.loadp("/home/ros/Downloads/ms/codes/DC_Service/Can/pluse.dbc")
    dbc_path = "/home/ros/Downloads/ms/codes/DC_Service/Can/pluse.dbc"
    db = cantools.database.load_file(dbc_path)
    messages  = db.messages
    signals = []
    for message in messages:
        for signal in message.signals:
            signals.append(signal)

    for signal in signals:
        print(signal.name)
        
    signal_name = "LHRPulseCounter"

    blf_file = "/home/ros/Downloads/ms/codes/DC_Service/Can/Logging.blf"

    data = can.BLFReader(blf_file)

def read_can():
    import can
    import os
    import sys 
    import csv 
    
    dbc_file = "/home/ros/Downloads/ms/codes/DC_Service/Can/pluse.dbc"
    blf_file = "/home/ros/Downloads/ms/codes/DC_Service/Can/Logging.blf"
    
    db = cantools.database.load_file(dbc_file)

    # Define the name of the signal you want to extract
    signal_name1 = "LHRPulseCounter"
    signal_name2 = "RHRPulseCounter"
    # Extract the frame ID of the signal from the DBC file
    frame_id = None
    for message in db.messages:
        for signal in message.signals:
            if signal.name == signal_name1:
                frame_id = message.frame_id
                break

    if frame_id is None:
        print(f"Signal {signal_name1} {signal_name2} not found in DBC file.")
        exit()

    # Open BLF file
    log = can.BLFReader(blf_file)
    
    # Iterate over messages in BLF file
    # with can.BLFReader(blf_file) as log:
    num = input("please input you Distance traveled：")
    
    if num.isdigit():
        num = int(num)
        print("Distance traveled:", num)
    else:
        print("input error code is exit")
        sys.exit(1)
        
    txt_path = "/home/ros/Downloads/ms/codes/DC_Service/Can/pluse.csv"
    
    # with open(csv_path, "a", newline='', encoding="utf-8") as csvfile:
    #     csv_writer = csv.writer(csvfile)
    first = []
    last = []
    for msg in log:
        if msg.arbitration_id == frame_id:
            # Decode signal data
            decoded_signal = db.decode_message(frame_id, msg.data)
            diff = decoded_signal[signal_name2] - decoded_signal[signal_name1]
            # print(diff)
            decoded_signal_value2 = int(decoded_signal[signal_name2])
            decoded_signal_value1 = int(decoded_signal[signal_name1])
            first_digit = decoded_signal_value2
            first.append(first_digit)
            
            last_digit = decoded_signal_value1
            last.append(last_digit)
            
            fieldnames = ["LR_Pluse", "RR_Pluse"]
            with open(txt_path, "a", newline="", encoding="utf-8") as csvfile:
                csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                if csvfile.tell() == 0:
                    csv_writer.writeheader()
                csv_writer.writerow({"LR_Pluse": str(decoded_signal[signal_name1]), "RR_Pluse": str(decoded_signal[signal_name2])})

            print(f"{signal_name1}: {decoded_signal[signal_name1]} ，Timestamp: {msg.timestamp}\n")
            print(f"{signal_name2}: {decoded_signal[signal_name2]} ，Timestamp: {msg.timestamp}\n")

                
    if first is not None and last is not None:
        first_value = first[-1] - first[0]
        last_value = last[-1] - last[0]
        # print(first[-1] - first[0])
        rr = int(num) / first_value
        print(f"RR_Pluse：{first_value} ，左后脉冲平均值为：{rr} ")
        lr = int(num) /last_value
        print(f"LR_Pluse：{last_value} ，左后脉冲平均值为：{lr} ")
    else:
        print("No messages with the specified frame ID were found.")
        
            # ny = int(decoded_signal[signal_name2][-1] - decoded_signal[signal_name2][0])
            # print(ny)
        
            # fieldnames = ["LR_Pluse", "RR_Pluse"]
            # with open(txt_path, "a", newline="", encoding="utf-8") as csvfile:
            #     csv_writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            #     if csvfile.tell() == 0:
            #         csv_writer.writeheader()
            #     csv_writer.writerow({"LR_Pluse": str(lr), "RR_Pluse": str(rr)})

            # print(f"{signal_name1}: {decoded_signal[signal_name1]} ，Timestamp: {msg.timestamp}，左后脉冲平均值为：{lr}\n")
            # print(f"{signal_name2}: {decoded_signal[signal_name2]} ，Timestamp: {msg.timestamp}，右后脉冲平均值为：{rr}")
            # print(f"{signal_name2}: {decoded_signal[signal_name2]} : {msg.timestamp}")
            
if __name__ == '__main__':
    read_can()
# for msg in data:
#     print(msg)
# output = check_output(["blc", blf_file]).decode("utf-8").splitlines()
# print(output)
# for line in output:
#     parts = line.split()
#     if len(parts) >= 5 and parts[3] == "Rx" and parts[4] == signal_name:
#         signal_value = parts[6]  # 信号值在第6个位置
#         print(f"{signal_name}: {signal_value}")