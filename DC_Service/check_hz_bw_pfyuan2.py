#!/usr/bin/env python
# -*- coding: UTF-8 -*-

#@author:pfyuan2
#@author:shuaima6

from std_msgs.msg import String
from threading import Lock
from rospy.numpy_msg import numpy_msg
import sys
import argparse
import rosbag
import rostopic
import time
import math
import traceback
import signal
from multiprocessing import Process, Queue
NAME='rostopic_checker'

class ROSTopicBandwidth(object):
    def __init__(self, window_size=100):
        import threading
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.sizes =[]
        self.times =[]
        self.window_size = window_size or 100

    def callback(self, data):
        """ros sub callback"""
        import rospy
        with self.lock:
            try:
                t = time.time()
                self.times.append(t)
                # size = len(data._buff)
                # print("size",size)
                self.sizes.append(len(data._buff)) #AnyMsg instance
                assert(len(self.times) == len(self.sizes))

                if len(self.times) > self.window_size:
                    self.times.pop(0)
                    self.sizes.pop(0)
            except:
                traceback.print_exc()

    def print_bw(self,directPrint=False):
        """print the average publishing rate to screen"""

        if len(self.times) < 2:
            return
        with self.lock:
            n = len(self.times)
            tn = time.time()
            t0 = self.times[0]

            total = sum(self.sizes)
            bytes_per_s = total / (tn - t0)
            mean = total / n

            #std_dev = math.sqrt(sum((x - mean)**2 for x in self.sizes) /n)

            # min and max
            max_s = max(self.sizes)
            min_s = min(self.sizes)
        # print("unit: B",bytes_per_s, mean, min_s, max_s)
        #min/max and even mean are likely to be much smaller, but for now I prefer unit consistency
        if bytes_per_s < 1000.0:
            bw, mean, min_s, max_s = ["%.2fB"%v for v in [bytes_per_s, mean, min_s, max_s]]
        elif bytes_per_s < 1000000.0:
            bw, mean, min_s, max_s = ["%.2fKB"%(v/1000.0) for v in [bytes_per_s, mean, min_s, max_s]]
        else:
            bw, mean, min_s, max_s = ["%.2fMB"%(v/1000000.0) for v in [bytes_per_s, mean, min_s, max_s]]

        printStr = "average: %s/s mean: %s min: %s max: %s window: %s"%(bw, mean, min_s, max_s, n)
        if directPrint:
            print(printStr)

        return printStr

class ROSTopicHz(object):

    """
    ROSTopicHz receives messages for a topic and computes frequency stats
    """
    def __init__(self, window_size, filter_expr=None, use_wtime=False):
        import threading
        import rospy
        from collections import defaultdict
        self.lock = threading.Lock()
        self.last_printed_tn = 0
        self.msg_t0 = -1
        self.msg_tn = 0
        self.times = []
        self._last_printed_tn = defaultdict(int)
        self._msg_t0 = defaultdict(lambda: -1)
        self._msg_tn = defaultdict(int)
        self._times = defaultdict(list)
        self.filter_expr = filter_expr
        self.use_wtime = use_wtime

        # can't have infinite window size due to memory restrictions
        if window_size < 0:
            window_size = 50000
        self.window_size = window_size

    def get_last_printed_tn(self, topic=None):
        import rospy
        if topic is None:
            return self.last_printed_tn
        return self._last_printed_tn[topic]

    def set_last_printed_tn(self, value, topic=None):
        if topic is None:
            self.last_printed_tn = value
        self._last_printed_tn[topic] = value

    def get_msg_t0(self, topic=None):
        if topic is None:
            return self.msg_t0
        return self._msg_t0[topic]

    def set_msg_t0(self, value, topic=None):
        if topic is None:
            self.msg_t0 = value
        self._msg_t0[topic] = value

    def get_msg_tn(self, topic=None):
        if topic is None:
            return self.msg_tn
        return self._msg_tn[topic]

    def set_msg_tn(self, value, topic=None):
        if topic is None:
            self.msg_tn = value
        self._msg_tn[topic] = value

    def get_times(self, topic=None):
        if topic is None:
            return self.times
        return self._times[topic]

    def set_times(self, value, topic=None):
        if topic is None:
            self.times = value
        self._times[topic] = value

    def callback_hz(self, m, topic=None):
        import rospy
        """
        ros sub callback
        :param m: Message instance
        :param topic: Topic name
        """
        # #694: ignore messages that don't match filter
        if self.filter_expr is not None and not self.filter_expr(m):
            return
        with self.lock:
            curr_rostime = rospy.get_rostime() if not self.use_wtime else \
                    rospy.Time.from_sec(time.time())

            # time reset
            if curr_rostime.is_zero():
                if len(self.get_times(topic=topic)) > 0:
                    print("time has reset, resetting counters")
                    self.set_times([], topic=topic)
                return

            curr = curr_rostime.to_sec() if not self.use_wtime else \
                    rospy.Time.from_sec(time.time()).to_sec()
            if self.get_msg_t0(topic=topic) < 0 or self.get_msg_t0(topic=topic) > curr:
                self.set_msg_t0(curr, topic=topic)
                self.set_msg_tn(curr, topic=topic)
                self.set_times([], topic=topic)
            else:
                self.get_times(topic=topic).append(curr - self.get_msg_tn(topic=topic))
                self.set_msg_tn(curr, topic=topic)

            #only keep statistics for the last 10000 messages so as not to run out of memory
            if len(self.get_times(topic=topic)) > self.window_size - 1:
                self.get_times(topic=topic).pop(0)

    def get_hz(self, topic=None):
        import rospy
        """
        calculate the average publising rate

        @returns: tuple of stat results
            (rate, min_delta, max_delta, standard deviation, window number)
            None when waiting for the first message or there is no new one
        """
        if not self.get_times(topic=topic):
            return
        elif self.get_msg_tn(topic=topic) == self.get_last_printed_tn(topic=topic):
            return
        with self.lock:

            n = len(self.get_times(topic=topic))
            #rate = (n - 1) / (rospy.get_time() - self.msg_t0)
            mean = sum(self.get_times(topic=topic)) / n
            rate = 1./mean if mean > 0. else 0

            #std dev
            std_dev = math.sqrt(sum((x - mean)**2 for x in self.get_times(topic=topic)) /n)

            # min and max
            max_delta = max(self.get_times(topic=topic))
            min_delta = min(self.get_times(topic=topic))

            self.set_last_printed_tn(self.get_msg_tn(topic=topic), topic=topic)

        return rate, min_delta, max_delta, std_dev, n+1

    def print_hz(self, topics=(None,)):
        import rospy
        """
        print the average publishing rate to screen
        """
        if len(topics) == 1:
            ret = self.get_hz(topics[0])
            if ret is None:
                print("no new messages")
                return
            rate, min_delta, max_delta, std_dev, window = ret
            print("average rate: %.3f\n\tmin: %.3fs max: %.3fs std dev: %.5fs window: %s"%(rate, min_delta, max_delta, std_dev, window))
            return

        # monitoring multiple topics' hz
        header = ['topic', 'rate', 'min_delta', 'max_delta', 'std_dev', 'window']
        stats = {h: [] for h in header}
        for topic in topics:
            hz_stat = self.get_hz(topic)
            if hz_stat is None:
                continue
            rate, min_delta, max_delta, std_dev, window = hz_stat
            stats['window'].append(str(window))
            stats['topic'].append(topic)
            stats['rate'].append('{:.4}'.format(rate))
            stats['min_delta'].append('{:.4}'.format(min_delta))
            stats['max_delta'].append('{:.4}'.format(max_delta))
            stats['std_dev'].append('{:.4}'.format(std_dev))
            stats['window'].append(str(window))
        if not stats['topic']:
            print('no new messages')
            return
        print(_get_ascii_table(header, stats))

def _get_ascii_table(header, cols):
    import rospy
    # compose table with left alignment
    header_aligned = []
    col_widths = []
    for h in header:
        col_width = max(len(h), max(len(el) for el in cols[h]))
        col_widths.append(col_width)
        header_aligned.append(h.center(col_width))
        for i, el in enumerate(cols[h]):
            cols[h][i] = str(cols[h][i]).ljust(col_width)
    # sum of col and each 3 spaces width
    table_width = sum(col_widths) + 3 * (len(header) - 1)
    n_rows = len(cols[header[0]])
    body = '\n'.join('   '.join(cols[h][i] for h in header) for i in xrange(n_rows))
    table = '{header}\n{hline}\n{body}\n'.format(
        header='   '.join(header_aligned), hline='=' * table_width, body=body)
    return table



def getTopics(topics=None):
    import rospy
    allTopics = rospy.get_published_topics()
    topicNames = [topic[0] for topic in allTopics]
    topicNames.sort()
    try:
        topicNames.remove('/rosout')
        topicNames.remove('/rosout_agg')
        topicNames.remove('/clock')
    except:
        pass
    topicNames.sort()
    return topicNames

def bw():
    import rospy
    rospy.init_node(name=NAME+"_bw",anonymous=True,disable_rostime=True)
    topicNames = getTopics()
    callbacks={}

    # topicNames =[ "/sensor/fdc/camera/surround_right_190_1M/compressed" ]
    for topic in topicNames:
        rt = ROSTopicBandwidth(window_size=100)
        sub = rospy.Subscriber(topic, rospy.AnyMsg, rt.callback)
        callbacks[topic] = rt
    while not rospy.is_shutdown():
        # print("/sensor/fdc/camera/front_120_8M/compressed")
        # rt.print_bw()
        for topic in topicNames:
            strs = callbacks[topic].print_bw()
            if strs:
                print("{:<58} {}".format(topic,strs))
        # print("------------------------------------------------------------")
        time.sleep(1)

def hz():
    import rospy
    rospy.init_node(name=NAME+"_hz",anonymous=True,disable_rostime=True)

    topicNames = getTopics()
    rt = ROSTopicHz(window_size=5000)
    for topic in topicNames:
        rospy.Subscriber(topic, rospy.AnyMsg, rt.callback_hz, callback_args=topic,tcp_nodelay=False)
    while not rospy.is_shutdown():

        # callbacks[topic].print_hz(topics=topicNames)
        rt.print_hz(topics=topicNames)
        time.sleep(1)

def check():

    import platform
    import multiprocessing
    import subprocess
    import os
    import psutil
    import socket
    import sys
    import paramiko
    import re
    import csv
    import codecs


    output_path = '/home/ros/Downloads/check_output.txt'
    hostname = socket.gethostname()
    # print(hostname)
    cpu_module = platform.processor()
    cpu_counts = multiprocessing.cpu_count()
    mem = psutil.virtual_memory()
    total_bytes = mem.total
    total_human_readable_gb = (mem.total / 1024 ** 3) + 2



    # print("CPU 型号：", cpu_module + "CPU count：" , cpu_counts)
    output_hostname = subprocess.check_output("hostname" , shell=True)
    string_without_spaces = "".join(output_hostname.split())
    # print(string_without_spaces)
    output_nvidia = subprocess.check_output("nvidia-smi -L" , shell=True)
    output_nvidia2 = subprocess.check_output("nvidia-smi" , shell=True)
    output_cpu = subprocess.check_output("cat /proc/cpuinfo | grep 'model name' | uniq" , shell=True)
    output_cpu_num = subprocess.check_output("cat /proc/cpuinfo| grep 'physical id' | sort| uniq| wc -l" , shell=True)
    output_systeam = subprocess.check_output("cat /etc/issue" , shell=True)

    if os.path.exists(output_path):
        print("将执行的结果存放在：" , output_path)
    else:
        print("手动创建结果" , output_path)

    with open(os.path.join(output_path), 'w') as f:
        f.write("Hostname：" + output_hostname + "\n")
        f.write("Gpu：" + output_nvidia + "\n")
        f.write("CPU：" + output_cpu + "\n")
        f.write("CPU 数量：" + output_cpu_num + "\n")
        f.write("系统信息：" + output_systeam + "\n")
        f.write("Gpu详细信息："+ output_nvidia2 )

    if hostname == string_without_spaces:
        print("hostname is True")
    else:
        print("hostname is false please input：修改主机名的命令 sudo hostnamectl set-hostname xxxx(正确主机名）")
        print("检测到Hostname不对，代码自动退出，请按照上方命令修改正确再使用！" , hostname)
        sys.exit(0)

    if "NVIDIA GeForce RTX 3090" in output_nvidia:
        print("NVIDIA is True")
    else:
        print("Errors")
    if "Intel(R) Xeon(R) E-2278G CPU @ 3.40GHz" in output_cpu and int(output_cpu_num) == 1:
        print("CPU is True")
    else:
        print("Cpu is Errors")
    if total_human_readable_gb == 64:
        print("mem size is True")
    else:
        print("mem size is Errors")
    if "Ubuntu 18.04.6 LTS" in output_systeam:
        print("systeam version is True")
    else:
        print("Errors")

    """
    service ping

    """
    DC_host = '192.168.1.10'
    DC_side_host = '192.168.1.11'
    fleet_host = '192.168.1.166'

    DC_result = subprocess.call(['ping', '-c', '3', DC_host])
    DC_side_result = subprocess.call(['ping', '-c', '3', DC_side_host])
    fleet_result = subprocess.call(['ping', '-c', '3', fleet_host])

    if DC_result == 0:
        print("\nDC_service ping is True")
    else:
        print("DC_service ping is Down")
    if DC_side_result == 0:
        print("DC_side_service ping is True")
    else:
        print("DC_side_service ping is Down")
    if fleet_result == 0:
        print("Fleet ping is True\n")
    else:
        print("Fleet ping is Error")

    """
    paramiko

    """
    output_path2 = '/home/ros/Downloads/fleet_output.txt'
    port = 22
    username = 'ifly'
    password = 'auto'
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:

        ### get network code
        ssh.connect(fleet_host, port, username, password)

        stdin, stdout, stderr = ssh.exec_command('ifconfig')

        output = stdout.read().decode('utf-8')
        # print(output)
        niude = output.decode('utf-8')
        f = codecs.open(os.path.join(output_path2),'w', encoding='utf-8')
        f.write(niude + "\n")
        # with open(os.path.join(output_path2), 'wb') as f:
        #     f.write(("Hostname：" + output + "\n").encode('utf-8'))

        ### get nvme code
        stdin, stdout, stderr = ssh.exec_command('df -h')
        output2 = stdout.read().decode('utf-8')
        niude2 = output2.decode('utf-8')
        f.write(niude2 + "\n")
        ### get mem size code
        stdin, stdout, stderr = ssh.exec_command('sudo dmidecode -t memory | grep -E "Man|Size|Speed"')
        stdin.write('auto\n')
        stdin.flush()
        output3 = stdout.read().decode('utf-8')
        niude3 = output3.decode('utf-8')
        f.write(niude3 + "\n")
        sizes = re.findall(r'Size: (\d+) MB', output3)
        sizes_in_gb = [int(size) / 1024 for size in sizes]
        total_size = sum(sizes_in_gb)

        ### get media code
        stdin, stdout, stderr = ssh.exec_command('lsblk')
        output4 = stdout.read().decode('utf-8')
        #niude4 = output4.decode('utf-8')
        f.write(output4 + "\n")
        ### get fleet ptp code
        stdin, stdout, stderr = ssh.exec_command('ps -ef |grep ptp')
        output5 = stdout.read().decode('utf-8')
        niude5 = output5.decode('utf-8')
        f.write(niude5 + "\n")
        ### get network % code
        stdin, stdout, stderr = ssh.exec_command('ping -c 10 ' + fleet_host)  # 发送 10 个 ICMP 回显请求

        output6 = stdout.read().decode('utf-8')
        f.write(output6 + "\n")
        packet_loss = re.search(r'(\d+)% packet loss', output6)

        if packet_loss:
            packet_loss_rate = int(packet_loss.group(1))
            if packet_loss_rate < 1:
                print("网络可达，并且丢包率小于 1%")
            else:
                print("网络可达，但丢包率大于等于 1%")
        else:
            print("无法确定网络可达性")

        ### get network speed% code
        stdin, stdout, stderr = ssh.exec_command('sudo iperf3 -c 192.168.1.166 -i 1 -M 1000 -t 10')
        stdin.write('auto\n')
        stdin.flush()
        output7 = stdout.read().decode('utf-8')

        speed = None
        for line in output7.split('\n'):
            if 'sender' in line:
                parts = line.split(' ')
                speed = float(parts[-2])  # 带宽信息通常在输出的倒数第二个位置
                print(speed)
                break

        if speed is not None:
            if speed >= 9 * 1024:
                print("网络通信速率达到 9GB 以上")
            else:
                print("网络通信速率未达到 9GB")
        else:
            print("无法确定网络通信速率")

        if "eno1" in output and "eno2" in output and "eno3" in output and "eno4" in output:
            print("fleet network is True")
        else:
            print("Errors get fleet network false" , output)

        if "/dev/nvme0n1p1" in output2 and "/dev/nvme0n1p2" in output2 and "/dev/nvme0n1p3" in output2:
            print("Disk is True")
        else:
            print("Disk is Flase" , output2)

        if total_size == 16:
            print("Fleet mem is True")
        else:
            print("Fleet mem is False" , output3 )

        if "/media/data_collect" in output4:
            print("data_collect is mount")
        else:
            print("data_collect is no mount" , output4)

        if "usr/sbin/ptpd -s -E -i br0 -c /etc/ifly-recorder-base/ptp.cfg -f /var/log/ifly_recorder/ptp.log" in output5:
            print("fleet ptp is Running")
        else:
            print("fleet ptp is not Running" , output5)


    except paramiko.AuthenticationException:
        print("Authentication failed, please check your credentials")
    except paramiko.SSHException as sshException:
        print("Unable to establish SSH connection: %s" % sshException)
    except paramiko.BadHostKeyException as badHostKeyException:
        print("Unable to verify server's host key: %s" % badHostKeyException)
    finally:

        ssh.close()
    # gpu_names = output.strip().split('\n')
    # print(gpu_names)
    # for idx, name in enumerate(gpu_names):
    #     gpu_info.append({'id': idx, 'name': name})
    #     print(gpu_info)

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--bw', action='store_true', help='show bandwidth')
    parser.add_argument('--hz', action='store_true', help='show frequency')
    parser.add_argument('--check', action='store_true', help='show frequency')
    args = parser.parse_args()

    if args.bw:
        bw()
    elif args.hz:
        hz()
    elif args.check:
        check()
    else:
        print("请输入你需要执行的功能！如--hz或者--bw或者--check")
        # signalProcess = Process(target=signal.signal, args=(signal.SIGINT, signal.SIG_IGN))
        # bwProcess = Process(target=bw)
        # hzProcess = Process(target=hz)
        # signalProcess.start()
        # bwProcess.start()
        # hzProcess.start()
        # signalProcess.join()
        # bwProcess.join()
        # hzProcess.join()
        sys.exit(1)
