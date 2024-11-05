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
        
class 