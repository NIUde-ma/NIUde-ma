#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import subprocess
# 调用git命令获取指定目录下的最新提交号
def get_latest_commit(directory):
    result = subprocess.run(['git', 'log', '-n', '1', '--pretty=format:%H', '--', directory], stdout=subprocess.PIPE)
    output = result.stdout.decode().strip()
    return output

# 指定需要获取提交号的目录
src_directory = ['src/around_view_camera_perception',
                 'src/control',
                 'src/ehr',
                 'src/finite_state_manager',
                 'src/front_view_camera_perception',
                 'src/hmi_on_soc',
                 'src/koala',
                 'src/me_track',
                 'src/obstacle_fusion',
                 'src/planning',
                 'src/prediction',
                 'src/road_structure_fusion',
                 'src/uss_perception']

# 获取src目录下所有文件夹的最新提交号
subfolders = subprocess.run(['find', src_directory, '-type', 'd'], stdout=subprocess.PIPE)
subfolders_output = subfolders.stdout.decode().strip()
# 解析输出，获取文件夹列表
folders = subfolders_output.split('\n')
# 获取每个文件夹的最新提交号
for folder in folders:
    print(folder)
    commit = get_latest_commit(folder)
    print(f"{folder}: {commit}")