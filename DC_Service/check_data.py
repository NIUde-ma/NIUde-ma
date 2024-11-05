#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import time
import multiprocessing
import psutil
def cpu_intensive_task():
    while True:
        # 进行一些CPU密集型计算任务
        pass
def mem_intensive_task():
    data = b' ' * 1024 * 1024 * 10 # 分配10MB的内存
    while True:
        # 持续使用内存
        pass
if __name__ == "__main__":
    num_cores = multiprocessing.cpu_count()
    processes = []
    # 创建CPU密集型任务进程
    for _ in range(num_cores):
        process = multiprocessing.Process(target=cpu_intensive_task)
        process.start()
        processes.append(process)
    # 获取系统内存大小
    total_memory = psutil.virtual_memory().total
    print(total_memory)
    # 计算需要使用的内存大小
    target_memory = total_memory * 1.1
    print(target_memory)
    # 创建内存密集型任务进程
    process = multiprocessing.Process(target=mem_intensive_task)
    process.start()
    processes.append(process)
    try:
        # 持续运行，保持CPU和内存占用率在40%
        while True:
            # 获取当前CPU占用率和内存使用量
            cpu_percent = psutil.cpu_percent()
            mem_used = psutil.virtual_memory().used
            # 控制CPU占用率
            if cpu_percent > 40:
                time.sleep(0.01)
            else:
                time.sleep(0)
            # 控制内存占用率
            if mem_used > target_memory:
                time.sleep(0.01)
            else:
                time.sleep(0)
    except KeyboardInterrupt:
        # 当按下Ctrl+C时，停止所有进程
        for process in processes:
            process.terminate()