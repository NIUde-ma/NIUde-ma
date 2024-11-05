#!/usr/bin/python3
import os
import random
import socket
import sys
import time
import logging
import logging.handlers
import shutil
import json
import subprocess
from pathlib import Path
import re
from os import path
IMAGE = "artifacts.iflytek.com/auto-docker-product-public/autofpilotdevtools/ros_melodic"
IMAGE_TAG = "v0.1.1"
RELEASE_TAG = "v0.1.0"


def getCurrentUser():
    return os.environ.get("USER")


class Cfg:
    _instance = None

    def __new__(cls, *args, **kw):
        if cls._instance is None:
            cls._instance = object.__new__(cls, *args, **kw)
        return cls._instance

    def __init__(self):
        self.dockerPort = 8899
        self.savePath = f"/home/{getCurrentUser()}/.ros_dev"
        if not os.path.exists(self.savePath):
            os.makedirs(self.savePath)
        self.cfgPath = path.join(self.savePath, "cfg.json")
        if not os.path.exists(self.cfgPath):
            self.save()
        else:
            self.load()

    def dumpToJSON(self):
        return json.dumps(self.__dict__, sort_keys=True, indent=4)

    def save(self):
        with open(self.cfgPath, "w") as f:
            f.write(self.dumpToJSON())
            f.close()

    def load(self):
        if os.path.exists(self.cfgPath):
            with open(self.cfgPath, "r") as f:
                self.__dict__ = json.loads(f.read())

    def getDockerPort(self):
        return self.dockerPort

    def setDockerPort(self, port):
        self.dockerPort = port


cfg = Cfg()


class Filter(logging.Filter):
    FMTDCIT = {
        'ERROR': "\033[31m ERROR\033[0m",
        'INFO': "\033[32m INFO\033[0m",
        'DEBUG': "\033[37m DEBUG\033[0m",
        'WARN': "\033[33m WARN\033[0m",
        'WARNING': "\033[33m WARNING\033[0m",
        'CRITICAL': "\033[35m CRITICAL\033[0m",
    }

    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def filter(self, record: logging.LogRecord) -> bool:
        record.levelname = self.FMTDCIT.get(record.levelname)
        return True


class Logger:

    def __init__(self, name, level=logging.DEBUG, fmt="%(asctime)s %(name)s %(levelname)s %(lineno)s %(message)s", fmt_date="%H:%M:%S"):
        self.name = name
        self.level = level
        self.fmt = fmt
        self.fmt_date = fmt_date
        self.filter = Filter()

    def getLogger(self):
        fmter = logging.Formatter(self.fmt, self.fmt_date)
        ch = logging.StreamHandler()
        ch.setLevel(self.level)
        ch.setFormatter(fmter)
        ch.addFilter(self.filter)

        logger = logging.getLogger(self.name)
        logger.setLevel(self.level)
        logger.addHandler(ch)
        return logger


log = Logger("ros_dev").getLogger()


def printEnv():
    for key in os.environ:
        log.error(key)
        log.warning(os.environ[key])


def get_random_port():
    port = cfg.getDockerPort()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.bind(('0.0.0.0', port))
        s.close()
        return port
    except socket.error as e:
        s.close()

    while True:
        port = random.randint(8899, 8999)
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            s.bind(('0.0.0.0', port))
            s.close()
            cfg.setDockerPort(port)
            cfg.save()
            return port
        except socket.error as e:
            s.close()


class DockerCfg:
    def __init__(self, docker_image, docker_port, docker_name, docker_shell, debugMode=False):
        self.docker_image = docker_image
        self.docker_port = docker_port
        self.docker_name = docker_name
        self.docker_shell = docker_shell
        self.hostname = socket.gethostname()
        self.debugMode = debugMode
        self.script_path = os.path.dirname(os.path.abspath(__file__))
        self.haveGPU = self.checkIfHaveGPU()

        self.check()
        self.Display = self.getDisplay()
        # print("docker image:", self.docker_image)
        # print("docker port:", self.docker_port)
        # print("docker name:", self.docker_name)
        # print("docker shell:", self.docker_shell)
        # print("hostname:", self.hostname)
        # print("debugMode:", self.debugMode)

    def check(self):
        if not self.haveGPU:
            return
        dockerDaemonFile = "/etc/docker/daemon.json"
        if not os.path.exists(dockerDaemonFile):
            log.error(f"{dockerDaemonFile} not exist,please read\n")
            print("http://wiki.iflytek.com/pages/viewpage.action?pageId=476383812")
            sys.exit(1)

        with open(dockerDaemonFile, "r") as f:
            dockerDaemonCfg = f.read()
            if not "runtimes" in dockerDaemonCfg:
                log.error(
                    f"please add nvidia runtime to {dockerDaemonFile},please read\n")
                print("http://wiki.iflytek.com/pages/viewpage.action?pageId=476383812")
                sys.exit(1)
            if not "nvidia" in dockerDaemonCfg:
                log.error(
                    f"please add nvidia runtime to {dockerDaemonFile},please read\n")
                print("http://wiki.iflytek.com/pages/viewpage.action?pageId=476383812")
                sys.exit(1)

    def getDisplay(self):
        uid = os.getuid()
        # /tmp/.X11-unix
        xBackends = os.listdir("/tmp/.X11-unix")
        Display = ""
        if len(xBackends)==0:
            pass
        elif len(xBackends)==1:
            Display=xBackends[0]
        else:
            for xBackend in xBackends:
                print(xBackend)
                if xBackend.startswith("X"):
                    stat = os.stat(f"/tmp/.X11-unix/{xBackend}")
                    print(stat)
                    print(stat.st_uid,uid,stat.st_uid == uid)
                    if stat.st_uid == uid:
                        Display = xBackend
                        break
        if Display == "":
            log.error("你似乎没有启动X11服务，或者没有权限访问X11服务，请检查")
            log.error("你应该用RDP或者VNC连接到这台机器，然后在这台机器上启动X11服务")
            if not self.debugMode:
                sys.exit(1)
        Display = Display.replace("X", "")
        Display = f":{Display}"
        if Display== ":":
            log.warning("using default display :0")
            Display = ":0"
        log.info(f"Display={Display}")
        self.Display = Display
        return Display

    def checkIfHaveGPU(self):
        if os.path.exists("/dev/nvidia0"):
            return True
        else:
            return False

    def startCmdBuilder(self, daemon=False):
        USER = getCurrentUser()

        cmd = f"docker run "
        cmd += "-dt "
        if daemon:
            cmd += "-e ROS_MASTER_IP=192.168.2.10 -e ROS_IP=192.168.2.10 "

        cmd += "--privileged=true "
        cmd += f"--ulimit memlock=-1 --ulimit stack=67108864 "
        if self.haveGPU:
            cmd += "--gpus all --runtime=nvidia "
            cmd += f"-e NVIDIA_VISIBLE_DEVICES=all -e NVIDIA_DRIVER_CAPABILITIES=all "
            cmd += f"-v /usr/local/cuda:/usr/local/cuda "
        if not self.debugMode:
            cmd += f"--net=host "
        cmd += f"--ipc host "
        cmd += f"--hostname {self.hostname} "
        cmd += f"-e WHICH_CAR={self.hostname.replace('-','_')} "
        cmd += f"--name {self.docker_name} "
        cmd += f"-p {self.docker_port}:22 "
        cmd += f"-v /etc/localtime:/etc/localtime:ro "
        cmd += f"-v /etc/timezone:/etc/timezone:ro "
        cmd += f"-v /lib/modules:/lib/modules "
        cmd += f"-v /etc/default:/etc/default "
        cmd += f"-v /dev:/dev "
        cmd += f"-v /dev/shm:/dev/shm "
        cmd += f"-v /tmp/.X11-unix:/tmp/.X11-unix "
        cmd += f"-e DISPLAY={self.Display} "
        cmd += f"-e USER={USER} "
        cmd += f"-e GIT_DISCOVERY_ACROSS_FILESYSTEM=1 "
        cmd += f"-v /tmp/log:/tmp/log "
        cmd += f"--cap-add SYS_TIME "
        cmd += f"-v /home/{USER}/Downloads:/home/ros/Downloads "
        cmd += f"-v /home/{USER}/work/dev_ws:/home/ros/dev_ws "
        cmd += f"-v /data_cold/:/data_cold "
        cmd += f"-v /data_cold2/:/data_cold2 "
        if os.path.exists(f"/media/{USER}"):
            cmd += f"--mount type=bind,source=/media/{USER},target=/media,bind-propagation=rshared "
        if os.path.exists(f"/mnt/{USER}"):
            cmd += f"--mount type=bind,source=/mnt/{USER},target=/mnt/{USER},bind-propagation=rshared "
        # 如果用自己的电脑可以这样可以把宿主机的移动硬盘挂载到容器中
        cmd += f"-v {self.script_path}/scripts/boot:/home/ros/boot "
        cmd += f"-v {self.script_path}/vehicle_config:/calib:ro "
        cmd += f"{self.docker_image} "
        cmd += f"{self.docker_shell}"
        print(cmd)
        return cmd

    def create(self, force=False,daemon=False):
        cmd = self.startCmdBuilder(daemon=daemon)
        if force:
            self.remove()
        os.system(cmd)

    def attach(self):
        cmd = f"docker exec -it {self.docker_name} {self.docker_shell}"
        os.system(cmd)

    def start(self):
        created = os.popen(f"docker ps -a | grep {self.docker_name}").read()
        if not created:
            log.info(f"container {self.docker_name} not exist,create it")
            self.create()
        else:
            log.info(f"container {self.docker_name} exist,start it")
            ret = os.system(f"docker start {self.docker_name}")
            if ret != 0:
                log.error(
                    f"container {self.docker_name} start failed,remove it and create it")
                self.remove()
                self.create()

        # time.sleep(1)
        log.info(f"attaching to container {self.docker_name}")
        self.attach()

    def runRelease(self):
        self.create(force=True,daemon=True)
        print()
        log.info("ros_dev 容器启动成功!")

    def stop(self):
        log.info(f"stop container {self.docker_name}")
        cmd = f"docker stop {self.docker_name}"
        os.system(cmd)

    def restart(self):
        log.info(f"restart container {self.docker_name}")
        self.remove()
        # time.sleep(1)
        self.start()

    def remove(self):
        log.info(f"remove container {self.docker_name}")
        cmd = f"docker rm -f {self.docker_name}"
        os.system(cmd)

    def rebuild(self):
        log.info(f"recreate container {self.docker_name}")
        self.remove()
        self.start()

    def info(self):
        container_running = os.popen(
            f"docker ps -a | grep {self.docker_name}").read()
        if container_running:
            log.info(f"container {self.docker_name} is running")
        else:
            log.error(f"container {self.docker_name} is not running")


RUN = "run"
RUN_DEBUG = "rundebug"
RUN_RELEASE = "runrelease"
STOP = "stop"
RESTART = "restart"
REMOVE = "remove"
REBUILD = "rebuild"
INFO = "info"
PRINT = "print"


def printUsage():
    print(f"Usage: {args[0]} [run|stop|restart|remove|rebuild|info|print]\n")
    print(
        f"{args[0]} run     - run or exec container(container will be created automatically)")
    print(
        f"{args[0]} run_debug     - run or exec container in debug mode (network mode will not be host)")
    print(f"{args[0]} run_release     - run or exec container in release mode (container will be removed and created automatically)")
    print(f"{args[0]} stop    - stop container")
    print(f"{args[0]} restart - restart container")
    print(f"{args[0]} remove  - remove container")
    print(f"{args[0]} rebuild - remove old container and create new container")
    print(f"{args[0]} info    - show container info")
    print(f"{args[0]} print   - print docker run command")
    print(f"{args[0]} name    - print docker name and exit")


def removeStrSpecialChar(str):
    return re.sub(r"[^a-zA-Z0-9]", "", str)

def setupCoredump():
    coreDumpDir=os.path.join("/home",os.environ["HOME"],"Downloads/coredump")
    # create core dump dir if not exist
    if not os.path.exists(coreDumpDir):
        os.makedirs(coreDumpDir)
    # clean old core files older than 7 days
    now = time.time()
    for filename in os.listdir(coreDumpDir):
        if "core" in filename:
            filestamp = os.stat(os.path.join(coreDumpDir, filename)).st_mtime
            filecompare = now - 7 * 86400
            if  filestamp < filecompare:
                os.remove(os.path.join(coreDumpDir, filename))


if __name__ == "__main__":

    args = sys.argv
    if len(args) == 1:
        printUsage()
        exit(0)
    action = args[1]
    action = removeStrSpecialChar(action)
    currentUser = getCurrentUser()
    docker_name = f"ros_dev_{currentUser}"
    if action=="getName":
        print(docker_name)
        exit(0)
    setupCoredump()
    # Open X11 passthrough

    log.info(f"currentUser {currentUser}")
    docker_image = f"{IMAGE}:{IMAGE_TAG}"
    if action == RUN_RELEASE or action == RUN:
        docker_image = f"{IMAGE}:{RELEASE_TAG}"
    log.info(f"docker_image {docker_image}")
    docker_port = get_random_port()
    log.warning(f"mapping docker port 22 to {docker_port}")

    log.warning(f"docker container name {docker_name}")
    docker_shell = "zsh"
    log.info(f"docker shell {docker_shell}")


    docker = DockerCfg(docker_image, docker_port, docker_name, docker_shell,action==RUN_DEBUG)
    os.system(f"export DISPLAY={docker.Display};xhost +")
    if action == RUN:
        docker.start()
    elif action == RUN_DEBUG:
        docker.start()
    elif action == RUN_RELEASE:
        docker.runRelease()
    elif action == STOP:
        docker.stop()
    elif action == RESTART:
        docker.restart()
    elif action == REMOVE:
        docker.remove()
    elif action == REBUILD:
        docker.rebuild()
    elif action == INFO:
        docker.info()
    elif action == PRINT:
        docker.startCmdBuilder()
    else:
        printUsage()
        exit(0)