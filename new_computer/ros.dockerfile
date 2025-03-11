FROM ubuntu:focal

LABEL maintainer="shuaima6@iflytek.com"
ENV DEBIAN_FRONTEND noninteractive
RUN sed -i 's|ports.ubuntu.com|mirrors.ustc.edu.cn|g' /etc/apt/sources.list

# 安装 Python 3.9
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    python3.9 \
    python3.9-distutils \
    && rm -rf /var/lib/apt/lists/*

# 将 Python 3.9 设置为默认版本
RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.9 1 && \
    update-alternatives --set python3 /usr/bin/python3.9

# 确保 pip 指向 Python 3.9
RUN curl https://mirrors.aliyun.com/pypi/get-pip.py -o get-pip.py && \
    python3.9 get-pip.py && \
    rm get-pip.py

# 继续安装其他依赖
RUN apt-get clean
RUN apt-get update && apt-get install -y \
    tmux \
    net-tools \
    htop \
    iputils-clockdiff \
    openssh-server  passwd \
    vim \
    autossh \
    mc \
    ncdu \
    mc \
    git \
    wget \
    nmap \
    tree \
    tcpdump \
    libgl1-mesa-glx \
    g++ \
    gcc \
    libc6-dev \
    make \
    pkg-config \
    gcc-aarch64-linux-gnu \
    g++-aarch64-linux-gnu \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO=noetic
ENV LANG=C.UTF-8

RUN apt-get clean && apt-get update && apt-get install -y \
    dirmngr \
    gnupg2

# 添加 ROS 软件源
RUN apt-get update && apt-get install -y lsb-release
RUN gpg --keyserver 'hkp://keyserver.ubuntu.com:80' --keyserver-options auto-key-retrieve --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN gpg --export C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 > /usr/share/keyrings/ros.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/ros.gpg] https://mirrors.ustc.edu.cn/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list

RUN apt-get update && apt-get install -y \
    ros-noetic-desktop \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    gnupg \
    lsb-release \
    python3-rosinstall \
    python3-rosdep \
    python3-rosbag \
    python3-rospy \
    python3-rospkg \
    python3-pyparsing \
    python3-yaml \
    python3-future \
    libgl1-mesa-glx \
    #folium \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    golang \
    && rm -rf /var/lib/apt/lists/*

ENV PATH=$PATH:/usr/local/go/bin
RUN go version

ENV PYTHONUNBUFFERED=1
RUN pip config set global.index-url https://pypi.tuna.tsinghua.edu.cn/simple

# 安装常用的Python库
RUN pip3 install --no-cache-dir --trusted-host files.pythonhosted.org --upgrade pip \
    setuptools \
    wheel \
    ipython \
    numpy \
    scipy \
    folium \
    matplotlib \
    pandas \
    scikit-learn \
    scikit-build \
    opencv-python \
    paramiko \
    flasgger \
    requests \
    fake-useragent==2.0.3 \
    lxml \
    bs4 \
    lxml \
    moviepy \
    keras \
    --ignore-installed blinker

RUN apt-get update && apt-get install -y \
    libhdf5-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple h5py

# 安装TensorFlow1.15.0
RUN pip config set global.index-url https://pypi.org/simple
RUN pip3 install --no-cache-dir -i https://pypi.tuna.tsinghua.edu.cn/simple tensorflow


# RUN useradd -m -d /home/myuser -s /bin/bash myuser && \
#     echo 'myuser:auto' | chpasswd

RUN sed -i 's/^#\?PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config && \
    sed -i -E 's/^#?AllowTcpForwarding.*/AllowTcpForwarding yes/' /etc/ssh/sshd_config && \
    sed -i -E 's/^#?GatewayPorts.*/GatewayPorts yes/' /etc/ssh/sshd_config

RUN echo 'root:NIUde' | chpasswd && \
    rm -f /etc/ssh/ssh_host_rsa_key /etc/ssh/ssh_host_rsa_key.pub && \
    ssh-keygen -q -t rsa -f /etc/ssh/ssh_host_rsa_key -N "" && \
    service ssh start


ENV ROS_DISTRO=noetic
ENV LD_LIBRARY_PATH=/opt/ros/noetic/lib:$LD_LIBRARY_PATH
ENV PATH=/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

CMD ["tail", "-f", "/dev/null"]
