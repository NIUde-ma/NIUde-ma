#!/bin/bash
sudo cp /etc/apt/sources.list /etc/apt/sources.list_local
sync
sudo sed -i 's/cn.archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list
sudo sed -i 's/cn.archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list
sudo sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list
sudo sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list

sudo apt install -y apt-utils apt-transport-https ca-certificates \
htop ncdu git build-essential cmake \
zip unzip lrzsz htop lnav net-tools \
 git vim curl wget nano cmake python3-pip bash-completion language-pack-zh-hans \
iftop iotop bmon dstat htop vim jq tmux screen fish tcpdump nmap iotop nload socat conntrack ebtables ipset ipvsadm  network-manager

sudo systemctl disable --now apt-daily.timer
sudo systemctl disable --now apt-daily.service

sudo systemctl disable --now apt-daily-upgrade.timer
sudo systemctl disable --now apt-daily-upgrade.service

