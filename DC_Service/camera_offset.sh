#!/bin/bash
my_time=$1

usage(){
    echo -e "请输入你要获取offset的时间，单位是秒！！！\n"
}

if [ $# -ne 1 ]; then
    usage
    exit
fi

current_time=$(date +"%Y-%m-%d %H:%M:%S.%N")
echo $current_time

front_120_msg="/sensor/fdc/camera/front_120_8M/"
left_front_msg="/sensor/fdc/camera/front_left_100_2M/"
right_front_msg="/sensor/fdc/camera/front_left_100_2M/"
rear_msg="/sensor/fdc/camera/rear_100_2M/"
rear_left_msg="/sensor/fdc/camera/rear_left_100_2M/"
rear_right_msg="/sensor/fdc/camera/rear_right_100_2M/"

offset_path="/home/ros/Downloads/ms/offset"
mkdir -p /home/ros/Downloads/ms/clock
mkdir -p /home/ros/Downloads/ms/offset
# offset_path="/home/ros/Downloads/ms/offset"

log_path="/home/ros/Downloads/ms/clock"

front_log="/home/ros/Downloads/ms/clock/front.log"
left_front_log="/home/ros/Downloads/ms/clock/left_front.log"
right_front_log="/home/ros/Downloads/ms/clock/right_front.log"
rear_log="/home/ros/Downloads/ms/clock/rear.log"
rear_left_log="/home/ros/Downloads/ms/clock/rear_left.log"
rear_right_log="/home/ros/Downloads/ms/clock/rear_right.log"

source /home/ros/dev_ws/devel/setup.bash

tmux new-window -n camera_offset_window

tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -v
tmux split-window -h
tmux split-window -h

tmux select-layout even-horizontal

echo -e "clock offset srcipt"


tmux send-keys -t 0 "timeout $my_time rostopic echo $front_120_msg |grep exposure_starting_utc_time: > $front_log " Enter
sleep 2
tmux send-keys -t 1 "timeout $my_time rostopic echo $left_front_msg |grep exposure_starting_utc_time: > $left_front_log " Enter
sleep 2
tmux send-keys -t 2 "timeout $my_time rostopic echo $right_front_msg |grep exposure_starting_utc_time: > $right_front_log " Enter
sleep 2
tmux send-keys -t 3 "timeout $my_time rostopic echo $rear_msg |grep exposure_starting_utc_time: > $rear_log " Enter
sleep 2
tmux send-keys -t 4 "timeout $my_time rostopic echo $rear_left_msg |grep exposure_starting_utc_time: > $rear_left_log " Enter
sleep 2
tmux send-keys -t 5 "timeout $my_time rostopic echo $rear_right_msg |grep exposure_starting_utc_time: > $rear_right_log " Enter
sleep 2
#wait camera_echo for logs
echo -e "---camera_offset is Geting please wait $my_time---"

sleep $my_time

echo -e "camera_offset is Get Done！"

front_log_1=$(grep -oaP '(?<=exposure_starting_utc_time: )\d{3}' $front_log | cut -c 2-3)
left_front_log_1=$(grep -oaP '(?<=exposure_starting_utc_time: )\d{3}' $left_front_log | cut -c 2-3)
right_front_log_1="$(grep -oaP '(?<=exposure_starting_utc_time: )\d{3}' $right_front_log | cut -c 2-3)"
rear_log_1="$(grep -oaP '(?<=exposure_starting_utc_time: )\d{3}' $rear_log | cut -c 2-3)"
rear_left_log_1="$(grep -oaP '(?<=exposure_starting_utc_time: )\d{3}' $rear_left_log | cut -c 2-3)"
rear_right_log_1="$(grep -oaP '(?<=exposure_starting_utc_time: )\d{3}' $rear_right_log | cut -c 2-3)"

echo $front_log_1 > $offset_path/front_offset.log
echo $left_front_log_1 > $offset_path/left_front_offset.log
echo $right_front_log_1 > $offset_path/right_front_offset.log
echo $rear_log_1 > $offset_path/rear_offset.log
echo $rear_left_log_1 > $offset_path/rear_left_offset.log
echo $rear_right_log_1 > $offset_path/rear_right_offset.log


if [ -f "$offset_path/front_offset.log" ];then
    random_numbers=($(cat "$offset_path/front_offset.log"))
    number_counts=$(printf "出现次数<---Front—120 offset--->值：  %s\n" "${random_numbers[@]}" | sort | uniq -c)
    echo "$number_counts\n"
else
    echo "---Front_120_8M的offset获取失败---"
fi

if [ -f "$offset_path/left_front_offset.log" ];then
    random_numbers=($(cat "$offset_path/left_front_offset.log"))
    number_counts=$(printf "出现次数<---left_front offset--->值：  %s\n" "${random_numbers[@]}" | sort | uniq -c)
    echo "$number_counts\n"
else
    echo "---left_front的offset获取失败---"
fi

if [ -f "$offset_path/right_front_offset.log" ];then
    random_numbers=($(cat "$offset_path/right_front_offset.log"))
    number_counts=$(printf "出现次数<---right_front offset--->值：  %s\n" "${random_numbers[@]}" | sort | uniq -c)
    echo "$number_counts\n"
else
    echo "---right_front的offset获取失败---"
fi

if [ -f "$offset_path/rear_offset.log" ];then
    random_numbers=($(cat "$offset_path/rear_offset.log"))
    number_counts=$(printf "出现次数<---rear offset--->值：  %s\n" "${random_numbers[@]}" | sort | uniq -c)
    echo "$number_counts\n"
else
    echo "---rear的offset获取失败---"
fi

if [ -f "$offset_path/rear_left_offset.log" ];then
    random_numbers=($(cat "$offset_path/rear_left_offset.log"))
    number_counts=$(printf "出现次数<---rear_left offset--->值：  %s\n" "${random_numbers[@]}" | sort | uniq -c)
    echo "$number_counts\n"
else
    echo "---rear_left的offset获取失败---"
fi

if [ -f "$offset_path/rear_right_offset.log" ];then
    random_numbers=($(cat "$offset_path/rear_right_offset.log"))
    number_counts=$(printf "出现次数<---rear_right offset--->值：  %s\n" "${random_numbers[@]}" | sort | uniq -c)
    echo "$number_counts\n"
else
    echo "---rear_right的offset获取失败---"
fi

echo  当前处理完成的时间点为：$current_time > $log_path/Debug.log