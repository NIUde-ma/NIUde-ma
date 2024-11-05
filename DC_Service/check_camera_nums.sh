#!/bin/bash

usage(){
    echo "please input dir "

}

if [ $# -ne 1 ]; then
    usage
    exit
fi




    
if [ -d "$1/ofilm_surround_front_120_8M" ] && [ -d "$1/ofilm_surround_front_left_100_2M" ] && [ -d "$1/ofilm_surround_front_right_100_2M" ] && [ -d "$1/ofilm_surround_rear_100_2M" ] && [ -d "$1/ofilm_surround_rear_left_100_2M" ] && [ -d "$1/ofilm_surround_rear_right_100_2M" ];then
    echo -e "正在处理$filename，下的数据"
    find $1/ofilm_surround_front_120_8M -name "*.jpeg" |wc -l |xargs echo -e "front_120_8M_jpegs:"
    find $1/ofilm_surround_front_left_100_2M -name "*.jpeg" |wc -l |xargs echo -e "front_left_100_2M_jpegs:"
    find $1/ofilm_surround_front_right_100_2M -name "*.jpeg" |wc -l |xargs echo -e "front_right_100_2M_jpegs:"
    find $1/ofilm_surround_rear_100_2M/ -name "*.jpeg" |wc -l |xargs echo -e "surround_rear_100_2M_jpegs:"
    find $1/ofilm_surround_rear_left_100_2M -name "*.jpeg" |wc -l |xargs echo -e "surround_rear_left_100_2M_jpegs:"
    find $1/ofilm_surround_rear_right_100_2M -name "*.jpeg" |wc -l |xargs echo -e "rear_right_100_2M_jpegs:"
    echo "  "
else
    echo -e "ErrorS"
    exit
fi



# if [ -d "$1/ofilm_surround_front_120_8M" ] && [ -d "$1/ofilm_surround_front_left_100_2M" ] && [ -d "$1/ofilm_surround_front_right_100_2M" ] && [ -d "$1/ofilm_surround_rear_100_2M" ] && [ -d "$1/ofilm_surround_rear_left_100_2M" ] && [ -d "$1/ofilm_surround_rear_right_100_2M" ];then
#     echo -e "正在处理$filename，下的数据"
#     find $1/ofilm_surround_front_120_8M -name "*.jpeg" |wc -l |xargs echo -e "front_120_8M_jpegs:"
#     find $1/ofilm_surround_front_left_100_2M -name "*.jpeg" |wc -l |xargs echo -e "front_left_100_2M_jpegs:"
#     find $1/ofilm_surround_front_right_100_2M -name "*.jpeg" |wc -l |xargs echo -e "front_right_100_2M_jpegs:"
#     find $1/ofilm_surround_rear_100_2M/ -name "*.jpeg" |wc -l |xargs echo -e "surround_rear_100_2M_jpegs:"
#     find $1/ofilm_surround_rear_left_100_2M -name "*.jpeg" |wc -l |xargs echo -e "surround_rear_left_100_2M_jpegs:"
#     find $1/ofilm_surround_rear_right_100_2M -name "*.jpeg" |wc -l |xargs echo -e "rear_right_100_2M_jpegs:"
# else
#     echo -e "ErrorS"
#     exit
# fi