#!/bin/bash

usage(){
    echo -e "please input 'open' or 'stop'"
    echo -e "open == open top log"
    echo -e "stop == stop top log"
}

if [ $# -ne 1 ]; then
    usage
    exit
fi

command_script="111.sh"
command_line="./top_record.sh record /home/ros/Downloads/ms/codes/DC_Service"

if [ "$1" == "open" ]; then
    sed -i "s|^# *\($command_line\)$|\1|" "$command_script"
    echo "script is up"
elif [ "$1" == "stop" ]; then
    sed -i "s|^\($command_line\)$|#\1|" "$command_script"
    echo "script is stop"
else
    usage
fi