#!/bin/bash
usage(){
    echo -e "please input1"
    echo -e "record == record top.log"
    echo -e "stop == stop top PID"

    echo -e "please input2"
    echo -e "+ your directory"
}

if [ $# -ne 2 ]; then
    usage
    exit
fi

top_path=$2
num_runs=1

if [ "$1" == "record" ];then
    for ((i=1; i<=$num_runs; i++))
    do
        echo "=== Run $1 ==="
        top -b -d 1 -c > "$top_path/$(date +"%Y-%m-%d-%H:%M:%S")_top.txt"
        sleep 1
    done

elif [ "$1" == "stop" ];then
    ps -ef | grep top | grep -v grep | awk '{print $2}' | xargs kill -9 &> /dev/null
fi

