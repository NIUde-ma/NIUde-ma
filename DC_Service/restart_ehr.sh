#!/bin/sh
usage(){
    echo -e "注意以下是脚本的用法~别用错！\n"
    echo "-v position vehicle :1"
    echo "-e ehr not up"
 }

if [ $# -ne 1 ]; then
    usage
    exit
fi

if [ $1 == "-v" ];then
    ps -ef | grep -i ehr | grep -v grep | awk '{print $1}' | xargs kill -9 &> /dev/null &
    sync
    sleep 2
    chmod +x /asw/EhpAdapt/data_channel_sample/*.sh
    cd /asw/EhpAdapt/data_channel_sample/
    sh run_sample_bst.sh &> /dev/null &
    echo "Ehp start_component.sh"

    sleep 2
    export LD_LIBRARY_PATH=/asw/EHR/Lib:$LD_LIBRARY_PATH
    mainboard -d /asw/EHR/Res/dag/startall.dag &> /dev/null &
    sync
    echo -e "---EHR start---"
    
elif [ $1 == "-e" ];then
    ps -ef |grep -i pnc |grep -v grep |awk '{print $2}' |xargs kill -9 &> /dev/null &
    sync
    sleep 5
    echo -e "---Vehicle start---"
    sh /bsw/script/iflyauto_bsw_com_service_pnc.sh &> /dev/null &
    sleep 2

    ps -ef | grep -i ehr | grep -v grep | awk '{print $1}' | xargs kill -9 &> /dev/null &
    sync
    sleep 5
    echo -e "---EHR start---"
    export LD_LIBRARY_PATH=/asw/EHR/Lib:$LD_LIBRARY_PATH
    mainboard -d /asw/EHR/Res/dag/startall.dag &> /dev/null &
    sync
fi

echo -e "Service is Running Please Check Now"