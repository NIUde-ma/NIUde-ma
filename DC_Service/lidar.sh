#!/bin/bash

usage() {
    echo "Usage: ${BASH_SOURCE[0]} [options]"
    echo "Options:"
    echo "    -h, --help        Display this information"
    echo "    -p, --platform    Select which platform FDC1.0A/FDC2.0A"
    echo "    -s, --trigger_type 01/02"
    echo " trigger_type 01: external trigger "
    echo " trigger_type 02: internal trigger "
}

platform=FDC2.0A
trigger_type=01

if [ $# -ne 4 ];then
    usage
    exit
fi

until [ $# -eq 0 ]; do
    case $1 in
    -h | --help)
        usage
        exit
        ;;
    -p | --platform)
        echo $platform
        platform=$2
        shift
        ;;
    -s | --sensor_type)
        echo $platform
        trigger_type=$2
        shift
        ;;
    -*)
        echo "Invalid option: $1"
        echo "Try '$0 --help' for more information."
        exit 1
        ;;
    *)
        usage
        break
        ;;
    esac
    shift
done

if [ $trigger_type == "01" ];then
/bsw/tools/checkclock.sh
fi

if [[ $platform == "FDC1.0A" && $trigger_type == "01" ]];then
    echo "FDC1.0A 01 external trigger "
    /bsw/tools/isp-fsync -m outer -i 0 -f 10 -s 2 -w 50000 1 20
    /bsw/tools/isp-fsync -m outer -i 0 -f 10 -s 2 -w 50000 7 20
    sleep 1
    i2ctransfer -f -y 1 w3@0x40 0x02 0xd6 0x84
elif [[ $platform == "FDC1.0A" && $trigger_type == "02" ]];then
    echo "FDC1.0A 02 inner trigger "
    isp-fsync -i 0 -s 0 -w 50000 1 20
    isp-fsync -i 0 -s 0 -w 50000 7 20
    sleep 1
    i2ctransfer -f -y 1 w3@0x40 0x02 0xd6 0x84
elif [[ $platform == "FDC2.0A" && $trigger_type == "01" ]];then
    echo "FDC2.0A 01 external trigger "
    /bsw/tools/isp-fsync -m outer -i 0 -f 10 -s 2 -w 50000 1 20
    /bsw/tools/isp-fsync -m outer -i 0 -f 10 -s 2 -w 50000 4 20
    /bsw/tools/isp-fsync -m outer -i 0 -f 10 -s 2 -w 50000 7 20
    i2ctransfer -f -y 3 w3@0x40 0x02 0xbe 0x84
    i2ctransfer -f -y 3 w3@0x41 0x02 0xd6 0x84
    i2ctransfer -f -y 3 w3@0x42 0x02 0xbe 0x84
elif [[ $platform == "FDC2.0A" && $trigger_type == "02" ]];then
    echo "FDC2.0A 02 inner trigger "
    isp-fsync -s 0 -w 50000 1 20
    isp-fsync -s 0 -w 50000 4 20
    isp-fsync -s 0 -w 50000 7 20
    i2ctransfer -f -y 3 w3@0x40 0x02 0xbe 0x84
    i2ctransfer -f -y 3 w3@0x41 0x02 0xd6 0x84
    i2ctransfer -f -y 3 w3@0x42 0x02 0xbe 0x84