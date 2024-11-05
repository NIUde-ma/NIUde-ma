#!/bin/bash
set -x
usage(){
    echo -e "    -h, --help 如有疑问可以咨询shuaima6/jietan3!"
    echo -e "    -m, --module 设置域控主副角色： apa/scc"
}

if [ $# -ne 2 ]; then
    usage
    exit
fi

until [ $# -eq 0 ]; do
    case $1 in
    -h | --help)
        usage
        exit
        ;;
    -d | --module)
        echo $module
        module=$1
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


function check() {

scc_need_topics=('/iflytek/fusion/objects' '/iflytek/fusion/road_fusion' '/sensor/fdc/camera/front_30_8M' '/sensor/fdc/camera/front_120_8M/compressed')
apa_need_topic=('')

if [ "$module" == "scc" ]; then
    need_topics=("${scc_need_topics[@]}")
elif [ "$module" == "apa" ]; then
    need_topics=("${apa_need_topics[@]}")
else
    echo "Invalid module: $module"
    usage
    exit 1
fi

rosbag_file=$2
info=$(rosbag info "$rosbag_file")

for topic in "${need_topics[@]}"; do
    if echo "$info" | grep -q "$topic"; then
        echo "Topic $topic exists in $rosbag_file"
    else
        echo "Topic $topic does not exist in $rosbag_file"
    fi
done

}

check