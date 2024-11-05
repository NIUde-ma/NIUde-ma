#!/bin/bash

usage(){
    echo -e "    -h, --help 如有疑问可以咨询： shuaima6!"
    echo -e "    -m, --module 设置启动程序： scc/apa"
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
    -m | --module)
        echo $module
        module=$2
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

log_path='/opt/usr/shuaima6'
if [ -d $log_path ];then
   echo -e "$log_path is exists"
else
   mkdir -p $log_path
fi

mdc_hostname=$(hostname)
set_mdc_qos=$(mdc-tool devm set-qos-config "5|35|0|1|1|0")

current_path="$(cd "$( dirname "$0")" && pwd )"
echo ${current_path}
current_work_path="${current_path}/../"
echo ${current_work_path}

chmod +x ${current_work_path}/script/*
chmod +x -R ${current_work_path}/runtime_service/*

sleep 2

# bsw
COM_SERVICE_PNC_NODE_SH="${current_work_path}/script/com_service_pnc_node_exec_run.sh"

GNSS_SERVICE_NODE_SH="${current_work_path}/script/gnss_service_node_exec_run.sh"

HMI_SERVICE_SH="${current_work_path}/script/hmi_service_exec_run.sh"

MOBILEYE_PERCEPTION_SH="${current_work_path}/script/mobileye_perception_exec_run.sh"

PANORAMA_VIEW_CAMERA_NODE_SH="${current_work_path}/script/panorama_view_camera_node_exec_run.sh"
AROUND_VIEW_CAMERA_NODE_SH="${current_work_path}/script/around_view_camera_node_exec_run.sh"

SOC_A_COMMUNICATION_SH="${current_work_path}/script/soc_a_communication_exec_run.sh"
SOC_B_COMMUNICATION_SH="${current_work_path}/script/soc_b_communication_exec_run.sh"

# asw
KOALA_SERVICE_SH="${current_work_path}/script/localization_exec_run.sh"

HMI_ON_SOC_SH="${current_work_path}/script/hmi_on_soc_exec_run.sh"
FINITE_STATE_MANAGER_SH="${current_work_path}/script/finite_state_manager_exec_run.sh"

PANORAMA_VIEW_CAMERA_PERCEPTION_SH="${current_work_path}/script/panorama_view_camera_perception_exec_run.sh"
PANORAMA_VIEW_CAMERA_PERCEPTION_LANELINE_SH="${current_work_path}/script/panorama_view_camera_perception_laneline_exec_run.sh"
AROUND_VIEW_CAMERA_PERCEPTION_SH="${current_work_path}/script/around_view_camera_perception_exec_run.sh"

USS_PERCEPTION_SH="${current_work_path}/script/uss_perception_exec_run.sh"

OBJECT_FUSION_SH="${current_work_path}/script/obstacle_fusion_exec_run.sh"
STATIC_FUSION_SH="${current_work_path}/script/static_fusion_exec_run.sh"

CONTROL_SH="${current_work_path}/script/control_exec_run.sh"

PLANNING_SH="${current_work_path}/script/planning_exec_run.sh"

function run() {
    args=("$@")
    script=${args[0]}
    if [ -e "$script" ]; then
        echo "Running $script"
        if [ -n "${args[1]}" ]; then
            log_file="${args[1]}"
            # echo "$log_file"
            sleep 3
            taskset -c 1,2,3 "${script}" 2>&1 > "$log_file" &
        else
            sleep 5
            if [ $(basename "$script") == "panorama_view_camera_perception_exec_run.sh" ];then
                # sh "$script" &
                echo "${script}"
                taskset -c 0 "${script}" &
            else
                taskset -c 1,2,3 "${script}" &
            fi
        fi
    else
        echo "$script not found"
    fi
}

if [[ $mdc_hostname == "AOS_A" ]]; then
    $set_mdc_qos
    sync
    run $SOC_A_COMMUNICATION_SH

    sleep 2

    if [[ $module == "scc" ]]; then
        run $PANORAMA_VIEW_CAMERA_PERCEPTION_LANELINE_SH
    elif [[ $module == "apa" ]]; then
        run $AROUND_VIEW_CAMERA_NODE_SH
        run $AROUND_VIEW_CAMERA_PERCEPTION_SH
    else
        echo "[error] input param2 error!!!"
        exit 1
    fi

elif [[ $mdc_hostname == "AOS_B" ]]; then
    $set_mdc_qos
    sync
    
    # taskset -c 0 /home/ypfang2/camera_node/gea/script/panorama_view_camera_node_exec_run.sh > /dev/null 2>&1 &
    # sh /home/yxli29/workspace/cluster/gea/script/com_service_pnc_node_exec_run.sh > /dev/null 2&>1 &
    # cd /home/mdc/chenzhang20/com/soc_b_communication/gea/script/
    # ./soc_b_communication_exec_run.sh  > /dev/null 2&>1 &
    # cd -
    
    run $COM_SERVICE_PNC_NODE_SH
    run $GNSS_SERVICE_NODE_SH
    run $HMI_SERVICE_SH
    run $SOC_B_COMMUNICATION_SH

    run $KOALA_SERVICE_SH
    run $HMI_ON_SOC_SH
    run $FINITE_STATE_MANAGER_SH
    run $OBJECT_FUSION_SH
    run $STATIC_FUSION_SH
    run $CONTROL_SH 
    run $PLANNING_SH

    if [[ $module == "scc" ]]; then
        # run $MOBILEYE_PERCEPTION_SH
        run $PANORAMA_VIEW_CAMERA_NODE_SH
        run $PANORAMA_VIEW_CAMERA_PERCEPTION_SH
    elif [[ $module == "apa" ]]; then
        run $USS_PERCEPTION_SH
    else
        echo "[error] input param2 error!!!"
        exit 1
    fi
else
    echo "[error] input param1 error!!!"
    exit 1
fi