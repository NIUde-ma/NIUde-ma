#!/bin/bash

# set -x 

function_docker_env (){
    # find_docker_cmd="docker images |grep niude"
    if docker images | grep -q "niude"; then
        return 0
    else
        echo -e "docker images is not found"
        exit 1
    fi
}

function_docker_module (){
    function_docker_env
    if [ $? -eq 0 ]; then
        # echo -e "done"
        return 0
    else
        echo -e "docker envs is failed"
        exit 1
    fi
}

function_docker_run (){
    function_docker_module
    if [ $? -eq 0 ]; then
        if [ -z $(docker ps |grep  niude |awk '{print $1}') ] ;then
            docker run -d -ti --rm --name "$container_name" -v /home/qcraft/work:/root/work -p 9527:22 niude:0.2 /bin/bash -c "echo 'root:NIUde' | chpasswd && service ssh start && /bin/bash"
        else
            echo -e "docker is running now"
            docker exec -it $(docker ps |grep  niude |awk '{print $1}') bash
        fi
    else
        echo -e "docker env failed"
    fi
}



case $1 in 
    "stop" )
        docker stop $(docker ps -q)
    ;;
    "run" )
        function_docker_run
    ;;
    * )
        echo "Usage: $1 {stop [container]|stop|run}"
        echo "Examples:"
        echo "  $1 stop           # Stop niude containers"
        echo "  $1 run           # Run niude containers"
        exit 1
    ;;

esac