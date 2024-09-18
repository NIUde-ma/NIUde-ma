#!/bin/bash

help_int() {
    echo -e "module = restart is docker restart_ing"
    echo -e "module = run is docker running "
    echo -e "module = stop is docker stop"
    echo -e "name = select image"
    echo -e "your input 1 is module and your input 2 is name"
}

module=$1
name=$2

if [ $# -ne 2 ]; then
    help_int
    exit
fi

function get_container_id() {
    docker ps | grep "$1" | awk '{print $1}'
}

function go_to_docker() {
    local container_id=$(get_container_id "$1")
    docker exec -it "$container_id" /bin/bash
}

function restart_docker() {
    local container_id=$(get_container_id "$1")
    docker restart "$container_id"
}

function stop_docker() {
    local container_id=$(get_container_id "$1")
    docker stop "$container_id"
}

function my_docker_run() {
    local find_docker=$(docker ps | grep niude)
    if [[ -z "$find_docker" ]]; then
        if [[ "$module" == "run" && "$name" == "niude" ]]; then
            echo "docker is not up"
            local container_name="${name}_$(uuidgen | cut -d'-' -f1)"
            docker run -d -ti --rm --name "$container_name" -v /home/shuaima66/Downloads/:/root/Downloads -p 9527:22 niude:0.1 /bin/bash -c "echo 'root:NIUde' | chpasswd && service ssh start && /bin/bash"
            go_to_docker "niude"
        else
            help_int
        fi
    elif [[ "$module" == "restart" ]]; then
        echo "docker is restart ing"
        restart_docker "niude"
    elif [[ "$module" == "stop" ]]; then
        echo "docker is status stop"
        stop_docker "niude"
    else
        echo "docker is Running --->>> $find_docker"
        go_to_docker "niude"
    fi
}

function mdc_docker_run() {
    local find_docker=$(docker ps | grep deniu)
    if [[ -z "$find_docker" ]]; then
        if [[ "$module" == "run" && "$name" == "deniu" ]]; then
            echo "docker is not up" 
            local container_name="deniu_$(uuidgen | cut -d'-' -f1)"
            docker run -d -ti --rm --name "$container_name" -v /home/shuaima66/Downloads/:/root/Downloads -p 9528:22 niude:v0.1 /bin/bash -c "echo 'root:NIUde' | chpasswd && service ssh start && /bin/bash"
            go_to_docker "deniu"
        else
            help_int
        fi
    elif [[ "$module" == "restart" ]]; then
        echo "docker is restart ing"
        restart_docker "deniu"
    elif [[ "$module" == "stop" ]]; then
        echo "docker is status stop"
        stop_docker "deniu"
    else
        echo "docker is Running --->>> $find_docker"
        go_to_docker "deniu"
    fi
}

if [ "$name" == "mdc" ]; then
    mdc_docker_run
else
    my_docker_run
fi
