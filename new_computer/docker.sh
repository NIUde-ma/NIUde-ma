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
    local name=$2
    local docker_id=`docker ps | grep "$name" | tail -n +2 | awk '{print $1}'`
    if [ -z "$docker_id" ]; then
        echo "Docker container '$name' is not UP"
        exit 0
    else
        echo $docker_id
    fi
}

function go_to_docker() {
    local name=$2
    local container_id=$(get_container_id)
    if [ -z "$container_id" ]; then
        echo "Docker container '$name' is not UP"
        exit 0
    else 
        docker exec -it $container_id /bin/bash
    fi
}

function restart_docker() {
    local name=$2
    local container_id=$(get_container_id)
    if [ -z "$container_id" ]; then
        echo "Docker container '$name' is not UP"
        exit 0
    else 
        docker restart $container_id
    fi
}

function stop_docker() {
    local name=$2
    local container_id=$(get_container_id)
    if [ -z "$container_id" ]; then
        echo "Docker container '$name' is not UP"
        exit 0
    else       
        docker stop "$container_id"
    fi
}

function my_docker_run() {
    local find_docker=$(docker ps | grep "$name")
    if [[ -z "$find_docker" ]]; then
        if [[ "$module" == "run" ]]; then
            echo "Docker container '$name' is not UP"
            local container_name="${name}_$(uuidgen | cut -d'-' -f1)"
            docker run -d -ti --rm --name "$container_name" -v /home/shuaima66/Downloads/:/root/Downloads -p 9527:22 niude:0.1 /bin/bash -c "echo 'root:NIUde' | chpasswd && service ssh start && /bin/bash"
            go_to_docker
        else
            echo -e "Docker container '$name' is not UP"
            exit 0
        fi
    elif [[ "$module" == "restart" ]]; then
        echo "Docker container '$name' is restarting"
        restart_docker 
    elif [[ "$module" == "stop" ]]; then
        echo "Docker container '$name' is stopping"
        stop_docker 
    else
        echo "Docker container '$name' is Running --->>> $find_docker"
        go_to_docker 
    fi
}

function mdc_docker_run() {
    local find_docker=$(docker ps | grep "$name")
    if [[ -z "$find_docker" ]]; then
        if [[ "$module" == "run" ]]; then
            echo "Docker container '$name' is not UP" 
            local container_name="${name}_$(uuidgen | cut -d'-' -f1)"
            docker run -d -ti --rm --name "$container_name" -v /home/shuaima66/Downloads/:/root/Downloads -p 9528:22 niude:v0.1 /bin/bash -c "echo 'root:NIUde' | chpasswd && service ssh start && /bin/bash"
            go_to_docker 
        else
            echo -e "Docker container '$name' is not UP"
            exit 0
        fi
    elif [[ "$module" == "restart" ]]; then
        echo "Docker container '$name' is restarting"
        restart_docker
    elif [[ "$module" == "stop" ]]; then
        echo "Docker container '$name' is stopping"
        stop_docker 
    else
        echo "Docker container '$name' is Running --->>> $find_docker"
        go_to_docker 
    fi
}

if [ "$name" == "mdc" ]; then
    mdc_docker_run
else
    my_docker_run
fi

