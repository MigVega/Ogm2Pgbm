image_name=carto-test
instance_name=carto-instance-test

check_docker_instance_already_running() {
    if  [ ! "$(docker ps -a | grep $instance_name)" ]; then
        return 0
    fi
    return 1
}

delete_running_docker_instance() {
    if ! docker container rm --force "${instance_name}"; then
        return 1
    fi
    return 0
}


simulation_main() {
    xhost +local:docker               # allow window
    if ! check_docker_instance_already_running; then
        if ! delete_running_docker_instance; then
            return 1
        fi
    fi
    docker build -t $image_name . 
    
    docker run -it --rm \
	   --name $instance_name \
           -e DISPLAY=$DISPLAY \
	   -e QT_X11_NO_MITSHM=1 \
	   --device=/dev/dri \
	   --group-add video \
	   --device=/dev/snd:/dev/snd \
	   --group-add audio \
	   --net=host \
	   --privileged \
	   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           --volume="$HOME/.Xauthority:/root/.Xauthority" \
	   --volume="$(pwd)/workspace:/root/workspace" \
	   $image_name /bin/bash
 
    #docker run -it -gpus all --rm \
    #	   --env NVIDIA_VISIBLE_DEVICES=0 \ 
    
}

simulation_main
