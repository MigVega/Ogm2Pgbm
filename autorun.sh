#!/bin/bash

image_name=carto-test
instance_name=carto-instance-test

check_podman_instance_already_running() {
    if  [ ! "$(sudo podman ps -a | grep $instance_name)" ]; then
        return 0
    fi
    return 1
}

delete_running_podman_instance() {
    if ! sudo podman container rm --force "${instance_name}"; then
        return 1
    fi
    return 0
}


simulation_main() {
    xhost +local:podman               # allow window
    if ! check_podman_instance_already_running; then
        if ! delete_running_podman_instance; then
            return 1
        fi
    fi
    sudo podman build --cap-add=CAP_AUDIT_WRITE -t $image_name . 
    
    sudo podman run -it --rm \
        --name $instance_name \
        --env DISPLAY \
        --device nvidia.com/gpu=all \
        --network host \
        --privileged \
        --volume="$(pwd)/workspace:/root/workspace" \
        $image_name \
            /bin/bash
 
    
}

simulation_main
