docker run -it \
    --name=fsm_lidom_ros_container \
    --env="DISPLAY=$DISPLAY" \
    --net=host \
    --rm \
    li9i/fsm_lidom_ros:latest
