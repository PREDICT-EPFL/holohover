# Holohover


```bash
./holohover build   # build the image and compile the ROS2 packages
./holohover up      # start the discovery server, the foxglove bridge (ws://localhost:8765) and the workspace
./holohover shell   # open a sourced shell in the workspace container
./holohover -h      # all commands
```

## restart mocap
TODO understand and fix this command 
```bash
andrea@LA013:~/holohover$ docker compose -f docker/docker-compose.yml --profile mocap restart mocap
```

## start exp
```bash
ros2 launch holohover_utils air_hockey_real.launch.py record:=true
ros2 topic pub --once /h3/episode std_msgs/msg/Int32 "{data: -1}"
ros2 topic pub --once /h3/episode std_msgs/msg/Int32 "{data: 0}"
```

# Start controller 

```bash
ssh ubuntu@192.168.0.122 
~/holohover-docker/remote-launch/launchFC_loc.sh h3
```
