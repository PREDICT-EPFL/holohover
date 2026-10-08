# Sourced by ~/.bashrc and by the entrypoint: ROS 2 + holohover workspace environment.
source /opt/ros/humble/setup.bash
if [ -f /root/ros2_ws/install/setup.bash ]; then
    source /root/ros2_ws/install/setup.bash
fi

if [ -n "${PS1:-}" ] && [ -z "${HOLOHOVER_BANNER_SHOWN:-}" ]; then
    export HOLOHOVER_BANNER_SHOWN=1
    cat <<EOF
Holohover workspace (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}, discovery server ${ROS_DISCOVERY_SERVER:-none})
  ros2 node list                      # nodes visible through the discovery server
  ros2 launch holohover_utils <file>  # e.g. simulation_base.launch.py
  cd /root/ros2_ws && colcon build --symlink-install --packages-select <pkg>
  Sources: /root/ros2_ws/src/holohover   Data/rosbags: /root/data
EOF
fi
