

Steps to run :

## VM Container 

#### Launch ROS Discovery Server :
1. ssh xf11bm-ros1
2. `podman run -it --rm --network host --pid=host --ipc=host ghcr.io/bondada-a/beambot_img_v2:latest`
3. `fastdds discovery --server-id 0`

#### Launch Robot Launch file 
4. ssh into xf11bm-ros1 
5. `podman exec -it {container_name} bash`
6. `ros2 launch beambot beambot_bringup.launch.py enable_vision:=false`

## WS2 

1. `cd ~/source/cms-ros-client`
2. `pixi install`
3. `pixi shell -e ros2`
4. `cd /home/xf11bm/.ipython/profile_collection/users/2026-1/beamline/ABondada`
5. `colcon build --packages-select beambot_interfaces`
6. `export ROS_DISCOVERY_SERVER=10.68.82.42:11811`
7. `export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_super_client.xml`
8.  `export ROS_DOMAIN_ID=0`

To send goal
1. Safe sample transport
```
ros2 action send_goal --feedback /beambot_execution beambot_interfaces/action/MTCExecution "{full_json: '{\"start_gripper\": \"epick\", \"tasks\": [{\"task_type\": \"moveto\", \"target\": \"safe_sample_transport\"}], \"poses\": {\"safe_sample_transport\": [79.72, -69.5, -81.91, -117.92, -268.05, -157.15]}}'}" 
```

2. Safe tool exchange 
```
  ros2 action send_goal --feedback /beambot_execution beambot_interfaces/action/MTCExecution "{full_json: '{\"start_gripper\": \"epick\", \"tasks\": [{\"task_type\": \"moveto\", \"target\": \"safe_tool_exchange\"}], \"poses\": {\"safe_tool_exchange\": [152.33, -110.63, -100.17, -59.12, -270.43, -207.7]}}'}"
```


via ipython shell?

