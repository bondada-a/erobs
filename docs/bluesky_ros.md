## on xf11bm-ros1

### Run the podman container 

```bash
podman run -it --rm --network host --ipc=host --pid=host ghcr.io/bondada-a/erobs-jazzy:latest bash
```

### Discovery server env
```bash
export ROS_DISCOVERY_SERVER=10.65.2.151:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=$(pwd)/super_client_configuration_file.xml
export ROS_DOMAIN_ID=0
```

### Run the Robot ROS Stack
```bash
cd erobs
source install/setup.bash
export BEAMBOT_BEAMLINE_CONFIG="$(ros2 pkg prefix --share beambot)/config/cms_beamline.yaml"
ros2 launch beambot beambot_bringup.launch.py use_mock_hardware:=true enable_vision:=false
```

## On WS2

Clone erobs repo ?
```bash
https://github.com/bondada-a/erobs
```
### start pixi shell 
```bash
cd erobs 
pixi install
pixi shell -e ros2
```

### build client dependencies
```bash
colcon build --packages-select beambot_interfaces
```

### Discovery server env
```bash
export ROS_DISCOVERY_SERVER=10.65.2.151:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE=$(pwd)/super_client_configuration_file.xml
export ROS_DOMAIN_ID=0
```

### send a goal via ros action client
```bash
source install/setup.bash
export GOAL="$(cat src/cms/tasks/spincoat_to_hotplate.json)"
ros2 action send_goal /beambot_execution beambot_interfaces/action/MTCExecution "{full_json: '$GOAL'}" --feedback
```



### send goal via BSUI



```python
import rclpy
from bluesky import RunEngine
import bluesky.plan_stubs as bps
from bluesky_ros.mtc_ophyd_device import MTCExecutionDevice

rclpy.init()
robot = MTCExecutionDevice()
RE = RunEngine({})

# Pass a .json path (read into full_json) — or a raw JSON string.
# note : epick mock_hardware / fake_hardware arg is broken upstream , run tests with hande gripper ...
RE(bps.mv(robot, "src/cms/tasks/spincoat_to_hotplate.json"))

RE(bps.mv(robot, "src/cms/tasks/perf_safe_transport_to_safe_exchange.json"))

```

If `import bluesky_ros` fails, add the repo's `src/` to the path for that
shell: `export PYTHONPATH="$PWD/src:$PYTHONPATH"`. (bluesky_ros isn't a colcon
package, so `colcon build` doesn't install it — only `beambot_interfaces` is.)

