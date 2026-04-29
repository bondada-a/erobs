
Steps to run 
1. ssh into xf11bm-ws2
2. `podman run -it --rm --network host --pid=host --ipc=host --device nvidia.com/gpu=all ghcr.io/bondada-a/beambot_img_v2:latest`
3. `export ROS_DISCOVERY_SERVER=""`
4. `colcon build --packages-select`
5. 
```
  colcon build --packages-select beambot_interfaces \
    --cmake-args \
    -DPython3_EXECUTABLE=$(which python3) \
    -DPython_EXECUTABLE=$(which python3) \
    -DPYTHON_EXECUTABLE=$(which python3) \
    -DPYTHON_SOABI=cpython-311-x86_64-linux-gnu
```

