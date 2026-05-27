# WS2 - ROS-VM communication via discovery server

## ROS-VM Setup (xf11bm-ros1)


```bash
git clone https://github.com/bondada-a/erobs.git
cd erobs
source install/setup.bash
export ROS_DISCOVERY_SERVER=10.65.2.151:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```

## WS2 Setup (xf11bm-ws2)
```bash
cd /home/xf11bm/source/jazzy-cms-ros-client
pixi shell -e ros2 
export ROS_DISCOVERY_SERVER=10.65.2.151:11811
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
```

## Talker / Listener 
On either WS2 or ROS-VM to run talker / listener nodes

### Talker:
```bash
ros2 run demo_nodes_cpp talker
```

### Listener
```bash
ros2 run demo_nodes_cpp listener
```


## Other notes:

### Network test between ws2 & ros vm for port 7500:

on ws2
```bash
echo "probe" | ncat -u -w1 10.65.14.42 7500
```

on VM 
```bash
dmesg | grep "Connection Denied" | grep 10.68.80.222 | tail
``` 
Ideally shouldn't have any Connection Denied logs.



## Testing WS2 - ROS VM talker / listener with Discovery server running inside ROS VM 

## ROS VM 

#### Terminal 1: Discovery service on port 7999
```bash
fastdds discovery --server-id 0 --udp-address 10.65.14.42 --udp-port 7999
```

#### Terminal 2: Listener
```bash

