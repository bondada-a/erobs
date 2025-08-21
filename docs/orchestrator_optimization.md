# Orchestrator Optimization: Replacing Hardcoded Sleep Times

## Problem
The original orchestrator used hardcoded `std::this_thread::sleep_for()` calls that were inefficient and unreliable:

- **Line 73**: 15-second sleep before dashboard play
- **Line 113**: 10-second sleep after killing processes  
- **Line 206**: 10-second sleep after initial setup
- **Line 233**: 10-second sleep after tool exchange execution
- **Line 285**: 60-second sleep on failure

## Solution: Event-Driven Waiting

### 1. **Service Availability Waiting**
```cpp
bool wait_for_service(rclcpp::Node::SharedPtr node, const std::string& service_name, 
                     std::chrono::seconds timeout = std::chrono::seconds(30))
```
- **Before**: Hardcoded 15-second sleep before calling dashboard play
- **After**: Wait for service to actually become available (max 30s timeout)

### 2. **MoveIt Readiness Detection**
```cpp
bool wait_for_moveit_ready(rclcpp::Node::SharedPtr node, 
                          std::chrono::seconds timeout = std::chrono::seconds(30))
```
- **Before**: Assumed MoveIt was ready after node appeared
- **After**: Wait for planning scene to be published, indicating full readiness

### 3. **Robot Stability Monitoring**
```cpp
bool wait_for_robot_stable(rclcpp::Node::SharedPtr node, 
                          std::chrono::seconds timeout = std::chrono::seconds(30),
                          double velocity_threshold = 0.01)
```
- **Before**: Hardcoded 10-second sleep after robot movements
- **After**: Monitor joint velocities and wait for actual stability

### 4. **Process Termination Monitoring**
```cpp
void kill_all_and_wait()
```
- **Before**: Hardcoded 10-second sleep after sending SIGINT
- **After**: Actively poll process status and wait for graceful termination

### 5. **Failure Handling**
- **Before**: 60-second sleep on failure
- **After**: User prompt with Enter key, allowing immediate continuation

## Benefits

### **Performance Improvements**
- **Faster startup**: No unnecessary waiting when services are ready early
- **Faster execution**: Robot continues as soon as it's stable, not after arbitrary timeout
- **Faster recovery**: Immediate response to failures

### **Reliability Improvements**
- **More robust**: Waits for actual conditions rather than hoping timeouts are sufficient
- **Better error handling**: Clear feedback when services don't become available
- **Adaptive timing**: Works with different hardware speeds and network conditions

### **User Experience**
- **Interactive failures**: User can choose to continue or exit immediately
- **Better logging**: Clear indication of what the system is waiting for
- **Predictable behavior**: Consistent timing regardless of system load

## Implementation Details

### **Joint State Monitoring**
The robot stability detection monitors the `/joint_states` topic and waits for all joint velocities to fall below a threshold (default: 0.01 rad/s).

### **Planning Scene Monitoring**
MoveIt readiness is detected by waiting for the `/monitored_planning_scene` topic to start publishing, indicating the planning scene monitor is active.

### **Service Availability**
Uses ROS2's built-in `wait_for_service()` mechanism with configurable timeouts.

### **Process Management**
Uses `waitpid()` with `WNOHANG` flag to poll process status without blocking, allowing for graceful termination with fallback to force kill.

## Configuration

All timeouts are configurable:
- Service wait timeout: 30 seconds (default)
- MoveIt readiness timeout: 30 seconds (default)  
- Robot stability timeout: 30 seconds (default)
- Velocity threshold: 0.01 rad/s (default)

## Usage

The optimized orchestrator maintains the same interface but provides much better performance:

```bash
ros2 launch mtc_pipeline orchestrator_launch.launch.py poses_file:=recorded_poses.json robot_ip:=192.168.56.101
```

## Future Improvements

1. **Action-based waiting**: Use ROS2 action clients to wait for trajectory completion
2. **State machine integration**: Integrate with robot state machine for more precise state detection
3. **Dynamic timeouts**: Adjust timeouts based on robot model and task complexity
4. **Parallel monitoring**: Monitor multiple conditions simultaneously for even faster response
