# Development Notes

## 1. UR Driver Connection Issues in Docker/VM Environments

### Problem
When sending robot goals from containers over a VM, the UR driver loses connection intermittently:

```
Sending data through socket failed.
[WARN] Connection attempt on port 50003 while maximum number of clients (1) is already connected. Closing connection.
[INFO] Connection to reverse interface dropped.

[WARN] Connection attempt on port 50001 while maximum number of clients (1) is already connected. Closing connection.
[INFO] Connection to reverse interface dropped.
```

### Root Cause
Likely caused by network latency/jitter in Docker/VM environments. The UR driver's keepalive mechanism times out before packets arrive, causing it to think the connection is lost. When the delayed packet arrives, it's treated as a "new" connection attempt on an already-connected port.

### Attempted Fix
Increased `keep_alive_count` parameter in URDF xacro files:

```xml
<!-- Keep-alive: number of 20ms cycles without response before disconnect -->
<xacro:arg name="keep_alive_count" default="10"/>  <!-- 10 × 20ms = 200ms timeout -->
```

**Status**: Preliminary testing suggests this helps, but not fully validated.

### Files Modified
- `ur5e_robot_description/urdf/ur_standalone.xacro`
- `ur5e_robot_description/urdf/ur_with_zivid_hande.xacro`
- `ur5e_robot_description/urdf/ur_with_zivid_epick.xacro`
- `ur5e_robot_description/urdf/ur_with_zivid_pipettor.xacro`

### Reference
- https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/issues/418
