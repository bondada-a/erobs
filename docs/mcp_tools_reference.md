# MCP Tools Reference

Complete inventory of tools available to the LLM via the two MCP servers in `.mcp.json`.

**Total: 34 tools** (10 erobs + 24 ros-mcp)

---

## beambot-mcp-server — Beambot-Specific Tools

> Source: `src/beambot/mcp/beambot_mcp_server.py`
> Custom MCP server with persistent ROS2 node, TF buffer, and camera subscriptions.

### System & State

| Tool | Description | Key Parameters |
|------|-------------|----------------|
| `ping` | Verify the erobs MCP server is reachable. Returns "pong". | — |
| `get_robot_state` | Get system status, attached gripper, execution state, and joint angles in degrees. Safe to call anytime. | — |
| `get_recent_logs` | Read recent ROS2 log messages from `beambot_launch.log`. Use after failures to diagnose errors. | `severity`, `logger`, `count` |

### Pose Management

| Tool | Description | Key Parameters |
|------|-------------|----------------|
| `get_saved_poses` | List poses from the registry (`poses.yaml`). Supports substring filtering. | `filter` |
| `save_pose` | Save a pose by name — from explicit joint angles or the robot's current position. | `name`, `joints_deg` (optional) |
| `delete_pose` | Remove a named pose from the registry. | `name` |

### Vision & Perception

| Tool | Description | Key Parameters |
|------|-------------|----------------|
| `capture_image` | Capture an image (+point cloud) from Zivid (eye-in-hand, triggered) or ZED (external, streaming). Saves to disk. | `camera`, `mode` (`2d`/`3d`) |
| `detect_objects` | Run detection on the last capture. Requires `capture_image()` first. Returns pixel coords + optional 3D positions in base frame. | `method` (`hsv_color`, `circle`, `contour`, `marker`), `camera` |
| `get_point_3d` | Look up 3D world position of a pixel from the last point cloud. Useful for arbitrary points visible in the image. | `pixel_x`, `pixel_y`, `camera` |

### Transforms

| Tool | Description | Key Parameters |
|------|-------------|----------------|
| `get_tf_transform` | Look up TF transform between two frames. Returns translation, rotation (quat + RPY), and 4x4 matrix. | `source_frame` (default `flange`), `target_frame` (default `base_link`) |

---

## ros-mcp-server — Generic ROS2 Bridge

> Package: [`ros-mcp`](https://github.com/robotmcp/ros-mcp-server), run via `uvx ros-mcp`
> Connects to ROS2 via rosbridge WebSocket (port 9090).

### Connection

| Tool | Description |
|------|-------------|
| `connect_to_robot` | Set rosbridge IP/port and test connectivity. |
| `ping_robot` | Ping an IP and check if the rosbridge port is open. |

### Nodes

| Tool | Description |
|------|-------------|
| `get_nodes` | List all active ROS2 nodes. |
| `get_node_details` | Get a node's publishers, subscribers, and services. |

### Topics

| Tool | Description |
|------|-------------|
| `get_topics` | List all active topics with their message types. |
| `get_topic_type` | Get the message type of a specific topic. |
| `get_topic_details` | Get publishers, subscribers, and type info for a topic. |
| `get_message_details` | Get full field definitions of a ROS2 message type. |
| `subscribe_once` | Subscribe to a topic and return one message. |
| `subscribe_for_duration` | Subscribe to a topic for N seconds and return all messages. |
| `publish_once` | Publish a single message to a topic. |
| `publish_for_durations` | Publish a message repeatedly for a specified duration. |

### Services

| Tool | Description |
|------|-------------|
| `get_services` | List all active ROS2 services. |
| `get_service_type` | Get the type of a specific service. |
| `get_service_details` | Get request/response field definitions for a service. |
| `call_service` | Call a service with given arguments and return the response. |

### Actions

| Tool | Description |
|------|-------------|
| `get_actions` | List all active action servers. |
| `get_action_details` | Get goal/result/feedback field definitions for an action type. |
| `get_action_status` | Check the status of a previously sent action goal. |
| `send_action_goal` | Send a goal to an action server and wait for the result. **This is how we execute all robot tasks.** |
| `cancel_action_goal` | Cancel a running action goal by goal ID. |

### Parameters

| Tool | Description |
|------|-------------|
| `get_parameter` | Get a ROS2 parameter value from a node. |
| `set_parameter` | Set a ROS2 parameter on a node. |
| `has_parameter` | Check if a parameter exists on a node. |
| `delete_parameter` | Delete a parameter from a node. |
| `get_parameters` | List all parameters on a given node. |
| `get_parameter_details` | Get the type, range, and description of a parameter. |

### Robot Config

| Tool | Description |
|------|-------------|
| `get_verified_robot_spec` | Get specs (URDF, joints, limits) for a known robot model by name. |
| `get_verified_robots_list` | List all robot models in ros-mcp's built-in database. |
| `detect_ros_version` | Detect the ROS distribution on the connected system. |

### Images

| Tool | Description |
|------|-------------|
| `analyze_previously_received_image` | Load and return a previously saved image (from `subscribe_once`, etc.) for LLM analysis. |

---

## Usage Notes

- **All robot tasks** go through `send_action_goal` → `/beambot_execution`. Never call individual action servers directly.
- **Prefer beambot tools** for vision (`capture_image`, `detect_objects`, `get_point_3d`) and state (`get_robot_state`) — they handle QoS timing, TF lookups, and gripper tracking internally.
- **ros-mcp tools** are best for introspection (listing topics/nodes/services), calling arbitrary services, and debugging.
- On **Humble**, `get_action_details` can't auto-resolve action types — provide the type string explicitly (see action type mapping in `CLAUDE.md`).
- **Call `get_robot_state` first** before constructing any task JSON, to know the system status and attached gripper.
