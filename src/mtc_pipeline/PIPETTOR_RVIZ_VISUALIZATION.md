# Pipettor RViz Visualization Options

## Current Status ✅

The pipettor integration is **complete and functional**:
- ✅ Integrated into MTC pipeline architecture
- ✅ Descriptive logging (`"SUCK 50%"`, `"SET_LED (0,255,0)"`)
- ✅ Action feedback published
- ✅ Works through orchestrator
- ✅ Works through GUI

## RViz MTC Task Tree

**Status: ❌ Not Shown (By Design)**

Pipettor operations **do not appear** in RViz's MTC Task Constructor visualization panel because they are **non-motion actions**.

### Why?

MTC is designed for **motion planning tasks**:
- Robot trajectory planning
- Inverse kinematics solving
- Collision checking
- Cartesian path planning

Pipettor operations are **hardware actions** without motion:
- No joints move
- No trajectories needed
- No collision geometry changes
- Instant execution (no planning phase)

Adding them to MTC would require dummy/fake stages that serve no purpose.

## Visualization Options

If you want pipettor operations visible in RViz, here are your options:

### Option 1: Text Markers (Recommended)

Add real-time status display showing current pipettor operation.

**Implementation:**
```cpp
// In pipettor_stages.cpp or pipettor_action_server.cpp
#include <visualization_msgs/msg/marker.hpp>

auto marker_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
    "pipettor_status", 10);

// In execute_pipettor_operation():
visualization_msgs::msg::Marker marker;
marker.header.frame_id = "tool0";  // Or "pipette_base_link"
marker.header.stamp = node_->now();
marker.ns = "pipettor";
marker.id = 0;
marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
marker.action = visualization_msgs::msg::Marker::ADD;
marker.pose.position.z = 0.15;  // Above pipettor
marker.text = operation_name;  // "SUCK 50%"
marker.scale.z = 0.03;  // Text height
marker.color.r = 0.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
marker.color.a = 1.0;
marker.lifetime = rclcpp::Duration::from_seconds(2.0);
marker_pub_->publish(marker);
```

**In RViz:**
1. Add → Marker
2. Topic: `/pipettor_status`
3. See floating text above pipettor during operations

**Pros:**
- Simple to implement (~50 lines)
- Real-time status updates
- No performance impact
- Works with existing system

**Cons:**
- Not in MTC task tree (separate visualization)
- Requires RViz marker display config

---

### Option 2: Interactive Markers

Create interactive markers showing operation state with color coding.

**Visual Feedback:**
- 🟢 Green: Operation in progress
- 🔵 Blue: Waiting
- 🔴 Red: Error
- ⚪ White: Idle

**Implementation:**
```cpp
#include <interactive_markers/interactive_marker_server.hpp>

auto marker_server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "pipettor_controls", node_);

// Create sphere marker at pipettor location
interactive_markers::InteractiveMarker int_marker;
int_marker.header.frame_id = "pipette_base_link";
int_marker.name = "pipettor_status";
int_marker.description = operation_name;

visualization_msgs::msg::Marker sphere;
sphere.type = visualization_msgs::msg::Marker::SPHERE;
sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
sphere.color = operation_color;  // Based on state

visualization_msgs::msg::InteractiveMarkerControl control;
control.markers.push_back(sphere);
int_marker.controls.push_back(control);

marker_server_->insert(int_marker);
marker_server_->applyChanges();
```

**Pros:**
- Interactive (can click for details)
- Color-coded status
- Professional appearance

**Cons:**
- More complex (~150 lines)
- Requires interactive marker config in RViz

---

### Option 3: Custom RViz Panel Plugin

Build a dedicated RViz panel showing pipettor operations.

**Display:**
```
┌─────────────────────────────┐
│   Pipettor Operations       │
├─────────────────────────────┤
│ Current: SUCK 50%          │
│ Volume:  ████████░░ 80%    │
│ LED:     ⬤ (0, 255, 0)     │
│ Status:  ✓ Complete         │
│                             │
│ History:                    │
│  1. SUCK 50%      ✓         │
│  2. EXPEL 50%     ✓         │
│  3. EJECT_TIP     ✓         │
└─────────────────────────────┘
```

**Implementation:** Full Qt-based RViz plugin (~500 lines)

**Pros:**
- Dedicated UI
- Rich status display
- Operation history
- Professional integration

**Cons:**
- Significant development time
- Requires Qt knowledge
- Maintenance overhead

---

### Option 4: Timeline Display

Show operations on a timeline with other tasks.

**Implementation:**
Publish to `/diagnostics_agg` or custom timeline topic:
```cpp
#include <diagnostic_msgs/msg/diagnostic_array.hpp>

diagnostic_msgs::msg::DiagnosticArray msg;
diagnostic_msgs::msg::DiagnosticStatus status;
status.name = "Pipettor Operations";
status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
status.message = operation_name;

diagnostic_msgs::msg::KeyValue kv;
kv.key = "operation";
kv.value = operation_type;
status.values.push_back(kv);

msg.status.push_back(status);
diagnostics_pub_->publish(msg);
```

View with rqt_robot_monitor or custom timeline plugin.

**Pros:**
- Timeline view of all operations
- Standard ROS diagnostics
- Good for debugging

**Cons:**
- Not real-time visual in 3D space
- Requires additional tools

---

## Recommendation

For your use case, I recommend **Option 1: Text Markers**.

### Why?

1. **Quick to implement** - Can be done in 30 minutes
2. **Visible in RViz** - Shows up in 3D space near pipettor
3. **Real-time updates** - See operation changes instantly
4. **No additional tools** - Uses built-in RViz Marker display
5. **Low overhead** - Minimal performance impact

### Implementation Steps

1. **Modify pipettor_action_server.cpp:**
   - Add marker publisher
   - Publish marker when operation starts
   - Update marker during operation
   - Clear marker when done

2. **Configure RViz:**
   - Add Marker display
   - Set topic to `/pipettor_status`
   - Adjust text size/color as needed

3. **Test:**
   - Run pipettor operation
   - See text floating above pipettor in RViz

**Estimated time:** 30-45 minutes
**Lines of code:** ~50
**Complexity:** Low

---

## Alternative: Status in Existing Displays

You can also see pipettor status in:

### 1. RViz Topic Monitor
- Add → By topic → `/mtc_execution/_action/feedback`
- Shows: current_action, progress_percentage, status_message

### 2. RQt Runtime Monitor
```bash
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```
Shows action feedback in real-time

### 3. Console Logs
Already implemented - descriptive names in terminal output

---

## Decision Matrix

| Option | Visibility | Complexity | Time | Maintenance |
|--------|-----------|------------|------|-------------|
| Text Markers | ⭐⭐⭐⭐ | Low | 30min | Low |
| Interactive Markers | ⭐⭐⭐⭐⭐ | Medium | 2hrs | Medium |
| Custom Panel | ⭐⭐⭐⭐⭐ | High | 8hrs | High |
| Timeline | ⭐⭐⭐ | Low | 1hr | Low |
| Status Quo | ⭐⭐ | None | 0min | None |

---

## Next Steps

**To add visualization:**

1. **Choose option** (I recommend Option 1)
2. **Let me know** - I'll implement it for you
3. **Test in RViz** - Verify it meets your needs
4. **Iterate** - Adjust colors, position, timing as needed

**Current functionality works without visualization:**
- All operations execute correctly
- Feedback available via topics
- Logs show descriptive names
- GUI integration complete

Would you like me to implement **Option 1: Text Markers** for real-time RViz display?
