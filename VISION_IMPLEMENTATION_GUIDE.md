# Vision Grasping Implementation Guide

## Quick Start Path (What We'll Build)

Based on the analysis, we'll implement the **Hybrid MTC + Vision** approach in phases:

### Phase 1: Foundation (Today)
Create a clean vision detection wrapper that works with your existing system.

### Phase 2: TCP Setup (Today)
Define proper Tool Center Points for accurate grasping.

### Phase 3: MTC Integration (Tomorrow)
Extend your existing MTC stages with vision capabilities.

---

## Phase 1: Vision Detection Wrapper

### 1.1 Create Vision Detector Class

**File: `src/mtc_pipeline/include/mtc_pipeline/vision_detector.hpp`**
```cpp
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_srvs/srv/trigger.hpp>
#include <optional>

class VisionDetector {
public:
  VisionDetector(rclcpp::Node::SharedPtr node);

  // Main interface - detect object and return pose
  std::optional<geometry_msgs::msg::PoseStamped>
  detectObject(int tag_id, const std::string& reference_frame = "base_link");

  // Get last detection timestamp
  rclcpp::Time getLastDetectionTime() const { return last_detection_time_; }

  // Check if detection is recent (< 2 seconds old)
  bool isDetectionValid() const;

private:
  // Trigger camera capture
  bool triggerCapture();

  // Get tag pose from TF
  std::optional<geometry_msgs::msg::PoseStamped>
  getTagPose(int tag_id, const std::string& reference_frame);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr capture_client_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Time last_detection_time_;
};
```

### 1.2 Implementation

**File: `src/mtc_pipeline/src/vision_detector.cpp`**
```cpp
#include "mtc_pipeline/vision_detector.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

VisionDetector::VisionDetector(rclcpp::Node::SharedPtr node)
  : node_(node) {

  // Setup TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Setup capture client
  capture_client_ = node_->create_client<std_srvs::srv::Trigger>("/capture_2d");

  RCLCPP_INFO(node_->get_logger(), "VisionDetector initialized");
}

std::optional<geometry_msgs::msg::PoseStamped>
VisionDetector::detectObject(int tag_id, const std::string& reference_frame) {

  // Step 1: Trigger capture
  if (!triggerCapture()) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to capture image");
    return std::nullopt;
  }

  // Step 2: Get tag pose from TF
  auto pose = getTagPose(tag_id, reference_frame);

  if (pose) {
    last_detection_time_ = node_->get_clock()->now();
    RCLCPP_INFO(node_->get_logger(),
                "Tag %d detected at [%.3f, %.3f, %.3f]",
                tag_id, pose->pose.position.x,
                pose->pose.position.y, pose->pose.position.z);
  }

  return pose;
}

bool VisionDetector::triggerCapture() {
  // Your existing capture logic
  if (!capture_client_->wait_for_service(std::chrono::seconds(2))) {
    return false;
  }

  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = capture_client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node_, future, std::chrono::seconds(5)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    return false;
  }

  return future.get()->success;
}

std::optional<geometry_msgs::msg::PoseStamped>
VisionDetector::getTagPose(int tag_id, const std::string& reference_frame) {

  std::string tag_frame = "tag36h11:" + std::to_string(tag_id);

  try {
    auto transform = tf_buffer_->lookupTransform(
      reference_frame, tag_frame,
      tf2::TimePointZero,
      std::chrono::seconds(2));

    geometry_msgs::msg::PoseStamped pose;
    pose.header = transform.header;
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation = transform.transform.rotation;

    return pose;

  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(node_->get_logger(), "Could not get transform: %s", ex.what());
    return std::nullopt;
  }
}

bool VisionDetector::isDetectionValid() const {
  auto age = (node_->get_clock()->now() - last_detection_time_).seconds();
  return age < 2.0;  // Detection valid for 2 seconds
}
```

---

## Phase 2: Proper TCP Definition

### 2.1 Add Grasp Point to URDF

**File: `src/robotiq_hande_description/urdf/robotiq_hande_gripper.xacro`**
Add this to define the grasp point:

```xml
<!-- Add inside the robotiq_hande macro -->

<!-- TCP at grasp point (between fingers) -->
<link name="robotiq_hande_tcp">
  <visual>
    <geometry>
      <sphere radius="0.005"/>
    </geometry>
    <material name="red"/>
  </visual>
</link>

<joint name="robotiq_hande_tcp_joint" type="fixed">
  <parent link="robotiq_hande_end"/>
  <child link="robotiq_hande_tcp"/>
  <!-- Position at center of grasp, adjust Z for your objects -->
  <origin xyz="0 0 0.02" rpy="0 0 0"/>
</joint>
```

### 2.2 Grasp Configuration

**File: `src/mtc_pipeline/config/grasp_config.yaml`**
```yaml
grasp_config:
  hande:
    # TCP frame for planning
    tcp_frame: "robotiq_hande_tcp"

    # Approach parameters
    approach:
      direction: [0.0, 0.0, -1.0]  # From above
      min_distance: 0.05
      max_distance: 0.15

    # Retreat parameters
    retreat:
      direction: [0.0, 0.0, 1.0]   # Up
      min_distance: 0.05
      max_distance: 0.10

    # Grasp parameters
    grasp:
      pre_grasp_posture: "open"
      grasp_posture: "closed"

    # Object-specific offsets (tag-specific)
    object_offsets:
      small_tag:   [0.0, 0.0, 0.002]  # 2mm above tag
      default:     [0.0, 0.0, 0.005]  # 5mm above by default

  epick:
    tcp_frame: "epick_suction_tcp"
    approach:
      direction: [0.0, 0.0, -1.0]
      min_distance: 0.03
      max_distance: 0.10
    retreat:
      direction: [0.0, 0.0, 1.0]
      min_distance: 0.05
      max_distance: 0.10
```

---

## Phase 3: MTC Vision Stage

### 3.1 Create Vision-Aware Pick Stage

**File: `src/mtc_pipeline/include/mtc_pipeline/vision_pick_stage.hpp`**
```cpp
#pragma once
#include <moveit/task_constructor/stage.h>
#include "vision_detector.hpp"

namespace moveit_task_constructor {

class VisionPickStage : public Stage {
public:
  VisionPickStage(const std::string& name,
                  std::shared_ptr<VisionDetector> detector);

  void setTagID(int tag_id) { tag_id_ = tag_id; }
  void setGripperType(const std::string& type) { gripper_type_ = type; }

  void compute() override;

private:
  // Generate grasp pose from detected object
  geometry_msgs::msg::PoseStamped
  computeGraspPose(const geometry_msgs::msg::PoseStamped& object_pose);

  std::shared_ptr<VisionDetector> detector_;
  int tag_id_;
  std::string gripper_type_;

  // Loaded from config
  std::map<std::string, GraspConfig> grasp_configs_;
};

} // namespace
```

### 3.2 Integration with Existing Pipeline

**Modify: `src/mtc_pipeline/src/pick_place_task.cpp`**
```cpp
// Add vision detection option
void PickPlaceTask::init() {
  // ... existing code ...

  // Check if vision-based pick
  if (use_vision_) {
    // Create vision detector
    auto detector = std::make_shared<VisionDetector>(node_);

    // Add vision pick stage
    auto vision_pick = std::make_unique<VisionPickStage>("vision_pick", detector);
    vision_pick->setTagID(target_tag_id_);
    vision_pick->setGripperType(gripper_type_);

    task_->add(std::move(vision_pick));
  } else {
    // Existing fixed-pose pick logic
  }
}
```

---

## Testing Plan

### Test 1: Vision Detection Only
```bash
# Test detector standalone
ros2 run mtc_pipeline test_vision_detector --tag-id 1
```

### Test 2: TCP Verification
```bash
# Verify TCP is at correct position
ros2 run tf2_ros tf2_echo robotiq_hande_end robotiq_hande_tcp
```

### Test 3: Full Pick Pipeline
```bash
# Run complete vision pick
ros2 run mtc_pipeline test_vision_pick --tag-id 1 --gripper hande
```

---

## Implementation Order (Recommended)

### Day 1 (Today):
1. **[2 hours]** Implement VisionDetector class
2. **[1 hour]** Add TCP frames to URDF
3. **[1 hour]** Create grasp configuration YAML
4. **[1 hour]** Test vision detection standalone

### Day 2 (Tomorrow):
1. **[2 hours]** Create VisionPickStage
2. **[2 hours]** Integrate with existing MTC pipeline
3. **[1 hour]** Test with Hande gripper

### Day 3:
1. **[2 hours]** Add EPick support
2. **[1 hour]** Error handling improvements
3. **[2 hours]** Full system testing

---

## Key Advantages of This Design

1. **Minimal Changes**: Extends existing system rather than replacing
2. **Testable**: Each component can be tested independently
3. **Configurable**: YAML-based configuration for different grippers
4. **Robust**: Proper error handling at each stage
5. **Reusable**: VisionDetector can be used elsewhere
6. **Production Ready**: Following MTC best practices

---

## Quick Validation Tests

Before full implementation, validate the concept:

```python
# Quick test script: test_vision_concept.py
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_srvs.srv import Trigger

class QuickTest(Node):
    def __init__(self):
        super().__init__('quick_test')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.capture_client = self.create_client(Trigger, '/capture_2d')

    def test(self):
        # Trigger capture
        future = self.capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        # Get tag pose
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', 'tag36h11:1',
                rclpy.time.Time())
            print(f"Tag at: {transform.transform.translation}")
            return True
        except:
            print("No tag detected")
            return False

rclpy.init()
node = QuickTest()
success = node.test()
rclpy.shutdown()
```

---

## Next Steps

1. Start with VisionDetector implementation
2. Test detection accuracy
3. Define proper TCP frames
4. Integrate with MTC
5. Production testing

This design leverages your existing infrastructure while adding clean, testable vision capabilities.