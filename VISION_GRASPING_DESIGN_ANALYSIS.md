# Vision-Based Grasping Design Analysis

## System Requirements
- Detect AprilTags using Zivid camera
- Move robot to grasp detected objects
- Support multiple grippers (Hande, EPick)
- Integrate with existing MTC pipeline
- Reliable and maintainable architecture

## Approach 1: Direct TCP Movement
### Description
Move TCP directly to tag position using basic MoveIt commands.

### Implementation
```python
tag_pose = get_tag_from_tf()
move_group.set_pose_target(tag_pose)
move_group.go()
```

### Pros
- ✅ Simple to implement
- ✅ Quick to test
- ✅ Direct control

### Cons
- ❌ No collision checking with object
- ❌ No approach/retreat strategies
- ❌ No grasp planning
- ❌ Hard-coded offsets
- ❌ Poor error handling
- ❌ Not reusable

### Score: 3/10 - Too simplistic for production

---

## Approach 2: Pure MoveIt Task Constructor (MTC)
### Description
Use MTC to create complete pick pipelines with stages.

### Implementation
```cpp
// Create MTC task with stages:
// 1. Current State
// 2. Open Gripper
// 3. Generate Grasp Poses
// 4. Approach
// 5. Close Gripper
// 6. Attach Object
// 7. Retreat
```

### Pros
- ✅ Proper grasp planning
- ✅ Collision checking
- ✅ Approach/retreat strategies
- ✅ Multiple grasp candidates
- ✅ Robust error handling
- ✅ Already integrated in your system

### Cons
- ❌ Complex to configure
- ❌ Requires object collision geometry
- ❌ May be overkill for simple tags

### Score: 8/10 - Professional but complex

---

## Approach 3: Vision Action Server
### Description
Dedicated action server that handles vision + grasping.

### Implementation
```cpp
class VisionGraspActionServer {
  // Action: /vision_grasp
  // Goal: tag_id, grasp_strategy
  // Result: success, final_pose
  // Feedback: current_stage
}
```

### Pros
- ✅ Clean interface
- ✅ Reusable across projects
- ✅ Progress feedback
- ✅ Can be cancelled
- ✅ Encapsulates complexity

### Cons
- ❌ Another layer of abstraction
- ❌ Need to maintain action definitions
- ❌ Potential integration issues

### Score: 7/10 - Good modularity

---

## Approach 4: Service-Based Pipeline
### Description
Multiple services for each step: detect, plan, execute.

### Implementation
```yaml
Services:
  /detect_tag -> tag_pose
  /plan_grasp -> trajectory
  /execute_grasp -> success
```

### Pros
- ✅ Granular control
- ✅ Easy to test individual components
- ✅ Simple interfaces

### Cons
- ❌ Multiple service calls
- ❌ State management between calls
- ❌ No transaction support

### Score: 5/10 - Too fragmented

---

## Approach 5: Hybrid MTC + Vision Integration (RECOMMENDED)
### Description
Extend existing MTC stages with vision-aware pick stage.

### Architecture
```
Existing MTC Pipeline
    ↓
VisionPickStage (new)
    ├── Trigger Camera
    ├── Detect Tag
    ├── Generate Grasp
    └── Execute Pick
    ↓
Existing Place/Move Stages
```

### Implementation Strategy

#### Phase 1: Vision Detection Module
```cpp
class VisionDetector {
  // Encapsulates:
  // - Camera triggering
  // - AprilTag detection
  // - TF publishing
  // - Pose validation

  std::optional<geometry_msgs::PoseStamped>
  detectObject(int tag_id);
};
```

#### Phase 2: MTC Vision Stage
```cpp
class VisionPickStage : public MoveitTaskConstructorStage {
  // Integrates with existing MTC:
  // - Uses VisionDetector
  // - Generates grasp poses
  // - Plans approach/retreat
  // - Handles gripper-specific logic

  void computePickPose(const PoseStamped& object_pose,
                       const std::string& gripper_type);
};
```

#### Phase 3: Configuration System
```yaml
vision_pick_config:
  grippers:
    hande:
      tcp_frame: "robotiq_hande_grasp_point"
      approach_distance: 0.10
      grasp_offset: [0, 0, 0.02]
    epick:
      tcp_frame: "epick_suction_cup"
      approach_distance: 0.08
      grasp_offset: [0, 0, 0]
```

### Pros
- ✅ Leverages existing MTC infrastructure
- ✅ Minimal new code
- ✅ Gripper-agnostic design
- ✅ Configuration-driven
- ✅ Testable components
- ✅ Production-ready
- ✅ Maintains existing workflows

### Cons
- ❌ Requires understanding MTC
- ❌ Initial setup complexity

### Score: 9/10 - Best balance

---

## FINAL RECOMMENDATION: Hybrid MTC + Vision

### Why This Approach?
1. **Reuses existing infrastructure** - You already have MTC stages
2. **Minimal disruption** - Extends rather than replaces
3. **Gripper flexibility** - Configuration handles Hande/EPick differences
4. **Professional quality** - Production-ready with proper error handling
5. **Maintainable** - Clear separation of concerns

### Implementation Plan

#### Step 1: Create Vision Detection Service (Week 1)
- [ ] Create `vision_detection_service.cpp`
- [ ] Implement camera trigger + tag detection
- [ ] Add pose validation and filtering
- [ ] Test with known tag positions

#### Step 2: Define Grasp Configuration (Week 1)
- [ ] Create `grasp_configs.yaml` for each gripper
- [ ] Define TCP frames properly in URDF
- [ ] Set approach vectors and offsets
- [ ] Validate with manual testing

#### Step 3: Implement Vision Pick Stage (Week 2)
- [ ] Create `vision_pick_stage.cpp`
- [ ] Inherit from MTC stage base
- [ ] Integrate vision detection
- [ ] Add grasp generation logic

#### Step 4: Integration Testing (Week 2)
- [ ] Test with Hande gripper
- [ ] Test with EPick gripper
- [ ] Test error cases
- [ ] Performance optimization

#### Step 5: Production Deployment (Week 3)
- [ ] Add monitoring/logging
- [ ] Create operator documentation
- [ ] Setup CI/CD tests
- [ ] Deploy to production

### Key Design Decisions

1. **TCP Definition**
   - Define grasp point as TCP (not flange)
   - Hande: Between fingers at grasp center
   - EPick: At suction cup contact point

2. **Coordinate Frames**
   ```
   base_link
     ↓
   flange
     ↓
   gripper_base
     ↓
   gripper_tcp (grasp point)
   ```

3. **Grasp Strategy**
   - Top-down for table objects
   - Side approach for vertical surfaces
   - Configurable per object type

4. **Error Handling**
   - Retry detection 3 times
   - Validate pose is reachable
   - Check gripper state before/after
   - Report specific failure reasons

### Configuration Schema

```yaml
vision_pick:
  detection:
    camera_timeout: 5.0
    max_retries: 3
    min_confidence: 0.8

  grippers:
    hande:
      tcp_frame: "hande_tcp"
      approach:
        direction: [0, 0, -1]  # From above
        distance: 0.10
      grasp:
        pre_shape: "open"
        grasp_shape: "closed"
        force: 50.0

    epick:
      tcp_frame: "epick_tcp"
      approach:
        direction: [0, 0, -1]
        distance: 0.08
      grasp:
        vacuum_level: 80
        timeout: 2.0

  objects:
    small_tag:
      size: [0.008, 0.008, 0.001]
      grasp_offset: [0, 0, 0.002]
      approach_angle_tolerance: 0.2
```

### Testing Strategy

1. **Unit Tests**
   - Vision detection with mock camera
   - Grasp pose generation
   - Configuration loading

2. **Integration Tests**
   - Full pick sequence in simulation
   - Different tag positions
   - Error recovery

3. **System Tests**
   - Real robot with actual tags
   - Multiple grippers
   - Edge cases

### Success Metrics
- Detection rate > 95%
- Grasp success > 90%
- Cycle time < 10 seconds
- Zero safety incidents

---

## Decision Matrix

| Criteria | Direct | Pure MTC | Action Server | Services | Hybrid |
|----------|--------|----------|---------------|----------|---------|
| Simplicity | 10 | 5 | 7 | 8 | 7 |
| Robustness | 2 | 9 | 7 | 5 | 9 |
| Maintainability | 3 | 7 | 8 | 6 | 9 |
| Reusability | 2 | 8 | 9 | 7 | 9 |
| Integration | 4 | 9 | 6 | 5 | 10 |
| **Total** | **21** | **38** | **37** | **31** | **44** |

## Final Verdict: Hybrid MTC + Vision (Score: 44/50)

This approach provides the best balance of:
- Leveraging existing infrastructure
- Professional-grade robustness
- Maintainable architecture
- Gripper flexibility
- Production readiness