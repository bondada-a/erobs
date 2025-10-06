# JSON Simplification TODO List

## 1. ✅ Redundant Task Type Nesting
- **Current**: `{"task_type": "moveto", "target_type": "pose", "target": "pickup_approach"}`
- **Proposed**: `{"moveto": {"pose": "pickup_approach"}}` or `{"moveto_pose": "pickup_approach"}`
- **Impact**: Reduces nesting levels and makes JSON more readable

## 2. ⬜ Repetitive Movement Patterns
- **Issue**: Many repeated sequences (approach → target → action → approach)
- **Solution**: Create higher-level composite actions like `pick` and `place`
- **Example**: `{"pick": {"approach": "pickup_approach", "target": "pickup", "gripper_action": "close"}}`

## 3. ⬜ Gripper Action Verbosity
- **Current**: `{"task_type": "end_effector", "end_effector_type": "hande_gripper", "end_effector_action": "hande_closed"}`
- **Proposed**: `{"gripper": "close"}` (context-aware, knows current gripper)
- **Benefit**: 75% reduction in JSON for gripper actions

## 4. ⬜ Tool Exchange Complexity
- **Current**: Separate dock/load operations with approach poses
- **Proposed**: Single atomic operation `{"tool_exchange": {"from": "hande", "to": "epick"}}`
- **Benefit**: Handles approach poses and MoveIt restart internally

## 5. ⬜ Automatic Approach Pose Generation
- **Issue**: Manually defining `_approach` versions of every pose
- **Solution**: Add `approach_offset` field to poses for automatic generation
- **Example**: `"pickup": {"joints": [...], "approach_offset": [0, 0, 0.1]}`

## 6. ⬜ Fix Duplicate Movement Bug
- **Location**: Lines 106-110 in new_test_updated.json
- **Issue**: Moving to `vacuum_pickup` twice consecutively
- **Fix**: Verify correct sequence (should one be `vacuum_place`?)

## 7. ⬜ Implement Polymorphic Task Handlers
- **Current**: Long if-else chain in `execute_step()`
- **Proposed**: Map-based dispatch with lambda handlers
- **Benefit**: More maintainable and extensible

## 8. ⬜ Add Context Object
- **Purpose**: Track current gripper, robot state, poses
- **Benefit**: Eliminates redundant information in JSON
- **Example**: No need to specify `end_effector_type` when context knows current gripper

## 9. ⬜ Create Migration Script
- **Purpose**: Convert existing JSON files to new schema
- **Features**: Backward compatibility, validation, dry-run mode

## 10. ⬜ Update Action Server Parser
- **Files**: `mtc_orchestrator_action_server.cpp`
- **Changes**: Support both old and new JSON formats during transition

## Priority Order:
1. Task type nesting (simplest, highest impact)
2. Fix duplicate movement bug (correctness issue)
3. Gripper action verbosity (frequent operations)
4. Context object implementation
5. Update action server parser
6. Tool exchange simplification
7. Repetitive patterns
8. Automatic approach poses
9. Polymorphic handlers
10. Migration script

## Success Metrics:
- [ ] JSON file size reduced by >40%
- [ ] Code complexity reduced (fewer conditional branches)
- [ ] Easier to read and write task sequences
- [ ] Backward compatibility maintained