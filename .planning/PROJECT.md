# EROBS Robustness & Reliability

## Vision
Make the EROBS robotic beamline system robust enough for reliable autonomous operation, with clear debugging capabilities and a foundation for future AI-based enhancements.

## Problem Statement
The current system works but has reliability issues that make autonomous 24/7 operation challenging:
- Vision detection is lighting/contrast dependent with inconsistent results
- No automated tests make debugging and regression detection difficult
- Root causes of failures are unclear (vision vs planning vs execution)

## Goals
1. **Identify root causes** of current reliability issues through systematic debugging
2. **Add observability** to understand where failures occur in the pipeline
3. **Improve test coverage** for core orchestration logic
4. **Stabilize vision detection** for consistent object detection
5. **Foundation for future work**: AI detection, VLA control, etc.

## Non-Goals (for now)
- Complete rewrite of any subsystem
- Adding new robot types or grippers
- Bluesky integration improvements
- Performance optimization (speed)

## Success Criteria
- [ ] Root cause of detection inconsistency identified and documented
- [ ] Core orchestrator logic has unit test coverage
- [ ] Vision pipeline has reproducible test cases
- [ ] System can complete 10 consecutive pick-and-place cycles without failure

## Technical Context

### Architecture (from codebase analysis)
- **Orchestrator** (`orchestrator.py`): Central coordinator, JSON task dispatch
- **7 Action Servers**: Specialized handlers for different task types
- **MTC Stages**: Motion planning pipeline composition
- **Vision Layer**: Zivid camera with ArUco/circle/contour detection

### Known Issues (from CONCERNS.md)
1. **Vision Detection Inconsistency**: Lighting dependent, labels shift between captures
2. **Centroid Accuracy**: Robot doesn't hit exact center of detected objects
3. **No Unit Tests**: Orchestrator parsing, batching, gripper tracking untested
4. **Broad Exception Handlers**: Mask unexpected errors, make debugging hard

### Key Files
- `src/beambot/beambot/action_servers/orchestrator.py` - Central coordinator
- `src/beambot/beambot/camera/zivid.py` - Vision detection
- `src/beambot/beambot/stages/vision_stages.py` - Vision-guided motion
- `src/beambot/beambot/stages/pick_place_stages.py` - 9-stage pick/place

## Workflow Mode
**Milestone-based**: Focus on completing one milestone before moving to next.

---

## Milestone 1: Debugging & Observability

**Objective**: Identify where failures occur and add visibility into the pipeline.

### Phases

**Phase 1: Add Diagnostic Logging**
- Add structured logging at key decision points in orchestrator
- Log vision detection confidence scores, timing, and failure modes
- Create log analysis script to identify patterns

**Phase 2: Vision Pipeline Investigation**
- Create reproducible test images/point clouds from Zivid
- Compare detection results across lighting conditions
- Document which parameters affect detection stability

**Phase 3: Unit Tests for Orchestrator**
- Test JSON parsing logic (`_parse_goal`)
- Test task batching logic (`_group_into_batches`)
- Test gripper state tracking
- Mock infrastructure for ROS2 actions

**Phase 4: Integration Test Framework**
- Test harness using `use_fake_hardware:=true`
- Automated vision-to-motion sequence testing
- Failure injection to test error handling

**Phase 5: Document Findings**
- Root cause analysis document
- Prioritized fix list for Milestone 2
- Updated CONCERNS.md with findings

---

## Future Milestones (TBD after M1)

### Milestone 2: Vision Stabilization
Based on M1 findings, fix the identified vision issues.

### Milestone 3: Motion Planning Reliability
Implement Cartesian fallbacks, tune planners for pick-and-place workload.

### Milestone 4: AI Detection Integration
Add YOLOv8-based detection as alternative/complement to geometric methods.

### Milestone 5: Scientist Interface
Typed DSL, high-level plans library for easy scientist usage.

---

*Created: 2026-01-17*
*Last Updated: 2026-01-17*
