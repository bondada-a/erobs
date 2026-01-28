# EROBS - Extensible Robotic Beamline Scientist

## What This Is

Autonomous robotic sample handling system for synchrotron beamlines at NSLS-II. Integrates ROS2 robotics with Bluesky experiment orchestration to enable self-driving beamlines that can run 24/7 without human intervention. Currently deployed on a UR5e with swappable grippers (Hand-E, ePick, Pipettor) and Zivid 3D vision.

## Core Value

**Reliable, autonomous sample manipulation** — scientists should be able to write `yield from robot_plans.load_sample(robot, "A1")` without understanding robotics, and the system must work consistently across beamtimes.

## Requirements

### Validated

<!-- Shipped and confirmed working from existing codebase -->

- ✓ Three-tier action server architecture (Orchestrator → Servers → MTC Stages) — existing
- ✓ Pick-and-place operations with 9-stage MTC pipeline — existing
- ✓ Vision-guided operations with ArUco marker detection — existing
- ✓ Tool exchange between grippers with MoveIt lifecycle management — existing
- ✓ GUI client for task composition and execution monitoring — existing
- ✓ Bluesky Ophyd device integration (proof-of-concept in bsui container) — existing
- ✓ Multiple gripper support (Hand-E, ePick, Pipettor) — existing
- ✓ Task batching for performance (~1.5s saved per batched task) — existing
- ✓ Pause/resume functionality during execution — existing
- ✓ Camera-agnostic vision abstraction (factory pattern) — existing
- ✓ Point cloud obstacle avoidance via Octomap — existing

### Active

<!-- Current scope being built toward -->

**Communication Architecture (Priority: Research First)**
- [ ] Understand current Bluesky → EROBS communication path (Ophyd device, ROS2 action client, message flow)
- [ ] Document architecture with diagrams, written docs, and code walkthrough
- [ ] Research architecture options (REST API, message broker, ROS2 bridge)
- [ ] Check security constraints with beamline IT
- [ ] Design standalone EROBS with external Bluesky bridge

**Vision Improvements**
- [ ] Improve tagless detection reliability (circle/contour detection)
- [ ] Fix label stability (sample IDs shifting between captures)
- [ ] Improve centroid accuracy for grasp positioning
- [ ] Evaluate ML-based detection (YOLOv8) as fallback

**Robust Grasp Pipeline**
- [ ] Phase 1: Scaffold grasp pipeline with MTC GenerateGraspPose stages
- [ ] Phase 2: Tune planning for multi-candidate grasp pattern
- [ ] Phase 3: Add path constraints ("keep level" for transfers)
- [ ] Phase 4: Vision integration with grasp pipeline
- [ ] Phase 5: Scientist interface (typed DSL, high-level plans library)

### Out of Scope

<!-- Explicit boundaries -->

- Multi-robot support — single UR5e focus, no coordination layer needed yet
- Beamline-specific customizations — focus on generic framework first
- Production Bluesky container — beamline staff use their own Bluesky installation

## Context

**Current State:**
- Framework runs on local development machine with GPU for Zivid camera
- bsui container was proof-of-concept for Bluesky integration, isolated from real beamline
- Real beamline Bluesky runs on separate network with EPICS and other equipment
- Need to bridge beamline network (Bluesky) and robot network (EROBS)

**Technical Environment:**
- ROS 2 Humble on Ubuntu 22.04
- MoveIt 2 with MoveIt Task Constructor
- Zivid SDK 2.16.0 (requires GPU/OpenCL)
- Docker containers for deployment

**Known Issues (from codebase analysis):**
- Large monolithic files need refactoring (orchestrator.py: 1108 lines)
- Missing unit tests for core components
- Race condition in pause/resume state management
- Detection label instability between captures

## Constraints

- **No GPU on VM**: Camera processing must stay on GPU-equipped machine — affects deployment architecture
- **Mixed latency needs**: Some operations need real-time response, some can be batched
- **Team dependencies**: Security constraints require beamline IT input; architecture decisions need supervisor approval
- **Network separation**: Beamline network and robot network are separate — bridge required

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| Understand before redesign | Can't design new architecture without understanding current code and message flow | — Pending |
| Keep orchestrator pattern | Needed for tool exchange, batching, and state management | ✓ Good |
| MTC grasp stages over AI grasping | Objects are known (not arbitrary) — geometric approach more reliable | — Pending |
| Standalone EROBS | Decouple from Bluesky container to allow integration with any beamline's existing Bluesky | — Pending |

---
*Last updated: 2026-01-28 after initialization*
