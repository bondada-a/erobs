# Roadmap: EROBS

## Overview

Evolve EROBS from a local development prototype to a production-ready, standalone robotic framework that integrates with any beamline's existing Bluesky infrastructure. The journey starts with deep understanding of current architecture, then improves core capabilities (vision, grasping), and culminates in a redesigned communication architecture that decouples EROBS from Bluesky containers.

## Domain Expertise

None

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

- [ ] **Phase 1: Communication Research** — Deep understanding of Bluesky → EROBS path
- [ ] **Phase 2: Vision Improvements** — Reliable tagless detection with stable labeling
- [ ] **Phase 3: Robust Grasp Pipeline** — MTC-based grasp generation with multi-candidate planning
- [ ] **Phase 4: Communication Implementation** — Standalone EROBS with external Bluesky bridge

## Phase Details

### Phase 1: Communication Research
**Goal**: Complete understanding of current Bluesky → EROBS communication path with comprehensive documentation
**Depends on**: Nothing (first phase)
**Research**: Likely (understanding existing system architecture)
**Research topics**: Ophyd device wrapper internals, ROS2 action client flow, DDS networking between containers, Docker bridge network setup
**Plans**: TBD

Deliverables:
- Architecture diagram showing message flow from Bluesky to robot
- Written documentation explaining each component
- Annotated code walkthrough of the communication path

Key files to analyze:
- `src/bluesky_ros/mtc_ophyd_device.py` — Ophyd device wrapper
- `src/bluesky_ros/mtc_ophyd_device_async.py` — Async variant
- `src/beambot/beambot/action_servers/orchestrator.py` — Action server receiving goals
- `docker/bsui/Dockerfile` — Bluesky container setup

### Phase 2: Vision Improvements
**Goal**: Reliable tagless detection with stable labeling and accurate centroid positioning
**Depends on**: Phase 1 (can run in parallel while waiting for IT input)
**Research**: Unlikely (building on existing patterns in zivid.py)
**Plans**: TBD

Deliverables:
- Improved circle/contour detection reliability
- Stable sample labeling between captures
- Accurate centroid positioning for grasps
- Optional: ML-based detection (YOLOv8) evaluation

Key files:
- `src/beambot/beambot/camera/zivid.py` — Detection methods
- `src/beambot/beambot/stages/vision_stages.py` — Vision stage composition

### Phase 3: Robust Grasp Pipeline
**Goal**: MTC-based grasp generation with multi-candidate planning and path constraints
**Depends on**: Phase 2 (vision provides accurate sample positions)
**Research**: Likely (MTC grasp stages are new integration)
**Research topics**: MTC GenerateGraspPose stages, IK solver tuning, OMPL planner parameters, path constraints
**Plans**: TBD

Deliverables:
- GenerateGraspPose stages integrated into pick-and-place
- Multi-candidate grasp planning (fast query, fail fast pattern)
- "Keep level" path constraints for sample transfers
- Vision integration with grasp pipeline

Key files:
- `src/beambot/beambot/stages/base_stages.py` — MTC utilities
- `src/beambot/beambot/stages/pick_place_stages.py` — Pick/place composition

### Phase 4: Communication Implementation
**Goal**: Standalone EROBS with external Bluesky bridge meeting security requirements
**Depends on**: Phase 1 (research complete), beamline IT input on security
**Research**: Likely (network design decisions, security patterns)
**Research topics**: REST API vs message broker vs ROS2 bridge trade-offs, security patterns for cross-network communication, deployment architecture
**Plans**: TBD

Deliverables:
- Architecture design document (approved by supervisor)
- Standalone EROBS deployment (VM or container)
- Bridge mechanism for Bluesky communication
- Security requirements met (per beamline IT)

Blockers:
- Requires beamline IT input on security constraints
- Requires supervisor approval on architecture decisions

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Communication Research | 0/TBD | Not started | - |
| 2. Vision Improvements | 0/TBD | Not started | - |
| 3. Robust Grasp Pipeline | 0/TBD | Not started | - |
| 4. Communication Implementation | 0/TBD | Not started | - |
