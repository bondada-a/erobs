# Roadmap: EROBS

## Overview

Evolve EROBS from a local development prototype to a production-ready, standalone robotic framework that integrates with any beamline's existing Bluesky infrastructure.

**Key insight:** DSSI will build the Bluesky ↔ EROBS communication bridge. Our job is to prepare clean containers and documentation so they can do their work.

## Current Status

- ✅ Communication Research — completed (see `phases/01-communication-research/01-RESEARCH.md`)
- 🔄 Codebase cleanup — in progress on `refactor/codebase-cleanup` branch

## Phases

- [x] **Phase 0: Communication Research** — ✅ DONE — Deep understanding of Bluesky → EROBS path
- [ ] **Phase 1: DSSI Handoff Prep** — Clean containers + documentation for DSSI
- [ ] **Phase 2: Vision Improvements** — Reliable tagless detection with stable labeling
- [ ] **Phase 3: Robust Grasp Pipeline** — MTC-based grasp generation with multi-candidate planning
- [ ] **Phase 4: Integration Testing** — Test bridge once DSSI delivers it

## Phase Details

### Phase 0: Communication Research ✅ DONE
**Status**: Complete (2026-01-28)

Deliverables (in `phases/01-communication-research/01-RESEARCH.md`):
- Architecture diagram showing message flow from Bluesky to robot
- Ophyd device wrapping ROS2 ActionClient patterns
- DDS networking between containers
- Common pitfalls and solutions
- Open questions for Phase 4

### Phase 1: DSSI Handoff Prep
**Goal**: Prepare containers and documentation so DSSI can build the communication bridge
**Depends on**: Phase 0 (done)
**Priority**: HIGH — This is the blocker for Bluesky integration

**Context:**
- Current approach: DSSI group will create communication API between Bluesky (beamline network) and EROBS (robot network)
- Rocky's role: Provide clean container images + clear documentation
- Ball is in our court to unblock DSSI

Deliverables:
- [ ] Clean up container images (remove debug cruft, update base images)
- [ ] Fix critical bugs found in codebase review (see CLEANUP.md)
- [ ] Update old repo URLs in Dockerfiles (`nsls2/erobs` → `bondada-a/erobs`)
- [ ] Write container documentation (what each container does, how to run)
- [ ] Update README with current architecture
- [ ] Document the communication interface EROBS exposes (action servers, topics)
- [ ] Test containers run successfully on ws2

Key files:
- `docker/erobs-common-img/` — Main container
- `docker/bsui/` — Bluesky container (reference for DSSI)
- `CLEANUP.md` — Issues to fix

Blockers:
- None (we control this)

### Phase 2: Vision Improvements
**Goal**: Reliable tagless detection with stable labeling and accurate centroid positioning
**Depends on**: Can run in parallel with Phase 1
**Priority**: MEDIUM — Improves reliability but not blocking integration

Deliverables:
- [ ] Improved circle/contour detection reliability
- [ ] Stable sample labeling between captures
- [ ] Accurate centroid positioning for grasps
- [ ] Optional: ML-based detection (YOLOv8) evaluation

Key files:
- `src/beambot/beambot/camera/zivid.py` — Detection methods
- `src/beambot/beambot/stages/vision_stages.py` — Vision stage composition

### Phase 3: Robust Grasp Pipeline
**Goal**: MTC-based grasp generation with multi-candidate planning and path constraints
**Depends on**: Phase 2 (vision provides accurate sample positions)
**Priority**: MEDIUM — Enhancement after core reliability

Research topics:
- MTC GenerateGraspPose stages
- IK solver tuning
- OMPL planner parameters
- Path constraints ("keep level" for sample transfers)

Deliverables:
- [ ] GenerateGraspPose stages integrated into pick-and-place
- [ ] Multi-candidate grasp planning (fast query, fail fast pattern)
- [ ] "Keep level" path constraints for sample transfers
- [ ] Vision integration with grasp pipeline

Key files:
- `src/beambot/beambot/stages/base_stages.py` — MTC utilities
- `src/beambot/beambot/stages/pick_place_stages.py` — Pick/place composition

### Phase 4: Integration Testing
**Goal**: Test the bridge once DSSI delivers it, ensure end-to-end Bluesky → Robot works
**Depends on**: Phase 1 (our prep), DSSI delivering the bridge
**Priority**: HIGH (when unblocked)

Deliverables:
- [ ] Test DSSI's bridge mechanism with EROBS
- [ ] Verify Bluesky plans can control robot from beamline network
- [ ] Document deployment architecture
- [ ] Security requirements met (per beamline IT)
- [ ] Handoff to beamline staff

Blockers:
- Waiting on DSSI to build the bridge (unblocked by Phase 1)
- Beamline IT input on security constraints

## Progress

| Phase | Status | Notes |
|-------|--------|-------|
| 0. Communication Research | ✅ Done | Research complete, documented |
| 1. DSSI Handoff Prep | 🔄 In Progress | Codebase cleanup on branch |
| 2. Vision Improvements | Not started | Can parallelize |
| 3. Robust Grasp Pipeline | Not started | After vision |
| 4. Integration Testing | Blocked | Waiting on DSSI |

## Related Work (Not in Roadmap)

These are tracked elsewhere but related:
- **Paper writing** — Overleaf with Esther
- **Summer intern proposal** — Due before Feb 4
- **Isaac Sim integration** — Future exploration
- **VLA research** — Potential paper topic

---
*Last updated: 2026-01-31 — Restructured based on actual priorities and DSSI dependency*
