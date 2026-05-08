# Documentation Audit
*Date: 2026-03-06 | Branch: humble-experimental*

## Summary

Audited all 50+ markdown files across the repo. Found significant staleness in `.planning/` docs (last updated Jan 2026, reference non-existent branches), several conflicts between docs, and missing documentation for the MCP approach that is now the primary interaction model.

---

## 1. Root-Level Documentation

### README.md
- **Status**: PARTIALLY STALE
- **Issues**:
  - References "Pixi" as the package manager but pixi.toml has limited use (just pre-commit/ruff/bluesky deps)
  - Build instructions reference `pixi run build` but Docker is the actual build path
  - Missing any mention of MCP (Model Context Protocol) which is now the primary interaction model
  - Architecture diagram shows Bluesky as the top-level orchestrator, but MCP/Claude Code is now the primary interface
  - No mention of `start_mcp.sh` or `erobs_mcp_server.py`

### LICENSE / LICENSE_README
- **Status**: OK - BSD-3-Clause, standard BNL boilerplate

### .doecode.yml
- **Status**: OK - DOE code registration metadata

---

## 2. docs/ Directory

### docs/mcp_architecture_design.md
- **Status**: CURRENT - Detailed MCP architecture design
- **Notes**: This is the most up-to-date architectural doc, describes the two-server MCP approach (ros-mcp-server + erobs custom server)

### docs/mcp_ros_reference.md
- **Status**: CURRENT - Reference doc for ros-mcp-server tools
- **Notes**: Comprehensive tool reference, recently added

### docs/development_notes.md
- **Status**: CURRENT - Development notes including MCP/cartesian goal work

### docs/Container_documentation.md
- **Status**: PARTIALLY STALE
- **Issues**:
  - Still references older container setup patterns
  - Should include MCP server container integration
  - Some paths may be outdated

### docs/ArUco_detection.md
- **Status**: STALE
- **Issues**:
  - Brief doc from early development
  - Doesn't reflect current Zivid-native ArUco detection or the multi-method detection (circle/contour/HSV)
  - Should reference `erobs_mcp_server.py` detect_objects tool

### docs/State_flow.md
- **Status**: STALE
- **Issues**:
  - References old C++ implementation state machine
  - Doesn't reflect current Python orchestrator or MCP interaction flow
  - Should be updated or removed

### docs/ws2-container-setup.md
- **Status**: STALE
- **Issues**:
  - References specific ws2 workstation setup
  - Some Docker commands may be outdated

### docs/Robustness_tests.md
- **Status**: STALE
- **Issues**:
  - References tests from early development
  - Doesn't reflect current test infrastructure

### docs/beamline_config_diagram.tex/.pdf
- **Status**: OK - Architecture diagram for beamline config system

### docs/architecture_diagram.tex/.pdf
- **Status**: PARTIALLY STALE - Pre-MCP architecture, still shows old flow

### docs/current_blockers.tex/.pdf
- **Status**: LIKELY STALE - Dated blockers document

### docs/epics_ros_bridge_architecture.tex/.pdf
- **Status**: OK - Reference architecture for EPICS bridge (future work)

---

## 3. .planning/ Directory

### .planning/PROJECT.md
- **Status**: STALE (last updated 2026-01-28)
- **Issues**:
  - "Active" section lists items now completed (MCP approach supersedes DSSI bridge)
  - "Known Issues" mentions "orchestrator.py: 1108 lines" but orchestrator is now 1063 lines (cleaned up)
  - Doesn't mention MCP at all
  - References `refactor/codebase-cleanup` branch which doesn't exist on current remote

### .planning/ROADMAP.md
- **Status**: STALE (last updated 2026-01-31)
- **Issues**:
  - Phase 1 "DSSI Handoff Prep" may be superseded by MCP approach
  - References `refactor/codebase-cleanup` branch
  - Phase 4 "Integration Testing" blocked on DSSI - but MCP approach may change this
  - Doesn't mention MCP-based interaction model at all

### .planning/STATE.md
- **Status**: STALE (last updated 2026-01-31)
- **Issues**:
  - "Current focus: Phase 1 - DSSI Handoff Prep"
  - Active branch listed as `refactor/codebase-cleanup` (doesn't exist)
  - 20% progress estimate is outdated
  - Should reflect MCP work on `humble-experimental`

### .planning/CODEBASE_NOTES.md
- **Status**: PARTIALLY STALE
- **Issues**:
  - Contains useful architectural analysis but from pre-MCP era
  - References C++ orchestrator which no longer exists
  - Some file path references may be outdated

### .planning/CLEANUP.md
- **Status**: PARTIALLY STALE
- **Issues**:
  - Many cleanup items may already be addressed
  - Doesn't reflect MCP-era priorities

### .planning/REVIEW_CONFIGS.md
- **Status**: STALE
- **Issues**:
  - Pre-MCP review of config files
  - Some recommendations may already be implemented

### .planning/REVIEW_BUILD_DEPLOY.md
- **Status**: STALE
- **Issues**:
  - Pre-MCP build/deploy review
  - Docker workflow may have changed

### .planning/REVIEW_INTERFACES.md
- **Status**: STALE
- **Issues**:
  - Comprehensive interface review but pre-MCP
  - Doesn't include MCP tool interfaces

### .planning/REVIEW_ROBOT_DESCRIPTIONS.md
- **Status**: STALE
- **Issues**:
  - URDF/XACRO review from Jan 2026
  - May still be relevant for physical setup

### .planning/REVIEW_DOCUMENTATION.md
- **Status**: STALE (meta-review of docs from Jan 2026)

### .planning/codebase/ARCHITECTURE.md
- **Status**: STALE - Pre-MCP architecture description

### .planning/codebase/STRUCTURE.md
- **Status**: PARTIALLY STALE - Package structure still valid but missing MCP

### .planning/codebase/STACK.md
- **Status**: OK - Technology stack description still mostly valid

### .planning/codebase/INTEGRATIONS.md
- **Status**: STALE - Missing MCP integration

### .planning/codebase/CONCERNS.md
- **Status**: STALE - Some concerns may be resolved

### .planning/codebase/CONVENTIONS.md
- **Status**: OK - Coding conventions still apply

### .planning/codebase/TESTING.md
- **Status**: STALE - Testing strategy needs MCP-era update

### .planning/config.json
- **Status**: OK - Planning configuration

### .planning/phases/01-communication-research/01-RESEARCH.md
- **Status**: STALE but HISTORICAL - Research completed, may be superseded by MCP

---

## 4. Package READMEs

### src/beambot/ (no README)
- **Status**: MISSING
- **Issue**: Main package has no README. Should document architecture, action servers, MCP server

### src/bluesky_ros/README.md
- **Status**: STALE
- **Issues**:
  - References old Bluesky integration approach
  - Doesn't mention MCP-based alternative

### src/bluesky_ros/COMMAND_REFERENCE.md
- **Status**: OK - Reference for Bluesky commands

### src/mtc_gui/README.md
- **Status**: OK - GUI documentation

### src/aruco_pose/README.md
- **Status**: STALE - Old ArUco detection package, may not be in active use

### src/vision/README.md
- **Status**: OK - Vision subsystem repos reference

### src/end_effectors/README.md
- **Status**: OK - End effector repos reference

### src/pdf/pdf_beamtime/README.md
- **Status**: STALE - PDF beamline C++ implementation, superseded by beambot Python

### src/demos/hello_orchestrator/README.md
- **Status**: OK - Demo/tutorial

### src/demos/hello_orchestrator_py/README.md
- **Status**: OK - Demo/tutorial

### src/custom-ur-descriptions/*/README.md
- **Status**: OK - URDF/MoveIt config documentation

---

## 5. Docker READMEs

### docker/README.md
- **Status**: PARTIALLY STALE - Overview of Docker setup

### docker/erobs-common-img/README.md
- **Status**: PARTIALLY STALE - Main container docs

### docker/bsui-minimal/README.md
- **Status**: OK - Minimal Bluesky container

### docker/azure-kinect/README.md
- **Status**: STALE - Azure Kinect no longer in active use

### docker/archive/erobs-common-img/README.md
- **Status**: ARCHIVED - Old container docs

### scripts/pdf-launch-scripts/README.md
- **Status**: STALE - PDF beamline launch scripts

---

## 6. Master CLAUDE.md

### /root/clawd/erobs-claude-config/CLAUDE.md
- **Status**: CURRENT and COMPREHENSIVE
- **Notes**: Well-maintained, includes architecture, task format, calibration history, current work items
- **Minor issues**:
  - References "Step 1: Pre-filtering" at the end but cuts off (appears truncated)
  - Some work items in "Current Work" may be completed
  - Missing documentation for MCP tools (erobs_mcp_server.py capabilities)

---

## 7. Cross-Document Conflicts

| Conflict | Files | Description |
|----------|-------|-------------|
| Architecture mismatch | README.md vs docs/mcp_architecture_design.md | README shows Bluesky-first architecture; MCP doc shows Claude Code as primary interface |
| Branch references | .planning/STATE.md, ROADMAP.md | Reference `refactor/codebase-cleanup` branch that doesn't exist; current work is on `humble-experimental` |
| DSSI vs MCP approach | .planning/ROADMAP.md vs docs/mcp_architecture_design.md | Roadmap focuses on DSSI handoff; MCP design suggests different integration path |
| Orchestrator file size | .planning/PROJECT.md | Claims "1108 lines" but current file is 1063 lines |
| Detection methods | docs/ArUco_detection.md vs erobs_mcp_server.py | Old doc only covers ArUco; MCP server supports HSV color, circles, contours, ArUco |

---

## 8. Missing Documentation

| Topic | Priority | Notes |
|-------|----------|-------|
| MCP server tools reference | HIGH | erobs_mcp_server.py tools (capture_image, detect_objects, get_tf_transform) need user-facing docs |
| End-to-end MCP workflow | HIGH | How to use Claude Code + MCP to interact with the robot |
| beambot package README | MEDIUM | Main package lacks any README |
| Updated architecture diagram | MEDIUM | Current diagram is pre-MCP |
| Hardware capabilities doc | LOW | What Zivid/HandE/ePick can do beyond current use |
| Test documentation | LOW | What tests exist, how to run them |

---

## Recommendations

1. **HIGH**: Update README.md to reflect MCP-first architecture
2. **HIGH**: Archive or update .planning/ docs with MCP-era status
3. **HIGH**: Create MCP tools reference documentation
4. **MEDIUM**: Add beambot package README
5. **MEDIUM**: Update or remove stale docs (ArUco_detection.md, State_flow.md, Robustness_tests.md)
6. **LOW**: Update Docker documentation for MCP server integration
