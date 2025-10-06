# MTC Pipeline Consistency TODO

## 🔴 Critical Issues (Fix First)

### [X] 1. PickPlace JSON Structure Mismatch ✅
**Location:** `pickplace_action_server.cpp:18-19` vs orchestrator
**Problem:**
- Action server creates arrays: `step["pick_poses"] = {goal.pick_approach, goal.pick_target}`
- Orchestrator sends individual fields: `pick_approach`, `pick_target`, etc.
- This creates unnecessary transformation layer

**Action:**
- [X] Decide on format: individual fields ✅
- [X] Update action server to pass individual fields (no array conversion) ✅
- [X] Update pick_place_stages.cpp to expect individual fields ✅
- [X] Build successful ✅
- [ ] Test pick_place operations (pending manual test)

**Files modified:**
- `src/pickplace_action_server.cpp:17-21` - Removed array conversion
- `src/pick_place_stages.cpp:68-80, 96-150` - Changed to use individual fields

---

### [DEFERRED] 2. Hardcoded Gripper Configuration ⏸️
**Location:** `pick_place_stages.cpp:10-12`
**Problem:** Only supports Hande gripper, not Epick

**Current:**
```cpp
constexpr const char* GRIPPER_GROUP = "hande_gripper";
constexpr const char* GRIPPER_OPEN_STATE = "hande_open";
constexpr const char* GRIPPER_CLOSED_STATE = "hande_closed";
```

**Status:** Deferred - User will add Epick support later when needed

**Action (when ready):**
- [ ] Create gripper configuration map (hande, epick)
- [ ] Make gripper states dynamic based on `step["gripper"]` value
- [ ] Add validation for unsupported grippers
- [ ] Test with both Hande and Epick

**Files to modify:**
- `src/pick_place_stages.cpp:10-12, 60-62, 92, 109, 146`

---

### [X] 3. Missing JSON Parse Error Handling ✅
**Location:** `base_action_server.hpp:83`
**Problem:** No explicit try-catch around json::parse()

**Action:**
- [X] Add explicit try-catch around `nlohmann::json::parse(goal->poses_json)` ✅
- [X] Return proper error result on parse failure (abort with error message) ✅
- [X] Improved error message from "Execution failed" to "Stage execution failed" ✅
- [X] Build successful ✅

**Files modified:**
- `include/mtc_pipeline/base_action_server.hpp:71-115` - Added nested try-catch for JSON parsing with specific error handling

---

## 🟡 Important Issues (Fix Soon)

### [X] 4. File Naming Consistency ✅
**Problem:** Mixed naming conventions

**Action:**
- [X] Renamed `src/pickplace_action_server.cpp` → `src/pick_place_action_server.cpp` ✅
- [X] Renamed `src/toolexchange_action_server.cpp` → `src/tool_exchange_action_server.cpp` ✅
- [X] Updated all CMakeLists.txt references (executable names, dependencies, linking, features, install) ✅
- [X] Build successful ✅
- [X] Verified executable names in install directory ✅

**Files modified:**
- `src/pickplace_action_server.cpp` → `src/pick_place_action_server.cpp` (renamed)
- `src/toolexchange_action_server.cpp` → `src/tool_exchange_action_server.cpp` (renamed)
- `CMakeLists.txt` lines 67-68, 78-79, 86-87, 93-94, 101-102 - Updated all references

**Result:**
All action server files now use consistent snake_case naming:
- ✅ `end_effector_action_server`
- ✅ `moveto_action_server`
- ✅ `pick_place_action_server`
- ✅ `tool_exchange_action_server`

---

### [X] 5. Complete Naming Consistency (Files, Nodes, Topics) ✅
**Problem:** Inconsistent naming across files, node names, and topic names

**Action:**
- [X] Renamed `src/moveto_action_server.cpp` → `src/move_to_action_server.cpp` ✅
- [X] Renamed `src/moveto_stages.cpp` → `src/move_to_stages.cpp` ✅
- [X] Renamed `include/mtc_pipeline/moveto_stages.hpp` → `include/mtc_pipeline/move_to_stages.hpp` ✅
- [X] Updated CMakeLists.txt library sources (line 46) ✅
- [X] Updated CMakeLists.txt executable (line 69) ✅
- [X] Updated CMakeLists.txt dependencies (line 80) ✅
- [X] Updated CMakeLists.txt linking (line 88) ✅
- [X] Updated CMakeLists.txt features (line 95) ✅
- [X] Updated CMakeLists.txt install (line 103) ✅
- [X] Updated #include in move_to_action_server.cpp ✅
- [X] Updated #include in move_to_stages.cpp ✅
- [X] Updated all topic names in orchestrator (line 58-61) ✅
- [X] Updated all node names and topic names in action servers ✅
- [X] Clean build successful ✅

**Files modified:**
- `src/moveto_action_server.cpp` → `src/move_to_action_server.cpp` (renamed, updated include, node name, topic name)
- `src/moveto_stages.cpp` → `src/move_to_stages.cpp` (renamed, updated include)
- `include/mtc_pipeline/moveto_stages.hpp` → `include/mtc_pipeline/move_to_stages.hpp` (renamed)
- `src/pick_place_action_server.cpp:8` - Updated node name and topic name
- `src/tool_exchange_action_server.cpp:8` - Updated node name and topic name
- `src/mtc_orchestrator_action_server.cpp:58-61` - Updated 3 topic names
- `CMakeLists.txt` - Updated 6 locations
- `launch/modular_action_servers.launch.py` - Updated all executable and node names (lines 34-56, 80-84)

**Result - All naming is now consistent with snake_case using underscores:**

**Executables:**
- ✅ `end_effector_action_server`
- ✅ `move_to_action_server`
- ✅ `pick_place_action_server`
- ✅ `tool_exchange_action_server`

**Node names:**
- ✅ `"end_effector_action_server"`
- ✅ `"move_to_action_server"`
- ✅ `"pick_place_action_server"`
- ✅ `"tool_exchange_action_server"`

**Topic names:**
- ✅ `"end_effector_action"`
- ✅ `"move_to_action"`
- ✅ `"pick_place_action"`
- ✅ `"tool_exchange_action"`

---

### [X] 6. Function Naming Consistency ✅
**Problem:** Mixed camelCase and snake_case

**Action:**
- [X] Renamed all 11 functions in base_stages.hpp ✅
- [X] Renamed 2 functions in pick_place_stages.hpp ✅
- [X] Updated all implementations in base_stages.cpp ✅
- [X] Updated all implementations in pick_place_stages.cpp ✅
- [X] Updated all call sites in pick_place_stages.cpp ✅
- [X] Updated all call sites in move_to_stages.cpp ✅
- [X] Updated all call sites in tool_exchange_stages.cpp ✅
- [X] Updated all call sites in end_effector_stages.cpp ✅
- [X] Build successful ✅

**Functions renamed (13 total):**

**base_stages.hpp/cpp:**
- `createTaskTemplate()` → `create_task_template()`
- `loadPlanExecute()` → `load_plan_execute()`
- `jointsFromDegrees()` → `joints_from_degrees()`
- `defaultJointNames()` → `default_joint_names()`
- `defaultArmGroupName()` → `default_arm_group_name()`
- `defaultIkFrame()` → `default_ik_frame()`
- `degToRad()` → `deg_to_rad()`
- `makePipelinePlanner()` → `make_pipeline_planner()`
- `makeCartesianPlanner()` → `make_cartesian_planner()`
- `makeJointInterpolationPlanner()` → `make_joint_interpolation_planner()`
- `createRelativeMoveStage()` → `create_relative_move_stage()`

**pick_place_stages.hpp/cpp:**
- `makeMoveToNamedStage()` → `make_move_to_named_stage()`
- `makeGripperStage()` → `make_gripper_stage()`

**Files modified:**
- `include/mtc_pipeline/base_stages.hpp` - 11 function declarations
- `src/base_stages.cpp` - 11 function implementations
- `include/mtc_pipeline/pick_place_stages.hpp` - 2 function declarations
- `src/pick_place_stages.cpp` - 2 implementations + 13 call sites
- `src/move_to_stages.cpp` - 7 call sites
- `src/tool_exchange_stages.cpp` - 6 call sites
- `src/end_effector_stages.cpp` - 3 call sites

**Result:** All functions now use consistent snake_case naming

---

### [REVIEWED] 7. JSON Field Handling Patterns ✅ No Changes Needed
**Problem:** Three different patterns for accessing fields

**Analysis:** After review, the three patterns are being used **correctly** for their specific purposes:

1. **`.at()` - for REQUIRED fields** - Throws exception if missing
   - Example: `step.at("operation")`, `step.at("end_effector_type")`
   - Purpose: Fail fast on configuration errors ✅

2. **`.value("key", default)` - for OPTIONAL fields** - Returns default if missing
   - Example: `step.value("planning_type", "joint")`, `step.value("return_home", true)`
   - Purpose: Provide sensible defaults ✅

3. **`.contains()` - for CONDITIONAL logic** - Check existence to determine code path
   - Example: `if (step.contains("direction") && step.contains("distance"))`
   - Purpose: Branch between alternative execution paths ✅

**Decision:** This is **NOT an inconsistency** - it's good design! Each pattern serves a different purpose. No changes needed.

**Status:** Reviewed and approved as correct usage

---

### [X] 8. Timeout Specification Style ✅
**Problem:** Mixed styles (chrono literals vs explicit)

**Current:**
- Style 1: `wait_for_service(30s)` (needs `using namespace std::chrono_literals;`)
- Style 2: `wait_for_service(std::chrono::seconds(10))` (explicit)

**Action:**
- [X] Choose one style (decided: use chrono literals in .cpp only) ✅
- [X] Update all timeout specifications consistently ✅
- [X] Move `using namespace std::chrono_literals;` from header to .cpp files ✅
- [X] Build successful ✅

**Decision:** Use chrono literals (Style 1) with namespace declaration in .cpp files only (not headers) for:
- More readable code (30s vs std::chrono::seconds(30))
- Modern C++ idiom (C++14 standard)
- No namespace pollution (only in .cpp files)

**Files modified:**
- `include/mtc_pipeline/mtc_orchestrator_action_server.hpp` - Removed namespace from header
- `src/mtc_orchestrator_action_server.cpp` - Added namespace, ready for literals
- `src/mtc_action_client_example.cpp` - Added namespace and converted 6 timeout specifications

---

## 🟢 Nice to Have (Future Improvements)

### [X] 9. Lambda vs Method Helpers Consistency ✅
**Problem:** Inconsistent approach - ToolExchangeStages had redundant lambdas

- ToolExchangeStages: Used lambdas that just wrapped existing functions
- PickPlaceStages: Uses class methods (kept as-is, they're actually reusable)
- MoveToStages: No helpers (correct - uses inline code)

**Action:**
- [X] Removed all 3 redundant lambdas from ToolExchangeStages ✅
- [X] Used `create_relative_move_stage()` directly instead of `addRelativeMoveStage` lambda ✅
- [X] Used inline MoveTo stage creation instead of `addNamedMoveStage` lambda (same pattern as move_to_stages.cpp) ✅
- [X] Removed `addDockShiftStage` lambda - logic now inline ✅
- [X] Build successful ✅

**Decision:**
- Use existing base class functions directly (no unnecessary wrappers)
- Inline simple logic instead of lambdas when not capturing complex state
- Keep PickPlaceStages methods as they provide actual reusable abstractions

**Files modified:**
- `src/tool_exchange_stages.cpp` - Removed 3 lambdas (lines 30-62), simplified code by ~30 lines

---

### [ ] 10. Constant Definition Style
**Problem:** Four different styles

**Approaches:**
1. Anonymous namespace with constexpr
2. Anonymous namespace with const + complex types
3. Static class members
4. Inline magic numbers

**Action:**
- [ ] Standardize on:
  - Anonymous namespace for file-scoped constants
  - Static class members for class-specific config
  - No inline magic numbers
- [ ] Extract all magic numbers to named constants
- [ ] Refactor existing constants to follow pattern

**Files to review:**
- `src/pick_place_stages.cpp:9-16`
- `src/base_stages.cpp:17-28`
- `include/mtc_pipeline/base_stages.hpp:19-35`
- `src/mtc_orchestrator_action_server.cpp` (magic numbers)

---

### [ ] 11. Error Message Consistency
**Problem:** Some detailed, some generic

**Action:**
- [ ] Define error message template: "Failed to {action}: {reason}"
- [ ] Always include context (what failed, why, expected)
- [ ] Update generic messages to be specific
- [ ] Ensure error_message field in results is populated

**Files to audit:**
- `include/mtc_pipeline/base_action_server.hpp:90`
- `src/mtc_orchestrator_action_server.cpp` (multiple locations)

---

### [ ] 12. Validation Location
**Problem:** Validation scattered between orchestrator and stages

**Action:**
- [ ] Define validation policy:
  - Orchestrator validates top-level structure
  - Action servers validate action-specific fields
  - Stages assume valid input or fail fast
- [ ] Consolidate validation logic
- [ ] Document validation boundaries

**Files to review:**
- `src/mtc_orchestrator_action_server.cpp:106-153`
- All *_stages.cpp files

---

### [ ] 13. Public Helper Methods Design
**Problem:** PickPlaceStages exposes helpers, others don't

**Action:**
- [ ] Decide if helpers should be public (for reuse) or private (implementation detail)
- [ ] Make PickPlaceStages helpers private if not needed externally
- [ ] Document public helper methods if they are part of the API

**Files to review:**
- `include/mtc_pipeline/pick_place_stages.hpp:21-33`

---

### [ ] 14. Namespace Alias Consistency
**Problem:** Redefined in multiple files

**Current:**
- base_stages.hpp defines: `namespace mtc = moveit::task_constructor;`
- Most .cpp files redefine it

**Action:**
- [ ] Keep alias in base_stages.hpp
- [ ] Remove redundant definitions in derived classes
- [ ] Verify all derived classes include base_stages.hpp

**Files to modify:**
- `src/moveto_stages.cpp:4`
- `src/pick_place_stages.cpp:7`
- `src/tool_exchange_stages.cpp:6`
- `src/end_effector_stages.cpp:5`

---

### [ ] 15. Section Comment Style
**Problem:** Three different styles

**Styles:**
1. `// === SECTION NAME ===`
2. `// ============================================================================`
3. Minimal or none

**Action:**
- [ ] Choose one style (recommend: `// ============================================================================`)
- [ ] Apply consistently across all files
- [ ] Add section comments where missing

**Files to update:**
- All source files

---

### [ ] 16. Function Documentation (Doxygen)
**Problem:** No function documentation

**Action:**
- [ ] Add Doxygen comments to all public methods
- [ ] Include @brief, @param, @return
- [ ] Generate documentation to verify

**Files to document:**
- All header files in `include/mtc_pipeline/`
- Focus on public API first

---

### [ ] 17. Planning Type Configuration
**Problem:** Not all actions support planning_type

**Has planning_type:**
- MoveToAction
- PickPlaceAction

**Missing:**
- ToolExchangeAction (hardcoded pipeline + cartesian)
- EndEffectorAction (hardcoded joint interpolation)

**Action:**
- [ ] Decide if all actions should support planning_type
- [ ] Add parameter if needed OR document why it's hardcoded
- [ ] Update action definitions

**Files to review:**
- `action/ToolExchangeAction.action`
- `action/EndEffectorAction.action`
- `src/tool_exchange_stages.cpp`
- `src/end_effector_stages.cpp`

---

### [ ] 18. Return Home Feature
**Problem:** Only PickPlaceStages has optional return_home

**Action:**
- [ ] Decide if this should be universal in BaseStages
- [ ] OR handle at orchestrator level for all actions
- [ ] OR remove if not needed consistently

**Files to review:**
- `src/pick_place_stages.cpp:158-164`

---

### [ ] 19. Architecture Documentation
**Problem:** Orchestrator doesn't use BaseActionServer (by design)

**Action:**
- [ ] Document why orchestrator is different (manages processes, calls other actions)
- [ ] Add architecture diagram or README explaining design
- [ ] Clarify BaseActionServer is for modular actions only

**Files to document:**
- Create `architecture.md` or add to README
- Add comment in `mtc_orchestrator_action_server.hpp`

---

### [ ] 20. Process Management Extraction
**Problem:** SimpleProcessManager embedded in .cpp file

**Action:**
- [ ] Decide if it should be:
  - In header (for potential reuse)
  - Separate file (better organization)
  - Stay embedded (orchestrator-specific)
- [ ] Add comment explaining decision
- [ ] Extract if reusable

**Files to review:**
- `src/mtc_orchestrator_action_server.cpp:3-42`

---

## Testing Checklist

After each fix:
- [ ] Code compiles (`colcon build --packages-select mtc_pipeline`)
- [ ] No new warnings
- [ ] Existing tests pass (if any)
- [ ] Manual test of affected functionality
- [ ] Update this checklist

---

## Progress Tracking

**Critical:** 2/3 complete ✅ (1 deferred)
**Important:** 4/5 complete ✅
**Nice to Have:** 1/12 complete ✅

**Total:** 7/20 complete (35%) + 1 deferred

---

## Notes

- Start with Critical issues first
- Each issue can be tackled independently
- Create git commits after each completed item
- Test thoroughly before moving to next item
- Update this file as you progress
