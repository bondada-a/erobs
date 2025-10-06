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

### [ ] 2. Hardcoded Gripper Configuration
**Location:** `pick_place_stages.cpp:10-12`
**Problem:** Only supports Hande gripper, not Epick

**Current:**
```cpp
constexpr const char* GRIPPER_GROUP = "hande_gripper";
constexpr const char* GRIPPER_OPEN_STATE = "hande_open";
constexpr const char* GRIPPER_CLOSED_STATE = "hande_closed";
```

**Action:**
- [ ] Create gripper configuration map (hande, epick)
- [ ] Make gripper states dynamic based on `step["gripper"]` value
- [ ] Add validation for unsupported grippers
- [ ] Test with both Hande and Epick

**Files to modify:**
- `src/pick_place_stages.cpp:10-12, 60-62, 92, 109, 146`

---

### [ ] 3. Missing JSON Parse Error Handling
**Location:** `base_action_server.hpp:83`
**Problem:** No try-catch around json::parse()

**Action:**
- [ ] Add try-catch around `nlohmann::json::parse(goal->poses_json)` in base_action_server.hpp
- [ ] Return proper error result on parse failure
- [ ] Ensure consistent error handling across all JSON parsing

**Files to modify:**
- `include/mtc_pipeline/base_action_server.hpp:83`

---

## 🟡 Important Issues (Fix Soon)

### [ ] 4. File Naming Consistency
**Problem:** Mixed naming conventions

**Current state:**
- ✅ `moveto_action_server.cpp`
- ✅ `end_effector_action_server.cpp`
- ❌ `pickplace_action_server.cpp` (should be `pick_place_action_server.cpp`)
- ❌ `toolexchange_action_server.cpp` (should be `tool_exchange_action_server.cpp`)

**Action:**
- [ ] Rename `src/pickplace_action_server.cpp` → `src/pick_place_action_server.cpp`
- [ ] Rename `src/toolexchange_action_server.cpp` → `src/tool_exchange_action_server.cpp`
- [ ] Update CMakeLists.txt references
- [ ] Test build

**Files to modify:**
- Rename 2 source files
- `CMakeLists.txt`

---

### [ ] 5. Action Topic Name Consistency
**Location:** `mtc_orchestrator_action_server.cpp:58-61`
**Problem:** Inconsistent naming

**Current:**
- `"moveto_action"` → should be `"move_to_action"`
- ✅ `"end_effector_action"`
- `"toolexchange_action"` → should be `"tool_exchange_action"`
- `"pickplace_action"` → should be `"pick_place_action"`

**Action:**
- [ ] Update action topic names to snake_case with underscores
- [ ] Update action server constructors to match
- [ ] Test all action communication

**Files to modify:**
- `src/mtc_orchestrator_action_server.cpp:58-61`
- `src/moveto_action_server.cpp:8`
- `src/pickplace_action_server.cpp:8`
- `src/toolexchange_action_server.cpp:8`

---

### [ ] 6. Function Naming Consistency
**Problem:** Mixed camelCase and snake_case

**Current camelCase functions:**
- `loadPlanExecute()` → `load_plan_execute()`
- `jointsFromDegrees()` → `joints_from_degrees()`
- `degToRad()` → `deg_to_rad()`
- `makePipelinePlanner()` → `make_pipeline_planner()`
- `makeCartesianPlanner()` → `make_cartesian_planner()`
- `makeJointInterpolationPlanner()` → `make_joint_interpolation_planner()`
- `makeMoveToNamedStage()` → `make_move_to_named_stage()`
- `makeGripperStage()` → `make_gripper_stage()`

**Action:**
- [ ] Rename all camelCase functions to snake_case
- [ ] Update all call sites
- [ ] Test build and functionality

**Files to modify:**
- `include/mtc_pipeline/base_stages.hpp`
- `src/base_stages.cpp`
- `include/mtc_pipeline/pick_place_stages.hpp`
- `src/pick_place_stages.cpp`
- All files calling these functions

---

### [ ] 7. JSON Field Handling Patterns
**Problem:** Three different patterns for accessing fields

**Current approaches:**
1. `.at()` - throws if missing
2. `.value("key", default)` - returns default if missing
3. `.contains()` + explicit check

**Action:**
- [ ] Define standard pattern:
  - Required fields: `.at()` with try-catch
  - Optional fields: `.value("key", default)`
  - Conditional logic: `.contains()`
- [ ] Document pattern in README or code comments
- [ ] Audit all JSON field access
- [ ] Refactor to follow pattern

**Files to audit:**
- All *_stages.cpp files
- `mtc_orchestrator_action_server.cpp`

---

### [ ] 8. Timeout Specification Style
**Problem:** Mixed styles (chrono literals vs explicit)

**Current:**
- Style 1: `wait_for_service(30s)` (needs `using namespace std::chrono_literals;`)
- Style 2: `wait_for_service(std::chrono::seconds(10))` (explicit)

**Action:**
- [ ] Choose one style (recommend explicit for no namespace pollution)
- [ ] Update all timeout specifications consistently
- [ ] Remove `using namespace std::chrono_literals;` if going explicit

**Files to modify:**
- `src/mtc_orchestrator_action_server.cpp` (multiple locations)
- `src/mtc_action_client_example.cpp`

---

## 🟢 Nice to Have (Future Improvements)

### [ ] 9. Lambda vs Method Helpers Consistency
**Problem:** Inconsistent approach

- ToolExchangeStages: Uses lambdas (lines 30-62)
- PickPlaceStages: Uses class methods (lines 34-63)
- MoveToStages: No helpers

**Action:**
- [ ] Define guideline: Use class methods for reusable/testable helpers
- [ ] Use lambdas for one-off helpers with captured state
- [ ] Refactor ToolExchangeStages to use methods if helpers are reusable
- [ ] OR refactor PickPlaceStages to use lambdas if helpers are specific

**Files to review:**
- `src/tool_exchange_stages.cpp`
- `src/pick_place_stages.cpp`
- `include/mtc_pipeline/pick_place_stages.hpp`

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

**Critical:** 1/3 complete ✅
**Important:** 0/5 complete
**Nice to Have:** 0/12 complete

**Total:** 1/20 complete (5%)

---

## Notes

- Start with Critical issues first
- Each issue can be tackled independently
- Create git commits after each completed item
- Test thoroughly before moving to next item
- Update this file as you progress
