---
phase: 1-rosbag-analysis
plan: 01
type: execute
---

<objective>
Validate the vision_accuracy_analyzer.py against actual robot behavior to establish trustworthy baselines.

Purpose: Before using analyzer metrics to define "good" performance, we must verify the analyzer is calculating correctly. Mismatched offsets, wrong frame transforms, or incorrect timing logic would invalidate all baseline measurements.

Output: Validated analyzer with documented offset values, test results confirming accuracy, and initial baseline metrics from existing rosbags.
</objective>

<execution_context>
~/.claude/get-shit-done/workflows/execute-phase.md
~/.claude/get-shit-done/templates/summary.md
</execution_context>

<context>
@.planning/PROJECT.md
@.planning/STATE.md
@.planning/phases/1-rosbag-analysis/1-CONTEXT.md
@.planning/codebase/CONCERNS.md
@.planning/codebase/TESTING.md

@recorded_bags/analysis/vision_accuracy_analyzer.py
@src/beambot/beambot/stages/vision_stages.py

**Key context from CONTEXT.md:**
- Existing analyzer shows 1.35mm mean error, 48/48 detections matched
- Y-axis has highest variance (1.14mm std)
- Some markers (4, 5, 9, 19) show 2-3mm errors vs ~0.8mm for others
- Need to validate before trusting these numbers

**Offset values to reconcile:**

| Source | ePick z_offset | Hand-E z_offset |
|--------|----------------|-----------------|
| Analyzer (GRIPPER_Z_OFFSETS) | 0.0264m | -0.02m |
| vision_stages.py (_detect_current_gripper) | 0.023m | -0.02m |
| vision_stages.py (fallback if "epick" in name) | 0.1m | -0.02m |
| Analyzer (TOOL_TO_TIP_OFFSETS) | 0.203m | 0.145m |

**Frame transform issue:**
- TCP pose broadcaster publishes in 'base' frame
- ArUco detections are in 'base_link' frame
- Analyzer applies 180° Z rotation (x_base_link = -x_base, y_base_link = -y_base)
- Need to verify this matches actual TF relationship
</context>

<tasks>

<task type="auto">
  <name>Task 1: Verify TF frame relationships</name>
  <files>recorded_bags/analysis/vision_accuracy_analyzer.py</files>
  <action>
  Run a quick TF inspection on an existing rosbag to verify the base → base_link relationship:

  1. Use rosbags library to extract /tf_static for base → base_link transform
  2. Confirm the 180° Z rotation assumption in the analyzer's _load_tcp_poses method
  3. Document the actual transform in a comment in the analyzer

  If the transform differs from assumed 180° Z rotation, update the analyzer's frame conversion logic.

  Verification approach: Extract a few TCP poses and ArUco detections at the same timestamp, manually verify they align when the frame transform is applied.
  </action>
  <verify>Run analyzer on a test bag and check that detection coordinates and TCP coordinates are in the same reference frame (positions should be comparable, not mirrored or offset by large amounts)</verify>
  <done>TF relationship documented in analyzer comments; if transform was wrong, it's now fixed</done>
</task>

<task type="auto">
  <name>Task 2: Reconcile z_offset values with vision_stages.py</name>
  <files>recorded_bags/analysis/vision_accuracy_analyzer.py, src/beambot/beambot/stages/vision_stages.py</files>
  <action>
  Update the analyzer's offset constants to match the actual values used in vision_stages.py:

  1. Extract the ACTUAL z_offset values from vision_stages.py:
     - ePick: 0.023m (from _detect_current_gripper, line 1203)
     - Hand-E: -0.02m (from _detect_current_gripper, line 1212)

  2. Update GRIPPER_Z_OFFSETS dict in analyzer:
     - 'epick': 0.023 (was 0.0264)
     - 'hande': -0.02 (unchanged)

  3. Add a comment explaining these values match vision_stages.py:_detect_current_gripper()

  4. For TOOL_TO_TIP_OFFSETS, verify by inspecting the static TF in a rosbag:
     - Extract tool0 → epick_tip transform from /tf_static
     - Compare with hardcoded 0.203m value
     - Update if different

  IMPORTANT: Do NOT change SAMPLE_OFFSET_X/Y values (0.02, 0.0) - these already match vision_stages.py lines 983-984.
  </action>
  <verify>Grep analyzer for GRIPPER_Z_OFFSETS and verify values match vision_stages.py</verify>
  <done>Analyzer z_offset values match production code exactly; discrepancies documented</done>
</task>

<task type="auto">
  <name>Task 3: Run validated analyzer and establish baseline</name>
  <files>recorded_bags/analysis/vision_accuracy_analyzer.py</files>
  <action>
  Run the now-validated analyzer on available rosbags to establish baseline metrics:

  1. Find available rosbags:
     ```bash
     ls -la recorded_bags/data/
     ```

  2. Run analyzer on the most relevant bag (sample_to_spincoat or similar vision operation):
     ```bash
     cd recorded_bags/analysis
     python vision_accuracy_analyzer.py ../data/[bag_name] --gripper epick
     ```

  3. Capture the output:
     - Mean position error (mm)
     - Per-axis breakdown (X, Y, Z)
     - Per-marker statistics
     - Orientation error

  4. Document in a new file recorded_bags/analysis/BASELINE.md:
     - Date of baseline
     - Rosbag used
     - Key metrics
     - Interpretation (what do these numbers mean for the system?)

  5. Compare with previous results (1.35mm mean) - note if validation changes significantly affected measurements.
  </action>
  <verify>BASELINE.md exists with documented metrics; analyzer runs without errors</verify>
  <done>Quantified baseline established; metrics documented; interpretation provided</done>
</task>

</tasks>

<verification>
Before declaring plan complete:
- [ ] Analyzer's frame transform logic verified against actual TF data
- [ ] GRIPPER_Z_OFFSETS values match vision_stages.py exactly
- [ ] TOOL_TO_TIP_OFFSETS verified against static TF chain
- [ ] Analyzer runs successfully on at least one rosbag
- [ ] BASELINE.md created with documented metrics
- [ ] No Python errors or warnings during analyzer execution
</verification>

<success_criteria>
- Analyzer offset values reconciled with vision_stages.py
- Frame transform logic documented and verified correct
- Baseline metrics established from rosbag analysis
- Results documented in BASELINE.md with interpretation
</success_criteria>

<output>
After completion, create `.planning/phases/1-rosbag-analysis/1-01-SUMMARY.md` using the summary template:

# Phase 1 Plan 01: Analyzer Validation Summary

**[What changed in the analyzer and what baseline was established]**

## Accomplishments
- [Frame transform verification results]
- [Offset reconciliation details]
- [Baseline metrics]

## Files Created/Modified
- `recorded_bags/analysis/vision_accuracy_analyzer.py` - Updated offsets
- `recorded_bags/analysis/BASELINE.md` - Baseline documentation

## Decisions Made
- [Any offset value choices or interpretations]

## Issues Encountered
- [Any discrepancies found between analyzer and production code]

## Next Phase Readiness
- [Ready for Phase 1 Plan 02 or Phase 2]
</output>
