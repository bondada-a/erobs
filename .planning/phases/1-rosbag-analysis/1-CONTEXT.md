# Phase 1: Rosbag Analysis & Baseline Validation - Context

**Gathered:** 2026-01-17
**Status:** Ready for planning

<vision>
## How This Should Work

Analyze existing rosbags to understand where issues come from before adding new logging. The workflow is simple: **record bag → run script → get report**.

The idea is to build analysis tools that extract metrics from recorded robot runs, establishing quantified baselines that define "good" performance. Only if the required data is missing from the bags would we resort to adding new logging.

I already have `vision_accuracy_analyzer.py` that computes vision-to-motion positioning error. Before expanding to other metrics, I want to validate that this analyzer is calculating correctly - the frame transforms, gripper offsets, and motion-complete timing all need verification.

The end goal is having numbers that define acceptable performance: "if position error is below X mm, if planning time is below Y seconds, the system is working."

</vision>

<essential>
## What Must Be Nailed

- **Validate the existing analyzer** - Ensure frame transforms (base vs base_link), gripper/tool offsets, and motion-complete detection are all correct
- **Quantified baselines** - Establish numbers that define "good" so I can measure the system objectively
- **Analysis-first debugging** - Extract insights from recorded data rather than adding instrumentation

</essential>

<boundaries>
## What's Out of Scope

- **Fixing issues** - This phase is about measuring and understanding, not fixing yet
- **New hardware integration** - Focus on analyzing what we have
- **ML/AI detection** - Stick to geometric detection analysis, AI comes later
- **Real-time monitoring** - This is post-hoc analysis of recorded bags, not live monitoring

</boundaries>

<specifics>
## Specific Ideas

**Existing infrastructure:**
- `recorded_bags/sample_to_spincoat/` - 3.7GB rosbag with ~12 minutes of robot operation
- `vision_accuracy_analyzer.py` - Already computes ArUco marker detection → TCP positioning error
- Results show 1.35mm mean error, 48/48 detections matched

**Report format:**
- Detailed breakdown with all metrics, plots, per-operation stats
- Similar to current analyzer output: JSON results + PNG visualization

**Metrics to eventually analyze (after validating current tool):**
- Motion planning times
- Gripper success rates
- End-to-end cycle times

</specifics>

<notes>
## Additional Context

The current vision accuracy results (1.35mm mean error) suggest the vision system is actually working well. The interesting patterns to investigate:
- Y-axis has largest variance (1.14mm std vs 0.54mm X, 0.04mm Z)
- Some markers (4, 5, 9, 19) have 2-3mm errors vs ~0.8mm for others
- This could indicate spatial calibration issues rather than detection problems

The immediate next step is validating the analyzer's correctness before trusting these baselines.

</notes>

---

*Phase: 1-rosbag-analysis*
*Context gathered: 2026-01-17*
