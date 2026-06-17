# Survey Mapping — point-cloud map of the beamline cell

Build a single point-cloud (and optional mesh) of the environment around the
UR5e by capturing Zivid 3D frames from several wrist viewpoints and merging
them in `base_link`.

These are **standalone scripts** — they add **no** task type to the orchestrator
and touch **no** ROS package. Mapping is an occasional, offline-ish activity, so
it lives outside the production task schema.

## The idea in one paragraph

The arm base is fixed, so `base_link` is effectively world-fixed. The Zivid is
wrist-mounted, and its pose in `base_link` is fully determined by joint FK plus
the static hand-eye calibration — both published as TF. So if we record each
cloud (in `zivid_optical_frame`) **alongside the full TF tree**, we can, offline,
look up `base_link ← zivid_optical_frame` *at each cloud's timestamp*, transform
every cloud into `base_link`, and concatenate. No online merge, fully replayable.

## Workflow (3 scripts)

```
teach_survey_poses.py  ->  survey_poses.yaml
        (move arm by hand, press Enter to save each viewpoint)

run_survey.py          ->  survey_session/   (a rosbag)
        (drive to each pose, trigger Zivid, WAIT for the cloud, record bag)

merge_survey_bag.py    ->  survey_map.ply
        (offline: bag -> one cloud in base_link, voxel-downsampled)
```

All three are run after sourcing the workspace:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### 1. Teach the viewpoints

Put the UR in **freedrive / teach mode**, then:

```bash
python3 scripts/survey_mapping/teach_survey_poses.py
```

Move the wrist to a viewpoint, press **Enter** to save; repeat until the camera
has pointed at everything you want in the map; press **q** to write
`survey_poses.yaml`. (`u` undoes the last save.)

> Aim for overlapping views from varied angles — pure top-down won't capture
> vertical faces. 6–12 viewpoints is typical.

The output uses the same `name: [j1..j6]` degree schema as `src/cms/poses.yaml`,
so the poses are reusable elsewhere if you want.

### 2. Run the survey (live robot)

```bash
python3 scripts/survey_mapping/run_survey.py \
    --poses scripts/survey_mapping/survey_poses.yaml \
    --bag-out survey_session
```

For each pose it: moves there (via the `beambot_moveto` action — same
collision-aware MTC planning as everything else), settles, triggers a Zivid
capture, and **waits for the actual cloud to publish before moving on**. A
`ros2 bag record` of the cloud + TF + joint topics is spawned automatically and
finalized cleanly at the end.

Useful flags:

| flag | meaning |
|---|---|
| `--dry-run` | move + settle only, no captures — sanity-check the motion path first |
| `--no-bag` | don't spawn the recorder (you run `ros2 bag record` yourself) |
| `--settle 2.0` | dwell longer after arrival before capturing (vibration) |
| `--no-set-settings` | skip the Specular preset; use the driver's launched settings |

**Always `--dry-run` first** to confirm every pose is reachable and the path is
collision-free before recording for real.

### 3. Merge offline (no robot)

```bash
python3 scripts/survey_mapping/merge_survey_bag.py \
    --bag survey_session --out survey_map.ply --voxel 0.005
```

Produces `survey_map.ply` — open in CloudCompare / MeshLab / open3d.

| flag | meaning |
|---|---|
| `--voxel 0.005` | voxel leaf size in m (5 mm); `0` keeps every point |
| `--max-range 2.0` | drop points >2 m from the camera (far-wall noise) |
| `--target-frame` | merge frame (default `base_link`) |
| `--denoise` | statistical outlier removal — **needs open3d** |
| `--mesh out.ply` | also write a Poisson surface mesh — **needs open3d** |

## Dependencies

Core path (teach / run / TF-merge / voxel / PLY) is **pure numpy + ROS** and
needs nothing extra — `sensor_msgs_py`, `rosbag2_py`, `tf_transformations` are
already present.

`--denoise` and `--mesh` additionally need open3d (not installed by default):

```bash
pip install open3d
```

## Why it's built this way (gotchas baked in)

- **Zivid is trigger-only.** The cloud topic publishes *only* on a capture
  trigger. A bag of just moves records zero clouds — every pose must fire a
  capture. `run_survey.py` does this explicitly.
- **The cloud lands ~3–4 s after the capture service returns**, stamped ~300 ms
  late. If we moved on at service-return, the cloud would publish mid-motion and
  its TF would be wrong. `run_survey.py` therefore **blocks on the cloud topic**,
  arm held still, before advancing. This is the single most important detail.
- **We trigger the bare `/capture`** (`std_srvs/Trigger`) after applying the
  `manufacturing_specular.yml` preset, because that preset has
  `Sampling.Color: rgb` — real color. The older
  `trigger_zivid_capture.py` uses `/capture_and_detect_markers`, which inherits
  the driver's launched `scene_capture` preset (`Sampling.Color: grayscale`),
  giving R=G=B clouds. For a colored map we want Specular RGB + `/capture`.
  Note `/capture` captures with whatever `settings_file_path` is set to, so the
  preset must be applied *before* triggering — which `run_survey.py` does.
- **TF-only merge is usually enough.** Hand-eye calibration residuals are
  <1.53 mm / <0.33° (2026-03-27), so no ICP is wired in. If you see
  ghosting/seams, ICP refinement is the next step — deliberately left out to
  keep this a clean baseline.
```
