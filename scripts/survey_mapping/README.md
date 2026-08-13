# Survey mapping

Create a registered point-cloud map of the robot cell from multiple
wrist-mounted Zivid captures. The workflow is standalone and does not add an
orchestrator task type.

## Requirements

Source ROS and the workspace before running any script:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

The live survey requires the robot, Zivid camera, and `beambot_moveto` action
server. Start the stack without its general-purpose bag recorder:

```bash
./start_mcp_nobag.sh
```

## Workflow

### 1. Record viewpoints

Put the robot in freedrive mode, then record each camera viewpoint:

```bash
python3 scripts/survey_mapping/teach_survey_poses.py
```

Press Enter to save, `u` to undo, and `q` to finish. The default output is
`survey_poses.yaml`. Use `--output PATH`, `--prefix NAME`, or `--append` when
needed.

Aim for overlapping views from varied angles. Keep the intended scene inside
the camera field of view at every pose.

### 2. Capture a survey

Always validate motion before enabling captures:

```bash
python3 scripts/survey_mapping/run_survey.py --dry-run
```

Then record the survey:

```bash
python3 scripts/survey_mapping/run_survey.py \
  --poses scripts/survey_mapping/survey_poses.yaml \
  --bag-out survey_session
```

For each pose, the runner moves through the collision-aware `beambot_moveto`
action, waits for vibration to settle, triggers `/capture`, and holds position
until a fresh `/points/xyzrgba` cloud arrives. It records the cloud, TF, and
joint states required by the offline merge.

Useful options:

- `--max-poses N`: run only the first N poses.
- `--settle SECONDS`: change the post-move settling time.
- `--no-bag`: use an externally managed recorder.
- `--no-set-settings`: keep the Zivid driver's current capture settings.

### 3. Merge the bag

No robot or live ROS graph is required:

```bash
python3 scripts/survey_mapping/merge_survey_bag.py \
  --bag survey_session \
  --out survey_map.ply \
  --voxel 0.005
```

Each cloud is transformed into `base_link` at its capture timestamp, range
filtered, voxel-downsampled, and written as a binary PLY. Open the result in
CloudCompare, MeshLab, or Open3D.

If an older bag lacks moving-arm TF, provide the matching robot URDF with
`--urdf PATH`; the merger will reconstruct camera poses from `/joint_states`.

## Recording details

- `run_survey.py` applies `manufacturing_specular.yml` before capture unless
  disabled. This preset provides RGB point-cloud data.
- Zivid publishes the cloud several seconds after the capture service returns.
  The runner waits for the cloud before moving to prevent bad transforms.
- `tf_qos_override.yaml` forces volatile QoS for `/tf`, preserving transforms
  from both TF publishers in the recorded bag.
