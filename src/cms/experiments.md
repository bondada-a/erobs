# Experiments

## Experiment 1: Sample to Hotplate and Discard

### Parameters
- sample_tag: 0              # ArUco tag ID on the sample to pick up
- hotplate_tag: 30           # ArUco tag near the hotplate
- discard_tag: 27            # ArUco tag at the discard bin
- gripper: epick

### Part A: Sample to Hotplate

1. **Pick up sample**
   - vision_target("sample", tag_id={sample_tag}) — moves to sample_scan_1, detects tag, moves 20mm right of tag
   - vacuum on
   - move 100mm backward (retreat with sample)

2. **Transport to hotplate**
   - move to hotplate_scan

3. **Place on hotplate**
   - vision_moveto tag={hotplate_tag}, offset_direction="right", offset_distance=0.0512
   - vacuum off
   - move 100mm backward (retreat)

4. **Return**
   - move to hotplate_scan

### Part B: Discard from Hotplate

1. **Pick up sample from hotplate**
   - vision_moveto tag={hotplate_tag}, offset_direction="right", offset_distance=0.0512
   - vacuum on
   - move 100mm backward (retreat with sample)

2. **Transport to discard**
   - move to hotplate_scan
   - move to sample_scan_1

3. **Discard sample**
   - vision_moveto tag={discard_tag}, offset_direction="right", offset_distance=0.040, z_offset=0.010 — 40mm right, 10mm above tag (hovering)
   - vacuum off (drop sample)
   - move 100mm backward (retreat)

4. **Return for next sample**
   - move to sample_scan_1

---

## Experiment 2: (template)

### Parameters
- (define per experiment)

### Protocol
1. (define steps)
