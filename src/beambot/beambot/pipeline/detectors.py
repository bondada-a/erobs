"""Built-in detectors for the vision-task pipeline (issue #88).

Each detector is a thin adapter that DELEGATES to the proven VisionEngine
detection methods — it does not reimplement capture, detection, or the
TF-at-capture transform. Detection algorithms stay the single source of truth
in beambot.detection (called transitively via VisionEngine). This is what lets
the migration be faithful: we route through the exact code that works today.

Contract: detect(ctx) -> PoseStamped | None  (None = detection failed).
`ctx` carries the goal and a VisionEngine handle (see vision_task_stages.py).
"""

from beambot.pipeline.registry import register_detector


@register_detector("marker")
def detect_marker(ctx):
    """ArUco marker detection -> base_link pose.

    Marker branch: cache first, then multi-position averaging if scan positions
    were given, then single-shot (the routing the old VisionMoveTo handler used).
    """
    vision = ctx.vision
    goal = ctx.goal

    cached = vision.get_cached_pose(goal.tag_id)
    if cached is not None:
        pos = cached.pose.position
        vision.logger.info(
            f"Using cached pose for tag {goal.tag_id}: "
            f"[{pos.x * 1000:.2f}, {pos.y * 1000:.2f}, {pos.z * 1000:.2f}] mm"
        )
        return cached

    if ctx.scan_positions is not None:
        return vision.detect_tag_multiposition(
            tag_id=goal.tag_id,
            scan_positions=ctx.scan_positions,
            timeout=goal.timeout,
            settle_time=vision._settle_time,
        )

    return vision.detect_and_transform_tag(goal.tag_id, goal.timeout)


@register_detector("sample_roi")
def detect_sample_roi(ctx):
    """Classical-CV sample detection within an ROI anchored to an ArUco tag."""
    vision = ctx.vision
    goal = ctx.goal
    strategy = goal.strategy or "farthest_edge"
    edge_inset_mm = goal.edge_inset_mm or 6.5
    vision.logger.info(
        f"Using sample_roi detection (tag {goal.tag_id}, "
        f"strategy={strategy}, inset={edge_inset_mm}mm)"
    )
    return vision.detect_and_transform_sample_roi(
        tag_id=goal.tag_id,
        strategy=strategy,
        edge_inset_mm=edge_inset_mm,
        timeout=goal.timeout,
    )


# --- spincoater detectors (2D flash capture -> angle, issue #88 PR2) ---------
# These return the raw detection DICT (center_px, angle_mod90, ...), not a pose.
# The j6_snap goal computer consumes angle_mod90; there is no TF transform
# because the spincoater path is joint-space, not cartesian. Capture and
# detection both stay single-source-of-truth: capture_2d (camera) +
# detect_spincoater_* (beambot.detection).


def _capture_2d_for_spincoater(ctx):
    """Shared 2D flash capture used by both spincoater detectors."""
    from beambot.camera.zivid import capture_2d

    ctx.vision.logger.info("spincoater: capturing 2D image...")
    return capture_2d(ctx.vision.rclpy_node, timeout=15.0)


@register_detector("spincoater_pocket")
def detect_spincoater_pocket(ctx):
    """Detect the empty pocket in the red chuck (HSV/CV). Returns a dict|None."""
    from beambot.detection import detect_spincoater_pocket as _detect

    image = _capture_2d_for_spincoater(ctx)
    if image is None:
        ctx.vision.logger.error("spincoater_pocket: 2D capture failed")
        return None
    detection = _detect(image)
    if detection is not None:
        ctx.vision.logger.info(
            f"pocket detected — angle_mod90={detection['angle_mod90']:.1f}°, "
            f"aspect={detection['aspect']:.2f}, solidity={detection['solidity']:.2f}"
        )
    return detection


@register_detector("spincoater_sample")
def detect_spincoater_sample(ctx):
    """Detect the sample wafer on the chuck (YOLO seg). Returns a dict|None."""
    from beambot.detection import detect_spincoater_sample as _detect

    image = _capture_2d_for_spincoater(ctx)
    if image is None:
        ctx.vision.logger.error("spincoater_sample: 2D capture failed")
        return None
    detection = _detect(image)
    if detection is not None:
        ctx.vision.logger.info(
            f"sample detected — angle_mod90={detection['angle_mod90']:.1f}°, "
            f"confidence={detection['confidence']:.2f}, center={detection['center_px']}"
        )
    return detection
