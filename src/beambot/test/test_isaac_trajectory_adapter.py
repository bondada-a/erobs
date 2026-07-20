from types import SimpleNamespace

from beambot.isaac_trajectory_adapter import interpolate_positions, validate_trajectory


def _point(seconds, positions):
    whole = int(seconds)
    return SimpleNamespace(
        time_from_start=SimpleNamespace(sec=whole, nanosec=int((seconds - whole) * 1e9)),
        positions=positions,
    )


def test_interpolate_positions():
    points = [_point(0.0, [0.0, 2.0]), _point(2.0, [2.0, 4.0])]
    assert interpolate_positions(points, 1.0) == [1.0, 3.0]
    assert interpolate_positions(points, 5.0) == [2.0, 4.0]


def test_validate_trajectory_accepts_reordered_joints():
    assert validate_trajectory(["b", "a"], [_point(1.0, [2.0, 1.0])], ["a", "b"]) is None


def test_validate_trajectory_rejects_missing_joint_and_bad_times():
    assert "missing" in validate_trajectory(["a"], [_point(1.0, [1.0])], ["a", "b"])
    error = validate_trajectory(
        ["a", "b"], [_point(1.0, [1.0, 2.0]), _point(1.0, [2.0, 3.0])], ["a", "b"]
    )
    assert "strictly increasing" in error
