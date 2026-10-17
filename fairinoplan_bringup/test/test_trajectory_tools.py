"""Unit tests of fairino_trajectory_tools (run: python3 -m pytest test/ in the
package, with scripts/ on PYTHONPATH)."""

import math
import os
import sys

import numpy as np
import pinocchio as pin
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import fairino_trajectory_tools as tt  # noqa: E402

DOWN = np.array([0.0, 1.0, 0.0, 0.0])  # flange pointing down (180 deg about x)
QD = np.radians([120, 120, 120, 180, 180, 180])
QDD = np.full(6, math.radians(540))


def wp(x, y, z, wxyz=DOWN):
    return tt.Waypoint(np.array([x, y, z]), np.asarray(wxyz, dtype=float))


def l_path():
    return [wp(0.5, -0.2, 0.4), wp(0.5, 0.0, 0.4), wp(0.7, 0.0, 0.4)]


def max_turn_deg(points):
    d = np.diff(points, axis=0)
    d = d[np.linalg.norm(d, axis=1) > 1e-9]
    d /= np.linalg.norm(d, axis=1)[:, None]
    cos = np.clip(np.sum(d[:-1] * d[1:], axis=1), -1, 1)
    return float(np.degrees(np.max(np.arccos(cos))))


def test_straight_segments_stop_where_the_direction_changes():
    path = tt.sample_path(l_path())
    k = path.waypoint_samples[1]
    assert np.linalg.norm(path.positions[k] - [0.5, 0.0, 0.4]) < 1e-9
    assert path.stop[k] and path.stop[0] and path.stop[-1]
    assert max_turn_deg(path.positions) > 80
    assert set(path.segment.tolist()) == {0, 1}


def test_collinear_straight_segments_do_not_stop():
    path = tt.sample_path([wp(0.5, -0.2, 0.4), wp(0.5, 0.0, 0.4), wp(0.5, 0.2, 0.4)])
    assert not np.any(path.stop[1:-1])


def test_curve_waypoint_is_smooth_and_visited_without_stop():
    waypoints = l_path()
    waypoints[1].curve = True
    waypoints[1].tangent = tt.default_tangent(waypoints, 1)
    assert np.linalg.norm(waypoints[1].tangent) == pytest.approx(0.35 * 0.2, rel=1e-6)
    assert tt.control_points(waypoints, 0) is not None and tt.control_points(waypoints, 1) is not None
    path = tt.sample_path(waypoints)
    assert np.min(np.linalg.norm(path.positions - [0.5, 0.0, 0.4], axis=1)) < 1e-9
    assert not np.any(path.stop[1:-1])
    assert max_turn_deg(path.positions) < 10.0


def test_zero_length_segment_is_reported():
    with pytest.raises(tt.PathError) as error:
        tt.sample_path([wp(0.5, 0.0, 0.4), wp(0.5, 0.0, 0.4)])
    assert error.value.segments == [0]


def test_part_of_a_path_keeps_the_whole_path_samples_and_stops_at_its_ends():
    waypoints = l_path() + [wp(0.7, 0.2, 0.4)]
    for k in (1, 2):
        waypoints[k].curve = True
        waypoints[k].tangent = tt.default_tangent(waypoints, k)
    whole = tt.sample_path(waypoints)
    assert tt.path_part({}, 4) == (0, 3)
    assert tt.path_part({"part": [1, 2]}, 4) == (1, 2)
    assert tt.path_part({"part": [3, 3]}, 4) == (2, 3)  # at least one segment
    part = tt.slice_path(whole, 1, 2)
    a, b = whole.waypoint_samples[1], whole.waypoint_samples[2]
    assert np.allclose(part.positions, whole.positions[a:b + 1])
    assert not whole.stop[a] and not whole.stop[b]  # passed without a stop in the whole path
    assert part.stop[0] and part.stop[-1] and not np.any(part.stop[1:-1])
    assert part.length[0] == 0.0 and part.length[-1] == pytest.approx(whole.length[b] - whole.length[a])
    assert part.waypoint_samples == [0, b - a]
    assert set(part.segment.tolist()) == {1}  # segment indices of the whole path

def test_orientation_is_interpolated_with_smooth_ends():
    end = tt.matrix_to_quat_wxyz(pin.rpy.rpyToMatrix(math.pi, 0.0, 0.5))
    path = tt.sample_path([wp(0.5, 0.0, 0.4), wp(0.6, 0.0, 0.4, end)])
    middle = path.wxyz[len(path.wxyz) // 2]
    assert tt.angle_between_wxyz(middle, DOWN) == pytest.approx(0.25, abs=0.03)
    early = path.wxyz[len(path.wxyz) // 20]
    assert tt.angle_between_wxyz(early, DOWN) < 0.01


def test_pure_rotation_has_path_length():
    end = tt.matrix_to_quat_wxyz(pin.rpy.rpyToMatrix(math.pi, 0.0, 1.0))
    path = tt.sample_path([wp(0.5, 0, 0.4), wp(0.5, 0, 0.4, end)])
    assert path.length[-1] == pytest.approx(1.0 * tt.ROTATION_LENGTH_M_PER_RAD, rel=1e-3)


def test_parameterize_respects_limits_and_rest():
    s = np.linspace(0.0, 1.0, 400)
    q = np.outer(s, np.array([1.0, -0.5, 0.8, 0.0, 0.3, 1.2]))
    stops = np.zeros(len(s), dtype=bool)
    stops[[0, -1]] = True
    traj = tt.parameterize_path(q, s, np.full(len(s), np.inf), stops, QD, QDD, 0.004)
    vel, acc = tt.limit_ratios(traj, QD, QDD)
    assert vel <= 1.0 + 1e-6 and acc <= 1.0 + 1e-6
    assert np.allclose(traj.velocities[0], 0.0) and np.allclose(traj.velocities[-1], 0.0)
    assert np.allclose(traj.positions[-1], q[-1])
    # Time-optimal: at least one joint reaches 90 % of a limit.
    assert max(vel, acc) > 0.9


def test_fit_to_limits_slows_down_a_too_fast_recording():
    t = np.linspace(0.0, 0.5, 200)
    q = np.outer(np.sin(np.pi * t / 0.5) ** 2, np.full(6, 1.5))
    traj = tt.fit_to_limits(t, q, QD, QDD, 0.004)
    assert traj.info["time_scale"] > 1.0
    vel, acc = tt.limit_ratios(traj, QD, QDD)
    assert vel <= 1.0 + 1e-6 and acc <= 1.0 + 1e-6


def test_simplify_keeps_corners():
    line1 = np.linspace([0.5, -0.2, 0.4], [0.5, 0.0, 0.4], 100)
    line2 = np.linspace([0.5, 0.0, 0.4], [0.7, 0.0, 0.4], 100)[1:]
    positions = np.vstack([line1, line2])
    keep = tt.simplify_poses(positions, np.tile(DOWN, (len(positions), 1)))
    assert keep == [0, 99, len(positions) - 1]


def test_recording_to_path_and_files(tmp_path):
    samples = []
    for k, y in enumerate(np.linspace(-0.2, 0.2, 81)):
        x = 0.5 + 0.1 * math.sin(math.pi * k / 80)
        samples.append({"t": 0.01 * k, "pose": tt.pose_to_fairino(np.array([x, y, 0.4]), DOWN), "joint_positions": [0.0] * 6})
    payload = tt.recording_payload(
        robot_name="fr20", urdf_path="/x/urdf/fr20.urdf", joint_names=["j1"], samples=samples, segments=[], name="t", tool_number=0
    )
    assert payload["fbx_path"] == "/x/fbx/fr20.fbx"
    waypoints, segments = tt.recording_to_path(payload)
    assert len(waypoints) >= 3 and len(segments) == len(waypoints) - 1
    assert all(w.curve for w in waypoints[1:-1])
    assert sum(s.duration for s in segments) == pytest.approx(0.8, rel=1e-6)
    import json

    path = tt.write_json(tmp_path / "p.json", tt.path_payload("p", waypoints, segments))
    loaded_w, loaded_s = tt.path_from_payload(json.loads(path.read_text()))
    assert np.allclose(loaded_w[1].tangent, waypoints[1].tangent) and loaded_w[1].curve
    assert loaded_s[0].duration == pytest.approx(segments[0].duration)


def test_recording_to_path_follows_the_recording_within_tolerance():
    """Back-and-forth motion with uneven speed: the converted path must stay
    within the tolerance of the recording both ways (no loops/overshoot)."""
    from scipy.spatial import cKDTree

    t = np.linspace(0.0, 6.0, 1500)
    x = 0.5 + 0.15 * np.sin(2.0 * t) * np.exp(-0.1 * t)  # reverses several times
    y = 0.2 * np.sin(0.7 * t) ** 3  # slow, then fast
    z = 0.4 + 0.05 * np.cos(3.0 * t)
    points = np.column_stack([x, y, z])
    samples = [{"t": float(ti), "pose": tt.pose_to_fairino(p, DOWN)} for ti, p in zip(t, points)]
    waypoints, segments = tt.recording_to_path({"samples": samples}, position_tol=0.003)
    path = tt.sample_path(waypoints, 0.001)
    fine = np.vstack([points[:-1] + (points[1:] - points[:-1]) * f for f in np.linspace(0, 1, 8, endpoint=False)])
    assert np.max(cKDTree(path.positions).query(points)[0]) < 0.0035
    assert np.max(cKDTree(fine).query(path.positions)[0]) < 0.0035
    assert sum(s.duration for s in segments) == pytest.approx(6.0, rel=1e-6)


def test_old_path_files_load_as_straight_fastest_segments():
    old = {
        "format": "fairino_path",
        "version": 2,
        "waypoints": [{"position_m": [0.5, 0.0, 0.4], "wxyz": list(DOWN)}, {"position_m": [0.6, 0.0, 0.4], "wxyz": list(DOWN)}],
        "segments": [{"shape": "bezier", "c1_m": [0, 0, 0], "c2_m": [0, 0, 0], "speed_m_s": 0.4}],
    }
    waypoints, segments = tt.path_from_payload(old)
    assert len(segments) == 1 and segments[0].duration is None and not waypoints[0].curve


def test_fairino_pose_round_trip():
    q = tt.matrix_to_quat_wxyz(pin.rpy.rpyToMatrix(0.3, -0.2, 1.1))
    p, back = tt.fairino_to_pose(tt.pose_to_fairino(np.array([0.1, 0.2, 0.3]), q))
    assert np.allclose(p, [0.1, 0.2, 0.3]) and tt.angle_between_wxyz(q, back) < 1e-9


@pytest.fixture(scope="module")
def path_ik():
    from ament_index_python.packages import get_package_share_path

    urdf = (get_package_share_path("fairino_description") / "urdf" / "fairino20_v6.urdf").read_text()
    return tt.PathIk(urdf, ["j1", "j2", "j3", "j4", "j5", "j6"])


def test_path_ik_follows_a_path_with_requested_segment_times(path_ik):
    home = np.radians([0, -90, 90, -90, -90, 0])
    start = path_ik.fk(home)
    rot = tt.matrix_to_quat_wxyz(start.rotation)
    p0 = start.translation
    waypoints = [
        tt.Waypoint(p0, rot),
        tt.Waypoint(p0 + [0.0, 0.2, 0.0], rot, curve=True),
        tt.Waypoint(p0 + [0.0, 0.2, -0.2], rot),
    ]
    waypoints[1].tangent = tt.default_tangent(waypoints, 1)
    path = tt.sample_path(waypoints)
    q = path_ik.follow(path, home)
    for k in range(0, len(q), 25):
        assert np.linalg.norm(path_ik.fk(q[k]).translation - path.positions[k]) < 1e-5
    fast = tt.parameterize_path(q, path.length, path.speed, path.stop, QD, QDD, 0.004, path.segment)
    assert len(fast.segment_fastest_s) == 2
    assert fast.segment_planned_s == pytest.approx(fast.segment_fastest_s, rel=1e-6)
    wanted = [None, 3.0 * fast.segment_fastest_s[1]]
    slow = tt.parameterize_path(q, path.length, path.speed, path.stop, QD, QDD, 0.004, path.segment, wanted)
    vel, acc = tt.limit_ratios(slow, QD, QDD)
    assert vel <= 1.0 + 1e-6 and acc <= 1.0 + 1e-6
    assert slow.segment_planned_s[1] == pytest.approx(wanted[1], rel=0.03)
    assert slow.segment_planned_s[0] == pytest.approx(fast.segment_planned_s[0], rel=0.15)
    tcp = np.array([path_ik.fk(x).translation for x in fast.positions])
    speed = np.linalg.norm(np.diff(tcp, axis=0), axis=1) / 0.004
    middle = int(np.argmin(np.linalg.norm(tcp - waypoints[1].position, axis=1)))
    assert speed[middle] > 0.05  # curve waypoint: no stop
    # a requested time shorter than the fastest one is ignored
    too_fast = tt.parameterize_path(q, path.length, path.speed, path.stop, QD, QDD, 0.004, path.segment, [0.01, None])
    assert too_fast.segment_planned_s[0] == pytest.approx(fast.segment_planned_s[0], rel=1e-6)


def test_smooth_recording_removes_stale_samples_and_timing_noise():
    rng = np.random.default_rng(1)
    t = np.cumsum(rng.uniform(0.006, 0.010, 300))
    q = np.outer(0.5 * (1 - np.cos(np.pi * (t - t[0]) / (t[-1] - t[0]))), np.ones(6))
    q[100] = q[99]  # stale repeat
    noisy_t = t + rng.uniform(-0.002, 0.002, len(t))
    noisy_t.sort()
    grid, smooth = tt.smooth_recording(noisy_t, q)
    assert np.allclose(np.diff(grid)[:-1], 0.004)
    assert np.max(np.abs(np.diff(smooth, 2, axis=0))) / 0.004**2 < 20.0
    assert np.allclose(smooth[-1], q[-1])


def test_fit_recorded_timing_stretches_only_locally():
    t = np.linspace(0.0, 4.0, 1001)
    s = 0.5 * (1 - np.cos(np.pi * t / 4.0))
    s = s + 0.004 * np.exp(-((t - 2.0) / 0.02) ** 2)  # one short jerk
    q = np.outer(s, np.ones(6))
    traj = tt.fit_recorded_timing(t, q, QD, QDD, 0.004)
    vel, acc = tt.limit_ratios(traj, QD, QDD)
    assert vel <= 1.0 + 1e-6 and acc <= 1.0 + 1e-6
    assert traj.info["locally_stretched"] == 1.0
    assert traj.times[-1] < 1.5 * 4.0  # a uniform slowdown would need far more


def test_tracking_report_measures_distance_to_the_planned_path(path_ik):
    t = np.linspace(0.0, 2.0, 501)
    home = np.radians([0, -90, 90, -90, -90, 0])
    plan_q = home + np.outer(0.5 * (1 - np.cos(np.pi * t / 2.0)), np.radians([30, 10, -10, 0, 0, 0]))
    late = np.interp(t, t + 0.1, t)  # 0.1 s late, but on the path
    act_q = home + np.outer(0.5 * (1 - np.cos(np.pi * late / 2.0)), np.radians([30, 10, -10, 0, 0, 0]))
    report = tt.tracking_report(t, plan_q, t, act_q, path_ik.fk)
    assert report["max_deviation_mm"] < 0.5
    off = act_q.copy()
    off[200:300, 0] += np.radians(1.0)  # ~1 deg of J1 at > 0.5 m reach
    servoj = np.array([[0.004 * k, (2 * k) % 10000, 0, 0, 0.001] for k in range(500)], dtype=float)
    report = tt.tracking_report(t, plan_q, t, off, path_ik.fk, servoj)
    assert report["max_deviation_mm"] > 5.0
    assert report["servoj_min_rate"] == pytest.approx(500, rel=0.02)
    assert report["servoj_expected_rate"] == 1000


def _prototype_look_at(position, point):
    """apply_look_at_orientation of the FAIRINO Viser prototype (fixed up vector)."""
    z = (point - position) / np.linalg.norm(point - position)
    up = np.array([0.0, 0.0, 1.0])
    if abs(z @ up) > 0.95:
        up = np.array([0.0, 1.0, 0.0])
    x = np.cross(up, z)
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def test_look_at_keeps_the_roll_continuous():
    point = np.array([0.5, 0.0, 0.0])
    positions = np.column_stack([np.linspace(0.3, 0.7, 401), np.full(401, 0.05), np.full(401, 0.6)])
    rotation = tt.quat_wxyz_to_matrix(DOWN)
    worst, worst_prototype, previous, previous_prototype = 0.0, 0.0, None, None
    for p in positions:
        rotation = tt.look_at_rotation(rotation, p, point)
        z = (point - p) / np.linalg.norm(point - p)
        assert rotation[:, 2] @ z > 1 - 1e-9
        prototype = _prototype_look_at(p, point)
        if previous is not None:
            worst = max(worst, np.linalg.norm(pin.log3(previous.T @ rotation)))
            worst_prototype = max(worst_prototype, np.linalg.norm(pin.log3(previous_prototype.T @ prototype)))
        previous, previous_prototype = rotation, prototype
    assert worst < math.radians(1.0)  # 1 mm steps: well below a degree per step
    assert worst_prototype > math.radians(45.0)  # the prototype's roll jump


def test_sampled_path_looks_at_the_point_and_round_trips(tmp_path):
    import json

    point = np.array([0.6, 0.0, 0.0])
    waypoints = [wp(0.4, -0.2, 0.5), wp(0.6, 0.1, 0.5), wp(0.8, -0.1, 0.45)]
    waypoints[1].curve = True
    waypoints[1].tangent = tt.default_tangent(waypoints, 1)
    path = tt.sample_path(waypoints, 0.002, look_at=point)
    for p, q in zip(path.positions, path.wxyz):
        z = tt.quat_wxyz_to_matrix(q)[:, 2]
        assert z @ ((point - p) / np.linalg.norm(point - p)) > 1 - 1e-9
    shown = tt.apply_look_at(waypoints, point)
    assert np.allclose(shown[1].wxyz, path.wxyz[path.waypoint_samples[1]])
    payload = json.loads(tt.write_json(tmp_path / "p.json", tt.path_payload("p", waypoints, [tt.Segment()] * 2, point)).read_text())
    assert np.allclose(tt.look_at_from_payload(payload), point)
    assert tt.look_at_from_payload(tt.path_payload("p", waypoints, [tt.Segment()] * 2)) is None
    with pytest.raises(tt.PathError):
        tt.sample_path(waypoints, 0.002, look_at=np.array([0.6, 0.1, 0.5]))  # on the path


def test_dwell_rests_at_the_waypoint_and_round_trips():
    waypoints = [wp(0.5, -0.2, 0.4), wp(0.5, 0.0, 0.4), wp(0.5, 0.2, 0.4)]  # collinear: no corner stop
    waypoints[1].dwell = 1.5
    path = tt.sample_path(waypoints)
    assert path.stop[path.waypoint_samples[1]]
    assert tt.Waypoint.from_dict(waypoints[1].to_dict()).dwell == 1.5
    q = np.zeros((len(path.positions), 6))
    q[:, 0] = path.length  # a joint path that follows the path coordinate
    traj = tt.parameterize_path(q, path.length, path.speed, path.stop, QD, QDD, 0.004, path.segment)
    t_wp = traj.segment_planned_s[0]
    rested = tt.insert_dwells(traj, [t_wp], [1.5], 0.004)
    assert rested.times[-1] == pytest.approx(traj.times[-1] + 1.5, abs=1e-9)
    assert np.all(np.diff(rested.times) > 0)
    hold = (rested.times > t_wp + 0.01) & (rested.times < t_wp + 1.49)
    assert np.ptp(rested.positions[hold, 0]) < 1e-9 and np.all(rested.velocities[hold] == 0.0)
    assert rested.positions[-1, 0] == pytest.approx(traj.positions[-1, 0])


def test_drawing_becomes_curve_waypoints_with_a_stop_where_the_gizmo_was_held():
    dt = 0.02
    t1 = np.arange(0.0, 1.0, dt)  # 0.2 m along x in 1 s
    t2 = np.arange(1.0, 1.8, dt)  # held still 0.8 s
    t3 = np.arange(1.8, 2.8, dt)  # quarter circle
    pos1 = np.stack([0.5 + 0.2 * t1, np.zeros_like(t1), np.full_like(t1, 0.4)], axis=1)
    pos2 = np.repeat([[0.7, 0.0, 0.4]], len(t2), axis=0)
    a = (t3 - 1.8) * (np.pi / 2)
    pos3 = np.stack([0.7 + 0.1 * np.sin(a), 0.1 - 0.1 * np.cos(a), np.full_like(a, 0.4)], axis=1)
    times = np.concatenate([t1, t2, t3])
    positions = np.vstack([pos1, pos2, pos3])
    quats = np.repeat([DOWN], len(times), axis=0)
    waypoints, segments = tt.drawing_to_path(times, positions, quats)
    assert len(segments) == len(waypoints) - 1
    assert np.allclose(waypoints[0].position, pos1[0])
    stops = [w for w in waypoints if w.dwell > 0]
    assert len(stops) == 1 and stops[0].dwell == pytest.approx(0.78, abs=0.03)
    assert np.linalg.norm(stops[0].position - [0.7, 0.0, 0.4]) < 0.002
    assert np.linalg.norm(waypoints[-1].position - pos3[-1]) < 1e-9
    assert sum(s.duration for s in segments) == pytest.approx(2.8 - dt - 0.78, abs=0.05)  # motion only
    tt.sample_path(waypoints)  # a valid path (no zero-length segment)
    # Released and grabbed again without moving: nothing to add.
    w, s = tt.drawing_to_path(times[:5] * 0.0 + np.arange(5) * dt, np.repeat(pos1[:1], 5, axis=0), quats[:5])
    assert len(w) == 1 and not s
