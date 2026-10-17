"""Trajectory recording, path geometry and time parameterization.

Shared by the Viser UI (drawing, recording, files) and the planner (sampling,
IK, timing), so what the operator sees is exactly what is planned. No ROS
dependency; numpy, scipy and pinocchio only.

Conventions
-----------
* Poses are poses of the URDF frame ``flange`` (= TCP of FAIRINO tool 0) in
  ``base_link``: position [m], orientation as quaternion (w, x, y, z).
* Files use the FAIRINO WebApp convention in addition: x, y, z [mm] and
  rx, ry, rz [deg] about the fixed axes X, Y, Z.
* Joint values are radians in the order j1..j6.

Path model
----------
A path is a list of waypoints (full TCP poses) and one segment between each
pair of consecutive waypoints; the first waypoint is the robot's TCP when the
path was started. A segment is straight unless one of its waypoints is a
*curve* waypoint: such a waypoint has a tangent handle, the path passes
through it as a cubic Bezier (control points waypoint +/- tangent, as in the
FAIRINO Viser prototype) and the robot does not stop there. Where straight
segments meet with a change of direction the robot stops.

Timing: by default every segment is traversed as fast as the joint limits
allow; a segment can be given a longer duration [s].

Orientation is interpolated with slerp along each segment with a smoothstep
profile (zero angular rate at both ends), so the angular velocity never jumps
at a waypoint.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path

import numpy as np
import pinocchio as pin
from scipy.interpolate import CubicSpline, make_lsq_spline

PATH_FORMAT = "fairino_path"
RECORDING_FORMAT_VERSION = 1
SHAPES = ("line", "bezier")
# Path length assigned to a rotation so that pure reorientations take time.
ROTATION_LENGTH_M_PER_RAD = 0.2
DEFAULT_STEP_M = 0.002
MIN_SPEED_M_S = 0.005
MAX_SPEED_M_S = 2.0


class PathError(RuntimeError):
    """Planning failed for a reason the operator can fix (with details)."""

    def __init__(
        self,
        message: str,
        segments: list[int] | None = None,
        fraction: float | None = None,
        position: np.ndarray | None = None,
    ):
        super().__init__(message)
        self.segments = segments or []
        self.fraction = fraction
        self.position = None if position is None else [float(v) for v in position]  # where (base_link)


# ---------------------------------------------------------------------------
# Pose helpers
# ---------------------------------------------------------------------------


def quat_wxyz_to_matrix(wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    return pin.Quaternion(w, x, y, z).normalized().matrix()


def matrix_to_quat_wxyz(rotation: np.ndarray) -> np.ndarray:
    q = pin.Quaternion(np.asarray(rotation, dtype=float))
    q.normalize()
    return np.array([q.w, q.x, q.y, q.z])


def smoothstep(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def slerp_wxyz(a, b, u: float) -> np.ndarray:
    qa = pin.Quaternion(*[float(v) for v in a]).normalized()
    qb = pin.Quaternion(*[float(v) for v in b]).normalized()
    q = qa.slerp(float(u), qb)
    return np.array([q.w, q.x, q.y, q.z])


def angle_between_wxyz(a, b) -> float:
    return float(np.linalg.norm(pin.log3(quat_wxyz_to_matrix(a).T @ quat_wxyz_to_matrix(b))))


def pose_to_fairino(position: np.ndarray, wxyz) -> list[float]:
    """[x, y, z mm, rx, ry, rz deg] as the FAIRINO WebApp shows it."""
    rpy = np.degrees(pin.rpy.matrixToRpy(quat_wxyz_to_matrix(wxyz)))
    p = np.asarray(position, dtype=float) * 1000.0
    return [float(p[0]), float(p[1]), float(p[2]), float(rpy[0]), float(rpy[1]), float(rpy[2])]


def fairino_to_pose(values) -> tuple[np.ndarray, np.ndarray]:
    values = [float(v) for v in values]
    rotation = pin.rpy.rpyToMatrix(*np.radians(values[3:6]))
    return np.array(values[:3]) / 1000.0, matrix_to_quat_wxyz(rotation)


# ---------------------------------------------------------------------------
# Waypoints, segments and geometry
# ---------------------------------------------------------------------------


@dataclass
class Waypoint:
    position: np.ndarray
    wxyz: np.ndarray
    curve: bool = False
    tangent: np.ndarray = field(default_factory=lambda: np.zeros(3))
    dwell: float = 0.0  # rest at this waypoint [s] (between two segments)
    blend: float = 0.0  # corner blend radius [m]: the path rounds the corner instead of stopping
    stroke: int | None = None  # inner waypoint of a drawn stroke (servo view): not shown, rebuilt from the drawing

    def to_dict(self) -> dict:
        return {
            "position_m": [float(v) for v in self.position],
            "wxyz": [float(v) for v in self.wxyz],
            "fairino_pose": pose_to_fairino(self.position, self.wxyz),
            "curve": bool(self.curve),
            "tangent_m": [float(v) for v in self.tangent],
            "dwell_s": float(self.dwell),
            "blend_m": float(self.blend),
            **({"stroke": int(self.stroke)} if self.stroke is not None else {}),
        }

    @staticmethod
    def from_dict(data: dict) -> "Waypoint":
        wxyz = np.asarray(data["wxyz"], dtype=float)
        tangent = np.asarray(data.get("tangent_m", [0.0, 0.0, 0.0]), dtype=float)
        return Waypoint(
            np.asarray(data["position_m"], dtype=float),
            wxyz / np.linalg.norm(wxyz),
            curve=bool(data.get("curve", False)) and np.linalg.norm(tangent) > 1e-9,
            tangent=tangent,
            dwell=max(0.0, float(data.get("dwell_s") or 0.0)),
            blend=max(0.0, float(data.get("blend_m") or 0.0)),
            stroke=None if data.get("stroke") is None else int(data["stroke"]),
        )

    def copy(self) -> "Waypoint":
        return Waypoint(
            self.position.copy(), self.wxyz.copy(), self.curve, self.tangent.copy(), self.dwell, self.blend, self.stroke
        )


@dataclass
class Segment:
    """Segment from waypoint k to k + 1. duration None: as fast as possible."""

    duration: float | None = None
    origin: str | None = None  # "drawn" (servo view), "recorded" (Edit as Path), None (drawn as a line / curve)
    stroke: int | None = None  # part of a drawn stroke (servo view)
    speed: float | None = None  # drawn strokes: 0 = the drawn duration, 1 = the fastest, between: mixed

    def to_dict(self) -> dict:
        data = {"duration_s": None if self.duration is None else float(self.duration)}
        if self.origin:
            data["origin"] = self.origin
        if self.stroke is not None:
            data["stroke"] = int(self.stroke)
        if self.speed is not None:
            data["speed"] = float(self.speed)
        return data

    @staticmethod
    def from_dict(data: dict) -> "Segment":
        value = data.get("duration_s")
        return Segment(
            None if value is None or float(value) <= 0.0 else float(value), data.get("origin") or None,
            None if data.get("stroke") is None else int(data["stroke"]),
            None if data.get("speed") is None else float(data["speed"]),
        )

    def copy(self) -> "Segment":
        return Segment(self.duration, self.origin, self.stroke, self.speed)

    def request(self):
        """Duration for the planner: a number, or (drawn, speed) for a drawn stroke."""
        return (self.duration, self.speed) if self.speed is not None and self.duration else self.duration


def default_tangent(waypoints: list[Waypoint], i: int) -> np.ndarray:
    """Tangent along the direction previous -> next waypoint, 35 % of the
    shorter neighbouring segment (the prototype's snap fraction)."""
    n = len(waypoints)
    prev_point = waypoints[max(i - 1, 0)].position
    next_point = waypoints[min(i + 1, n - 1)].position
    direction = next_point - prev_point
    lengths = []
    if i > 0:
        lengths.append(np.linalg.norm(waypoints[i].position - prev_point))
    if i < n - 1:
        lengths.append(np.linalg.norm(next_point - waypoints[i].position))
    size = 0.35 * min(lengths) if lengths else 0.0
    norm = np.linalg.norm(direction)
    return direction / norm * size if norm > 1e-9 else np.zeros(3)


def control_points(waypoints: list[Waypoint], k: int) -> tuple[np.ndarray, np.ndarray] | None:
    """Bezier control points of segment k, or None for a straight segment."""
    a, b = waypoints[k], waypoints[k + 1]
    if not (a.curve or b.curve):
        return None
    c1 = a.position + (a.tangent if a.curve else np.zeros(3))
    c2 = b.position - (b.tangent if b.curve else np.zeros(3))
    return c1, c2


def _start_direction(waypoints: list[Waypoint], k: int) -> np.ndarray:
    cp = control_points(waypoints, k)
    if cp is not None and np.linalg.norm(cp[0] - waypoints[k].position) > 1e-9:
        return cp[0] - waypoints[k].position
    return (cp[1] if cp is not None else waypoints[k + 1].position) - waypoints[k].position


def _end_direction(waypoints: list[Waypoint], k: int) -> np.ndarray:
    cp = control_points(waypoints, k)
    if cp is not None and np.linalg.norm(waypoints[k + 1].position - cp[1]) > 1e-9:
        return waypoints[k + 1].position - cp[1]
    return waypoints[k + 1].position - (cp[0] if cp is not None else waypoints[k].position)


def _unit(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-12 else np.zeros(3)


@dataclass
class SampledPath:
    positions: np.ndarray  # (N, 3)
    wxyz: np.ndarray  # (N, 4)
    speed: np.ndarray  # (N,) max path speed [m/s]
    stop: np.ndarray  # (N,) bool: robot must be at rest here
    segment: np.ndarray  # (N,) segment index of the sample
    waypoint_samples: list[int]  # sample index of each waypoint
    length: np.ndarray  # (N,) cumulative path coordinate [m] (incl. rotation)


def segment_points(waypoints: list[Waypoint], k: int, step: float = DEFAULT_STEP_M) -> tuple[np.ndarray, np.ndarray]:
    a, b = waypoints[k], waypoints[k + 1]
    p0, p3 = a.position, b.position
    cp = control_points(waypoints, k)
    p1, p2 = cp if cp is not None else (p0 + (p3 - p0) / 3.0, p0 + 2.0 * (p3 - p0) / 3.0)
    rough = np.linalg.norm(p1 - p0) + np.linalg.norm(p2 - p1) + np.linalg.norm(p3 - p2)
    rotation = angle_between_wxyz(a.wxyz, b.wxyz) * ROTATION_LENGTH_M_PER_RAD
    count = int(max(2, math.ceil(max(rough, rotation) / step) + 1))
    u = np.linspace(0.0, 1.0, count)[:, None]
    points = ((1 - u) ** 3) * p0 + 3 * ((1 - u) ** 2) * u * p1 + 3 * (1 - u) * (u**2) * p2 + (u**3) * p3
    lengths = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    fraction = lengths / lengths[-1] if lengths[-1] > 1e-9 else u[:, 0]
    quats = np.array([slerp_wxyz(a.wxyz, b.wxyz, f) for f in smoothstep(fraction)])
    return points, quats


BLEND_MAX_FRACTION = 0.5  # of the shorter neighbouring segment: blends of neighbours meet at most
BLEND_MIN_ANGLE_RAD = math.radians(1.0)  # below this there is no corner to round


def corner_angle(waypoints: list[Waypoint], k: int) -> float:
    """Change of direction at interior waypoint k [rad] (0 at the ends)."""
    if k <= 0 or k >= len(waypoints) - 1:
        return 0.0
    d_in, d_out = _unit(_end_direction(waypoints, k - 1)), _unit(_start_direction(waypoints, k))
    if np.linalg.norm(d_in) == 0.0 or np.linalg.norm(d_out) == 0.0:
        return 0.0
    return float(math.acos(float(np.clip(np.dot(d_in, d_out), -1.0, 1.0))))


def blend_limit(waypoints: list[Waypoint], k: int) -> float:
    """Largest blend radius at waypoint k [m]."""
    if k <= 0 or k >= len(waypoints) - 1:
        return 0.0
    w = waypoints[k]
    return BLEND_MAX_FRACTION * min(
        float(np.linalg.norm(w.position - waypoints[k - 1].position)),
        float(np.linalg.norm(waypoints[k + 1].position - w.position)),
    )


def blend_radius(waypoints: list[Waypoint], k: int) -> float:
    """Blend radius used at waypoint k [m]: 0 at the ends, at curve waypoints
    (already smooth) and without a corner; at most BLEND_MAX_FRACTION of the
    shorter neighbouring segment. With a stop (dwell) the robot rests at the
    middle of the rounding."""
    if k <= 0 or k >= len(waypoints) - 1:
        return 0.0
    w = waypoints[k]
    if w.blend <= 0.0 or w.curve or corner_angle(waypoints, k) < BLEND_MIN_ANGLE_RAD:
        return 0.0
    return min(w.blend, blend_limit(waypoints, k))


def _trim(points: np.ndarray, quats: np.ndarray, start: float, end: float, step: float) -> tuple[np.ndarray, np.ndarray]:
    """Samples of one segment without its first `start` and last `end`
    metres of path (exact cut points interpolated; samples closer than half
    a step to a cut are dropped: nearly equal neighbours make the numerical
    path derivatives, hence the curvature speed limit, explode)."""
    if start <= 0.0 and end <= 0.0:
        return points, quats
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    a, b = start, s[-1] - end

    def at(value: float) -> tuple[np.ndarray, np.ndarray]:
        i = int(np.clip(np.searchsorted(s, value) - 1, 0, len(s) - 2))
        f = 0.0 if s[i + 1] - s[i] < 1e-12 else (value - s[i]) / (s[i + 1] - s[i])
        return points[i] + f * (points[i + 1] - points[i]), slerp_wxyz(quats[i], quats[i + 1], float(np.clip(f, 0, 1)))

    if b - a < 1e-6:  # two roundings meet (each at half the segment): one point is left
        pa, qa = at(min(a, s[-1]))
        return pa[None, :], qa[None, :]
    inner = (s > a + 0.5 * step) & (s < b - 0.5 * step)
    pa, qa = at(a)
    pb, qb = at(b)
    return np.vstack([pa, points[inner], pb]), np.vstack([qa, quats[inner], qb])


def _length(points: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def _direction_at(points: np.ndarray, quats: np.ndarray, distance: float) -> np.ndarray:
    """Unit direction of travel at a path distance along one segment."""
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    i = int(np.clip(np.searchsorted(s, float(np.clip(distance, 0.0, s[-1]))) - 1, 0, len(s) - 2))
    return _unit(points[i + 1] - points[i])


def _orientation_at(points: np.ndarray, quats: np.ndarray, distance: float) -> np.ndarray:
    """Orientation at a path distance along samples of one segment."""
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    distance = float(np.clip(distance, 0.0, s[-1]))
    i = int(np.clip(np.searchsorted(s, distance) - 1, 0, len(s) - 2))
    f = 0.0 if s[i + 1] - s[i] < 1e-12 else (distance - s[i]) / (s[i + 1] - s[i])
    return slerp_wxyz(quats[i], quats[i + 1], float(np.clip(f, 0.0, 1.0)))


def _blend_curve(p_in, d_in, p_out, d_out, radius: float, step: float, incoming, outgoing):
    """Round corner from p_in (end of the trimmed incoming segment, direction
    d_in) to p_out: cubic Bezier tangent to both segments (for straight
    segments the quadratic one with its control point at the corner). The
    orientation continues the one of the sharp corner (incoming segment up to
    the middle, then the outgoing one), so the turning rate has no step at
    the ends of the rounding (a step made the curvature speed limit collapse)."""
    c1, c2 = p_in + (2.0 / 3.0) * radius * d_in, p_out - (2.0 / 3.0) * radius * d_out
    count = int(max(3, math.ceil(2.0 * radius / step) + 1))
    u = np.linspace(0.0, 1.0, count)[:, None]
    points = ((1 - u) ** 3) * p_in + 3 * ((1 - u) ** 2) * u * c1 + 3 * (1 - u) * (u**2) * c2 + (u**3) * p_out
    lengths = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points, axis=0), axis=1))])
    fraction = lengths / lengths[-1] if lengths[-1] > 1e-12 else u[:, 0]
    (in_points, in_quats), (out_points, out_quats) = incoming, outgoing
    in_length = float(np.sum(np.linalg.norm(np.diff(in_points, axis=0), axis=1)))
    quats = []
    for f in fraction:
        x = 2.0 * radius * float(f)  # distance along the sharp corner p_in -> corner -> p_out
        quats.append(
            _orientation_at(in_points, in_quats, in_length - radius + x) if x <= radius
            else _orientation_at(out_points, out_quats, x - radius)
        )
    return points, np.array(quats), lengths


def segment_pieces(waypoints: list[Waypoint], step: float = DEFAULT_STEP_M) -> list[tuple[np.ndarray, np.ndarray]]:
    """Samples (positions, wxyz) of every segment as the robot moves: corners
    with a blend radius are cut and rounded; each half of the rounding
    belongs to the segment on its side, so a segment ends at the middle of
    the rounding (the closest point to the waypoint)."""
    count = len(waypoints)
    radii = [blend_radius(waypoints, k) for k in range(count)]
    raw = [segment_points(waypoints, k, step) for k in range(count - 1)]
    pieces = [_trim(points, quats, radii[k], radii[k + 1], step) for k, (points, quats) in enumerate(raw)]
    for k in range(1, count - 1):
        if radii[k] <= 0.0:
            continue
        (pa, qa), (pb, qb) = pieces[k - 1], pieces[k]
        d_in = _direction_at(*raw[k - 1], _length(raw[k - 1][0]) - radii[k])
        d_out = _direction_at(*raw[k], radii[k])
        points, quats, lengths = _blend_curve(pa[-1], d_in, pb[0], d_out, radii[k], step, raw[k - 1], raw[k])
        half = int(np.argmin(np.abs(lengths - 0.5 * lengths[-1])))
        pieces[k - 1] = (np.vstack([pa, points[1:half + 1]]), np.vstack([qa, quats[1:half + 1]]))
        pieces[k] = (np.vstack([points[half:], pb[1:]]), np.vstack([quats[half:], qb[1:]]))
    return pieces


def corner_stops(waypoints: list[Waypoint]) -> list[bool]:
    """True where the robot must stop: path start/end and every straight
    corner with a change of direction (> 1 degree)."""
    stops = [True] * len(waypoints)
    for k in range(1, len(waypoints) - 1):
        if waypoints[k].curve or blend_radius(waypoints, k) > 0.0:
            stops[k] = False  # a curve, or a corner rounded by its blend radius
            continue
        d_in = _unit(_end_direction(waypoints, k - 1))
        d_out = _unit(_start_direction(waypoints, k))
        if np.linalg.norm(d_in) > 0 and np.linalg.norm(d_out) > 0:
            stops[k] = float(np.dot(d_in, d_out)) < math.cos(math.radians(1.0))
    return stops


# ---------------------------------------------------------------------------
# Look at a point
# ---------------------------------------------------------------------------

LOOK_AT_MIN_DISTANCE_M = 0.05


def rotation_between(a: np.ndarray, b: np.ndarray, fallback_axis: np.ndarray | None = None) -> np.ndarray:
    """Smallest rotation that turns direction a into direction b."""
    a = a / np.linalg.norm(a)
    b = b / np.linalg.norm(b)
    cross = np.cross(a, b)
    sin, cos = float(np.linalg.norm(cross)), float(np.clip(a @ b, -1.0, 1.0))
    if sin < 1e-9:
        if cos > 0.0:
            return np.eye(3)
        axis = fallback_axis if fallback_axis is not None else np.cross(a, [1.0, 0.0, 0.0])
        if np.linalg.norm(axis) < 1e-9:
            axis = np.cross(a, [0.0, 1.0, 0.0])
        return pin.exp3(axis / np.linalg.norm(axis) * math.pi)
    return pin.exp3(cross / sin * math.atan2(sin, cos))


def look_at_rotation(rotation: np.ndarray, position: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Tool orientation whose z axis points from position at point, reached
    from `rotation` by the smallest rotation. Applied step by step this keeps
    the roll about the tool axis continuous; a fixed "up" vector (the FAIRINO
    Viser prototype's look-at) makes the roll jump by up to 180 deg where the
    up vector is switched and spin near the vertical."""
    direction = np.asarray(point, dtype=float) - np.asarray(position, dtype=float)
    if np.linalg.norm(direction) < 1e-9:
        return rotation
    return rotation_between(rotation[:, 2], direction, fallback_axis=rotation[:, 0]) @ rotation


def look_at_orientations(positions: np.ndarray, start_wxyz: np.ndarray, point: np.ndarray) -> np.ndarray:
    """Orientations (wxyz) along positions, each looking at point, carried
    from the previous one (the first from start_wxyz)."""
    rotation = quat_wxyz_to_matrix(start_wxyz)
    quats = np.empty((len(positions), 4))
    for i, p in enumerate(positions):
        rotation = look_at_rotation(rotation, p, point)
        quats[i] = matrix_to_quat_wxyz(rotation)
    return quats


def apply_look_at(waypoints: list[Waypoint], point: np.ndarray | None) -> list[Waypoint]:
    """Copies of the waypoints with the orientation the path has there when
    it looks at point (as planned); unchanged without a point."""
    copies = [w.copy() for w in waypoints]
    if point is None or len(waypoints) == 0:
        return copies
    try:
        path = sample_path(waypoints, DEFAULT_STEP_M, look_at=point)
        for w, k in zip(copies, path.waypoint_samples):
            w.wxyz = path.wxyz[k].copy()
    except PathError:
        quats = look_at_orientations(np.array([w.position for w in waypoints]), waypoints[0].wxyz, np.asarray(point))
        for w, q in zip(copies, quats):
            w.wxyz = q
    return copies


def sample_path(waypoints: list[Waypoint], step: float = DEFAULT_STEP_M, look_at: np.ndarray | None = None) -> SampledPath:
    """Dense samples of the path. With look_at the tool z axis points at that
    point on every sample (the waypoints' own orientations are not used,
    except the first one's roll)."""
    if len(waypoints) < 2:
        raise PathError("a path needs at least two waypoints")
    if look_at is not None:
        look_at = np.asarray(look_at, dtype=float)
        # Positions only: the orientation comes from the point.
        waypoints = [w.copy() for w in waypoints]
        for w in waypoints:
            w.wxyz = waypoints[0].wxyz.copy()
    stops = corner_stops(waypoints)
    for k in range(len(waypoints) - 1):
        if np.linalg.norm(waypoints[k + 1].position - waypoints[k].position) < 1e-6 and angle_between_wxyz(
            waypoints[k].wxyz, waypoints[k + 1].wxyz
        ) < 1e-6:
            raise PathError(f"segment {k + 1} has zero length (waypoints {k + 1} and {k + 2} are equal)", segments=[k])
    pieces = segment_pieces(waypoints, step)
    positions, quats, speed, stop, owner = [], [], [], [], []
    waypoint_samples = [0] * len(waypoints)
    for k in range(len(waypoints) - 1):
        points, qs = pieces[k]
        first = 0 if k == 0 else 1
        if k > 0 and (stops[k] or waypoints[k].dwell > 0.0):
            stop[-1] = True  # corner, or a rest at the waypoint
        for i in range(first, len(points)):
            positions.append(points[i])
            quats.append(qs[i])
            speed.append(MAX_SPEED_M_S)
            stop.append(False)
            owner.append(k)
        waypoint_samples[k + 1] = len(positions) - 1
    stop_arr = np.asarray(stop, dtype=bool)
    stop_arr[0] = stop_arr[-1] = True
    pos, qts = np.asarray(positions), np.asarray(quats)
    if look_at is not None:
        distance = np.linalg.norm(pos - look_at, axis=1)
        if distance.min() < LOOK_AT_MIN_DISTANCE_M:
            near = sorted({int(owner[i]) for i in np.flatnonzero(distance < LOOK_AT_MIN_DISTANCE_M)})
            raise PathError(
                f"the path passes within {1000 * LOOK_AT_MIN_DISTANCE_M:.0f} mm of the look-at point", segments=near
            )
        qts = look_at_orientations(pos, waypoints[0].wxyz, look_at)
    dp = np.linalg.norm(np.diff(pos, axis=0), axis=1)
    dr = np.array([angle_between_wxyz(qts[i], qts[i + 1]) for i in range(len(qts) - 1)])
    length = np.concatenate([[0.0], np.cumsum(np.sqrt(dp**2 + (ROTATION_LENGTH_M_PER_RAD * dr) ** 2))])
    return SampledPath(pos, qts, np.asarray(speed, dtype=float), stop_arr, np.asarray(owner), waypoint_samples, length)


def path_part(payload: dict, waypoint_count: int) -> tuple[int, int]:
    """Waypoint indices (first, last) a planner request covers: "part" or the
    whole path."""
    last = waypoint_count - 1
    part = payload.get("part")
    if not part:
        return 0, last
    first = int(np.clip(int(part[0]), 0, max(last - 1, 0)))
    return first, int(np.clip(int(part[1]), first + 1, last))


def slice_path(path: SampledPath, first: int, last: int) -> SampledPath:
    """Samples from waypoint first to waypoint last (indices of the whole
    path), at rest at both ends. Segment indices stay those of the whole path."""
    a, b = path.waypoint_samples[first], path.waypoint_samples[last] + 1
    stop = path.stop[a:b].copy()
    stop[0] = stop[-1] = True
    segment = path.segment[a:b].copy()
    segment[0] = first  # the waypoint sample ends the previous segment
    return SampledPath(
        path.positions[a:b],
        path.wxyz[a:b],
        path.speed[a:b],
        stop,
        segment,
        [i - a for i in path.waypoint_samples[first:last + 1]],
        path.length[a:b] - path.length[a],
    )


# ---------------------------------------------------------------------------
# Inverse kinematics along a Cartesian path
# ---------------------------------------------------------------------------


class PathIk:
    """Damped least-squares IK that follows a dense Cartesian path, seeded by
    the previous sample, so the joint configuration never switches branch."""

    def __init__(self, urdf_xml: str, joint_names: list[str], tip_frame: str = "flange"):
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        self.frame = self.model.getFrameId(tip_frame)
        self.base = self.model.getFrameId("base_link")
        self.q_index = [self.model.joints[self.model.getJointId(n)].idx_q for n in joint_names]
        self.v_index = [self.model.joints[self.model.getJointId(n)].idx_v for n in joint_names]

    def full(self, q6: np.ndarray) -> np.ndarray:
        q = pin.neutral(self.model)
        q[self.q_index] = q6
        return q

    def fk(self, q6: np.ndarray) -> pin.SE3:
        pin.framesForwardKinematics(self.model, self.data, self.full(q6))
        return self.data.oMf[self.base].inverse() * self.data.oMf[self.frame]

    def solve(self, target: pin.SE3, seed: np.ndarray, iterations: int = 30, tol: float = 1e-6) -> tuple[np.ndarray, float]:
        q = self.full(seed)
        damping = 1e-6
        error = np.inf
        for _ in range(iterations):
            pin.framesForwardKinematics(self.model, self.data, q)
            current = self.data.oMf[self.base].inverse() * self.data.oMf[self.frame]
            err = pin.log6(current.inverse() * target).vector
            error = float(np.linalg.norm(err))
            if error < tol:
                break
            jac = pin.computeFrameJacobian(self.model, self.data, q, self.frame, pin.LOCAL)[:, self.v_index]
            dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(6), err)
            step = np.zeros(self.model.nv)
            step[self.v_index] = dq
            q = pin.integrate(self.model, q, step)
        return q[self.q_index].copy(), error

    @staticmethod
    def reach_hint(q: np.ndarray, position: np.ndarray) -> str:
        """Why a pose is not reachable, from the closest configuration q the IK
        found (an operator hint, UR-like arm)."""
        if abs(math.sin(q[2])) < 0.1:
            return " (arm fully stretched, j3 ≈ 0°)"
        if abs(math.sin(q[4])) < 0.1:
            return " (wrist singularity, j5 ≈ 0°)"
        if np.linalg.norm(np.asarray(position)[:2]) < 0.35:
            return " (too close to the robot's base axis)"
        return ""

    def follow(self, path: SampledPath, seed: np.ndarray, max_step_rad: float = 0.25) -> np.ndarray:
        """Joint positions for every sample. Raises PathError at the first
        unreachable sample or configuration jump."""
        q = np.asarray(seed, dtype=float)
        result = np.zeros((len(path.positions), len(q)))
        for k, (p, wxyz) in enumerate(zip(path.positions, path.wxyz)):
            target = pin.SE3(quat_wxyz_to_matrix(wxyz), p)
            q_new, error = self.solve(target, q)
            if error > 1e-4:
                raise PathError(
                    f"pose out of the robot's reach{self.reach_hint(q_new, p)}"
                    f" at {100.0 * k / max(1, len(path.positions) - 1):.0f} % of the path",
                    segments=[int(path.segment[k])],
                    fraction=k / max(1, len(path.positions) - 1),
                    position=p,
                )
            if k > 0 and np.max(np.abs(q_new - q)) > max_step_rad:
                raise PathError(
                    f"joint configuration jump (singularity) at {100.0 * k / (len(path.positions) - 1):.0f} % of the path",
                    segments=[int(path.segment[k])],
                    fraction=k / max(1, len(path.positions) - 1),
                    position=p,
                )
            result[k] = q_new
            q = q_new
        return result


# ---------------------------------------------------------------------------
# Time parameterization
# ---------------------------------------------------------------------------


@dataclass
class TimedTrajectory:
    times: np.ndarray
    positions: np.ndarray
    velocities: np.ndarray
    accelerations: np.ndarray
    info: dict
    segment_fastest_s: list = field(default_factory=list)
    segment_planned_s: list = field(default_factory=list)
    waypoint_times_s: list = field(default_factory=list)  # arrival at each waypoint [s] (Cartesian paths)


def _derivative(values: np.ndarray, s: np.ndarray) -> np.ndarray:
    ds = np.maximum(np.gradient(s), 1e-9)
    return np.gradient(values, axis=0) / ds[:, None]


def _spline_resample(times: np.ndarray, positions: np.ndarray, dt: float) -> TimedTrajectory:
    keep = np.concatenate([[True], np.diff(times) > 1e-9])
    times, positions = times[keep], positions[keep]
    spline = CubicSpline(times, positions, axis=0, bc_type="clamped")
    count = int(math.ceil(times[-1] / dt)) + 1
    t = np.minimum(np.arange(count) * dt, times[-1])
    t[-1] = times[-1]
    return TimedTrajectory(t, spline(t), spline(t, 1), spline(t, 2), {})


def smooth_recording(
    times: np.ndarray, positions: np.ndarray, knot_spacing: float = 0.04, dt: float = 0.004
) -> tuple[np.ndarray, np.ndarray]:
    """Measured joint samples -> smooth, uniformly sampled motion.

    /joint_states is stamped when the broadcaster publishes, while the FAIRINO
    feedback frame behind it can be up to 4 ms older, and a frame is sometimes
    published twice. Differentiating that directly gave apparent accelerations
    of > 100 rad/s^2 for a motion executed at 9.4 rad/s^2. Repeated (stale)
    samples are dropped and a least-squares cubic B-spline with knots every
    knot_spacing seconds (a ~1/(2 knot_spacing) Hz low-pass that accepts
    irregular samples) is fitted per joint."""
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    keep = np.ones(len(times), dtype=bool)
    moved = np.max(np.abs(np.diff(positions, axis=0)), axis=1)
    for k in range(1, len(times) - 1):
        stale = moved[k - 1] == 0.0 and moved[k] > 0.0 and k >= 2 and moved[k - 2] > 0.0
        too_close = times[k] - times[k - 1] < 1e-3
        keep[k] = not (stale or too_close)
    times, positions = times[keep], positions[keep]
    duration = times[-1] - times[0]
    if duration < 4 * knot_spacing or len(times) < 8:
        return times - times[0], positions
    t = times - times[0]
    interior = np.arange(knot_spacing, duration - knot_spacing * 0.5, knot_spacing)
    # Drop knots without data in their span (Schoenberg-Whitney condition).
    interior = np.array([k for k in interior if np.any(np.abs(t - k) < knot_spacing)])
    knots = np.concatenate([[0.0] * 4, interior, [duration] * 4])
    spline = make_lsq_spline(t, positions, knots, k=3, axis=0)
    grid = np.minimum(np.arange(int(math.ceil(duration / dt)) + 1) * dt, duration)
    grid[-1] = duration
    smooth = spline(grid)
    smooth[0], smooth[-1] = positions[0], positions[-1]
    return grid, smooth


def limit_ratios(traj: TimedTrajectory, qd_max: np.ndarray, qdd_max: np.ndarray) -> tuple[float, float]:
    vel = float(np.max(np.abs(traj.velocities) / qd_max)) if len(traj.times) else 0.0
    acc = float(np.max(np.abs(traj.accelerations) / qdd_max)) if len(traj.times) else 0.0
    return vel, acc


def fit_to_limits(
    times: np.ndarray, positions: np.ndarray, qd_max: np.ndarray, qdd_max: np.ndarray, dt: float, max_iterations: int = 12
) -> TimedTrajectory:
    """Spline through timed samples; time is stretched until joint velocity and
    acceleration stay within the limits."""
    scale = 1.0
    traj = _spline_resample(times, positions, dt)
    for _ in range(max_iterations):
        vel, acc = limit_ratios(traj, qd_max, qdd_max)
        if vel <= 1.0 and acc <= 1.0:
            break
        factor = max(vel, math.sqrt(acc), 1.0) * 1.03
        scale *= factor
        traj = _spline_resample(times * scale, positions, dt)
    vel, acc = limit_ratios(traj, qd_max, qdd_max)
    if vel > 1.02 or acc > 1.02:
        raise PathError(f"trajectory exceeds the joint limits (velocity {vel:.2f}, acceleration {acc:.2f} of the limit)")
    traj.info = {"time_scale": scale, "velocity_ratio": vel, "acceleration_ratio": acc}
    return traj


def fit_recorded_timing(
    times: np.ndarray,
    positions: np.ndarray,
    qd_max: np.ndarray,
    qdd_max: np.ndarray,
    dt: float,
    window_s: float = 0.12,
    iterations: int = 25,
) -> TimedTrajectory:
    """Keeps the recorded timing wherever it respects the joint limits and
    stretches time only around the places where it does not (a measured
    motion can contain short jerks, e.g. SimMachine catching up after a
    stall). Falls back to a uniform slowdown (fit_to_limits) at the end."""
    times = np.asarray(times, dtype=float)
    intervals = np.diff(times)
    original = float(times[-1] - times[0])
    stretched = False
    for _ in range(iterations):
        t = np.concatenate([[0.0], np.cumsum(intervals)])
        traj = _spline_resample(t, positions, dt)
        ratio = np.maximum(
            np.max(np.abs(traj.velocities) / qd_max, axis=1),
            np.sqrt(np.max(np.abs(traj.accelerations) / qdd_max, axis=1)),
        )
        bad = np.where(ratio > 1.0)[0]
        if len(bad) == 0:
            break
        stretched = True
        factor = np.ones(len(intervals))
        for k in bad:
            centre = traj.times[k]
            span = np.where(np.abs(t[:-1] - centre) < window_s)[0]
            if len(span) == 0:
                span = np.array([int(np.clip(np.searchsorted(t, centre) - 1, 0, len(intervals) - 1))])
            # Cosine taper: full stretch at the violation, none at the window edge.
            weight = 0.5 * (1.0 + np.cos(np.pi * np.clip(np.abs(t[span] - centre) / window_s, 0.0, 1.0)))
            factor[span] = np.maximum(factor[span], 1.0 + (ratio[k] * 1.05 - 1.0) * weight)
        intervals = intervals * factor
    t = np.concatenate([[0.0], np.cumsum(intervals)])
    traj = fit_to_limits(t, positions, qd_max, qdd_max, dt)
    traj.info["recorded_duration_s"] = original
    traj.info["locally_stretched"] = 1.0 if stretched else 0.0
    traj.info["time_scale"] = float(traj.times[-1] / original) if original > 0 else 1.0
    return traj


def _path_limits(q: np.ndarray, s: np.ndarray, qd_max: np.ndarray, qdd_max: np.ndarray):
    """Per-sample path speed limits from the joint velocity limits and the
    curvature term (half the acceleration budget), and the path acceleration
    limit (the other half)."""
    dq = np.abs(_derivative(q, s))
    ddq = np.abs(_derivative(_derivative(q, s), s))
    with np.errstate(divide="ignore", invalid="ignore"):
        v_joint = np.min(np.where(dq > 1e-9, qd_max / dq, np.inf), axis=1)
        v_curv = np.min(np.where(ddq > 1e-9, np.sqrt(0.5 * qdd_max / ddq), np.inf), axis=1)
        a_path = np.min(np.where(dq > 1e-9, 0.5 * qdd_max / dq, np.inf), axis=1)
    return v_joint, v_curv, np.minimum(a_path, 50.0)


def _segment_bounds(segment: np.ndarray) -> list[tuple[int, int]]:
    """(first, last) sample index of each segment; first is the shared sample
    where the previous segment ends."""
    bounds = []
    for k in range(int(segment.max()) + 1):
        idx = np.where(segment == k)[0]
        bounds.append((max(0, int(idx[0]) - 1) if k > 0 else 0, int(idx[-1])))
    return bounds


def parameterize_path(
    q: np.ndarray,
    s: np.ndarray,
    max_path_speed: np.ndarray,
    stops: np.ndarray,
    qd_max: np.ndarray,
    qdd_max: np.ndarray,
    dt: float,
    segment: np.ndarray | None = None,
    durations: list[float | None] | None = None,
) -> TimedTrajectory:
    """Timing along a fixed joint path q(s): as fast as the path speed limit,
    the joint velocity/acceleration limits and the rest at stop samples allow;
    segments with a requested duration (longer than their fastest time) are
    slowed down to it.

    Velocity limit curve (path speed, joint velocity, curvature term) and
    forward/backward acceleration passes; where the exact spline still exceeds
    a joint limit only that region is slowed down; then requested durations
    are met by capping the speed of their segment (bisection); a uniform
    slowdown (fit_to_limits) is the last fallback. info carries the fastest
    and the planned duration of every segment."""
    s = np.asarray(s, dtype=float)
    v_joint, v_curv, a_path = _path_limits(q, s, qd_max, qdd_max)
    v_lim = np.minimum.reduce([np.asarray(max_path_speed, dtype=float), v_joint, v_curv])
    v_lim = np.where(np.isfinite(v_lim), v_lim, 10.0)
    v_lim[np.asarray(stops, dtype=bool)] = 0.0
    ds = np.diff(s)
    window = 8

    def timing(limit: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        v = limit.copy()
        for k in range(len(v) - 1):
            v[k + 1] = min(v[k + 1], math.sqrt(v[k] ** 2 + 2.0 * a_path[k] * ds[k]))
        for k in range(len(v) - 2, -1, -1):
            v[k] = min(v[k], math.sqrt(v[k + 1] ** 2 + 2.0 * a_path[k + 1] * ds[k]))
        mean = 0.5 * (v[:-1] + v[1:])
        with np.errstate(divide="ignore", invalid="ignore"):
            dt_seg = np.where(
                mean > 1e-9, ds / np.maximum(mean, 1e-12), 2.0 * np.sqrt(ds / np.maximum(a_path[:-1], 1e-6))
            )
        return np.concatenate([[0.0], np.cumsum(dt_seg)]), v

    # The curvature term is only estimated above: slow down only where the
    # exact spline exceeds a limit (a uniform slowdown would penalize the
    # whole path for one tight spot).
    def local_slowdown(v_lim: np.ndarray) -> int:
        iterations = 0
        for iterations in range(1, 16):
            times, v = timing(v_lim)
            traj = _spline_resample(times, q, dt)
            ratio_v = np.max(np.abs(traj.velocities) / qd_max, axis=1)
            ratio_a = np.max(np.abs(traj.accelerations) / qdd_max, axis=1)
            factor = np.minimum(1.0 / np.maximum(ratio_v, 1e-9), 1.0 / np.sqrt(np.maximum(ratio_a, 1e-9)))
            bad = np.where(factor < 1.0)[0]
            if len(bad) == 0:
                break
            path_index = np.clip(np.searchsorted(times, traj.times[bad]), 0, len(v_lim) - 1)
            for k, f in zip(path_index, factor[bad]):
                lo, hi = max(0, k - window), min(len(v_lim), k + window + 1)
                v_lim[lo:hi] = np.minimum(v_lim[lo:hi], v[lo:hi] * f * 0.95)
        return iterations

    local_iterations = local_slowdown(v_lim)

    bounds = _segment_bounds(np.asarray(segment)) if segment is not None else []
    times, v = timing(v_lim)
    fastest = [float(times[b] - times[a]) for a, b in bounds]
    # Requested durations: cap the speed inside the segment until it takes
    # that long (neighbours keep their own timing; two sweeps settle the
    # coupling through the acceleration passes).
    if bounds and durations:
        base = v_lim.copy()
        caps = np.full(len(bounds), np.inf)
        # A duration may come with a speed (drawn strokes): the time is then
        # between the drawn one (speed 0) and the fastest (speed 1).
        targets = []
        for k, d in enumerate(durations[: len(bounds)]):
            speed = None
            if isinstance(d, (tuple, list)):
                d, speed = d
            if d is not None and speed is not None and d > fastest[k]:
                d = d - float(speed) * (d - fastest[k])
            targets.append(d)
        wanted = [(k, float(d)) for k, d in enumerate(targets) if d is not None and d > fastest[k] * 1.001]
        for _ in range(2 if len(wanted) > 1 else 1):
            for k, target in wanted:
                a, b = bounds[k]
                lo, hi = 1e-4, float(max(np.max(v[a : b + 1]), 1e-3))
                for _ in range(22):
                    mid = 0.5 * (lo + hi)
                    caps[k] = mid
                    limit = base.copy()
                    for j, (a2, b2) in enumerate(bounds):
                        if np.isfinite(caps[j]):
                            limit[a2 : b2 + 1] = np.minimum(limit[a2 : b2 + 1], caps[j])
                    t_try, _ = timing(limit)
                    if t_try[b] - t_try[a] > target:
                        lo = mid
                    else:
                        hi = mid
                caps[k] = 0.5 * (lo + hi)
        for j, (a2, b2) in enumerate(bounds):
            if np.isfinite(caps[j]):
                v_lim[a2 : b2 + 1] = np.minimum(v_lim[a2 : b2 + 1], caps[j])
        # A capped segment changes speed abruptly at its ends: settle that
        # locally too, instead of slowing the whole path down uniformly.
        local_iterations += local_slowdown(v_lim)
        times, v = timing(v_lim)
    traj = fit_to_limits(times, q, qd_max, qdd_max, dt)
    scale = traj.info.get("time_scale", 1.0)
    traj.info["path_speed_max_m_s"] = float(np.max(v))
    traj.info["local_slowdown_iterations"] = float(local_iterations)
    traj.segment_fastest_s = [f * scale for f in fastest]
    traj.segment_planned_s = [float(times[b] - times[a]) * scale for a, b in bounds]
    return traj


def insert_dwells(traj: TimedTrajectory, at_times: list[float], dwells: list[float], dt: float) -> TimedTrajectory:
    """Rests of dwells[i] seconds at at_times[i] (where the trajectory is at
    rest, e.g. a waypoint with a stop): the motion after it starts later."""
    pairs = sorted((float(t), float(d)) for t, d in zip(at_times, dwells) if d > 0.0)
    if not pairs:
        return traj
    times, columns = traj.times, (traj.positions, traj.velocities, traj.accelerations)
    out_t: list[np.ndarray] = []
    out: list[list[np.ndarray]] = [[], [], []]
    shift, last = 0.0, 0
    for t_stop, dwell in pairs:
        k = int(np.searchsorted(times, t_stop))
        out_t.append(times[last:k] + shift)
        for o, c in zip(out, columns):
            o.append(c[last:k])
        hold = np.arange(0.0, dwell, dt)
        q = np.array([np.interp(t_stop, times, traj.positions[:, j]) for j in range(traj.positions.shape[1])])
        out_t.append(t_stop + shift + hold)
        out[0].append(np.repeat(q[None, :], len(hold), axis=0))
        out[1].append(np.zeros((len(hold), q.size)))
        out[2].append(np.zeros((len(hold), q.size)))
        shift += dwell
        last = k
    out_t.append(times[last:] + shift)
    for o, c in zip(out, columns):
        o.append(c[last:])
    return TimedTrajectory(
        np.concatenate(out_t), *(np.vstack(o) for o in out), dict(traj.info),
        list(traj.segment_fastest_s), list(traj.segment_planned_s),
    )


# ---------------------------------------------------------------------------
# Recordings -> waypoints
# ---------------------------------------------------------------------------


def simplify_poses(
    positions: np.ndarray, wxyz: np.ndarray, position_tol: float = 0.003, rotation_tol: float = math.radians(2.0)
) -> list[int]:
    """Ramer-Douglas-Peucker on TCP poses (position distance to the chord and
    orientation distance to the chord's slerp). Returns kept indices."""
    keep = {0, len(positions) - 1}
    stack = [(0, len(positions) - 1)]
    while stack:
        a, b = stack.pop()
        if b - a < 2:
            continue
        chord = positions[b] - positions[a]
        length = np.linalg.norm(chord)
        worst, worst_index = 0.0, -1
        for k in range(a + 1, b):
            if length > 1e-9:
                u = float(np.clip(np.dot(positions[k] - positions[a], chord) / length**2, 0.0, 1.0))
            else:
                u = (k - a) / (b - a)
            dist = np.linalg.norm(positions[a] + u * chord - positions[k]) / position_tol
            rot = angle_between_wxyz(slerp_wxyz(wxyz[a], wxyz[b], u), wxyz[k]) / rotation_tol
            score = max(dist, rot)
            if score > worst:
                worst, worst_index = score, k
        if worst > 1.0:
            keep.add(worst_index)
            stack.append((a, worst_index))
            stack.append((worst_index, b))
    return sorted(keep)


def tracking_report(
    plan_t: np.ndarray,
    plan_q: np.ndarray,
    act_t: np.ndarray,
    act_q: np.ndarray,
    fk,
    servoj: np.ndarray | None = None,
) -> dict:
    """How well an executed trajectory followed its plan.

    plan_t/plan_q: planned times and joints; act_t/act_q: measured joints
    (/joint_states) during the execution; fk(q) -> pin.SE3 of the TCP.
    servoj: optional rows (t, executed ServoJ counter mod 10000, flow skips,
    limited points, cmdT) from /fairino_robot/status.
    Deviation = distance of every measured TCP position to the planned TCP
    path (spatial, so a robot that is late but on the path counts as 0)."""
    from scipy.spatial import cKDTree

    plan_q = np.asarray(plan_q, dtype=float)
    act_q = np.asarray(act_q, dtype=float)
    stride = max(1, len(plan_q) // 20000)
    planned = np.array([fk(q).translation for q in plan_q[::stride]] + [fk(plan_q[-1]).translation])
    # Densify the planned TCP polyline so the distance is to the line, not
    # to its vertices.
    fine = np.vstack([planned[:-1] + (planned[1:] - planned[:-1]) * f for f in np.linspace(0.0, 1.0, 4, endpoint=False)]
                     + [planned[-1:]])
    measured = np.array([fk(q).translation for q in act_q])
    deviation = cKDTree(fine).query(measured)[0]
    moving = np.flatnonzero(np.max(np.abs(np.diff(act_q, axis=0)), axis=1) > 1e-5)
    duration = float(act_t[moving[-1] + 1] - act_t[moving[0]]) if len(moving) else 0.0
    report = {
        "max_deviation_mm": round(1000.0 * float(deviation.max()), 2),
        "mean_deviation_mm": round(1000.0 * float(deviation.mean()), 3),
        "p95_deviation_mm": round(1000.0 * float(np.percentile(deviation, 95)), 2),
        "worst_at_s": round(float(act_t[int(np.argmax(deviation))] - (act_t[moving[0]] if len(moving) else act_t[0])), 2),
        "executed_s": round(duration, 3),
        "planned_s": round(float(plan_t[-1]), 3),
    }
    if servoj is not None and len(servoj) >= 2:
        servoj = np.asarray(servoj, dtype=float)
        executed = np.concatenate([[0.0], np.cumsum(np.mod(np.diff(servoj[:, 1]), 10000.0))])
        rates = []
        for start in np.arange(servoj[0, 0], servoj[-1, 0] - 0.5, 0.25):
            window = (servoj[:, 0] >= start) & (servoj[:, 0] <= start + 0.5)
            if np.count_nonzero(window) >= 2:
                t, n = servoj[window, 0], executed[window]
                rates.append((n[-1] - n[0]) / (t[-1] - t[0]))
        report.update(
            servoj_min_rate=round(min(rates)) if rates else None,
            servoj_mean_rate=round(float(executed[-1] / (servoj[-1, 0] - servoj[0, 0]))),
            servoj_expected_rate=round(1.0 / servoj[-1, 4]) if servoj[-1, 4] > 0 else None,
            servoj_limited_points=int(servoj[-1, 3] - servoj[0, 3]),
            servoj_flow_skips=int(servoj[-1, 2] - servoj[0, 2]),
        )
    return report


def _recording_tangent(points: np.ndarray, i: int) -> np.ndarray:
    """Symmetric tangent of curve waypoint i of a converted recording: along
    the bisector of the two neighbouring chords, at most a third of the
    shorter one (no loops where the spacing is uneven); zero where the
    direction reverses by more than 90 degrees (the robot stops there)."""
    a, b = points[i] - points[i - 1], points[i + 1] - points[i]
    la, lb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if la < 1e-9 or lb < 1e-9 or float(a @ b) < 0.0:
        return np.zeros(3)
    direction = a / la + b / lb
    norm = float(np.linalg.norm(direction))
    return direction / norm * min(la, lb) / 3.0 if norm > 1e-9 else np.zeros(3)


def _segment_deviation(positions: np.ndarray, waypoints: list[Waypoint], k: int, first: int, last: int):
    """Largest distance between segment k and the recording samples
    first..last it replaces, both ways (samples to curve and curve to the
    recorded polyline). Returns (distance, sample index to split at)."""
    from scipy.spatial import cKDTree

    curve, _ = segment_points(waypoints, k, 0.001)
    recorded = positions[first:last + 1]
    d_samples, _ = cKDTree(curve).query(recorded)
    fine = np.vstack([recorded[:-1] + (recorded[1:] - recorded[:-1]) * f for f in np.linspace(0.0, 1.0, 8, endpoint=False)]
                     + [recorded[-1:]])
    d_curve, nearest = cKDTree(fine).query(curve)
    if d_samples[1:-1].size and d_samples[1:-1].max() >= d_curve.max():
        return float(d_samples.max()), first + 1 + int(np.argmax(d_samples[1:-1]))
    # The curve bulges away from the recording: split at the sample nearest
    # to the worst curve point.
    split = first + int(np.clip(round(nearest[int(np.argmax(d_curve))] / 8), 1, max(1, last - first - 1)))
    return float(d_curve.max()), split


def recording_to_path(
    recording: dict, position_tol: float = 0.003, max_waypoints: int = 3000
) -> tuple[list[Waypoint], list[Segment]]:
    """Simplified recording as curve waypoints (no stops, except where the
    motion reverses); each segment keeps the duration it had in the
    recording (limited to the fastest feasible when planned).

    RDP picks the waypoints; then every segment is compared with the samples
    it replaces and split until the curve stays within position_tol of the
    recording both ways (measured on SimMachine recordings: 55 mm overshoot
    with the earlier Catmull-Rom tangents, < 3 mm with this)."""
    samples = recording["samples"]
    if len(samples) < 2:
        raise PathError("the recording has fewer than two samples")
    poses = [fairino_to_pose(s["pose"]) for s in samples]
    positions = np.array([p for p, _ in poses])
    quats = np.array([q for _, q in poses])
    times = np.array([float(s["t"]) for s in samples])
    return poses_to_path(times, positions, quats, position_tol, max_waypoints)


def poses_to_path(
    times: np.ndarray, positions: np.ndarray, quats: np.ndarray, position_tol: float = 0.003, max_waypoints: int = 3000,
    rotation_tol: float = math.radians(2.0),
) -> tuple[list[Waypoint], list[Segment]]:
    """Timed TCP poses -> curve waypoints and segments with their durations
    (see recording_to_path)."""
    keep = simplify_poses(positions, quats, position_tol, rotation_tol)

    def build(keep: list[int]) -> list[Waypoint]:
        points = positions[keep]
        waypoints = [Waypoint(positions[k].copy(), quats[k].copy()) for k in keep]
        for i in range(1, len(waypoints) - 1):
            waypoints[i].tangent = _recording_tangent(points, i)
            waypoints[i].curve = bool(np.linalg.norm(waypoints[i].tangent) > 1e-9)
        return waypoints

    # A segment's shape depends on its two waypoints and their neighbours
    # (tangents): cache the deviation by those four sample indices.
    cache: dict[tuple, tuple[float, int]] = {}
    while True:
        waypoints = build(keep)
        splits = []
        for k in range(len(keep) - 1):
            if keep[k + 1] - keep[k] < 2:
                continue
            key = (keep[max(k - 1, 0)], keep[k], keep[k + 1], keep[min(k + 2, len(keep) - 1)])
            if key not in cache:
                cache[key] = _segment_deviation(positions, waypoints, k, keep[k], keep[k + 1])
            distance, split = cache[key]
            if distance > position_tol and keep[k] < split < keep[k + 1]:
                splits.append(split)
        if not splits or len(keep) + len(splits) > max_waypoints:
            break
        keep = sorted(set(keep) | set(splits))
    segments = [Segment(float(times[keep[n + 1]] - times[keep[n]]), "recorded") for n in range(len(keep) - 1)]
    return waypoints, segments


DRAWING_DWELL_MIN_S = 0.5  # holding the gizmo still at least this long is a stop
DRAWING_STILL_M = 0.001
DRAWING_STILL_RAD = math.radians(1.0)


def drawing_to_path(
    times: np.ndarray,
    positions: np.ndarray,
    quats: np.ndarray,
    dwell_min_s: float = DRAWING_DWELL_MIN_S,
    position_tol: float = 0.003,
    rotation_tol: float = math.radians(2.0),
) -> tuple[list[Waypoint], list[Segment]]:
    """Gizmo motion drawn while it was held (servo view) -> waypoints and
    segments. Holding the gizmo still for dwell_min_s or longer is a stop: a
    waypoint with that rest (dwell). The moving parts become curve waypoints
    with their drawn durations (poses_to_path). The first waypoint is the
    first sample."""
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    quats = np.asarray(quats, dtype=float)
    n = len(times)
    if n == 0:
        return [], []

    def still(a: int, k: int) -> bool:
        return (
            np.linalg.norm(positions[k] - positions[a]) < DRAWING_STILL_M
            and angle_between_wxyz(quats[a], quats[k]) < DRAWING_STILL_RAD
        )

    # Still intervals (first, last sample) lasting dwell_min_s or longer.
    stills, a = [], 0
    for k in range(1, n + 1):
        if k < n and still(a, k):
            continue
        if times[k - 1] - times[a] >= dwell_min_s:
            stills.append((a, k - 1))
        a = k
    waypoints = [Waypoint(positions[0].copy(), quats[0].copy())]
    segments: list[Segment] = []
    cursor = 0
    for a, b in stills + [(n - 1, n - 1)]:
        if a > cursor and any(not still(cursor, k) for k in range(cursor + 1, a + 1)):
            piece_wps, piece_segs = poses_to_path(
                times[cursor:a + 1], positions[cursor:a + 1], quats[cursor:a + 1], position_tol, rotation_tol=rotation_tol
            )
            for segment in piece_segs:
                segment.origin = "drawn"
            waypoints += piece_wps[1:]
            segments += piece_segs
        if b > a:
            waypoints[-1].dwell += float(times[b] - times[a])
        cursor = b
    # Equal neighbours (a short pause inside a moving part) would be a
    # zero-length segment: merge them.
    k = 1
    while k < len(waypoints):
        if (
            np.linalg.norm(waypoints[k].position - waypoints[k - 1].position) < 1e-6
            and angle_between_wxyz(waypoints[k].wxyz, waypoints[k - 1].wxyz) < 1e-6
        ):
            waypoints[k - 1].dwell += waypoints[k].dwell
            pause = segments[k - 1].duration or 0.0
            del waypoints[k]
            del segments[k - 1]
            if k - 1 < len(segments) and segments[k - 1].duration is not None:
                segments[k - 1] = Segment(segments[k - 1].duration + pause, segments[k - 1].origin)
        else:
            k += 1
    return waypoints, segments


# ---------------------------------------------------------------------------
# Files
# ---------------------------------------------------------------------------


def default_fbx_path_for_urdf(urdf_path: str) -> str:
    resolved = Path(urdf_path)
    return str(resolved.parent.parent / "fbx" / f"{resolved.stem}.fbx")


def recording_payload(
    *,
    robot_name: str,
    urdf_path: str,
    joint_names: list[str],
    samples: list[dict],
    segments: list[dict],
    name: str,
    tool_number: int | None,
) -> dict:
    """Same keys as trajectory_io.build_trajectory_payload of the FAIRINO
    Viser prototype (Blender replay reads samples[*].joint_positions and
    fbx_path), plus frame/unit information."""
    return {
        "format_version": RECORDING_FORMAT_VERSION,
        "name": name,
        "robot_name": robot_name,
        "urdf_path": urdf_path,
        "fbx_path": default_fbx_path_for_urdf(urdf_path),
        "joint_names": list(joint_names),
        "base_transform": np.eye(4).tolist(),
        "frame": "flange",
        "base_frame": "base_link",
        "pose_units": "x, y, z [mm]; rx, ry, rz [deg] about the fixed axes X, Y, Z (FAIRINO WebApp)",
        "joint_units": "rad",
        "fairino_tool_number": tool_number,
        "sample_count": len(samples),
        "segment_count": len(segments),
        "total_duration_s": float(samples[-1]["t"]) if samples else 0.0,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "source": "fairino_ros2 Viser UI",
        "segments": segments,
        "samples": samples,
    }


def path_payload(
    name: str, waypoints: list[Waypoint], segments: list[Segment], look_at: np.ndarray | None = None,
    strokes: dict | None = None,
) -> dict:
    payload = {
        "format": PATH_FORMAT,
        "version": 3,
        "name": name,
        "frame": "flange",
        "base_frame": "base_link",
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "waypoints": [w.to_dict() for w in waypoints],
        "segments": [s.to_dict() for s in segments],
        # Point the tool z axis looks at along the whole path [m, base_link];
        # null: the waypoints' own orientations.
        "look_at_m": None if look_at is None else [float(v) for v in look_at],
    }
    if strokes:
        # Drawn strokes (servo view): the drawing's samples, filter tolerance
        # and speed, so they can be tuned again after loading.
        payload["strokes"] = strokes
    return payload


def look_at_from_payload(payload: dict) -> np.ndarray | None:
    point = payload.get("look_at_m")
    return None if point is None else np.asarray(point, dtype=float)


def path_from_payload(payload: dict) -> tuple[list[Waypoint], list[Segment]]:
    """Path file or planner request -> waypoints and segments. Older files
    (version 1/2) load as straight segments at the fastest timing."""
    if payload.get("format", PATH_FORMAT) != PATH_FORMAT:
        raise ValueError("not a fairino_path file")
    version = int(payload.get("version", 3))
    if version < 3:
        waypoints = [
            Waypoint(np.asarray(w["position_m"], dtype=float), np.asarray(w["wxyz"], dtype=float) / np.linalg.norm(w["wxyz"]))
            for w in payload["waypoints"]
        ]
        return waypoints, [Segment() for _ in range(max(0, len(waypoints) - 1))]
    waypoints = [Waypoint.from_dict(w) for w in payload["waypoints"]]
    segments = [Segment.from_dict(s) for s in payload.get("segments", [])]
    if len(segments) != max(0, len(waypoints) - 1):
        segments = [Segment() for _ in range(max(0, len(waypoints) - 1))]
    return waypoints, segments


def safe_name(text: str, default: str) -> str:
    cleaned = "".join(c if c.isalnum() or c in "-_" else "_" for c in text.strip())
    return cleaned.strip("_") or default


def write_json(path: Path, payload: dict) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, indent=1), encoding="utf-8")
    tmp.replace(path)  # atomic: a crash never leaves half a file
    return path
