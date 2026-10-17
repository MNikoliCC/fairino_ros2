"""Viser panels for trajectory recording/replay and the Cartesian path editor.

Both panels only build requests: planning, IK and timing run in the planner
(/fairinoplan/path_goal), and every motion goes through the same
plan -> preview -> Execute flow as a normal plan. Recording only reads
/joint_states. See docs/trajectory_recording_and_paths.md.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import pinocchio as pin
import viser
from std_msgs.msg import String

import fairino_trajectory_tools as tt

JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]
TIMING_LABELS = ["Recorded timing", "Fastest within limits"]


def resolve_trajectory_dir(configured: str) -> Path:
    """Configured dir, else the bind-mounted /ros2_ws/trajectories, else ~/.ros."""
    candidates = [configured] if configured else []
    candidates += [os.environ.get("FAIRINO_TRAJECTORY_DIR", ""), "/ros2_ws/trajectories"]
    for candidate in candidates:
        if candidate and os.path.isdir(candidate) and os.access(candidate, os.W_OK):
            return Path(candidate)
    fallback = Path.home() / ".ros" / "fairino_trajectories"
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def list_json(directory: Path) -> list[str]:
    if not directory.is_dir():
        return []
    files = sorted(directory.glob("*.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    return [f.name for f in files]


# ===========================================================================
# Recording and replay
# ===========================================================================


AUTOSAVE_NAME = "runtime_trajectory.json"  # running recording (prototype name), not listed


class RecordingPanel:
    """Records the measured motion (any mode) from /joint_states.

    Armed -> recording starts with the first motion (0.25 s pre-roll, so it
    starts at rest) -> pauses 0.5 s after the motion stops (so it ends at
    rest) -> resumes with the next motion. Pauses are not part of the time
    axis. Samples: t, FAIRINO-style TCP pose (frame flange), joints [rad]."""

    PRE_ROLL_S = 0.25
    STILL_S = 0.5
    MOTION_RAD_S = math.radians(0.5)

    def __init__(self, ui, directory: Path, urdf_path: str, record_rate_hz: float):
        self._ui = ui
        self._dir = directory / "recordings"
        self._dir.mkdir(parents=True, exist_ok=True)
        self._urdf_path = urdf_path
        self._period = 1.0 / record_rate_hz
        self._lock = threading.Lock()
        self._model = ui.model
        self._data = self._model.createData()
        self._state = "idle"  # idle | armed | recording | paused
        self._samples: list[dict] = []
        self._segments: list[dict] = []
        self._pre_roll: deque = deque()
        self._active_time = 0.0
        self._segment_start_stamp = 0.0
        self._segment_start_index = 0
        self._segment_mode = "planning"
        self._last_stamp = None
        self._last_q = None
        self._last_kept = -1e9
        self._last_motion = 0.0
        self._dirty = False
        self._loaded: dict | None = None
        self._loaded_name = ""
        self._trail = None
        self._trail_count = -1
        self._trail_source = None
        self._trail_done = 0
        self._trail_points: list = []

    # ------------------------------------------------------------------ GUI
    def build_controls(self, gui) -> None:
        """Record ↔ Stop & Save (one button), shared by both views: a
        recording continues across servo and executed motion."""
        self._record_button = gui.add_button(
            "Record", color="red", icon=viser.Icon.PLAYER_RECORD,
            hint="Arms the recorder: it starts with the next motion (servo or Execute), pauses while the robot "
            "rests and continues with the next motion, in either view",
        )
        self._status = gui.add_markdown("", visible=False)
        self._discard_button = gui.add_button("Discard Recording", icon=viser.Icon.TRASH, visible=False)
        self._record_button.on_click(lambda _: self._arm() if self._record_button.label == "Record" else self._stop_and_save())
        self._discard_button.on_click(lambda _: self._discard())

    def build_library(self, gui) -> list:
        """Recordings folder (trajectory view): library, replay, edit as path."""
        folder = gui.add_folder("Recordings", expand_by_default=False)
        with folder:
            self._name = gui.add_text("Name", "recording", hint="File name of the next Stop & Save")
            self._library = gui.add_dropdown("Recording", ["—"])
            # The orange TCP trail shows the loaded recording (and the one
            # being recorded); Unload removes both.
            self._library_buttons = gui.add_button_group("Files", ["Load", "Unload", "Refresh"])
            self._loaded_info = gui.add_markdown("No recording loaded.")
            self._timing = gui.add_dropdown("Replay timing", TIMING_LABELS, initial_value=TIMING_LABELS[0])
            self._replay_button = gui.add_button(
                "Plan Replay", icon=viser.Icon.ROUTE,
                hint="Approach + recorded motion at the Speed of Plan & Execute; check the preview, then Execute",
            )
            self._tolerance = gui.add_number("Simplify tolerance [mm]", 3.0, min=0.5, max=50.0, step=0.5)
            self._edit_button = gui.add_button(
                "Edit as Path", icon=viser.Icon.PENCIL, hint="Turns the loaded recording into editable waypoints (Path)"
            )
        self._library_buttons.on_click(lambda e: self._library_action(e.target.value))
        self._replay_button.on_click(lambda _: self._plan_replay())
        self._edit_button.on_click(lambda _: self._edit_as_path())
        self._refresh_library()
        return [folder]

    # ------------------------------------------------------------ recording
    def on_joint_state(self, stamp: float, q: np.ndarray) -> None:
        with self._lock:
            if self._state == "idle":
                self._last_stamp, self._last_q = stamp, q
                return
            moving = False
            if self._last_q is not None and self._last_stamp is not None and stamp > self._last_stamp:
                moving = np.max(np.abs(q - self._last_q)) / (stamp - self._last_stamp) > self.MOTION_RAD_S
            self._last_stamp, self._last_q = stamp, q
            if moving:
                self._last_motion = stamp
            if self._state in ("armed", "paused"):
                self._pre_roll.append((stamp, q))
                while self._pre_roll and stamp - self._pre_roll[0][0] > self.PRE_ROLL_S:
                    self._pre_roll.popleft()
                if moving:
                    self._start_segment()
                return
            # recording
            if stamp - self._last_kept >= self._period - 1e-4:
                self._append(stamp, q)
            if stamp - self._last_motion > self.STILL_S:
                self._end_segment()

    def _start_segment(self) -> None:
        servo = self._ui.servo_status()
        self._segment_mode = "servo" if servo is not None and servo.mode == "servo" else "planning"
        self._segment_start_stamp = self._pre_roll[0][0]
        self._segment_start_index = len(self._samples)
        self._state = "recording"
        self._last_kept = -1e9
        for stamp, q in list(self._pre_roll):
            if stamp - self._last_kept >= self._period - 1e-4:
                self._append(stamp, q)
        self._pre_roll.clear()

    def _append(self, stamp: float, q: np.ndarray) -> None:
        full = pin.neutral(self._model)
        for name, value in zip(JOINT_NAMES, q):
            full[self._ui.joint_q_index[name]] = value
        pin.framesForwardKinematics(self._model, self._data, full)
        tcp = self._data.oMf[self._ui.base_frame].inverse() * self._data.oMf[self._ui.tip_frame]
        t = self._active_time + (stamp - self._segment_start_stamp)
        if self._samples and t <= self._samples[-1]["t"]:
            t = self._samples[-1]["t"] + 1e-4
        self._samples.append(
            {
                "t": round(float(t), 6),
                "pose": [round(v, 4) for v in tt.pose_to_fairino(tcp.translation, tt.matrix_to_quat_wxyz(tcp.rotation))],
                "joint_positions": [round(float(v), 7) for v in q],
            }
        )
        self._last_kept = stamp
        self._dirty = True

    def _end_segment(self) -> None:
        if self._state != "recording":
            return
        if len(self._samples) > self._segment_start_index:
            start, end = self._segment_start_index, len(self._samples) - 1
            self._segments.append(
                {
                    "mode": self._segment_mode,
                    "command_pose": None,
                    "start_index": start,
                    "end_index": end,
                    "start_time": self._samples[start]["t"],
                    "end_time": self._samples[end]["t"],
                }
            )
            # Next segment starts one sample period later (pauses are cut).
            self._active_time = self._samples[end]["t"] + self._period
        self._state = "paused"
        self._pre_roll.clear()

    def _arm(self) -> None:
        with self._lock:
            if self._state != "idle":
                self._ui.set_result("Record: already armed or recording")
                return
            self._samples, self._segments = [], []
            self._active_time = 0.0
            self._pre_roll.clear()
            self._state = "armed"
        self._ui.set_result("Record: armed — recording starts with the next motion")

    def _discard(self) -> None:
        with self._lock:
            self._state = "idle"
            self._samples, self._segments = [], []
            self._pre_roll.clear()
        self._draw_trail(force=True)
        self._ui.set_result("Record: discarded")

    def _stop_and_save(self) -> None:
        with self._lock:
            if self._state == "recording":
                self._end_segment()
            samples, segments = list(self._samples), list(self._segments)
            self._state = "idle"
        if len(samples) < 2:
            self._ui.set_result("Record: nothing recorded (the robot did not move)")
            return
        name = tt.safe_name(str(self._name.value), "recording")
        payload = self._payload(name, samples, segments)
        path = tt.write_json(self._dir / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.json", payload)
        tt.write_json(self._dir / AUTOSAVE_NAME, payload)
        # Saved, not loaded: the recording's trail goes away; Load shows it.
        self._refresh_library(select=path.name)
        self._draw_trail(force=True)
        self._ui.set_result(
            f"Record: saved {path.name} ({len(samples)} samples, {payload['total_duration_s']:.1f} s) — Load to replay or edit it"
        )

    def _payload(self, name: str, samples: list[dict], segments: list[dict]) -> dict:
        robot = self._ui.robot_status()
        return tt.recording_payload(
            robot_name="fairino20_v6",
            urdf_path=self._urdf_path,
            joint_names=JOINT_NAMES,
            samples=samples,
            segments=segments,
            name=name,
            tool_number=None if robot is None else int(robot.tool_number),
        )

    def tick(self) -> None:
        """2 Hz: autosave of the running recording and the trail."""
        with self._lock:
            autosave = self._dirty and self._state in ("recording", "paused") and len(self._samples) > 1
            samples, segments = (list(self._samples), list(self._segments)) if autosave else ([], [])
            self._dirty = False
        if autosave:
            tt.write_json(self._dir / AUTOSAVE_NAME, self._payload("runtime", samples, segments))
        self._draw_trail()

    def status_text(self) -> str:
        with self._lock:
            state, count = self._state, len(self._samples)
            duration = self._samples[-1]["t"] if self._samples else 0.0
            segments = len(self._segments) + (1 if state == "recording" else 0)
        icon = {"idle": "⚪", "armed": "🟡", "recording": "🔴", "paused": "⏸️"}[state]
        text = {
            "idle": "Idle.",
            "armed": "Armed — waiting for motion.",
            "recording": "Recording.",
            "paused": "Paused — robot at rest; continues with the next motion.",
        }[state]
        if state == "idle":
            return f"{icon} {text}"
        return f"{icon} **{text}**  \n{count} samples · {duration:.1f} s · {segments} segment(s)"

    def refresh(self, plan_mode: bool) -> None:
        with self._lock:
            state = self._state
        active = state != "idle"
        label, color, icon = (
            ("Stop & Save Recording", "red", viser.Icon.PLAYER_STOP) if active else ("Record", "red", viser.Icon.PLAYER_RECORD)
        )
        if self._record_button.label != label:
            self._record_button.label = label
            self._record_button.icon = icon
            self._record_button.color = color
        for handle in (self._status, self._discard_button):
            if handle.visible != active:
                handle.visible = active
        text = self.status_text()
        if self._status.content != text:
            self._status.content = text
        has_loaded = self._loaded is not None and len(self._loaded.get("samples", [])) > 1
        self._replay_button.disabled = not (plan_mode and has_loaded and state == "idle")
        self._edit_button.disabled = not has_loaded

    # ------------------------------------------------------------- library
    def _refresh_library(self, select: str | None = None) -> None:
        # The autosave of the running recording is not a library entry.
        files = [f for f in list_json(self._dir) if f != AUTOSAVE_NAME]
        self._library.options = tuple(files) if files else ("—",)
        if select and select in files:
            self._library.value = select

    def _library_action(self, action: str) -> None:
        if action == "Refresh":
            self._refresh_library()
            return
        if action == "Unload":
            name, self._loaded, self._loaded_name = self._loaded_name, None, ""
            self._describe_loaded()
            self._draw_trail(force=True)
            self._ui.set_result(f"Unload: {name}" if name else "Unload: no recording loaded")
            return
        name = str(self._library.value)
        if name == "—":
            return
        try:
            payload = json.loads((self._dir / name).read_text(encoding="utf-8"))
            if len(payload.get("samples", [])) < 2:
                raise ValueError("fewer than two samples")
        except Exception as exc:  # noqa: BLE001
            self._ui.set_result(f"Load: {name} failed ({exc})")
            return
        self._loaded, self._loaded_name = payload, name
        self._describe_loaded()
        self._draw_trail(force=True)
        self._ui.set_result(f"Load: {name}")

    def loaded_name(self) -> str:
        return Path(self._loaded_name).stem if self._loaded_name else "recording"

    def _describe_loaded(self) -> None:
        p = self._loaded
        if p is None:
            self._loaded_info.content = "No recording loaded."
            return
        modes = sorted({s.get("mode", "?") for s in p.get("segments", [])}) or ["—"]
        self._loaded_info.content = (
            f"**{self._loaded_name}**  \n{p.get('sample_count', len(p['samples']))} samples · "
            f"{p.get('total_duration_s', 0.0):.1f} s · {p.get('segment_count', 0)} segment(s) ({', '.join(modes)}) · "
            f"created {p.get('created_at', '?')}"
        )

    TRAIL_SPACING_M = 0.002

    def _draw_trail(self, force: bool = False) -> None:
        with self._lock:
            live = self._state in ("recording", "paused") and len(self._samples) > 1
            samples = self._samples if live else (self._loaded or {}).get("samples", [])
            count = len(samples)
            if not force and count == self._trail_count:
                return
            # Only the samples added since the last call are new (the list
            # grows while recording); anything else starts over.
            if samples is not self._trail_source or count < self._trail_done:
                self._trail_source, self._trail_done, self._trail_points = samples, 0, []
            fresh = [s["pose"][:3] for s in samples[self._trail_done:]]
            self._trail_done = count
            kept = self._trail_points
        self._trail_count = count
        # A point every 2 mm of TCP travel, chosen from the start: points
        # already drawn never change while the recording grows (picking every
        # n-th sample with n growing made the trail jump), and the polyline
        # is fine enough to show the curves.
        for p in np.asarray(fresh, dtype=float).reshape(-1, 3) / 1000.0:
            if not kept or np.linalg.norm(p - kept[-1]) >= self.TRAIL_SPACING_M:
                kept.append(p)
        points = list(kept)
        if fresh and np.linalg.norm(np.asarray(fresh[-1]) / 1000.0 - points[-1]) > 1e-6:
            points.append(np.asarray(fresh[-1], dtype=float) / 1000.0)
        if self._trail is not None:
            self._trail.remove()
            self._trail = None
        if len(points) > 1:
            points = np.asarray(points)
            self._trail = self._ui.server.scene.add_line_segments(
                "/fairino/recording_trail",
                points=np.stack([points[:-1], points[1:]], axis=1),
                colors=(240, 120, 20),
                thickness=0.003,
            )

    # --------------------------------------------------------------- replay
    def _plan_replay(self) -> None:
        if not self._ui.plan_mode_ready("Plan Replay"):
            return
        p = self._loaded
        samples = p["samples"]
        request = {
            "type": "joint_path",
            "name": self._loaded_name,
            "joint_names": list(p.get("joint_names", JOINT_NAMES)),
            "times": [float(s["t"]) for s in samples],
            "positions": [list(map(float, s["joint_positions"])) for s in samples],
            "timing": "original" if self._timing.value == TIMING_LABELS[0] else "optimal",
        }
        self._ui.path_goal_pub.publish(String(data=json.dumps(request)))
        self._ui.set_result(f"Plan Replay: {self._loaded_name} sent to the planner ({self._timing.value})")
        self._ui.show_view("trajectory")  # the plan and Execute are there

    def _edit_as_path(self) -> None:
        try:
            waypoints, segments = tt.recording_to_path(self._loaded, position_tol=float(self._tolerance.value) / 1000.0)
        except Exception as exc:  # noqa: BLE001
            self._ui.set_result(f"Edit as Path: {exc}")
            return
        name = Path(self._loaded_name).stem
        self._ui.path_editor.load_path(waypoints, segments, name)
        self._ui.set_result(f"Edit as Path: {len(waypoints)} waypoints from {self._loaded_name} (Path)")
        self._ui.show_view("trajectory")


# ===========================================================================
# Path editor
# ===========================================================================

COLOR_SEGMENT = (40, 170, 70)
COLOR_SELECTED = (245, 200, 0)
COLOR_FAILED = (230, 50, 50)
COLOR_DRAFT = COLOR_SEGMENT
COLOR_HANDLE = (0, 185, 225)
SAME_POINT_M = 5e-4


_REDRAW = object()  # draft-line key that never equals a drawn state


def axes_options(mode: str) -> tuple[str, str]:
    """Gizmo axes as a pair of check boxes: base_link (move only) / tool."""
    return (f"{'☑' if mode == 'base' else '☐'} base_link", f"{'☑' if mode == 'tool' else '☐'} tool")


def _mode_options(mode: str) -> tuple[str, str]:
    """Labels of the Draw/Edit pair: a check box look, exactly one ticked."""
    return tuple(f"{'☑' if m == mode else '☐'} {m}" for m in ("Draw", "Edit"))


def _align_axis(wxyz: np.ndarray, direction: np.ndarray) -> np.ndarray:
    """Smallest rotation of a gizmo so that the signed local axis closest to
    `direction` lies exactly on it (prototype: orient_curve_gizmo_toward_neighbor)."""
    norm = float(np.linalg.norm(direction))
    if norm < 1e-9:
        return wxyz
    target = direction / norm
    rotation = tt.quat_wxyz_to_matrix(wxyz)
    axes = np.hstack([rotation, -rotation]).T  # +x, +y, +z, -x, -y, -z in world
    best = axes[int(np.argmax(axes @ target))]
    cross = np.cross(best, target)
    dot = float(np.clip(best @ target, -1.0, 1.0))
    if np.linalg.norm(cross) < 1e-9:
        return wxyz  # already on the line (best is the closest axis, so dot > 0)
    angle = math.atan2(float(np.linalg.norm(cross)), dot)
    delta = pin.exp3(cross / np.linalg.norm(cross) * angle)
    return tt.matrix_to_quat_wxyz(delta @ rotation)


def gui_number(gui, label: str, value: float):
    return gui.add_number(label, value, min=0.0, max=600.0, step=0.1, hint="Time of this segment [s]; 0 = as fast as possible")


def _scope_options(scope: str, gizmo: bool = True) -> tuple[str, ...]:
    """What Plan plans; "To gizmo" only while the trajectory gizmo is shown
    (Draw mode)."""
    return (
        f"{'☑' if scope == 'path' else '☐'} Whole path",
        f"{'☑' if scope == 'part' else '☐'} Segments",
    ) + ((f"{'☑' if scope == 'gizmo' else '☐'} To gizmo",) if gizmo else ())


def _short(message: str) -> str:
    """Planner message without the hints meant for the log."""
    return message.split(" — ")[0].strip()


def _same_pose(a: tt.Waypoint, b: tt.Waypoint) -> bool:
    """Within 1 mm and 1 deg."""
    return bool(np.linalg.norm(a.position - b.position) < 1e-3 and tt.angle_between_wxyz(a.wxyz, b.wxyz) < np.radians(1.0))


def _pose_text(position: np.ndarray, wxyz: np.ndarray) -> str:
    """'x, y, z mm · rx, ry, rz°' in the FAIRINO convention (no '-0.0')."""
    values = [round(v, 1) + 0.0 for v in tt.pose_to_fairino(position, wxyz)]
    return f"{values[0]:.1f}, {values[1]:.1f}, {values[2]:.1f} mm · {values[3]:.1f}, {values[4]:.1f}, {values[5]:.1f}°"


class PathEditor:
    """Cartesian path editor with two modes.

    Draw: the TCP gizmo is the pen; a light green line runs from the last
    waypoint to it (red as soon as that pose cannot be reached) and Confirm
    Waypoint fixes it; clicking an earlier waypoint adds a segment back to it.
    Edit: the pen is hidden; clicking a waypoint shows a gizmo to move/rotate
    it and two tangent handles (toward the previous and the next waypoint, at
    35 % while straight); dragging a handle bends the path through the
    waypoint as a Bezier (the other handle mirrors it), as in the FAIRINO
    Viser prototype.

    The path is checked (failing segments red, fastest time per segment) or,
    with Plan continuously, planned after every change. Execute is in the
    Plan & Execute folder of the trajectory view.

    Servo view: while the servo gizmo is held, its motion is recorded from the
    last waypoint (holding it still is a stop); Confirm Waypoint turns that
    drawing into waypoints and ends it with a waypoint at the gizmo."""

    CHECK_DELAY_S = 0.4
    DRAFT_CHECK_PERIOD_S = 0.3
    SNAP_FRACTION = 0.35
    DRAG_SETTLE_S = 0.6  # a handle is re-aligned this long after its last drag event
    CHIP_START_M = 0.05  # numbered spheres of a shared point
    CHIP_STEP_M = 0.04

    def __init__(self, ui, directory: Path):
        self._ui = ui
        self._dir = directory / "paths"
        self._dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.RLock()
        self._waypoints: list[tt.Waypoint] = []
        self._segments: list[tt.Segment] = []
        self._mode = "Draw"
        self._seg: int | None = None
        self._wp: int | None = None
        self._fastest: list[float] = []
        self._planned: list[float] = []
        self._failed: set[int] = set()
        self._plan_text = ""
        self._path_time: float | None = None  # duration of the editor's current plan (without approach)
        self._undo: list = []
        self._redo: list = []
        self._last_edit = 0.0
        self._changed_at = 0.0
        self._check_due = False
        self._stale_plan = False
        self._ours = False  # the latest plan came from this editor
        # What Plan plans: "path" (whole), "part" (waypoints _range) or "gizmo".
        self._scope = "path"
        self._range = (0, 1)  # waypoint indices of the part (from, to)
        self._plan_label = ""  # what that plan is, for the UI
        self._request_id = 0
        self._draft_id = 0
        self._draft_failed = False
        self._draft_checked_key = None
        self._draft_sent_at = 0.0
        self._last_plan_request = 0.0
        self._last_q = None
        self._last_motion = 0.0
        self._dirty = True
        self._updating = False
        self._handles: list = []
        self._dragged_handle: str | None = None
        self._drag_time = 0.0
        self._realign_due = False
        self._draft = None
        self._draft_key = _REDRAW
        # Drawing in the servo view: (t, position, wxyz) of the servo gizmo
        # while it is held; t runs only while it is held.
        self._sketch: list[tuple[float, np.ndarray, np.ndarray]] = []
        self._sketch_held = False
        self._sketch_clock: float | None = None
        self._sketch_t = 0.0
        self._sketch_handle = None
        self._sketch_key = None

    # ------------------------------------------------------------------ GUI
    def build_path(self, gui):
        """The Path folder (drawing, modes, plan); returns the folder."""
        scene = self._ui.server.scene
        folder = gui.add_folder("Path")
        with folder:
            self._new_button = gui.add_button(
                "New Trajectory", icon=viser.Icon.ROUTE,
                hint="Starts a new trajectory at the robot's TCP and puts the gizmo there",
            )
            self._summary = gui.add_markdown("")
            # Rows of small buttons: what is edited, the gizmo's axes, and
            # the orientation shortcut (TCP gizmo in Draw, waypoint in Edit).
            self._mode_select = gui.add_button_group("Mode", _mode_options("Draw"))
            # Axes of the trajectory gizmo (the servo gizmo has its own); both
            # move and rotate, only the axes differ. Hidden while point
            # tracking is on.
            self._gizmo_axes = gui.add_button_group("Gizmo axes", axes_options("tool"))
            self._orient_button = gui.add_button_group("Orientation", ["Look at point"])
            self._confirm_button = gui.add_button(
                "Confirm Waypoint", color="green", icon=viser.Icon.CHECK,
                hint="Adds the TCP gizmo pose as the next waypoint (end of the light green line)",
            )
            self._edit_buttons = gui.add_button_group("Edit", ["Undo", "Redo", "Clear"])
        self._build_scene_gizmos(scene)
        self._mode_select.on_click(lambda e: self._on_mode(str(e.target.value)))
        self._new_button.on_click(lambda _: self._new_path())
        self._confirm_button.on_click(lambda _: self._confirm())
        self._edit_buttons.on_click(lambda e: self._edit(e.target.value))
        self._gizmo_axes.on_click(
            lambda e: self._ui.set_gizmo_axes("base" if "base_link" in str(e.target.value) else "tool")
        )
        self._orient_button.on_click(lambda _: self._orient_to_point())  # the red tracking point
        return folder

    def build_plan_controls(self, gui) -> None:
        """What to plan (inside the UI's Plan & Execute folder): the whole
        path, a part of it (From → To) or a free motion to the gizmo."""
        self._scope_select = gui.add_button_group("Plan", _scope_options("path"))
        self._from_select = gui.add_dropdown("From waypoint", ["1"], visible=False)
        self._to_select = gui.add_dropdown("To waypoint", ["1"], visible=False)
        self._scope_select.on_click(lambda e: self._on_scope(str(e.target.value)))
        self._from_select.on_update(lambda _: self._on_range("from"))
        self._to_select.on_update(lambda _: self._on_range("to"))

    def build_plan_button(self, gui) -> None:
        """Plan continuously and Plan (below the UI's Speed)."""
        self._auto_plan = gui.add_checkbox(
            "Plan continuously", False,
            hint="On: what is chosen is planned again after every change and the preview repeats. Off: press "
            "Plan; the preview plays the plan once (the path is only checked: red segments, fastest times).",
        )
        self._plan_button = gui.add_button(
            "Plan", icon=viser.Icon.ROUTE_2, hint="Plans what is chosen above; the preview plays it, then Execute"
        )
        self._auto_plan.on_update(lambda _: self._on_auto_plan())
        self._plan_button.on_click(lambda _: self._plan())

    def continuous(self) -> bool:
        """Plan continuously: the preview repeats."""
        return bool(self._auto_plan.value)

    def build_details(self, gui) -> list:
        """Waypoint and Segments folders; returns them."""
        # Waypoint editing: the gizmo (Draw) or a waypoint (Edit), its pose,
        # exact offsets and the table of all waypoints.
        folders = [gui.add_folder("Waypoint")]
        with folders[-1]:
            self._wp_select = gui.add_dropdown(
                "Waypoint", ["gizmo"], hint="gizmo: the drawing gizmo (Draw); a number: that waypoint (Edit)"
            )
            self._coord_info = gui.add_markdown("")
            self._dwell_number = gui.add_number(
                "Stop [s]", 0.0, min=0.0, max=600.0, step=0.1, visible=False,
                hint="Rest at this waypoint before the next segment (0: no stop; not at the first or last waypoint)",
            )
            self._straight_button = gui.add_button(
                "Straight", hint="Removes the curve of the selected waypoint", visible=False
            )
            self._delete_wp = gui.add_button("Delete Waypoint", icon=viser.Icon.TRASH, visible=False)
            self._move_by = gui.add_vector3("Move [mm]", (0.0, 0.0, 0.0), step=0.1)
            self._rotate_by = gui.add_vector3("Rotate [deg]", (0.0, 0.0, 0.0), step=0.1)
            self._offset_frame = gui.add_dropdown(
                "Frame", ["base_link", "local"], initial_value="base_link",
                hint="Axes of the offset: robot base, or the gizmo's / waypoint's own axes",
            )
            self._apply_offset = gui.add_button(
                "Apply Offset", hint="Moves the gizmo (Draw) or the selected waypoint (Edit) by the offset"
            )
            self._waypoint_table = gui.add_markdown("")
        # All segments at once: one time field per segment, its label says
        # the fastest possible time (0 = as fast as possible).
        self._seg_folder = gui.add_folder("Segments")
        folders.append(self._seg_folder)
        with self._seg_folder:
            self._seg_info = gui.add_markdown("")
            self._seg_page = gui.add_dropdown("Show", ["—"], visible=False)
        self._seg_inputs: dict[int, object] = {}
        self._seg_labels: dict[int, str] = {}
        self._seg_values: dict[int, float] = {}
        self._seg_rows: tuple = ()
        self._dwell_number.on_update(lambda _: self._on_dwell())
        self._straight_button.on_click(lambda _: self._waypoint_action("Straight"))
        self._wp_select.on_update(lambda _: self._on_select_waypoint())
        self._delete_wp.on_click(lambda _: self._delete_selected_waypoint())
        self._seg_page.on_update(lambda _: self._refresh_segment_table())
        self._apply_offset.on_click(lambda _: self._on_apply_offset())
        return folders

    def build_files(self, gui) -> list:
        """Path files (Files view)."""
        folder = gui.add_folder("Path files")
        with folder:
            self._file_name = gui.add_text("Name", "path")
            self._files = gui.add_dropdown("Path file", ["—"])
            self._file_buttons = gui.add_button_group("Files", ["Save", "Load", "Refresh"])
        self._file_buttons.on_click(lambda e: self._file(e.target.value))
        self._refresh_files()
        return [folder]

    def _build_scene_gizmos(self, scene) -> None:
        self._pose_gizmo = scene.add_transform_controls(
            "/fairino/path_pose", scale=0.16, depth_test=False, disable_sliders=False, visible=False
        )
        self._prev_gizmo = scene.add_transform_controls(
            "/fairino/path_tangent_prev", scale=0.08, depth_test=False, disable_rotations=True, visible=False
        )
        self._next_gizmo = scene.add_transform_controls(
            "/fairino/path_tangent_next", scale=0.08, depth_test=False, disable_rotations=True, visible=False
        )
        self._pose_gizmo.on_update(lambda _: self._on_pose_gizmo())
        # A gizmo that is being dragged is never moved from here: a pose set
        # during a drag makes it jump (seen with point tracking, where the
        # shown orientation differs from the gizmo's own).
        self._held: set[str] = set()
        self._pose_ref: np.ndarray | None = None  # waypoint rotation when a base_link drag started
        for key, gizmo in (("pose", self._pose_gizmo), ("prev", self._prev_gizmo), ("next", self._next_gizmo)):
            gizmo.on_drag_start(lambda _, key=key: self._on_drag_start(key))
            gizmo.on_drag_end(lambda _, key=key: self._on_drag_end(key))
        self._prev_gizmo.on_update(lambda _: self._on_tangent_gizmo(previous=True))
        self._next_gizmo.on_update(lambda _: self._on_tangent_gizmo(previous=False))
        self._axes = "tool"

    # ------------------------------------------------------------- helpers
    def _snapshot(self, merge: bool = False) -> None:
        now = time.monotonic()
        if merge and now - self._last_edit < 0.4:
            self._last_edit = now
            return
        self._last_edit = now
        self._undo.append(([w.copy() for w in self._waypoints], [s.copy() for s in self._segments]))
        del self._undo[:-100]
        self._redo.clear()

    def _changed(self) -> None:
        """Caller holds the lock: redraw and check/plan shortly. A plan made
        before the change no longer matches the drawing."""
        self._dirty = True
        self._failed.clear()
        self._planned = []
        self._changed_at = time.monotonic()
        self._check_due = len(self._waypoints) >= 2
        self._plan_text = ""
        self._path_time = None
        self._stale_plan = self._ours and not bool(self._auto_plan.value)
        self._draft_checked_key = None

    @staticmethod
    def _wp_label(j: int) -> str:
        return str(j + 1)

    def _groups(self, waypoints: list[tt.Waypoint]) -> list[list[int]]:
        """Waypoints at the same position (e.g. a path back to an earlier
        waypoint) share one sphere and one label."""
        groups: list[list[int]] = []
        for j, wp in enumerate(waypoints):
            for g in groups:
                if np.linalg.norm(waypoints[g[0]].position - wp.position) < SAME_POINT_M:
                    g.append(j)
                    break
            else:
                groups.append([j])
        return groups

    def load_path(self, waypoints: list[tt.Waypoint], segments: list[tt.Segment], name: str) -> None:
        with self._lock:
            self._snapshot()
            self._waypoints = [w.copy() for w in waypoints]
            self._segments = [s.copy() for s in segments]
            self._seg = 0 if segments else None
            self._wp = None
            self._file_name.value = name
            self._fastest = []
            self._changed()
        # Not in servo mode: a loaded path may end far away from the robot.
        self._gizmo_to_end(servo_too=False)
        self._sync_fields()

    # ------------------------------------------------------------------ modes
    def _on_mode(self, clicked: str) -> None:
        self._set_mode("Edit" if "Edit" in clicked else "Draw")

    def _set_mode(self, mode: str) -> None:
        with self._lock:
            self._mode = mode
            if mode == "Draw":
                self._wp = None
            self._dirty = True
            self._draft_key = _REDRAW
        self._ui.set_tcp_gizmo_visible(mode == "Draw")
        self._updating = True
        try:
            self._mode_select.options = _mode_options(mode)
        finally:
            self._updating = False
        # Edit hides the trajectory gizmo: no "To gizmo" (the other two
        # fill the row); back to the whole path if it was chosen.
        if mode == "Edit" and self._scope == "gizmo":
            self._apply_scope("path")
        else:
            self._scope_select.options = _scope_options(self._scope, gizmo=mode == "Draw")
        self._sync_fields()

    def _on_auto_plan(self) -> None:
        if self._auto_plan.value:
            with self._lock:
                if len(self._waypoints) >= 2:
                    self._check_due = True
                    self._changed_at = 0.0
        else:
            # Off: the plan made continuously and its preview go away now;
            # only the check runs until Plan.
            self._clear_plan()
            with self._lock:
                self._check_due = len(self._waypoints) >= 2
                self._changed_at = 0.0
                self._dirty = True

    # -------------------------------------------------------------- drawing
    def _new_path(self, reset_gizmo: bool = True) -> None:
        pose = self._ui.robot_tcp_pose()
        if pose is None:
            self._ui.set_result("New Trajectory: no robot pose yet")
            return
        with self._lock:
            self._snapshot()
            self._waypoints = [tt.Waypoint(*pose)]
            self._segments = []
            self._seg, self._wp = None, None
            self._fastest = []
            self._changed()
        self._sync_fields()
        self._clear_plan()
        self._set_mode("Draw")
        if reset_gizmo:
            self._ui.reset_gizmo_to_robot()
            self._ui.set_result("New Trajectory: starts at the robot TCP — move the gizmo and press Confirm Waypoint")

    def _append(self, position: np.ndarray, wxyz: np.ndarray, label: str) -> None:
        with self._lock:
            last = self._waypoints[-1]
            if np.linalg.norm(last.position - position) < 1e-4 and tt.angle_between_wxyz(last.wxyz, wxyz) < 1e-3:
                self._ui.set_result(f"{label}: same pose as the last waypoint")
                return
            self._snapshot()
            self._waypoints.append(tt.Waypoint(position.copy(), wxyz.copy()))
            self._segments.append(tt.Segment())
            self._seg = len(self._segments) - 1
            self._changed()
        self._sync_fields()

    def _confirm(self) -> None:
        pose = self._ui.gizmo_pose()
        if pose is None:
            self._ui.set_result("Confirm: no gizmo pose yet")
            return
        if not self._waypoints:
            self._new_path(reset_gizmo=False)  # the gizmo stays on the pose being confirmed
        self._append(pose[0], pose[1], "Confirm")

    def _edit(self, action: str) -> None:
        clear = False
        with self._lock:
            if action == "Undo" and self._undo:
                self._redo.append(([w.copy() for w in self._waypoints], [s.copy() for s in self._segments]))
                self._waypoints, self._segments = self._undo.pop()
            elif action == "Redo" and self._redo:
                self._undo.append(([w.copy() for w in self._waypoints], [s.copy() for s in self._segments]))
                self._waypoints, self._segments = self._redo.pop()
            elif action == "Clear":
                self._snapshot()
                self._waypoints, self._segments = [], []
                clear = True
            else:
                return
            self._clamp_selection()
            self._fastest = []
            self._changed()
        if clear or len(self._waypoints) < 2:
            self._clear_plan(any_plan=clear)
        if action in ("Undo", "Redo"):
            self._gizmo_to_end()
        self._sync_fields()

    def _gizmo_to_end(self, servo_too: bool = True) -> None:
        """Rule: after Undo, Redo, Delete Waypoint and Load the TCP gizmo is on
        the last waypoint (the end of the path), where drawing continues. In
        servo mode the robot follows the gizmo; servo_too=False leaves it."""
        with self._lock:
            last = self._waypoints[-1].copy() if self._waypoints else None
        if last is None:
            return
        servo = self._ui.servo_status()
        if not servo_too and servo is not None and servo.state == "ACTIVE":
            return
        self._ui.set_gizmo_pose(last.position, last.wxyz)

    def _clear_plan(self, any_plan: bool = False) -> None:
        """Removes the plan (planner, preview, planned path): the editor's own
        plan, or with any_plan whatever plan is shown (Clear)."""
        if self._ours or any_plan:
            self._ui.path_goal_pub.publish(String(data=json.dumps({"type": "clear"})))
            self._ours = False
        with self._lock:
            self._plan_text = ""
            self._planned = []
            self._plan_label = ""
            self._path_time = None

    def _delete_selected_waypoint(self) -> None:
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            self._snapshot()
            del self._waypoints[i]
            if self._segments:
                del self._segments[min(i, len(self._segments) - 1)]
            self._wp = None
            self._clamp_selection()
            self._fastest = []
            self._changed()
        if len(self._waypoints) < 2:
            self._clear_plan()
        self._gizmo_to_end()
        self._sync_fields()

    def _clamp_selection(self) -> None:
        if self._seg is not None and self._seg >= len(self._segments):
            self._seg = len(self._segments) - 1 if self._segments else None
        if self._wp is not None and self._wp >= len(self._waypoints):
            self._wp = None

    # ----------------------------------------------------- waypoint editing
    def _click_group(self, group: list[int]) -> None:
        with self._lock:
            mode = self._mode
            last = len(self._waypoints) - 1
        if mode == "Draw":
            # Draw: the gizmo goes to the clicked waypoint; Confirm Waypoint
            # then adds the segment back to it.
            if last in group:
                self._ui.set_result("Draw: that is the last waypoint")
                return
            wp = self._waypoints[group[0]]
            self._ui.set_gizmo_pose(wp.position.copy(), wp.wxyz.copy())
            self._ui.set_result(f"Gizmo on waypoint {self._wp_label(group[0])} — Confirm Waypoint to go back to it")
            return
        with self._lock:
            # Edit: select; clicking a shared point again cycles through it.
            if self._wp in group and len(group) > 1:
                self._wp = group[(group.index(self._wp) + 1) % len(group)]
            else:
                self._wp = group[0]
            if self._segments:
                self._seg = max(0, min(self._wp - 1, len(self._segments) - 1))
            self._dirty = True
        self._sync_fields()

    def _waypoint_action(self, action: str) -> None:
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            if action == "Straight":
                self._snapshot()
                self._set_tangent(i, np.zeros(3))
                self._changed()
        self._sync_fields()

    def _set_tangent(self, i: int, tangent: np.ndarray) -> None:
        """Caller holds the lock. A zero tangent makes the waypoint straight."""
        wp = self._waypoints[i]
        wp.tangent = np.asarray(tangent, dtype=float)
        wp.curve = bool(np.linalg.norm(wp.tangent) > 1e-6)

    def _handle_positions(self, waypoints: list[tt.Waypoint], i: int):
        """Previous/next tangent handle of waypoint i: waypoint -/+ tangent for
        a curve; 35 % toward the neighbours while straight."""
        wp = waypoints[i]
        prev_pos = next_pos = None
        if wp.curve:
            if i > 0:
                prev_pos = wp.position - wp.tangent
            if i < len(waypoints) - 1:
                next_pos = wp.position + wp.tangent
        else:
            if i > 0:
                prev_pos = wp.position + self.SNAP_FRACTION * (waypoints[i - 1].position - wp.position)
            if i < len(waypoints) - 1:
                next_pos = wp.position + self.SNAP_FRACTION * (waypoints[i + 1].position - wp.position)
        return prev_pos, next_pos

    def _on_pose_gizmo(self) -> None:
        if self._updating:
            return
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            self._snapshot(merge=True)
            wp = self._waypoints[i]
            wp.position = np.asarray(self._pose_gizmo.position, dtype=float).copy()
            if self._ui.tracking_on():
                # Point tracking: the moved waypoint looks at the point.
                point = self._ui.tracking_point()
                if np.linalg.norm(point - wp.position) > tt.LOOK_AT_MIN_DISTANCE_M:
                    wp.wxyz = tt.matrix_to_quat_wxyz(
                        tt.look_at_rotation(tt.quat_wxyz_to_matrix(wp.wxyz), wp.position, point)
                    )
            else:
                gizmo = tt.quat_wxyz_to_matrix(np.asarray(self._pose_gizmo.wxyz, dtype=float))
                if self._ui.gizmo_axes() == "tool":
                    rotation = gizmo  # the gizmo is the waypoint pose
                else:
                    # base_link axes: the gizmo's rotation since the drag
                    # started (it starts at the base axes) turns the waypoint.
                    reference = self._pose_ref if self._pose_ref is not None else tt.quat_wxyz_to_matrix(wp.wxyz)
                    rotation = gizmo @ reference
                wp.wxyz = tt.matrix_to_quat_wxyz(rotation)
            self._changed()

    def _on_drag_start(self, key: str) -> None:
        self._held.add(key)
        if key == "pose":
            with self._lock:
                i = self._wp
                if i is not None and i < len(self._waypoints):
                    self._pose_ref = tt.quat_wxyz_to_matrix(self._waypoints[i].wxyz)

    def _on_drag_end(self, key: str) -> None:
        self._held.discard(key)
        if key == "pose":
            self._pose_ref = None
        with self._lock:
            self._dirty = True  # show the final pose/handles

    def _on_tangent_gizmo(self, previous: bool) -> None:
        if self._updating:
            return
        with self._lock:
            i = self._wp
            if i is None or i >= len(self._waypoints):
                return
            self._snapshot(merge=True)
            self._dragged_handle, self._drag_time = ("prev" if previous else "next"), time.monotonic()
            self._realign_due = True
            wp = self._waypoints[i]
            handle = np.asarray((self._prev_gizmo if previous else self._next_gizmo).position, dtype=float)
            # The other handle mirrors this one (path tangent through the waypoint).
            self._set_tangent(i, (wp.position - handle) if previous else (handle - wp.position))
            self._changed()

    def _on_select_waypoint(self) -> None:
        if self._updating:
            return
        value = str(self._wp_select.value)
        if value == "gizmo":
            if self._mode != "Draw":
                self._set_mode("Draw")
            return
        with self._lock:
            self._wp = int(value) - 1 if value.isdigit() else None
            if self._wp is not None and self._segments:
                self._seg = max(0, min(self._wp - 1, len(self._segments) - 1))
            self._dirty = True
        if self._wp is not None and self._mode != "Edit":
            self._set_mode("Edit")
        self._sync_fields()

    # ------------------------------------------------------- segment table
    SEG_PAGE = 25

    def _on_seg_time(self, k: int, handle) -> None:
        if self._updating:
            return
        value = float(handle.value)
        with self._lock:
            if k >= len(self._segments):
                return
            self._snapshot(merge=True)
            self._segments[k].duration = value if value > 0.0 else None
            self._seg = k  # highlighted in the 3D view
            self._changed()
        self._seg_values[k] = value

    def _seg_label(self, k: int, fastest: float | None, failed: bool) -> str:
        # Short enough for one line in the label column.
        name = f"{k + 1}→{k + 2}"
        if failed:
            return f"{name} ❌"
        return f"{name} ≥{fastest:.2f}s" if fastest is not None else f"{name} …"

    def _refresh_segment_table(self) -> None:
        with self._lock:
            durations = [s.duration for s in self._segments]
            fastest = list(self._fastest)
            failed = set(self._failed)
        n = len(durations)
        pages = [(i, min(i + self.SEG_PAGE, n)) for i in range(0, n, self.SEG_PAGE)]
        names = tuple(f"{a + 1}–{b}" for a, b in pages) or ("—",)
        if tuple(self._seg_page.options) != names:
            self._updating = True
            try:
                self._seg_page.options = names
                self._seg_page.value = names[0]
            finally:
                self._updating = False
        self._seg_page.visible = n > self.SEG_PAGE
        page = names.index(self._seg_page.value) if self._seg_page.value in names else 0
        rows = tuple(range(*pages[page])) if pages else ()
        if rows != self._seg_rows:
            # Rows changed (segments added/removed, other page): rebuild.
            for handle in self._seg_inputs.values():
                handle.remove()
            self._seg_inputs, self._seg_labels, self._seg_values = {}, {}, {}
            with self._seg_folder:
                for k in rows:
                    label = self._seg_label(k, fastest[k] if k < len(fastest) else None, k in failed)
                    value = round(durations[k], 2) if durations[k] else 0.0
                    handle = gui_number(self._ui.server.gui, label, value)
                    handle.on_update(lambda _, k=k, handle=handle: self._on_seg_time(k, handle))
                    self._seg_inputs[k], self._seg_labels[k], self._seg_values[k] = handle, label, value
            self._seg_rows = rows
        self._updating = True
        try:
            for k, handle in self._seg_inputs.items():
                label = self._seg_label(k, fastest[k] if k < len(fastest) else None, k in failed)
                if label != self._seg_labels.get(k):
                    handle.label = label
                    self._seg_labels[k] = label
                value = round(durations[k], 2) if durations[k] else 0.0
                if abs(value - self._seg_values.get(k, -1.0)) > 1e-6:  # changed elsewhere (undo, load)
                    handle.value = value
                    self._seg_values[k] = value
        finally:
            self._updating = False
        too_fast = [
            f"{self._wp_label(k)} → {k + 2}"
            for k in range(min(n, len(fastest)))
            if durations[k] is not None and fastest[k] is not None and durations[k] < fastest[k] * 0.999
        ]
        text = "Time per segment [s]; 0 = as fast as possible; ≥ = the fastest possible." if n else "No segments yet."
        if too_fast:
            text += f"  \nFaster than possible (the fastest time is used): {', '.join(too_fast)}"
        if self._seg_info.content != text:
            self._seg_info.content = text

    def _on_apply_offset(self) -> None:
        move_mm = np.asarray(self._move_by.value, dtype=float)
        rotate_deg = np.asarray(self._rotate_by.value, dtype=float)
        move = move_mm / 1000.0
        rotate = pin.rpy.rpyToMatrix(*np.radians(rotate_deg))
        applied = (
            f"{', '.join(f'{v:g}' for v in move_mm)} mm, {', '.join(f'{v:g}' for v in rotate_deg)}° "
            f"({self._offset_frame.value})"
        )
        with self._lock:
            target_wp = self._wp if self._mode == "Edit" else None
        if target_wp is not None:
            with self._lock:
                wp = self._waypoints[target_wp]
                position, rotation = wp.position, tt.quat_wxyz_to_matrix(wp.wxyz)
        else:
            pose = self._ui.gizmo_pose()
            if pose is None:
                return
            position, rotation = pose[0], tt.quat_wxyz_to_matrix(pose[1])
        if self._offset_frame.value == "local":
            new_position, new_rotation = position + rotation @ move, rotation @ rotate
        else:
            new_position, new_rotation = position + move, rotate @ rotation
        if target_wp is not None:
            with self._lock:
                self._snapshot()
                self._waypoints[target_wp].position = new_position
                self._waypoints[target_wp].wxyz = tt.matrix_to_quat_wxyz(new_rotation)
                self._changed()
            self._ui.set_result(f"Offset {applied} applied to waypoint {self._wp_label(target_wp)}")
        else:
            self._ui.set_gizmo_pose(new_position, tt.matrix_to_quat_wxyz(new_rotation))
            self._ui.set_result(f"Offset {applied} applied to the TCP gizmo")

    def _sync_fields(self) -> None:
        with self._lock:
            n_wp = len(self._waypoints)
            i = self._wp
            wp = self._waypoints[i] if i is not None and i < n_wp else None
        self._updating = True
        try:
            edit = self._mode == "Edit"
            self._wp_select.options = (
                ("gizmo",) + tuple(self._wp_label(j) for j in range(n_wp)) + (("—",) if edit and wp is None else ())
            )
            # From → To of the planned part.
            labels = tuple(str(j + 1) for j in range(n_wp)) or ("1",)
            if tuple(self._from_select.options) != labels:
                self._from_select.options = labels
                self._to_select.options = labels
            if n_wp >= 2:
                with self._lock:
                    first, last = self._part() if self._scope == "part" else self._range
                first, last = min(first, n_wp - 2), min(max(last, 1), n_wp - 1)
                self._from_select.value, self._to_select.value = str(first + 1), str(last + 1)
            self._wp_select.value = "gizmo" if not edit else self._wp_label(i) if wp is not None else "—"
            self._dwell_number.value = round(wp.dwell, 2) if wp is not None else 0.0
        finally:
            self._updating = False

    # -------------------------------------------------------------- planning
    def _request(self, waypoints: list[tt.Waypoint] | None = None, segments: list[tt.Segment] | None = None) -> dict:
        waypoints = self._waypoints if waypoints is None else waypoints
        segments = self._segments if segments is None else segments
        point = self.look_at_point()
        return {
            "name": tt.safe_name(str(self._file_name.value), "path"),
            "waypoints": [w.to_dict() for w in waypoints],
            "segments": [s.to_dict() for s in segments],
            "look_at_m": None if point is None else [float(v) for v in point],
        }

    # -------------------------------------------------------- point tracking
    def look_at_point(self) -> np.ndarray | None:
        """Paths are planned with the waypoints' own orientations (the point
        only turns waypoints on request: Orient to point)."""
        return None

    def on_view(self, view: str) -> None:
        """Servo view: the path stays visible, its editing gizmos and the
        drawing line are hidden."""
        with self._lock:
            self._dirty = True
            self._draft_key = _REDRAW

    def on_gizmo_axes(self, mode: str) -> None:
        """Trajectory gizmo axes changed (here, or by point tracking)."""
        self._axes = mode
        self._gizmo_axes.options = axes_options(mode if not self._ui.tracking_on() else "base")
        with self._lock:
            self._dirty = True  # re-place the waypoint gizmo with the new axes

    def _orient_to_point(self) -> None:
        """Turns the tool z axis of the selected waypoint (Edit) or of the TCP
        gizmo (Draw) toward the point, by the smallest rotation."""
        point = self._ui.tracking_point()
        with self._lock:
            i = self._wp if self._mode == "Edit" else None
            if i is not None and i < len(self._waypoints):
                wp = self._waypoints[i]
                if np.linalg.norm(point - wp.position) < tt.LOOK_AT_MIN_DISTANCE_M:
                    self._ui.set_result("Orient to point: the waypoint is at the point")
                    return
                self._snapshot()
                wp.wxyz = tt.matrix_to_quat_wxyz(
                    tt.look_at_rotation(tt.quat_wxyz_to_matrix(wp.wxyz), wp.position, point)
                )
                self._changed()
                label = self._wp_label(i)
            else:
                label = None
        if label is not None:
            self._ui.set_result(f"Orient to point: waypoint {label} looks at the point")
            return
        pose = self._ui.gizmo_pose()
        if pose is None:
            return
        rotation = tt.look_at_rotation(tt.quat_wxyz_to_matrix(pose[1]), pose[0], point)
        self._ui.set_gizmo_pose(pose[0], tt.matrix_to_quat_wxyz(rotation))
        self._ui.set_result("Orient to point: the TCP gizmo looks at the point")

    def _shown_waypoints(self, waypoints: list[tt.Waypoint]) -> list[tt.Waypoint]:
        return waypoints

    def _send_check(self) -> None:
        with self._lock:
            if len(self._waypoints) < 2:
                return
            self._check_due = False
            self._request_id += 1
            request = dict(self._request(), request_id=f"path-{self._request_id}")
        self._ui.path_check_pub.publish(String(data=json.dumps(request)))

    def _plan(self, automatic: bool = False) -> None:
        if self._scope == "gizmo":
            if not automatic:
                self._ui.plan_to_gizmo()
            return
        if not automatic and not self._ui.plan_mode_ready("Plan"):
            return
        with self._lock:
            if len(self._waypoints) < 2:
                if not automatic:
                    self._ui.set_result("Plan: confirm at least one waypoint")
                return
            first, last = self._part()
            # The whole path with the part: the planner moves the part as
            # within the whole path (same joint motion).
            request = dict(self._request(), type="cartesian_path", part=[first, last])
            self._plan_label = (
                f"path {first + 1}→{last + 1}" if (first, last) == (0, len(self._waypoints) - 1)
                else f"segments {first + 1}→{last + 1}"
            )
            self._plan_text = "⏳"
            self._check_due = False
            self._last_plan_request = time.monotonic()
            self._dirty = True
        self._ours = True
        self._ui.path_goal_pub.publish(String(data=json.dumps(request)))
        if not automatic:
            self._ui.set_result(f"Plan: {self._plan_label} sent to the planner")

    # ----------------------------------------------------------- plan scope
    def _part(self) -> tuple[int, int]:
        """Waypoint indices planned: the whole path, or From → To. Caller holds the lock."""
        last = len(self._waypoints) - 1
        if self._scope != "part":
            return 0, last
        first, end = self._range
        first = int(np.clip(first, 0, max(last - 1, 0)))
        end = int(np.clip(end, first + 1, last))
        return first, end

    def plan_label(self) -> str:
        """What the editor's last plan covers ("path 1→6", "segments 4→5")."""
        return self._plan_label

    def _on_scope(self, clicked: str) -> None:
        self._apply_scope("part" if "Segments" in clicked else "gizmo" if "gizmo" in clicked else "path")

    def _apply_scope(self, scope: str) -> None:
        self._scope = scope
        self._scope_select.options = _scope_options(self._scope, gizmo=self._mode == "Draw")
        part = self._scope == "part"
        self._from_select.visible = part
        self._to_select.visible = part
        with self._lock:
            self._dirty = True  # the part is highlighted
            if self._scope != "gizmo" and len(self._waypoints) >= 2:
                self._changed()  # plan (or check) what is chosen now
        self._ui.set_result(
            {"path": "Plan: the whole path", "part": "Plan: the segments From → To",
             "gizmo": "Plan: a free motion (RRT + TOPP-RA) to the trajectory gizmo"}[self._scope]
        )

    def _on_range(self, changed: str) -> None:
        if self._updating:
            return
        try:
            first, last = int(self._from_select.value) - 1, int(self._to_select.value) - 1
        except ValueError:
            return
        with self._lock:
            # The other field gives way: From 3 with To 2 -> To 4.
            if last <= first:
                if changed == "from":
                    last = first + 1
                else:
                    first = last - 1
            self._range = (first, last)
            first, last = self._part()
            self._range = (first, last)
            if self._scope == "part" and len(self._waypoints) >= 2:
                self._changed()
        self._updating = True
        try:
            self._from_select.value, self._to_select.value = str(first + 1), str(last + 1)
        finally:
            self._updating = False

    def _maybe_check_draft(self) -> None:
        """While drawing: is the pose at the gizmo reachable from the last
        waypoint? The draft line turns red before Confirm if not."""
        if self._mode != "Draw":
            return
        gizmo = self._ui.gizmo_pose()
        with self._lock:
            if gizmo is None or not self._waypoints:
                return
            key = (tuple(np.round(gizmo[0], 4)), tuple(np.round(gizmo[1], 4)), len(self._waypoints))
            now = time.monotonic()
            if key == self._draft_checked_key or now - self._draft_sent_at < self.DRAFT_CHECK_PERIOD_S:
                return
            last = self._waypoints[-1]
            if np.linalg.norm(last.position - gizmo[0]) < 1e-4 and tt.angle_between_wxyz(last.wxyz, gizmo[1]) < 1e-3:
                self._draft_checked_key = key
                self._draft_failed = False
                return
            self._draft_checked_key = key
            self._draft_sent_at = now
            self._draft_id += 1
            waypoints = self._waypoints + [tt.Waypoint(gizmo[0].copy(), gizmo[1].copy())]
            segments = self._segments + [tt.Segment()]
            request = dict(self._request(waypoints, segments), request_id=f"draft-{self._draft_id}")
        self._ui.path_check_pub.publish(String(data=json.dumps(request)))

    def on_check(self, result: dict) -> None:
        rid = str(result.get("request_id"))
        with self._lock:
            if rid == f"draft-{self._draft_id}":
                draft_segment = len(self._segments)
                failed = {int(k) for k in result.get("failed_segments", [])}
                was = self._draft_failed
                self._draft_failed = (not result.get("ok")) and draft_segment in failed
                if was != self._draft_failed:
                    self._draft_key = _REDRAW  # redraw the line in its new colour
                return
            if rid != f"path-{self._request_id}":
                return  # answer to an older drawing
            if result.get("ok"):
                self._failed = set()
                self._fastest = list(result.get("segment_fastest_s", []))
            else:
                self._failed = {int(k) for k in result.get("failed_segments", [])}
                where = ", ".join(f"{self._wp_label(k)} → {k + 2}" for k in sorted(self._failed))
                self._plan_text = f"❌ {_short(result.get('message', ''))}" + (f" (segment {where})" if where else "")
            self._dirty = True

    def on_plan_status(self, status: dict) -> None:
        kind = status.get("kind")
        state = status.get("state")
        if kind in ("joint_goal", "joint_path") and state in ("planning", "planned"):
            self._ours = False  # another plan (Plan, Home, replay) replaced ours
            return
        if kind != "cartesian_path":
            return
        with self._lock:
            offset = int(status.get("first_segment", 0))  # segment indices are the whole path's
            n_seg = len(self._segments)
            if state == "failed":
                self._failed = {int(k) for k in status.get("failed_segments", [])}
                where = ", ".join(f"{self._wp_label(k)} → {k + 2}" for k in sorted(self._failed))
                self._plan_text = f"❌ {_short(status.get('message', ''))}" + (f" (segment {where})" if where else "")
            elif state == "planned":
                self._failed.clear()
                fastest = list(self._fastest) + [None] * max(0, n_seg - len(self._fastest))
                planned: list = [None] * n_seg
                for i, value in enumerate(status.get("segment_fastest_s", [])):
                    if offset + i < n_seg:
                        fastest[offset + i] = value
                for i, value in enumerate(status.get("segment_planned_s", [])):
                    if offset + i < n_seg:
                        planned[offset + i] = value
                self._fastest = fastest[:n_seg]
                self._planned = planned
                self._path_time = status.get("path_duration_s")
                self._plan_text = "✅"
            elif state == "succeeded":
                self._plan_text = ""  # the plan was used; planned again when continuous
                self._path_time = None
            self._dirty = True

    def _robot_resting_elsewhere(self) -> bool:
        q = self._ui.robot_q()
        if q is None:
            return False
        now = time.monotonic()
        if self._last_q is None or np.max(np.abs(q - self._last_q)) > 1e-4:
            self._last_motion = now
        self._last_q = q
        if now - self._last_motion < 0.5:
            return False
        start = self._ui.planned_start()
        return start is not None and np.max(np.abs(q - start)) > 0.003

    # ----------------------------------------------------------------- files
    def _refresh_files(self) -> None:
        files = list_json(self._dir)
        self._files.options = tuple(files) if files else ("—",)

    def _file(self, action: str) -> None:
        if action == "Refresh":
            self._refresh_files()
            return
        if action == "Save":
            with self._lock:
                if len(self._waypoints) < 2:
                    self._ui.set_result("Save: the path has no segment yet")
                    return
                name = tt.safe_name(str(self._file_name.value), "path")
                payload = tt.path_payload(name, self._waypoints, self._segments)
            path = tt.write_json(self._dir / f"{name}.json", payload)
            self._refresh_files()
            self._files.value = path.name
            self._ui.set_result(f"Save: {path.name} ({len(payload['waypoints'])} waypoints)")
            return
        name = str(self._files.value)
        if name == "—":
            return
        try:
            payload = json.loads((self._dir / name).read_text(encoding="utf-8"))
            waypoints, segments = tt.path_from_payload(payload)
            point = tt.look_at_from_payload(payload)
        except Exception as exc:  # noqa: BLE001
            self._ui.set_result(f"Load: {name} failed ({exc})")
            return
        if point is not None:
            # Older file planned to look at a point: the waypoints get the
            # orientation they had along that path.
            waypoints = tt.apply_look_at(waypoints, point)
        self.load_path(waypoints, segments, Path(name).stem)
        self._ui.show_view("trajectory")
        self._ui.set_result(
            f"Load: {name} ({len(waypoints)} waypoints" + (", looks at a point)" if point is not None else ")")
        )

    # --------------------------------------------------------------- drawing
    def tick(self) -> None:
        with self._lock:
            if self._realign_due and time.monotonic() - self._drag_time >= self.DRAG_SETTLE_S:
                self._realign_due = False
                self._dirty = True  # the dragged handle gets its axis back on the line
            due = self._check_due and time.monotonic() - self._changed_at >= self.CHECK_DELAY_S
            stale, self._stale_plan = self._stale_plan, False
        if stale:
            self._clear_plan()  # the plan no longer matches the drawing
        auto = bool(self._auto_plan.value)
        if due:
            # Plan continuously: the plan also reports failing segments and
            # fastest times; otherwise only the check runs.
            self._plan(automatic=True) if auto else self._send_check()
        elif auto and self._ours:
            servo = self._ui.servo_status()
            plan = self._ui.plan_status()
            idle = servo is None or servo.state in ("DISABLED", "HOLD")
            if (
                idle and plan.get("state") in ("planned", "succeeded")
                and time.monotonic() - self._last_plan_request > 1.5 and self._robot_resting_elsewhere()
            ):
                self._plan(automatic=True)  # the robot moved since planning: new approach
        self._maybe_check_draft()
        self._draw_draft()
        self._draw_sketch()
        with self._lock:
            if not self._dirty:
                return
            self._dirty = False
            waypoints = [w.copy() for w in self._waypoints]
            k, i, failed, mode = self._seg, self._wp, set(self._failed), self._mode
            scope_part = self._part() if self._scope == "part" and len(self._waypoints) >= 2 else None
            if scope_part is not None:
                k = None  # the part is shown instead of a single segment
        waypoints = self._shown_waypoints(waypoints)
        scene = self._ui.server.scene
        # Children (labels, numbered spheres) were added after their parent
        # frames: remove in reverse order.
        for handle in reversed(self._handles):
            handle.remove()
        self._handles = []
        for g, group in enumerate(self._groups(waypoints)):
            first = waypoints[group[0]]
            frame = scene.add_frame(
                f"/fairino/path/wp_{g}", wxyz=tuple(first.wxyz), position=tuple(first.position),
                axes_length=0.04, axes_radius=0.002,
            )
            selected = i in group
            color = COLOR_SELECTED if selected else (240, 240, 240) if 0 in group else (40, 130, 230)
            sphere = scene.add_icosphere(f"/fairino/path/wp_{g}/dot", radius=0.011, color=color)
            sphere.on_click(lambda _, members=list(group): self._click_group(members))
            self._handles += [sphere, frame]
            if len(group) == 1:
                self._handles.append(scene.add_label(
                    f"/fairino/path/wp_{g}/label", self._wp_label(group[0]), position=(0.0, 0.0, 0.03)
                ))
            if len(group) > 1:
                # Shared point: one small numbered sphere per waypoint, in a
                # row off the point (the waypoint gizmo has no handles in its
                # negative octant), so each one can be picked directly.
                direction = -np.ones(3) / math.sqrt(3.0)
                chips = [direction * (self.CHIP_START_M + self.CHIP_STEP_M * n) for n in range(len(group))]
                self._handles.append(scene.add_line_segments(
                    f"/fairino/path/wp_{g}/chip_line", points=np.array([[np.zeros(3), chips[-1]]]),
                    colors=(150, 150, 150), thickness=0.0015,
                ))
                for n, j in enumerate(group):
                    chip = scene.add_icosphere(
                        f"/fairino/path/wp_{g}/chip_{j}", radius=0.008, position=tuple(chips[n]),
                        color=COLOR_SELECTED if j == i else (40, 130, 230),
                    )
                    chip.on_click(lambda _, member=j: self._click_group([member]))
                    chip_label = scene.add_label(
                        f"/fairino/path/wp_{g}/chip_{j}/label", self._wp_label(j), position=(0.0, 0.0, 0.015)
                    )
                    self._handles += [chip, chip_label]
        for j in range(len(waypoints) - 1):
            p0, p3 = waypoints[j].position, waypoints[j + 1].position
            cp = tt.control_points(waypoints, j)
            c1, c2 = cp if cp is not None else (p0 + (p3 - p0) / 3.0, p0 + 2.0 * (p3 - p0) / 3.0)
            in_part = scope_part is not None and scope_part[0] <= j < scope_part[1]
            color = COLOR_FAILED if j in failed else COLOR_SELECTED if (j == k or in_part) else COLOR_SEGMENT
            self._handles.append(
                scene.add_spline_cubic_bezier(
                    f"/fairino/path/segment_{j}",
                    points=np.array([p0, p3]),
                    control_points=np.array([c1, c2]),
                    thickness=0.006 if j == k or j in failed else 0.004,
                    color=color,
                    segments=64,
                )
            )
        self._updating = True
        try:
            editing = mode == "Edit" and i is not None and i < len(waypoints) and self._ui.view() == "trajectory"
            self._pose_gizmo.visible = editing
            base_axes = self._ui.gizmo_axes() == "base"
            tracking = self._ui.tracking_on()  # the point decides the orientation
            if self._pose_gizmo.disable_rotations != tracking:
                self._pose_gizmo.disable_rotations = tracking
            prev_pos = next_pos = None
            if editing:
                wp = waypoints[i]
                if "pose" not in self._held:
                    self._pose_gizmo.position = tuple(wp.position)
                    self._pose_gizmo.wxyz = (1.0, 0.0, 0.0, 0.0) if base_axes else tuple(wp.wxyz)
                prev_pos, next_pos = self._handle_positions(waypoints, i)
                segments_lines = [[wp.position, p] for p in (prev_pos, next_pos) if p is not None]
                if segments_lines:
                    self._handles.append(
                        scene.add_line_segments(
                            "/fairino/path/handles", points=np.array(segments_lines), colors=COLOR_HANDLE, thickness=0.002
                        )
                    )
            self._prev_gizmo.visible = prev_pos is not None
            self._next_gizmo.visible = next_pos is not None
            dragging = self._dragged_handle if time.monotonic() - self._drag_time < self.DRAG_SETTLE_S else None
            for name, gizmo, pos in (("prev", self._prev_gizmo, prev_pos), ("next", self._next_gizmo, next_pos)):
                if pos is None or name in self._held:
                    continue
                gizmo.position = tuple(pos)
                if name != dragging:
                    # One gizmo axis along the line waypoint -> handle; not
                    # while that handle is being dragged.
                    gizmo.wxyz = tuple(_align_axis(np.asarray(gizmo.wxyz, dtype=float), pos - waypoints[i].position))
        finally:
            self._updating = False

    def _draw_draft(self) -> None:
        """Light green line from the last waypoint (or the robot TCP) to the
        gizmo in Draw mode; red when that pose is not reachable."""
        gizmo = self._ui.gizmo_pose() if self._mode == "Draw" and self._ui.view() == "trajectory" else None
        with self._lock:
            start = self._waypoints[-1].position.copy() if self._waypoints else None
            failed = self._draft_failed
        if start is None and gizmo is not None:
            robot = self._ui.robot_tcp_pose()
            start = None if robot is None else robot[0]
        key = None if gizmo is None or start is None else (tuple(np.round(start, 5)), tuple(np.round(gizmo[0], 5)), failed)
        # Nothing to draw (Edit mode, no gizmo) always removes the line; the
        # key only skips redrawing an unchanged line. (Resetting the key to
        # None used to equal "nothing to draw" and left the line behind.)
        if key == self._draft_key and (key is not None or self._draft is None):
            return
        self._draft_key = key
        if self._draft is not None:
            self._draft.remove()
            self._draft = None
        if key is not None and np.linalg.norm(gizmo[0] - start) > 1e-4:
            self._draft = self._ui.server.scene.add_line_segments(
                "/fairino/path/draft",
                points=np.array([[start, gizmo[0]]]),
                colors=COLOR_FAILED if failed else COLOR_DRAFT,
                thickness=0.004,
            )

    def refresh(self, plan_mode: bool) -> None:
        with self._lock:
            n_wp, n_seg = len(self._waypoints), len(self._segments)
            i = self._wp
            wp = self._waypoints[i] if i is not None and i < n_wp else None
            plan_text = self._plan_text
            mode = self._mode
            curves = sum(1 for w in self._waypoints if w.curve)
            total = self._path_time
            label = self._plan_label
            length = sum(
                float(np.sum(np.linalg.norm(np.diff(tt.segment_points(self._waypoints, j, 0.01)[0], axis=0), axis=1)))
                for j in range(n_seg)
            )
            table = [(j, w.position * 1000.0, w.dwell) for j, w in enumerate(self._waypoints)]
        if n_wp == 0:
            self._summary.content = "Press **New Trajectory** (starts at the robot TCP), move the gizmo, **Confirm Waypoint**."
        else:
            # One line (the panel must not change its height): ✅ planned and
            # executable, ❌ cannot be executed (the reason is in the Robot
            # window), ⏳ planning, ⚪ not planned yet.
            icon = plan_text[:1] if plan_text else ("⚪" if n_seg else "")
            self._summary.content = (
                f"{icon} **{n_wp} waypoints · {n_seg} segments**"
                + (f" · {curves} curve(s)" if curves else "")
                + f" · {length * 1000.0:.0f} mm"
                + (f" · {total:.2f} s" + (f" ({label})" if label.startswith("segments") else "")
                   if total is not None and icon == "✅" else "")
            )
        self._refresh_segment_table()
        # Waypoint folder: the gizmo (Draw) or the selected waypoint (Edit).
        edit = mode == "Edit"
        if not edit:
            gizmo = self._ui.gizmo_pose()
            self._coord_info.content = (
                f"Gizmo: {_pose_text(gizmo[0], gizmo[1])}" if gizmo is not None else "Gizmo: —"
            ) + "  \n*Apply Offset moves the gizmo*"
        elif wp is not None:
            self._coord_info.content = (
                f"Waypoint {self._wp_label(i)}: {_pose_text(wp.position, wp.wxyz)}"
                + f"  \n*Apply Offset moves waypoint {self._wp_label(i)}*"
            )
        else:
            self._coord_info.content = "Pick a waypoint (sphere or list).  \n*Apply Offset needs a waypoint*"
        rows = []
        if table or not edit:
            rows = ["| # | x | y | z [mm] |", "|---|---:|---:|---:|"]
            for j, pos, dwell in table[:40]:
                name = self._wp_label(j) + (f" ⏸{dwell:.1f}s" if dwell > 0.0 else "")
                if edit and j == i:
                    name = f"**{name}**"
                rows.append(f"| {name} | {pos[0]:.1f} | {pos[1]:.1f} | {pos[2]:.1f} |")
            if not edit:
                gizmo = self._ui.gizmo_pose()
                if gizmo is not None:
                    g = gizmo[0] * 1000.0
                    rows.append(f"| **gizmo** | {g[0]:.1f} | {g[1]:.1f} | {g[2]:.1f} |")
        self._waypoint_table.content = "\n".join(rows)
        if self._confirm_button.visible == edit:  # nothing to confirm in Edit
            self._confirm_button.visible = not edit
        for handle in (self._straight_button, self._delete_wp, self._dwell_number):
            if handle.visible != edit:
                handle.visible = edit
        tracking = self._ui.tracking_on()
        for handle in (self._gizmo_axes, self._orient_button):  # point tracking decides the orientation
            if handle.visible == tracking:
                handle.visible = not tracking
        self._delete_wp.disabled = wp is None
        self._straight_button.disabled = wp is None or not wp.curve
        self._dwell_number.disabled = wp is None or i in (0, n_wp - 1)
        self._apply_offset.disabled = edit and wp is None
        self._plan_button.disabled = not plan_mode or (self._scope != "gizmo" and n_wp < 2)

    def problem(self) -> str:
        """Why the path cannot be executed (check or plan), or ""."""
        with self._lock:
            return self._plan_text[1:].strip() if self._plan_text.startswith("❌") else ""

    def _on_dwell(self) -> None:
        if self._updating:
            return
        value = max(0.0, float(self._dwell_number.value))
        with self._lock:
            i = self._wp
            if i is None or not 0 < i < len(self._waypoints) - 1:
                return
            if abs(self._waypoints[i].dwell - value) < 1e-9:
                return
            self._snapshot(merge=True)
            self._waypoints[i].dwell = value
            self._changed()
        self._ui.set_result(
            f"Waypoint {self._wp_label(i)}: " + (f"stop {value:.1f} s" if value > 0.0 else "no stop")
        )

    # ------------------------------------------------- drawing in the servo view
    SKETCH_DT_S = 0.02  # sampling period of the held servo gizmo
    SKETCH_MAX_SAMPLES = 60000  # 20 min at 50 Hz

    def servo_hold(self, held: bool) -> None:
        """The servo gizmo was grabbed (True) or released (False)."""
        with self._lock:
            self._sketch_held = bool(held)
            self._sketch_clock = None

    def servo_sample(self, position: np.ndarray, wxyz: np.ndarray) -> None:
        """One pose of the held servo gizmo (base_link). Time runs only while
        it is held: a pause after a release is not part of the drawing, holding
        it still is (a stop)."""
        now = time.monotonic()
        with self._lock:
            if not self._sketch_held or len(self._sketch) >= self.SKETCH_MAX_SAMPLES:
                return
            if self._sketch_clock is None:
                self._sketch_t += self.SKETCH_DT_S if self._sketch else 0.0  # grabbed again
            else:
                self._sketch_t += now - self._sketch_clock
            self._sketch_clock = now
            self._sketch.append((self._sketch_t, np.asarray(position, dtype=float).copy(), np.asarray(wxyz, dtype=float).copy()))

    def servo_drawing_text(self) -> str:
        with self._lock:
            sketch = self._sketch
            t = sketch[-1][0] if sketch else 0.0
            length = (
                float(np.sum(np.linalg.norm(np.diff(np.array([p for _, p, _ in sketch]), axis=0), axis=1)))
                if len(sketch) > 1 else 0.0
            )
        return f"Drawn since the last waypoint: {t:.1f} s · {length * 1000.0:.0f} mm"

    def servo_discard(self) -> None:
        with self._lock:
            had = bool(self._sketch)
            self._sketch, self._sketch_t, self._sketch_clock = [], 0.0, None
        self._ui.set_result("Drawing discarded" if had else "Nothing drawn to discard")

    def servo_undo(self) -> None:
        """Undo in the servo view (the servo gizmo stays where it is)."""
        with self._lock:
            possible = bool(self._undo)
        self._edit("Undo")
        with self._lock:
            n = len(self._waypoints)
        self._ui.set_result(f"Undo: {n} waypoint(s)" if possible else "Undo: nothing to undo")

    def servo_confirm(self) -> None:
        """Servo view: the drawing since the last waypoint (if any) becomes
        waypoints, ending with a waypoint at the servo gizmo; drawing then
        continues from there. Without a drawing it only adds that waypoint
        (teaching by driving the robot); the first one starts the path."""
        pose = self._ui.servo_gizmo_pose()
        with self._lock:
            sketch = self._sketch
            self._sketch, self._sketch_t, self._sketch_clock = [], 0.0, None
        if pose is None:
            self._ui.set_result("Confirm: no servo gizmo pose yet")
            return
        new_wps: list[tt.Waypoint] = []
        new_segs: list[tt.Segment] = []
        if len(sketch) >= 2:
            new_wps, new_segs = tt.drawing_to_path(
                np.array([t for t, _, _ in sketch]),
                np.array([p for _, p, _ in sketch]),
                np.array([q for _, _, q in sketch]),
            )
        end = tt.Waypoint(pose[0].copy(), pose[1].copy())
        if not new_wps:
            new_wps = [end]
        elif not _same_pose(new_wps[-1], end):
            new_wps.append(end)  # the gizmo moved after the drawing (e.g. turned toward the point)
            new_segs.append(tt.Segment())
        drawn = sketch[-1][0] if len(sketch) >= 2 else 0.0
        stops = sum(1 for w in new_wps if w.dwell > 0.0)
        with self._lock:
            merge = bool(self._waypoints) and _same_pose(self._waypoints[-1], new_wps[0])
            if merge and len(new_wps) == 1 and new_wps[0].dwell <= 0.0:
                self._ui.set_result("Confirm: the gizmo is on the last waypoint (nothing drawn)")
                return
            self._snapshot()
            if merge:
                self._waypoints[-1].dwell += new_wps[0].dwell  # the drawing starts at the last waypoint
                new_wps = new_wps[1:]
            elif self._waypoints:
                new_segs = [tt.Segment()] + new_segs  # straight from the last waypoint to the drawing
            self._waypoints += new_wps
            self._segments += new_segs
            self._seg = len(self._segments) - 1 if self._segments else None
            self._changed()
            n = len(self._waypoints)
        self._sync_fields()
        self._ui.set_result(
            f"Confirm: waypoint {n} at the gizmo"
            + (f" (+{len(new_wps) - 1} from the drawing, {drawn:.1f} s" + (f", {stops} stop(s)" if stops else "") + ")"
               if drawn > 0.0 else "")
        )

    def _draw_sketch(self) -> None:
        """Green line of the servo drawing (servo view), from the last waypoint."""
        show = self._ui.view() == "servo"
        with self._lock:
            points = [p for _, p, _ in self._sketch] if show else []
            start = self._waypoints[-1].position.copy() if self._waypoints and points else None
        key = (len(points), None if start is None else tuple(np.round(start, 5)))
        if key == self._sketch_key:
            return
        self._sketch_key = key
        if self._sketch_handle is not None:
            self._sketch_handle.remove()
            self._sketch_handle = None
        if len(points) < 2 and start is None:
            return
        line = np.array(([start] if start is not None else []) + points)
        if len(line) < 2:
            return
        travel = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(line, axis=0), axis=1))])
        keep = np.unique(np.concatenate([[0, 1], np.searchsorted(travel, np.arange(0.0, travel[-1], 0.002)), [len(line) - 1]]))
        keep = keep[keep < len(line)]
        line = line[keep]
        if len(line) < 2:
            return
        self._sketch_handle = self._ui.server.scene.add_line_segments(
            "/fairino/path/sketch", points=np.stack([line[:-1], line[1:]], axis=1), colors=COLOR_DRAFT, thickness=0.004
        )
