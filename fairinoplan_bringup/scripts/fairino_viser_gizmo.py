"""Target gizmo for the FAIRINO Viser UI.

One instance per view: the servo gizmo (the target the servo follows) and the
trajectory gizmo (drawing and planning; the robot does not follow it). Poses
are world poses (in this UI the world is base_link).

Gizmo axes:
  "tool"  the gizmo is the target pose: its arrows and rings are the tool axes.
  "base"  the gizmo rests at the base axes; moving it moves the target, a ring
          turns the kept orientation about that base axis, and after release
          the gizmo returns to the base axes. Small axes at the target show
          its orientation.
Point tracking: base axes without rotation rings; the orientation turns the
tool z axis toward the point by the smallest rotation from the previous one,
optionally keeping the roll of a reference (the robot) and limited to a turn
rate (while the robot follows).
"""

from __future__ import annotations

import threading
import time
from typing import Callable

import numpy as np
import pinocchio as pin

import fairino_trajectory_tools as tt


def _quat(rotation: np.ndarray) -> np.ndarray:
    q = pin.Quaternion(rotation)
    return np.array([q.w, q.x, q.y, q.z])


def _rotation(wxyz) -> np.ndarray:
    w, x, y, z = (float(v) for v in wxyz)
    return pin.Quaternion(w, x, y, z).normalized().matrix()


class TargetGizmo:
    def __init__(
        self,
        scene,
        name: str,
        *,
        base_rotation: np.ndarray,
        point: Callable[[], np.ndarray],
        roll_source: Callable[[], np.ndarray | None] | None = None,
        turn_rate: float | None = None,
        turning: Callable[[], bool] | None = None,
        on_change: Callable[[], None] | None = None,
        on_hold: Callable[[bool], None] | None = None,
        scale: float = 0.25,
    ) -> None:
        self._base_rotation = base_rotation
        self._point = point
        self._roll_source = roll_source
        self._turn_rate = turn_rate
        self._turning = turning or (lambda: False)
        self._on_change = on_change
        self._on_hold = on_hold
        self.held = False  # grabbed by the mouse
        self._lock = threading.RLock()
        self._mode = "tool"
        self._tracking = False
        self._rotation: np.ndarray | None = None  # kept target orientation (base axes / tracking)
        self._look_time = time.monotonic()
        self._suppress = False
        self._visible = True
        self.initialized = False
        self.controls = scene.add_transform_controls(name, scale=scale, depth_test=False, disable_sliders=False)
        self._axes = scene.add_frame(f"{name}_orientation", axes_length=0.12, axes_radius=0.005, visible=False)
        self._line = scene.add_line_segments(
            f"{name}_line", points=np.zeros((1, 2, 3)), colors=(230, 40, 40), thickness=0.0015, visible=False
        )
        self.controls.on_update(lambda _: self._moved())
        self.controls.on_drag_start(lambda _: self._hold(True))
        self.controls.on_drag_end(lambda _: self._drag_end())

    # ---------------------------------------------------------------- state
    def base_axes(self) -> bool:
        return self._tracking or self._mode == "base"

    def axes(self) -> str:
        """Effective axes: "base" or "tool" (point tracking uses base)."""
        return "base" if self.base_axes() else "tool"

    def tracking(self) -> bool:
        return self._tracking

    def _gizmo_rotation(self) -> np.ndarray:
        return _rotation(self.controls.wxyz)

    def _current(self) -> np.ndarray:
        """Target orientation now, without advancing a tracking turn."""
        if self._rotation is None or not self.base_axes():
            return self._gizmo_rotation()
        if self._tracking:
            return self._rotation
        # base axes: the gizmo's turn away from the base axes (during a drag)
        # turns the kept orientation about the base axes.
        return self._gizmo_rotation() @ self._base_rotation.T @ self._rotation

    def pose(self) -> pin.SE3 | None:
        """The target (world). With point tracking the orientation is turned
        toward the point (at most turn_rate while turning())."""
        with self._lock:
            if not self.initialized:
                return None
            position = np.asarray(self.controls.position, dtype=float)
            rotation = self._current()
            if self._tracking:
                rotation = self._look(rotation, position)
            return pin.SE3(rotation, position)

    def _desired(self, previous: np.ndarray, position: np.ndarray) -> np.ndarray:
        if self._roll_source is not None:
            reference = self._roll_source()
            if reference is not None:
                # Keep the reference's roll (the robot: its roll is chosen by
                # the servo when the roll is free).
                previous = tt.rotation_between(reference[:, 2], previous[:, 2]) @ reference
        return tt.look_at_rotation(previous, position, self._point())

    def _look(self, previous: np.ndarray, position: np.ndarray) -> np.ndarray:
        desired = self._desired(previous, position)
        now = time.monotonic()
        rotation = desired
        if self._turn_rate is not None and self._turning():
            step = self._turn_rate * float(np.clip(now - self._look_time, 0.0, 0.1))
            delta = pin.log3(previous.T @ desired)
            angle = float(np.linalg.norm(delta))
            if angle > step:
                rotation = previous @ pin.exp3(delta * (step / angle))
        self._look_time = now
        self._rotation = rotation
        return rotation

    def turning_left(self) -> float:
        """Angle [rad] still to turn toward the point (0 without tracking)."""
        with self._lock:
            if not self._tracking or self._rotation is None or not self.initialized:
                return 0.0
            desired = self._desired(self._rotation, np.asarray(self.controls.position, dtype=float))
            return float(np.linalg.norm(pin.log3(self._rotation.T @ desired)))

    # -------------------------------------------------------------- setters
    def set_pose(self, pose: pin.SE3) -> None:
        """Places the gizmo (no on_change): position and target orientation."""
        with self._lock:
            self._rotation = pose.rotation.copy()
            self._look_time = time.monotonic()
            self._write(pose.translation, self._base_rotation if self.base_axes() else pose.rotation)
            self.initialized = True
        self.refresh()

    def set_axes(self, mode: str) -> None:
        with self._lock:
            current = self._current() if self.initialized else None
            self._mode = mode
            self._apply(current)

    def set_tracking(self, on: bool, rotation: np.ndarray | None = None) -> None:
        """Point tracking on/off. rotation: the target orientation to keep
        (default: the current one)."""
        with self._lock:
            current = rotation if rotation is not None else (self._current() if self.initialized else None)
            self._tracking = on
            self._look_time = time.monotonic()
            self._apply(current)

    def set_visible(self, visible: bool) -> None:
        with self._lock:
            self._visible = bool(visible)
            if self.controls.visible != self._visible:
                self.controls.visible = self._visible
        self.refresh()

    def _apply(self, rotation: np.ndarray | None) -> None:
        if rotation is not None:
            self._rotation = rotation.copy()
        shown = self._base_rotation if self.base_axes() or rotation is None else rotation
        self._suppress = True
        try:
            if self.controls.disable_rotations != self._tracking:
                self.controls.disable_rotations = self._tracking  # the point decides
            self.controls.wxyz = _quat(shown)
        finally:
            self._suppress = False
        self.refresh()

    def _write(self, position, rotation: np.ndarray) -> None:
        self._suppress = True
        try:
            self.controls.position = tuple(float(v) for v in position)
            self.controls.wxyz = _quat(rotation)
        finally:
            self._suppress = False

    # -------------------------------------------------------------- events
    def _moved(self) -> None:
        if self._suppress:
            return
        if self._on_change is not None:
            self._on_change()
        self.refresh()

    def _hold(self, held: bool) -> None:
        self.held = held
        if self._on_hold is not None:
            self._on_hold(held)

    def _drag_end(self) -> None:
        self._hold(False)
        with self._lock:
            if not self.initialized or not self.base_axes() or self._tracking or self._rotation is None:
                return
            # base axes: keep the turned orientation, back to the base axes.
            self._rotation = self._current()
            self._write(self.controls.position, self._base_rotation)
        self.refresh()

    def refresh(self) -> None:
        """Small axes (orientation) and the line to the point."""
        with self._lock:
            show_axes = self._visible and self.initialized and self.base_axes() and self._rotation is not None
            show_line = self._visible and self.initialized and self._tracking
            if self._axes.visible != show_axes:
                self._axes.visible = show_axes
            if self._line.visible != show_line:
                self._line.visible = show_line
            if not (show_axes or show_line):
                return
            position = np.asarray(self.controls.position, dtype=float)
            if show_axes:
                self._axes.position = tuple(position)
                self._axes.wxyz = tuple(_quat(self._current()))
            if show_line:
                self._line.points = np.array([[position, self._point()]])
