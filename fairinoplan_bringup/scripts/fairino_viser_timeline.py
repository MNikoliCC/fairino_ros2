"""Trajectory timeline for the FAIRINO Viser UI: uPlot charts of the planned
trajectory over time along the bottom of the 3D view, from the left edge to
the control panel. A row of toggles chooses what is shown, any combination:
TCP speed, joint positions, joint velocities, joint accelerations; the
charts are stacked and share the time cursor. After an Execute the measured
values (positions, the controller's actual_qd / actual_qdd, the TCP speed
from the measured positions) are drawn over the plan as dashed lines.

Viser places panels by pixels only (``float(x, y, width, height)``) and does
not report the browser's size (the camera's image size is in device pixels and
follows the adaptive DPR). So every browser gets its own panel, and a small
script in the page reports the window width and the control panel's left edge
through a hidden text field; the panel is then placed with ``float``.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field

import numpy as np
import viser
import viser.uplot as uplot

MARGIN_PX = 12
HEADER_PX = 150  # tab bar, chart and joint toggles, title line
PLOT_AREA_PX = {1: 150, 2: 112, 3: 92, 4: 80}  # drawing area of each chart by the number shown
TOP_AXIS_PX = (2, 16)  # waypoint axis without / with numbers
X_AXIS_PX = (12, 34, 22)  # time axis hidden (charts above) / shown (bottom chart), its label
LEGEND_PX = {"tcp": 30, "q": 30, "qd": 30, "qdd": 30}  # legend and padding
CONTROL_RESERVE_PX = 24 * 16 + 15  # "large" control panel (24 em) + its gap, if it is not found
MAX_POINTS = 2000  # samples on the common time grid
LABEL_MIN_PX = 16  # waypoint numbers closer than this are left out
PLOT_INSET_PX = 190  # panel width minus the plot area (axes, padding)
JOINT_COLORS = ("#ff922b", "#51cf66", "#4dabf7", "#cc5de8", "#ff6b6b", "#fcc419")  # orange green blue purple red yellow
# Dashes of the joint range lines: FR20 ranges coincide (j1/j5/j6 ±175°,
# j2/j4 -265°/85°), so long, short and dotted dashes keep every colour visible.
LIMIT_DASHES = ([9, 9], [9, 9], [9, 9], [6, 12], [6, 12], [3, 15])  # same period: equal thirds where three coincide
SYNC_KEY = "fairino-timeline"
QD_LIMITS_DEG_S = (120.0, 180.0)  # j1-j3, j4-j6 (FR20 v_max, as max_joint_velocity_deg_s)
QDD_LIMIT_DEG_S2 = 540.0
GROUPS = (("tcp", "TCP"), ("q", "Positions"), ("qd", "Velocities"), ("qdd", "Accelerations"))
UNITS = {"tcp": "TCP mm/s", "q": "angle deg", "qd": "vel. deg/s", "qdd": "acc. deg/s²"}
# One series layout for every chart, so a legend click (synced by series
# index in uPlot) hides a joint on all charts: t, j1-j6 plan, j1-j6
# measured, TCP plan, TCP measured, waypoints. Series a chart does not draw
# are NaN and hidden from its legend (CSS class).
TCP_SERIES = 13
HIDDEN = "fx-hide"

# Reports "window width,control panel left" (CSS px) into the hidden field
# whenever they change; hides that field's row.
VIEWPORT_SCRIPT = """<img src="data:," style="display:none" onerror="
if(!window.__fairinoViewport){window.__fairinoViewport=1;let last='';
const st=document.createElement('style');st.textContent='.u-legend .u-series.fx-hide{display:none} .u-legend{pointer-events:none}';document.head.appendChild(st);
const find=()=>{const l=[...document.querySelectorAll('label')].find(e=>e.textContent.trim()==='viewport_px');
if(!l)return null;let r=l;while(r&&!r.querySelector('input'))r=r.parentElement;if(!r)return null;r.style.display='none';return r.querySelector('input');};
const tick=()=>{const inp=find();if(!inp)return;
const b=[...document.querySelectorAll('button')].find(x=>/[☐☑] Trajectory/.test(x.textContent));
let p=b;while(p&&p.parentElement&&p.parentElement.getBoundingClientRect().width<0.6*innerWidth)p=p.parentElement;
const left=p?Math.round(p.getBoundingClientRect().left):-1;const v=innerWidth+','+left;
if(v===last&&inp.value===v)return;last=v;
const set=Object.getOwnPropertyDescriptor(HTMLInputElement.prototype,'value').set;set.call(inp,v);
inp.dispatchEvent(new Event('input',{bubbles:true}));};
setInterval(tick,700);addEventListener('resize',tick);}">"""


@dataclass
class Motion:
    """Joint motion over time [s]: positions [deg], velocities [deg/s],
    accelerations [deg/s^2] (N x 6) and the TCP speed [mm/s] (N)."""

    times: np.ndarray
    tcp_speed: np.ndarray
    q: np.ndarray
    qd: np.ndarray
    qdd: np.ndarray


@dataclass
class _State:
    plan: Motion | None = None
    measured: Motion | None = None
    marks: list = field(default_factory=list)  # waypoint arrival times [s]
    labels: list = field(default_factory=list)  # their numbers
    title: str = "Planned trajectory: nothing planned"


def _toggle_options(shown: set) -> tuple:
    return tuple(f"{'☑' if key in shown else '☐'} {label}" for key, label in GROUPS)


JOINT_SWATCHES = ("🟧", "🟩", "🟦", "🟪", "🟥", "🟨")  # the joint colours


def _joint_options(joints: set, planned: bool, measured: bool) -> tuple:
    return tuple(f"{'☑' if j in joints else '☐'} {JOINT_SWATCHES[j - 1]} j{j}" for j in range(1, 7)) + (
        f"{'☑' if planned else '☐'} planned",
        f"{'☑' if measured else '☐'} measured",
    )


def _series(group: str, joints: set | None = None, planned: bool = True, measured: bool = True) -> tuple:
    """The common series layout; the ones this chart does not draw are hidden
    from its legend. joints / planned / measured: what the toggles show (kept
    in the chart's options, so a redraw, e.g. after a zoom, keeps it)."""
    joint = group != "tcp"
    joints = set(range(1, 7)) if joints is None else joints

    def cls(shown: bool) -> dict:
        return {} if shown else {"class": HIDDEN}

    return (
        uplot.Series(label="t [s]"),
        *(
            uplot.Series(label=f"j{j + 1}", scale="y", stroke=c, width=1.5, show=planned and j + 1 in joints, **cls(joint))
            for j, c in enumerate(JOINT_COLORS)
        ),
        *(
            uplot.Series(
                label=f"j{j + 1} m", scale="y", stroke=c, width=1, dash=[4, 3], show=measured and j + 1 in joints,
                **cls(joint),
            )
            for j, c in enumerate(JOINT_COLORS)
        ),
        uplot.Series(label="TCP plan", scale="y", stroke="#74c0fc", width=2, show=planned, **cls(not joint)),
        uplot.Series(
            label="TCP measured", scale="y", stroke="#ffa8a8", width=1, dash=[4, 3], show=measured, **cls(not joint)
        ),
        uplot.Series(
            label="waypoint", scale="y", stroke="#ffd43b", width=0, **cls(not joint),
            points=uplot.Series_Points(show=True, size=8, fill="#ffd43b", stroke="#ffd43b"),
        ),
    )


LEFT_AXIS_PX = 92  # value axis, the same on every chart so the time axes line up
RIGHT_AXIS_PX = 82  # limit labels on the right (the same width everywhere)
LABEL_GAP_PX = 11  # labels closer than this on one side collide
LABEL_FONT = "11px monospace"  # monospace: the coloured digits are drawn by separate axes and must line up
LABEL_GREY = "#adb5bd"


def _label(value: float, joints: list[int], side: str) -> list[tuple[str, str]]:
    """Limit label as (text, colour) pieces: joints first on the left (the
    value next to the chart), the value first on the right; the joint
    numbers in their colours, the rest grey. ASCII minus: a typographic one
    is not monospace in every font."""
    number = "0" if abs(value) < 1e-9 else f"{value:.0f}"
    if not joints:
        return [(number, LABEL_GREY)]
    names: list[tuple[str, str]] = [("j", LABEL_GREY)]
    for k, j in enumerate(sorted(joints)):
        if k:
            names.append((",", LABEL_GREY))
        names.append((str(j), JOINT_COLORS[j - 1]))
    return names + [(" " + number, LABEL_GREY)] if side == "left" else [(number + " ", LABEL_GREY)] + names


def _place_labels(owners: dict[float, list[int]], y_range: tuple[float, float], area_px: float, mirror: bool) -> tuple:
    """Every limit on the left unless it would collide there (then on the
    right); with mirror also on the right wherever it fits. Returns the
    values for the left and for the right."""
    px = area_px / max(y_range[1] - y_range[0], 1e-6)

    def fits(placed: list, value: float) -> bool:
        return all(abs(value - other) * px >= LABEL_GAP_PX for other in placed)

    left, right = [], []
    for value in sorted(owners):
        (left if fits(left, value) else right).append(value)
    if mirror:
        for value in sorted(left):
            if fits(right, value):
                right.append(value)
    return sorted(left), sorted(right)


def _label_axes(side: str, values: list[float], owners: dict, size: int, title: str | None, grid) -> list:
    """Axes drawing the labels of one side: one per colour, all of size 0 at
    the chart edge (so their texts overlay into one label), then an empty
    axis of the side's width (with the axis title on the left)."""
    labels = [_label(v, owners[v], side) for v in values]
    colours = [LABEL_GREY] + [c for c in JOINT_COLORS if any(c == colour for label in labels for _, colour in label)]
    axis_side = 3 if side == "left" else 1
    axes = []
    for colour in colours:
        texts = ["".join(t if c == colour else " " * len(t) for t, c in label) for label in labels]
        axes.append(uplot.Axis(
            scale="y", side=axis_side, size=0, gap=5, font=LABEL_FONT, stroke=colour,
            splits=values or [1e12], values=texts or [""],
            ticks=uplot.Axis_Ticks(show=True, size=8, stroke=LABEL_GREY if colour == LABEL_GREY else "rgba(0,0,0,0)"),
            grid=grid if colour == LABEL_GREY else uplot.Axis_Grid(show=False),
        ))
    axes.append(uplot.Axis(
        scale="y", side=axis_side, size=size, label=title, splits=[1e12], values=[""],
        ticks=uplot.Axis_Ticks(show=False), grid=uplot.Axis_Grid(show=False),
    ))
    return axes


def _value_axis(group: str, joint_limits: list, y_range: tuple[float, float], area_px: float, joints: set) -> list:
    """Value axes: ticks at 0 and the limits only, labelled with the joints
    they hold for (shared limits once), on the left and, where they fit, on
    the right. Positions: each joint's range as dashed lines in its colour;
    velocities, accelerations, TCP: grey dashed limit lines. TCP keeps its
    regular ticks on the left. Every chart has the same axis widths."""
    limit_grid = uplot.Axis_Grid(show=True, stroke="rgba(255, 255, 255, 0.32)", width=1, dash=[5, 4])
    no_grid = uplot.Axis_Grid(show=False)
    title = UNITS[group]
    if group == "tcp":  # regular ticks, the same on both sides
        return [
            uplot.Axis(scale="y", side=3, label=title, size=LEFT_AXIS_PX, space=18),
            uplot.Axis(scale="y", side=1, size=RIGHT_AXIS_PX, space=18, grid=no_grid),
        ]
    owners: dict[float, list[int]] = {0.0: []}
    if group == "q":
        limits = joint_limits
    elif group == "qd":
        limits = [(-QD_LIMITS_DEG_S[0], QD_LIMITS_DEG_S[0])] * 3 + [(-QD_LIMITS_DEG_S[1], QD_LIMITS_DEG_S[1])] * 3
    else:
        limits = []
        owners.update({-QDD_LIMIT_DEG_S2: [], QDD_LIMIT_DEG_S2: []})  # the same for all joints
    for j, (low, high) in enumerate(limits):
        if j + 1 not in joints:
            continue  # a hidden joint: no limit lines or labels
        for value in (round(low, 1), round(high, 1)):
            owners.setdefault(value, []).append(j + 1)
    left, right = _place_labels(owners, y_range, area_px, mirror=True)
    grid = no_grid if group == "q" else limit_grid  # positions: the coloured range lines below
    axes = _label_axes("left", left, owners, LEFT_AXIS_PX, title, grid)
    axes += _label_axes("right", right, owners, RIGHT_AXIS_PX, None, no_grid)
    if group == "q":
        axes.append(uplot.Axis(  # the 0 line, as on the other charts
            scale="y", side=1, size=0, gap=0, splits=[0.0], values=[""], ticks=uplot.Axis_Ticks(show=False),
            grid=limit_grid,
        ))
        for j, ((low, high), color, dash) in enumerate(zip(joint_limits, JOINT_COLORS, LIMIT_DASHES)):
            if j + 1 not in joints:
                continue
            axes.append(uplot.Axis(
                scale="y", side=1, size=0, gap=0, splits=[low, high], values=["", ""],
                ticks=uplot.Axis_Ticks(show=False), grid=uplot.Axis_Grid(show=True, stroke=color, width=1, dash=dash),
            ))
    return axes


def _axes(
    group: str, marks: list, labels: list, duration: float, width_px: float, numbers: bool, bottom: bool,
    joint_limits: list, y_range: tuple[float, float], area_px: float, joints: set | None = None,
) -> tuple:
    """Time axis (labelled on the bottom chart only; the others keep its grid),
    value axis, and at the top the waypoint arrival times: faint dashed
    lines, with their numbers on the top-most chart only."""
    shown, last_px = [], -1e9
    scale = max(width_px - PLOT_INSET_PX, 100.0) / max(duration, 1e-6)
    for t, label in zip(marks, labels):
        x = t * scale
        shown.append(label if numbers and x - last_px >= LABEL_MIN_PX else "")
        last_px = x if shown[-1] else last_px
    time_axis = (
        uplot.Axis(label="time [s]", size=X_AXIS_PX[1], labelSize=X_AXIS_PX[2])
        if bottom else uplot.Axis(size=X_AXIS_PX[0], stroke="rgba(0, 0, 0, 0)", ticks=uplot.Axis_Ticks(show=False))
    )
    joints = set(range(1, 7)) if joints is None else joints
    axes = [time_axis, *_value_axis(group, joint_limits, y_range, area_px, joints)]
    if marks:
        axes.append(uplot.Axis(
            scale="x", side=0, splits=list(marks), values=shown, size=TOP_AXIS_PX[numbers], gap=2,
            font="10px sans-serif", stroke="#c9a227",
            grid=uplot.Axis_Grid(show=True, stroke="rgba(255, 212, 59, 0.18)", width=1, dash=[3, 4]),
            ticks=uplot.Axis_Ticks(show=False),
        ))
    return tuple(axes)


def _range(plan: np.ndarray, measured: np.ndarray, floor: float | None = None, pad: float = 0.06) -> tuple[float, float]:
    """y range (Viser's uPlot does not autoscale on a data update): the plan,
    and the measured values without their 0.5 % most extreme samples (the
    SimMachine's catch-up spikes would flatten everything else)."""
    p = np.ravel(plan)[np.isfinite(np.ravel(plan))]
    m = np.ravel(measured)[np.isfinite(np.ravel(measured))]
    parts = [p] + ([np.percentile(m, [0.5, 99.5])] if m.size else [])
    values = np.concatenate(parts) if p.size or m.size else np.zeros(0)
    if values.size == 0:
        return (0.0, 1.0)
    low, high = float(values.min()), float(values.max())
    if floor is not None:
        low = min(low, floor)
    span = max(high - low, 1.0)
    return (low if floor is not None else low - pad * span, high + pad * span)


def _on_grid(grid: np.ndarray, motion: Motion | None, name: str) -> np.ndarray:
    """A motion's quantity on the common grid (NaN outside its time span)."""
    columns = 1 if name == "tcp_speed" else 6
    if motion is None or len(motion.times) < 2:
        return np.full((len(grid), columns), np.nan)
    values = np.asarray(getattr(motion, name), dtype=float).reshape(len(motion.times), -1)
    out = np.stack([np.interp(grid, motion.times, values[:, j]) for j in range(columns)], axis=1)
    out[(grid < motion.times[0]) | (grid > motion.times[-1])] = np.nan
    return out


def _limited(rng: tuple[float, float], limit: float) -> tuple[float, float]:
    """Velocity / acceleration range: always shows the limits, measured
    spikes beyond 1.3 x the limit leave the chart."""
    return (max(min(rng[0], -1.12 * limit), -1.3 * limit), min(max(rng[1], 1.12 * limit), 1.3 * limit))


def _chart_data(state: _State, joint_limits: list) -> dict:
    """Data tuple and y range of every chart, on one time grid."""
    end = max(
        float(state.plan.times[-1]) if state.plan is not None else 1.0,
        float(state.measured.times[-1]) if state.measured is not None else 0.0,
        1e-3,
    )
    grid = np.linspace(0.0, end, int(np.clip(end / 0.004, 2, MAX_POINTS)))
    marks = np.array(state.marks, dtype=float)
    if len(marks):
        grid = np.unique(np.concatenate([grid, marks]))
    plan, meas = state.plan, state.measured
    nan = np.full(len(grid), np.nan)
    tcp_p, tcp_m = _on_grid(grid, plan, "tcp_speed")[:, 0], _on_grid(grid, meas, "tcp_speed")[:, 0]
    wp = nan.copy()
    if len(marks):
        idx = np.searchsorted(grid, marks)
        wp[idx] = np.nan_to_num(tcp_p[idx])
    out = {
        "tcp": ((grid, *([nan] * 12), tcp_p, tcp_m, wp), _range(tcp_p, tcp_m, floor=0.0)),
        "duration": float(grid[-1]),
        "x": (0.0, float(grid[-1])),
    }
    for group in ("q", "qd", "qdd"):
        p, m = _on_grid(grid, plan, group), _on_grid(grid, meas, group)
        rng = _range(p, m)
        if group == "q" and joint_limits:
            limits = np.array(joint_limits, dtype=float)
            span = max(limits.max() - limits.min(), 1.0)
            rng = (min(rng[0], limits.min() - 0.06 * span), max(rng[1], limits.max() + 0.09 * span))  # room for the labels
        elif group == "qd":
            rng = _limited(rng, QD_LIMITS_DEG_S[1])
        elif group == "qdd":
            rng = _limited(rng, QDD_LIMIT_DEG_S2)
        out[group] = ((grid, *(p[:, j] for j in range(6)), *(m[:, j] for j in range(6)), nan, nan, nan), rng)
    return out


def _scales(charts: dict, key: str) -> dict:
    """Explicit x and y ranges (no autoscale in Viser's uPlot on updates)."""
    return {
        "x": uplot.Scale(time=False, auto=False, range=charts["x"]),
        "y": uplot.Scale(auto=False, range=charts[key][1]),
    }


def _heights(shown: list) -> dict:
    """Canvas height of each shown chart: drawing area plus its axes."""
    area = PLOT_AREA_PX[len(shown)]
    return {
        key: area + TOP_AXIS_PX[key == shown[0]] + (X_AXIS_PX[1] + X_AXIS_PX[2] if key == shown[-1] else X_AXIS_PX[0])
        for key in shown
    }


class TrajectoryTimeline:
    def __init__(self, server: viser.ViserServer, joint_limits_deg: list[tuple[float, float]] | None = None) -> None:
        self._server = server
        self._limits = list(joint_limits_deg or [])  # (lower, upper) per joint [deg]
        self._lock = threading.Lock()
        self._clients: dict[int, dict] = {}
        self._state = _State()
        self._charts = _chart_data(self._state, self._limits)
        server.on_client_connect(self._connect)
        server.on_client_disconnect(lambda client: self._clients.pop(client.client_id, None))

    # ------------------------------------------------------------- clients
    def _connect(self, client: viser.ClientHandle) -> None:
        gui = client.gui
        panel = gui.add_panel()
        entry = {
            "panel": panel, "size": None, "shown": {"tcp"}, "plots": {}, "joints": set(range(1, 7)),
            "planned": True, "measured": True,
        }
        with self._lock:
            charts, state = self._charts, self._state
        with panel.add_tab("Trajectory timeline", icon=viser.Icon.CHART_LINE):
            entry["toggles"] = gui.add_button_group("Show", _toggle_options(entry["shown"]))
            entry["title"] = gui.add_markdown(f"*{state.title}*")
            for key, _ in GROUPS:
                data, rng = charts[key]
                entry["plots"][key] = gui.add_uplot(
                    data, _series(key), scales=_scales(charts, key),
                    axes=_axes(
                        key, state.marks, state.labels, charts["duration"], 900.0, key == "tcp", key == "tcp",
                        self._limits, charts[key][1], PLOT_AREA_PX[1],
                    ),
                    height=PLOT_AREA_PX[1], legend=uplot.Legend(show=True, live=True),
                    cursor=uplot.Cursor(sync=uplot.Cursor_Sync(key=SYNC_KEY, setSeries=False)),  # shared cursor
                    visible=key in entry["shown"],
                )
            # Joints (with their limits), planned and measured curves on all
            # charts; kept here, not in the browser's legend, so a redraw keeps it.
            entry["joint_toggles"] = gui.add_button_group("Joints", _joint_options(entry["joints"], True, True))
            viewport = gui.add_text("viewport_px", "")
            gui.add_html(VIEWPORT_SCRIPT)
        entry["toggles"].on_click(lambda e: self._toggle(entry, str(e.target.value)))
        entry["joint_toggles"].on_click(lambda e: self._toggle_joint(entry, str(e.target.value)))
        viewport.on_update(lambda _: self._place(entry, str(viewport.value)))
        with self._lock:
            self._clients[client.client_id] = entry
        self._layout(entry)

    def _toggle(self, entry: dict, clicked: str) -> None:
        key = next(k for k, label in GROUPS if clicked.endswith(label))
        shown = set(entry["shown"]) ^ {key}
        if not shown:
            return  # at least one chart
        entry["shown"] = shown
        entry["toggles"].options = _toggle_options(shown)
        self._layout(entry)

    def _toggle_joint(self, entry: dict, clicked: str) -> None:
        if clicked.endswith("measured"):
            entry["measured"] = not entry["measured"]
        elif clicked.endswith("planned"):
            entry["planned"] = not entry["planned"]
        else:
            entry["joints"] = set(entry["joints"]) ^ {int(clicked[-1])}
        entry["joint_toggles"].options = _joint_options(entry["joints"], entry["planned"], entry["measured"])
        for key, plot in entry["plots"].items():
            plot.series = _series(key, entry["joints"], entry["planned"], entry["measured"])
        self._layout(entry)

    def _place(self, entry: dict, value: str) -> None:
        """Bottom of the viewport, from the left edge to the control panel."""
        try:
            width_px, left_px = (int(v) for v in value.split(","))
        except ValueError:
            return
        right = left_px if 0 < left_px < width_px else width_px - CONTROL_RESERVE_PX
        width = max(320, right - 2 * MARGIN_PX)
        if entry["size"] is not None and abs(entry["size"] - width) < 4:
            return
        entry["size"] = width
        self._layout(entry)

    def _layout(self, entry: dict) -> None:
        """Panel size for the shown charts; their heights, visibility and axes
        (waypoint numbers on the top-most one)."""
        shown = [key for key, _ in GROUPS if key in entry["shown"]]
        heights = _heights(shown)
        width = float(entry["size"] or 900)
        # Placed only when its width changes: a placement command re-places
        # (and re-opens) the panel, so new plans must not send one. No pinned
        # height: the panel follows its content (charts shown or minimized)
        # and stays anchored to the bottom.
        if entry.get("placed_width") != width:
            entry["placed_width"] = width
            entry["panel"].float(x=MARGIN_PX, y=-MARGIN_PX, width=width)
        with self._lock:
            state, charts = self._state, self._charts
        for key, plot in entry["plots"].items():
            visible = key in shown
            if visible:
                plot.height = heights[key]
                plot.axes = _axes(
                    key, state.marks, state.labels, charts["duration"], width, key == shown[0], key == shown[-1],
                    self._limits, charts[key][1], PLOT_AREA_PX[len(shown)], entry["joints"],
                )
            if plot.visible != visible:
                plot.visible = visible

    # ---------------------------------------------------------------- data
    def set_plan(
        self, plan: Motion | None, waypoint_times: list[float], title: str, first_waypoint: int = 1,
        labels: list[str] | None = None,
    ) -> None:
        """The plan waiting for Execute (None: nothing planned); a measured
        motion of an earlier execution is dropped. labels: the waypoints'
        numbers (default first_waypoint, first_waypoint + 1, ...)."""
        if plan is None:
            self._publish(_State())
            return
        end = float(plan.times[-1]) + 1e-6
        labels = labels if labels is not None else [str(first_waypoint + k) for k in range(len(waypoint_times))]
        numbered = [(float(t), label) for t, label in zip(waypoint_times, labels) if 0.0 <= t <= end]
        self._publish(_State(plan, None, [t for t, _ in numbered], [n for _, n in numbered], title))

    def set_measured(self, measured: Motion, note: str) -> None:
        """The executed motion of the current plan (time 0 = Execute)."""
        with self._lock:
            state = self._state
        if state.plan is None:
            return
        self._publish(_State(state.plan, measured, state.marks, state.labels, f"{state.title} · {note}"))

    def clear(self) -> None:
        self._publish(_State())

    def _publish(self, state: _State) -> None:
        charts = _chart_data(state, self._limits)
        with self._lock:
            self._state, self._charts = state, charts
            entries = list(self._clients.values())
        for entry in entries:
            try:
                entry["title"].content = f"*{state.title}*"
                for key, plot in entry["plots"].items():
                    plot.scales = _scales(charts, key)
                    plot.data = charts[key][0]
                self._layout(entry)
            except Exception:  # noqa: BLE001 - a client that just left
                pass
