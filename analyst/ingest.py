"""Load a mission folder into Records.

One Record per meaningful unit: a hazard CSV row, a log line or status block,
a lane of a flown track, or a per-file summary. 50 Hz tracks and per-frame
sightings are summarised (per lane / per hazard) so a mission stays at a few
hundred records. Every number in a Record's text is read from a data file.
"""
from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from .config import mission_dir
from .records import Record

# --------------------------------------------------------------------------
# Log parsing
# --------------------------------------------------------------------------

# survey/detector/ground_station:  [INFO] [1790944349.38] [survey_node_0]: msg
ROS_RE = re.compile(r"^\[(?P<level>\w+)\] \[(?P<ts>\d+\.\d+)\] \[(?P<node>[^\]]+)\]: (?P<msg>.*)$")
# launch.log:                      1790944340.71 [INFO] [launch]: msg
LAUNCH_RE = re.compile(r"^(?P<ts>\d+\.\d+) \[(?P<level>\w+)\] \[(?P<node>[^\]]+)\]: (?P<msg>.*)$")
HAZARD_ID_RE = re.compile(r"\bd\d+-\d+\b")


@dataclass
class LogLine:
    file: str            # relative to the mission folder
    lineno: int          # 1-based line of the entry's first line
    ts: float            # epoch seconds
    level: str
    node: str
    msg: str
    extra: list[str] = field(default_factory=list)   # continuation lines


def parse_log(path: Path, rel: str) -> list[LogLine]:
    out: list[LogLine] = []
    for i, raw in enumerate(path.read_text(errors="replace").splitlines(), 1):
        m = ROS_RE.match(raw) or LAUNCH_RE.match(raw)
        if m:
            out.append(LogLine(rel, i, float(m["ts"]), m["level"], m["node"], m["msg"]))
        elif out and raw.strip():
            out[-1].extra.append(raw.rstrip())
    return out


# First match wins, so specific patterns come before generic ones
# (e.g. the config line "takeover=on" must not count as a takeover event).
EVENT_PATTERNS: list[tuple[str, re.Pattern]] = [(n, re.compile(p)) for n, p in [
    ("verify", r"^VERIFY (PASS|FAIL|PARTIAL)"),
    ("waypoint_check", r"^\s*wp\d+ \("),
    ("waypoint_reached", r"^reached wp \d+/\d+"),
    ("hazard_new", r"^HAZARD d\d+-\d+"),
    ("hazard_reported", r"^hazard d\d+-\d+ from drone"),
    ("peer_hazard", r"^peer hazard"),
    ("duplicate_suppressed", r"already logged by drone"),
    ("detector_stats", r"\[OPEN\] camera"),
    ("gate", r"detection gate"),
    ("lane_done", r"^own lane \d+ done"),
    ("config", r"lane_spacing derived|^SWARM drone|^Survey ns|^detector up|class filter"
               r"|require_gate|inference device|loading YOLO|^torch |model warm-up"
               r"|shared hazard list|ground station up"),
    ("track_written", r"track CSV written"),
    ("state", r"^drone \d+: [A-Z_]+$"),
    ("swarm_status", r"^swarm status"),
    ("peer", r"^peer \d+ "),
    ("rtl", r"^RTL|returned home|[Ff]ailsafe|survey pattern complete|own band complete"),
    ("takeover", r"(?i)take ?over|took over|claim|orphan|silent|abort|never heard"),
    ("separation", r"(?i)separation|yield"),
    ("process", r"process (started|has finished)|sending signal|user interrupted"
                r"|log files can be found|logging verbosity"),
]]


def classify(msg: str) -> str:
    for name, pat in EVENT_PATTERNS:
        if pat.search(msg):
            return name
    return "other"


def node_drone(node: str, msg: str) -> int | None:
    m = re.search(r"_(\d+)$", node)      # survey_node_0, detector_node_2
    if m:
        return int(m.group(1))
    m = re.search(r"\bdrone (\d+)\b", msg)
    return int(m.group(1)) if m else None


def mission_logs(mdir: Path) -> dict[str, list[LogLine]]:
    logs = {}
    for p in sorted(mdir.rglob("*.log")):
        rel = str(p.relative_to(mdir))
        logs[rel] = parse_log(p, rel)
    return logs


def mission_t0(logs: dict[str, list[LogLine]]) -> float | None:
    """T0 = first line of launch.log (the launch start); else earliest log line."""
    for rel, lines in logs.items():
        if Path(rel).name == "launch.log" and lines:
            return lines[0].ts
    stamps = [ln.ts for lines in logs.values() for ln in lines]
    return min(stamps) if stamps else None


# Status table row:  "  0   SURVEY       0/3    3/8    30.3   13.1  10.0    50%  0.2s"
STATUS_ROW_RE = re.compile(
    r"^\s*(?P<drone>\d+)\s+(?P<state>[A-Z_]+)\s+(?P<band>-?\d+)/(?P<lane>-?\d+)\s+"
    r"(?P<done>\d+)/(?P<total>\d+)\s+(?P<n>-?[\d.]+)\s+(?P<e>-?[\d.]+)\s+(?P<alt>-?[\d.]+)\s+"
    r"(?P<batt>\d+)%\s+(?P<age>[\d.]+)s")


def parse_status_rows(extra: list[str]) -> list[dict]:
    rows = []
    for raw in extra:
        m = STATUS_ROW_RE.match(raw)
        if m:
            d = m.groupdict()
            rows.append({
                "drone": int(d["drone"]), "state": d["state"],
                "band": int(d["band"]), "lane": int(d["lane"]),
                "lanes_done": int(d["done"]), "lanes_total": int(d["total"]),
                "north": float(d["n"]), "east": float(d["e"]), "alt": float(d["alt"]),
                "batt_pct": int(d["batt"]), "age_s": float(d["age"]),
            })
    return rows


# --------------------------------------------------------------------------
# Survey-log facts the track segmentation needs
# --------------------------------------------------------------------------

WP_REACHED_RE = re.compile(r"^reached wp (\d+)/(\d+) \((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\)")
WP_CHECK_RE = re.compile(r"^\s*wp(\d+) \((-?[\d.]+),(-?[\d.]+),(-?[\d.]+)\) closest=([\d.]+)m (\w+)")
BAND_RE = re.compile(r"band (\d+) = shared east \[(-?[\d.]+), (-?[\d.]+)\], (\d+) lanes")
LOCAL_Y_RE = re.compile(r"y\[(-?[\d.]+),(-?[\d.]+)\]")


@dataclass
class SurveyLog:
    drone: int
    waypoints: list[tuple[float, float, float]]          # local NED, in order
    reached_ts: dict[int, float]                         # wp number (1-based) -> epoch
    band: int | None = None
    band_east: tuple[float, float] | None = None
    east_offset: float = 0.0                             # shared east - local east


def parse_survey_log(lines: list[LogLine], drone: int) -> SurveyLog:
    wps: dict[int, tuple[float, float, float]] = {}
    reached: dict[int, float] = {}
    sl = SurveyLog(drone, [], {})
    local_y_min = None
    for ln in lines:
        if m := WP_REACHED_RE.match(ln.msg):
            k = int(m.group(1))
            wps[k] = (float(m.group(3)), float(m.group(4)), float(m.group(5)))
            reached[k] = ln.ts
        elif m := WP_CHECK_RE.match(ln.msg):
            wps.setdefault(int(m.group(1)), (float(m.group(2)), float(m.group(3)), float(m.group(4))))
        if m := BAND_RE.search(ln.msg):
            sl.band = int(m.group(1))
            sl.band_east = (float(m.group(2)), float(m.group(3)))
        if ln.msg.startswith("Survey ns") and (m := LOCAL_Y_RE.search(ln.msg)):
            local_y_min = float(m.group(1))
    sl.waypoints = [wps[k] for k in sorted(wps)]
    sl.reached_ts = reached
    if sl.band_east is not None and local_y_min is not None:
        sl.east_offset = sl.band_east[0] - local_y_min
    return sl


# --------------------------------------------------------------------------
# Track segmentation (50 Hz samples -> one record per lane)
# --------------------------------------------------------------------------

def locate_waypoints(track: pd.DataFrame, wps, radius: float = 1.6) -> list[int]:
    """Index of the first sample within `radius` of each waypoint, in order
    (falls back to the closest sample after the previous waypoint)."""
    xyz = track[["x_ned", "y_ned", "z_ned"]].to_numpy()
    idx, start = [], 0
    for wp in wps:
        d = np.linalg.norm(xyz[start:] - np.asarray(wp), axis=1)
        if len(d) == 0:
            idx.append(len(xyz) - 1)
            continue
        hits = np.nonzero(d < radius)[0]
        j = start + int(hits[0] if len(hits) else np.argmin(d))
        idx.append(j)
        start = j
    return idx


def horizontal_distance(track: pd.DataFrame, i: int, j: int) -> float:
    seg = track.iloc[i:j + 1]
    return float(np.hypot(np.diff(seg.x_ned), np.diff(seg.y_ned)).sum())


# --------------------------------------------------------------------------
# Loaders: one function per file kind, each returns list[Record]
# --------------------------------------------------------------------------

def _rel_t(ts: float | None, t0: float | None) -> float | None:
    if ts is None or t0 is None or (isinstance(ts, float) and math.isnan(ts)):
        return None
    return round(float(ts) - t0, 2)


def _tstr(t: float | None) -> str:
    return f"T+{t:.1f}s" if t is not None else "T=?"


def _drone_of(path: Path) -> int | None:
    m = re.search(r"_d(\d+)", path.name)
    return int(m.group(1)) if m else None


STATUS_TEXT = {
    "own": "first report (own detection)",
    "peer": "first report",
    "update": "refinement",
    "peer_update": "refinement",
}


def load_run_json(path: Path, mid: str, t0) -> list[Record]:
    cfg = json.loads(path.read_text())
    text = ("mission config (run.json): "
            + ", ".join(f"{k}={v}" for k, v in cfg.items()))
    return [Record("run:config", path.name, "summary", mid, None, None, text, dict(cfg))]


def load_ground_hazards(path: Path, mid: str, t0) -> list[Record]:
    df = pd.read_csv(path)
    recs = []
    for i, r in enumerate(df.to_dict("records"), 1):
        kind = STATUS_TEXT.get(r["status"], r["status"])
        if kind == "refinement":
            kind = f"refinement v{r['version']}"
        t = _rel_t(r["t_s"], t0)
        text = (f"ground station hazard log: {r['hazard_id']} {kind} from drone {r['drone']}: "
                f"{r['class']} conf={r['conf']:.2f} at N={r['x_ned_north']:.2f} E={r['y_ned_east']:.2f}, "
                f"{r['n_sightings']} sightings, spread {r['spread_m']:.2f} m, first seen {_tstr(t)}")
        recs.append(Record(f"{path.stem}:r{i}", path.name, f"row {i}", mid, int(r["drone"]), t, text,
                           {"event": "hazard_log", "hazard_id": r["hazard_id"], "status": r["status"],
                            "version": int(r["version"]), "conf": float(r["conf"])}))
    return recs


def load_drone_hazard_list(path: Path, mid: str, t0) -> list[Record]:
    df = pd.read_csv(path)
    d = _drone_of(path)
    first = df.drop_duplicates("hazard_id")
    own = sorted(first[first.drone == d].hazard_id)
    peer = sorted(first[first.drone != d].hazard_id)
    text = (f"drone {d} onboard hazard list ({path.name}): {first.hazard_id.nunique()} unique "
            f"hazards in {len(df)} rows; own detections ({len(own)}): {', '.join(own) or 'none'}; "
            f"received from peers ({len(peer)}): {', '.join(peer) or 'none'}")
    return [Record(f"{path.stem}:summary", path.name, "summary", mid, d, None, text,
                   {"event": "hazard_list", "n_unique": int(first.hazard_id.nunique()),
                    "n_own": len(own), "n_peer": len(peer)})]


def load_geojson(path: Path, mid: str, t0) -> list[Record]:
    g = json.loads(path.read_text())
    feats = g.get("features", [])
    recs = []
    by_drone: dict[str, int] = {}
    for f in feats:
        p, (lon, lat) = f["properties"], f["geometry"]["coordinates"][:2]
        by_drone[str(p.get("drone"))] = by_drone.get(str(p.get("drone")), 0) + 1
        text = (f"final hazard map entry #{p['id']}: {p.get('class')} conf={p.get('confidence')} "
                f"at N={p.get('north_m')} E={p.get('east_m')} (lat {lat:.6f}, lon {lon:.6f}), "
                f"reported by drone {p.get('drone')}, detected at {p.get('detected_at_alt_m')} m altitude")
        recs.append(Record(f"map:{p['id']}", path.name, f"feature {p['id']}", mid,
                           int(p["drone"]) if str(p.get("drone", "")).isdigit() else None, None, text,
                           {"event": "final_map", **p}))
    per = ", ".join(f"drone {k}: {v}" for k, v in sorted(by_drone.items()))
    props = g.get("properties", {})
    text = (f"final hazard map ({path.name}): {len(feats)} hazards in total; by reporting drone: {per}; "
            f"frame: {props.get('frame', '?')}, home lat {props.get('home_lat')}, lon {props.get('home_lon')}")
    recs.insert(0, Record("map:summary", path.name, "summary", mid, None, None, text,
                          {"event": "final_map_summary", "n_features": len(feats)}))
    return recs


def load_targets(path: Path, mid: str, t0) -> list[Record]:
    df = pd.read_csv(path)
    recs = [Record(f"targets:{r.name}", path.name, f"row {i}", mid, None, None,
                   f"ground-truth target {r.name} placed at N={r.north} E={r.east}",
                   {"event": "truth", "name": r.name})
            for i, r in enumerate(df.itertuples(index=False), 1)]
    text = (f"ground-truth targets ({path.name}): {len(df)} targets placed: "
            + ", ".join(f"{r.name} (N={r.north}, E={r.east})" for r in df.itertuples(index=False)))
    recs.insert(0, Record("targets:summary", path.name, "summary", mid, None, None, text,
                          {"event": "truth_summary", "n_targets": len(df)}))
    return recs


def load_sightings(path: Path, mid: str, t0) -> list[Record]:
    df = pd.read_csv(path)
    d = _drone_of(path)
    recs = []
    for hid, g in df[df.hazard_id.notna()].groupby("hazard_id"):
        outs = ", ".join(f"{k} {v}" for k, v in g.outcome.value_counts().items())
        t1, t2 = _rel_t(g.t_rx.min(), t0), _rel_t(g.t_rx.max(), t0)
        text = (f"drone {d} camera sightings of hazard {hid} ({path.name}): {len(g)} person boxes "
                f"({outs}), conf {g.conf.min():.2f}-{g.conf.max():.2f}, {_tstr(t1)} to {_tstr(t2)}")
        recs.append(Record(f"{path.stem}:{hid}", path.name, f"hazard {hid}", mid, d, t1, text,
                           {"event": "sightings", "hazard_id": hid, "n": len(g)}))
    for outcome, g in df[df.hazard_id.isna()].groupby("outcome"):
        label = "below-threshold" if outcome == "below" else f"'{outcome}'"
        t1, t2 = _rel_t(g.t_rx.min(), t0), _rel_t(g.t_rx.max(), t0)
        text = (f"drone {d} {label} person boxes ({path.name}): {len(g)} boxes not recorded as "
                f"hazards, conf {g.conf.min():.2f}-{g.conf.max():.2f}, {_tstr(t1)} to {_tstr(t2)}")
        recs.append(Record(f"{path.stem}:{outcome}", path.name, f"outcome {outcome}", mid, d, t1, text,
                           {"event": "sightings_unrecorded", "outcome": outcome, "n": len(g)}))
    return recs


def load_track(path: Path, mid: str, t0, survey: SurveyLog | None) -> list[Record]:
    df = pd.read_csv(path)
    d = _drone_of(path)
    off = survey.east_offset if survey else 0.0
    dur = float(df.t_s.iloc[-1] - df.t_s.iloc[0])
    dist = horizontal_distance(df, 0, len(df) - 1)
    stem = f"track_d{d}"
    recs = [Record(f"{stem}:summary", path.name, "summary", mid, d, None,
                   f"drone {d} flown track ({path.name}): {len(df)} samples over {dur:.1f} s "
                   f"(~{(len(df) - 1) / dur:.0f} Hz), {dist:.1f} m flown horizontally, max altitude "
                   f"{-df.z_ned.min():.1f} m, local north {df.x_ned.min():.1f} to {df.x_ned.max():.1f} m, "
                   f"local east {df.y_ned.min():.1f} to {df.y_ned.max():.1f} m (shared east = local + {off:.1f} m)",
                   {"event": "track_summary", "samples": len(df), "duration_s": round(dur, 2),
                    "distance_m": round(dist, 1)})]
    if not survey or len(survey.waypoints) < 2:
        return recs
    idx = locate_waypoints(df, survey.waypoints)
    lane = 0
    for k in range(len(idx) - 1):
        (x1, y1, _), (x2, y2, _) = survey.waypoints[k], survey.waypoints[k + 1]
        if abs(y2 - y1) > 0.5 or abs(x2 - x1) < 10:      # not a lane: transit / lane change
            continue
        i, j = idx[k], idx[k + 1]
        seg = df.iloc[i:j + 1]
        ldist = horizontal_distance(df, i, j)
        ldur = float(seg.t_s.iloc[-1] - seg.t_s.iloc[0])
        ts1, ts2 = survey.reached_ts.get(k + 1), survey.reached_ts.get(k + 2)
        t1, t2 = _rel_t(ts1, t0), _rel_t(ts2, t0)
        when = f", {_tstr(t1)} to {_tstr(t2)}" if t1 is not None and t2 is not None else ""
        band = f"band {survey.band}, " if survey.band is not None else ""
        heading = "northbound" if x2 > x1 else "southbound"
        text = (f"drone {d} lane {lane} ({band}shared east {y1 + off:.1f} m): flew {heading} from "
                f"N={x1:.1f} to N={x2:.1f} m, {ldist:.1f} m in {ldur:.1f} s (mean {ldist / ldur:.2f} m/s, "
                f"mean altitude {-seg.z_ned.mean():.1f} m){when}")
        recs.append(Record(f"{stem}:lane{lane}", path.name, f"lane {lane}", mid, d, t1, text,
                           {"event": "lane", "lane": lane, "distance_m": round(ldist, 1),
                            "duration_s": round(ldur, 2)}))
        lane += 1
    # Return leg: last waypoint to the end of the track.
    i = idx[-1]
    seg = df.iloc[i:]
    end_d = math.hypot(df.x_ned.iloc[-1], df.y_ned.iloc[-1])
    recs.append(Record(f"{stem}:return", path.name, "return leg", mid, d, None,
                       f"drone {d} return leg (after last waypoint): {horizontal_distance(df, i, len(df) - 1):.1f} m "
                       f"in {float(seg.t_s.iloc[-1] - seg.t_s.iloc[0]):.1f} s, peak altitude "
                       f"{-seg.z_ned.min():.1f} m, last sample {end_d:.2f} m horizontally from its home",
                       {"event": "return_leg"}))
    return recs


LOG_DROP = {"gate", "waypoint_check", "waypoint_reached"}   # summarised per file instead


def load_log(rel: str, lines: list[LogLine], mid: str, t0) -> list[Record]:
    stem = Path(rel).stem
    recs: list[Record] = []
    gates: list[LogLine] = []
    checks: list[re.Match] = []
    for ln in lines:
        ev = classify(ln.msg)
        t = _rel_t(ln.ts, t0)
        drone = node_drone(ln.node, ln.msg)
        if ev == "gate":
            gates.append(ln)
            continue
        if ev == "waypoint_check":
            if m := WP_CHECK_RE.match(ln.msg):
                checks.append(m)
            continue
        if ev in LOG_DROP:
            continue
        fields = {"event": ev, "node": ln.node, "level": ln.level}
        hids = HAZARD_ID_RE.findall(ln.msg)
        if hids:
            fields["hazard_id"] = hids[0]
        if ev == "verify":
            m = re.search(r"took over: ([^|]+)", ln.msg)
            fields["took_over"] = m.group(1).strip() if m else None
            fields["topic"] = "takeover status"
        if ev == "duplicate_suppressed":
            fields["topic"] = "duplicate"
        if ev == "swarm_status":
            rows = parse_status_rows(ln.extra)
            fields["rows"] = rows
            parts = [f"drone {r['drone']} {r['state']} band/lane {r['band']}/{r['lane']} lanes "
                     f"{r['lanes_done']}/{r['lanes_total']} N={r['north']} E={r['east']} alt={r['alt']}m "
                     f"batt={r['batt_pct']}% age={r['age_s']}s" for r in rows]
            text = f"{ln.node} swarm status {_tstr(t)}: " + "; ".join(parts)
        else:
            text = f"{ln.node} {_tstr(t)}: {ln.msg}"
        recs.append(Record(f"{stem}:L{ln.lineno}", rel, f"line {ln.lineno}", mid, drone, t, text, fields))
    drone = node_drone(lines[0].node, "") if lines else None
    if gates:
        opens = [g for g in gates if "OPEN" in g.msg]
        recs.append(Record(f"{stem}:gate", rel, "summary", mid, drone, _rel_t(gates[0].ts, t0),
                           f"{lines[0].node} detection gate: opened {len(opens)} times, "
                           f"{len(gates) - len(opens)} 'closed' lines, first {_tstr(_rel_t(gates[0].ts, t0))}, "
                           f"last {_tstr(_rel_t(gates[-1].ts, t0))}",
                           {"event": "gate_summary", "n_open": len(opens)}))
    if checks:
        ok = sum(m.group(6) == "OK" for m in checks)
        cl = [float(m.group(5)) for m in checks]
        recs.append(Record(f"{stem}:wpcheck", rel, "summary", mid, drone, None,
                           f"{lines[0].node} post-flight waypoint check: {ok}/{len(checks)} waypoints OK, "
                           f"closest approach {min(cl):.2f}-{max(cl):.2f} m",
                           {"event": "waypoint_check_summary", "ok": ok, "n": len(checks)}))
    return recs


# --------------------------------------------------------------------------
# Entry point
# --------------------------------------------------------------------------

def file_kind(rel: str) -> str:
    name = Path(rel).name
    if name == "run.json":
        return "run_config"
    if name == "ground_hazards.csv":
        return "ground_hazards"
    if re.fullmatch(r"hazards_d\d+\.csv", name):
        return "drone_hazards"
    if name.endswith(".geojson"):
        return "hazard_map"
    if name in ("targets.csv", "targets_truth.csv"):
        return "truth"
    if re.fullmatch(r"sightings_d\d+\.csv", name):
        return "sightings"
    if re.fullmatch(r"survey_track_d\d+_\d+\.csv", name):
        return "track"
    if name.endswith(".log"):
        return "log"
    return "other"


def load_mission(mission: str) -> list[Record]:
    mdir = mission_dir(mission)
    if not mdir.is_dir():
        raise FileNotFoundError(mdir)
    logs = mission_logs(mdir)
    t0 = mission_t0(logs)
    surveys = {}
    for rel, lines in logs.items():
        m = re.fullmatch(r"survey_node_(\d+)\.log", Path(rel).name)
        if m:
            surveys[int(m.group(1))] = parse_survey_log(lines, int(m.group(1)))
    recs: list[Record] = []
    for p in sorted(mdir.rglob("*")):
        if not p.is_file():
            continue
        rel = str(p.relative_to(mdir))
        kind = file_kind(rel)
        if kind == "run_config":
            recs += load_run_json(p, mission, t0)
        elif kind == "ground_hazards":
            recs += load_ground_hazards(p, mission, t0)
        elif kind == "drone_hazards":
            recs += load_drone_hazard_list(p, mission, t0)
        elif kind == "hazard_map":
            recs += load_geojson(p, mission, t0)
        elif kind == "truth":
            recs += load_targets(p, mission, t0)
        elif kind == "sightings":
            recs += load_sightings(p, mission, t0)
        elif kind == "track":
            recs += load_track(p, mission, t0, surveys.get(_drone_of(p)))
        elif kind == "log":
            recs += load_log(rel, logs[rel], mission, t0)
    ids = [r.id for r in recs]
    dupes = {i for i in ids if ids.count(i) > 1}
    if dupes:
        raise ValueError(f"duplicate record ids: {sorted(dupes)}")
    return recs


# --------------------------------------------------------------------------
# Inventory
# --------------------------------------------------------------------------

SUPPORTS = {
    "run_config": "mission config, area, n_drones",
    "ground_hazards": "detections, refinement history, per-drone counts",
    "drone_hazards": "each drone's onboard list (convergence)",
    "hazard_map": "final hazard map",
    "truth": "ground truth -> found / FP / duplicates / geotag error",
    "sightings": "detection evidence, below-threshold boxes, cross-drone re-sightings",
    "track": "flown track -> per-lane distance, speed, timing",
    "log": "",
    "other": "not ingested",
}
LOG_SUPPORTS = {
    "survey_node": "lanes done, VERIFY, takeover status, RTL",
    "detector_node": "detections, cross-drone dedup, fps stats",
    "ground_station": "timeline, drone states, battery %, landing order",
    "launch": "process timeline (T0)",
}


def inventory(mission: str) -> list[dict]:
    mdir = mission_dir(mission)
    recs = load_mission(mission)
    per_file: dict[str, int] = {}
    for r in recs:
        per_file[r.source_file] = per_file.get(r.source_file, 0) + 1
    rows = []
    for p in sorted(mdir.rglob("*")):
        if not p.is_file():
            continue
        rel = str(p.relative_to(mdir))
        kind = file_kind(rel)
        cols, n = "-", None
        if p.suffix == ".csv":
            df = pd.read_csv(p)
            cols, n = ", ".join(df.columns), len(df)
        elif p.suffix == ".geojson":
            g = json.loads(p.read_text())
            n = len(g.get("features", []))
            cols = ", ".join(g["features"][0]["properties"]) if n else "-"
        elif p.suffix == ".json":
            cols, n = ", ".join(json.loads(p.read_text())), 1
        elif p.suffix == ".log":
            n = sum(1 for _ in p.open(errors="replace"))
        supports = SUPPORTS[kind]
        if kind == "log":
            supports = next((v for k, v in LOG_SUPPORTS.items() if p.stem.startswith(k)), "log lines")
        rows.append({"file": rel, "kind": kind, "rows": n, "columns": cols,
                     "records": per_file.get(rel, 0), "supports": supports})
    return rows
