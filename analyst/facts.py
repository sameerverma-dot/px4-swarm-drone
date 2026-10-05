"""Deterministic mission facts, computed from the data files with pandas and
regex log parsing. These are the ground truth for the report's numbers and
for the eval set. Nothing here calls an LLM.
"""
from __future__ import annotations

import json
import math
import re
from functools import lru_cache
from pathlib import Path

import pandas as pd

from . import config
from .ingest import (BAND_RE, classify, mission_logs, mission_t0, parse_status_rows)

LANE_SPACING_RE = re.compile(
    r"detection swath ([\d.]+) m \(camera footprint ([\d.]+) m\) at ([\d.]+) m altitude, "
    r"(\d+)% sidelap -> ([\d.]+) m")
VERIFY_RE = re.compile(
    r"^VERIFY (?P<status>\w+) \| drone (?P<drone>\d+) \| own lanes (?P<done>\d+)/(?P<total>\d+) \| "
    r"took over: (?P<took>[^|]+?) \| waypoints (?P<wp>\d+)/(?P<wpt>\d+) \| returned=(?P<ret>\w+) \| "
    r"why=(?P<why>[^|]+?) \|")
DEDUP_RE = re.compile(
    r"saw (?P<cls>\w+) at N=(?P<n>-?[\d.]+) E=(?P<e>-?[\d.]+) - already logged by drone (?P<owner>\d+) "
    r"as (?P<hid>d\d+-\d+)")
STATS_RE = re.compile(r"camera ([\d.]+) fps, inferred ([\d.]+) fps")
HOME_RE = re.compile(r"returned home \(d=([\d.]+) m\)")


def _r(x, nd=2):
    return None if x is None else round(float(x), nd)


def _tplus(ts, t0):
    return None if ts is None or t0 is None else round(ts - t0, 1)


def match_truth(hazards: list[dict], truth: pd.DataFrame, radius: float) -> dict:
    """One-to-one greedy matching, closest pairs first, within `radius`.
    Unmatched hazards within `radius` of some target are duplicates (a
    second flag on a found target); the rest are false positives."""
    pairs = sorted(
        (math.hypot(h["north"] - t.north, h["east"] - t.east), t.name, h["id"])
        for t in truth.itertuples(index=False) for h in hazards)
    used_t, used_h, matches = set(), set(), []
    for d, tname, hid in pairs:
        if d <= radius and tname not in used_t and hid not in used_h:
            used_t.add(tname)
            used_h.add(hid)
            matches.append((tname, hid, d))
    by_id = {h["id"]: h for h in hazards}
    rows = []
    for tname, hid, d in sorted(matches, key=lambda m: m[0]):
        t = truth[truth.name == tname].iloc[0]
        rows.append({"target": tname, "truth_north": float(t.north), "truth_east": float(t.east),
                     "hazard": hid, "drone": by_id[hid]["drone"], "error_m": _r(d)})
    dupes, fps = [], []
    for h in hazards:
        if h["id"] in used_h:
            continue
        near = min(math.hypot(h["north"] - t.north, h["east"] - t.east) for t in truth.itertuples())
        (dupes if near <= radius else fps).append(h["id"])
    errs = [m[2] for m in matches]
    missed = sorted(set(truth.name) - used_t)
    return {
        "match_radius_m": radius,
        "targets_truth": len(truth),
        "targets_found": len(matches),
        "missed_targets": missed,
        "false_positives": len(fps),
        "false_positive_hazards": fps,
        "duplicates": len(dupes),
        "duplicate_hazards": dupes,
        "geotag_error_m": ({"min": _r(min(errs)), "max": _r(max(errs)), "mean": _r(sum(errs) / len(errs))}
                           if errs else None),
        "matches": rows,
    }


@lru_cache(maxsize=8)
def compute_facts(mission: str) -> dict:
    mdir = config.mission_dir(mission)
    logs = mission_logs(mdir)
    t0 = mission_t0(logs)
    f: dict = {"mission_id": mission, "t0_epoch": t0,
               "time_note": "T+ times are seconds since the first launch.log line"}

    # --- config ---------------------------------------------------------
    run_p = mdir / "run.json"
    run = json.loads(run_p.read_text()) if run_p.exists() else {}
    f["n_drones"] = run.get("num_drones")
    if {"x_min", "x_max", "y_min", "y_max"} <= run.keys():
        f["area"] = {k: run[k] for k in ("x_min", "x_max", "y_min", "y_max")}
        f["area_m2"] = _r((run["x_max"] - run["x_min"]) * (run["y_max"] - run["y_min"]), 1)
    f["altitude_m"] = run.get("altitude")
    f["started"] = run.get("started")

    # --- hazards: ground station log (all reports, all refinements) -----
    gh_p = mdir / "ground_hazards.csv"
    hazards = []
    if gh_p.exists():
        gh = pd.read_csv(gh_p)
        final = gh.sort_values("version").groupby("hazard_id").tail(1).set_index("hazard_id")
        first = gh[gh.version == 0].set_index("hazard_id")
        for hid in sorted(gh.hazard_id.unique()):
            fr, fn = first.loc[hid] if hid in first.index else final.loc[hid], final.loc[hid]
            hazards.append({
                "id": hid, "drone": int(fn.drone), "class": fn["class"],
                "first_seen_t": _tplus(float(fr.t_s), t0),
                "first_conf": _r(fr.conf), "final_conf": _r(fn.conf),
                "north": _r(fn.x_ned_north), "east": _r(fn.y_ned_east),
                "versions": int(fn.version), "n_sightings": int(fn.n_sightings),
                "spread_m": _r(fn.spread_m)})
        f["n_unique_hazards"] = int(gh.hazard_id.nunique())
        f["hazards_per_drone"] = {str(k): int(v) for k, v in
                                  gh.drop_duplicates("hazard_id").groupby("drone").size().items()}
        f["classes"] = sorted(gh["class"].unique().tolist())
        first_h = min(hazards, key=lambda h: h["first_seen_t"])
        f["first_hazard"] = {"id": first_h["id"], "drone": first_h["drone"], "t": first_h["first_seen_t"]}
    f["hazards"] = hazards

    # --- final map ------------------------------------------------------
    map_p = next(iter(sorted(mdir.glob("*.geojson"))), None)
    map_hz = []
    if map_p:
        g = json.loads(map_p.read_text())
        for ft in g["features"]:
            p = ft["properties"]
            map_hz.append({"id": f"map#{p['id']}", "drone": int(p["drone"]), "conf": p["confidence"],
                           "north": float(p["north_m"]), "east": float(p["east_m"])})
        f["n_map_hazards"] = len(map_hz)
        f["map_conf_range"] = [min(h["conf"] for h in map_hz), max(h["conf"] for h in map_hz)] if map_hz else None

    # --- truth ----------------------------------------------------------
    truth_p = next((mdir / n for n in ("targets.csv", "targets_truth.csv") if (mdir / n).exists()), None)
    if truth_p is not None and map_hz:
        f["truth"] = match_truth(map_hz, pd.read_csv(truth_p), config.MATCH_RADIUS_M)
        f["truth"]["scored_against"] = map_p.name
    else:
        f["truth"] = None

    # --- per-drone survey logs -------------------------------------------
    drones: dict[str, dict] = {}
    takeover_lines = []
    for rel, lines in logs.items():
        m = re.fullmatch(r"survey_node_(\d+)\.log", Path(rel).name)
        if not m:
            continue
        d = int(m.group(1))
        info: dict = {"lanes_done_logged": 0}
        for ln in lines:
            if (mm := BAND_RE.search(ln.msg)):
                info["band"] = int(mm.group(1))
                info["band_east"] = [float(mm.group(2)), float(mm.group(3))]
                info["lanes_total"] = int(mm.group(4))
            if (mm := LANE_SPACING_RE.search(ln.msg)):
                f["lane_spacing"] = {"detection_swath_m": float(mm.group(1)),
                                     "camera_footprint_m": float(mm.group(2)),
                                     "altitude_m": float(mm.group(3)),
                                     "sidelap_pct": int(mm.group(4)),
                                     "lane_spacing_m": float(mm.group(5))}
            if ln.msg.startswith("own lane") and "done" in ln.msg:
                info["lanes_done_logged"] += 1
            if ln.msg.startswith("Offboard engaged"):
                info["offboard_t"] = _tplus(ln.ts, t0)
            if (mm := HOME_RE.search(ln.msg)):
                info["returned_home_d_m"] = float(mm.group(1))
            if (mm := VERIFY_RE.match(ln.msg)):
                info.update({"verify": mm["status"], "own_lanes": f"{mm['done']}/{mm['total']}",
                             "took_over": mm["took"].strip(), "waypoints": f"{mm['wp']}/{mm['wpt']}",
                             "returned": mm["ret"] == "True", "why": mm["why"].strip(),
                             "verify_t": _tplus(ln.ts, t0)})
            if classify(ln.msg) == "takeover":
                takeover_lines.append({"drone": d, "t": _tplus(ln.ts, t0), "msg": ln.msg})
        drones[str(d)] = info

    # --- ground station: states, battery, landing order -------------------
    gs = next((v for k, v in logs.items() if Path(k).name == "ground_station.log"), [])
    for ln in gs:
        if (mm := re.match(r"^drone (\d+): ([A-Z_]+)$", ln.msg)):
            st = drones.setdefault(mm.group(1), {}).setdefault("states", {})
            st.setdefault(mm.group(2), _tplus(ln.ts, t0))
        if ln.msg.startswith("swarm status"):
            for row in parse_status_rows(ln.extra):
                dd = drones.setdefault(str(row["drone"]), {})
                dd["batt_min_pct"] = min(dd.get("batt_min_pct", 101), row["batt_pct"])
                dd["batt_last_pct"] = row["batt_pct"]
                if row["state"] == "SURVEY":
                    dd["max_heartbeat_age_survey_s"] = max(dd.get("max_heartbeat_age_survey_s", 0.0),
                                                           row["age_s"])
    landed = sorted((v["states"]["LANDED"], int(k)) for k, v in drones.items()
                    if "LANDED" in v.get("states", {}))
    f["landing_order"] = [d for _, d in landed]
    offb = [v["offboard_t"] for v in drones.values() if "offboard_t" in v]
    if landed and offb:
        f["mission_duration_s"] = _r(landed[-1][0] - min(offb), 1)
        f["mission_duration_note"] = "first 'Offboard engaged' to last LANDED"

    # --- detector logs: dedup events, throughput ------------------------
    f["suppressed_duplicates"] = []
    for rel, lines in logs.items():
        m = re.fullmatch(r"detector_node_(\d+)\.log", Path(rel).name)
        if not m:
            continue
        d = m.group(1)
        cams, infs = [], []
        for ln in lines:
            if (mm := DEDUP_RE.search(ln.msg)):
                f["suppressed_duplicates"].append({
                    "seen_by_drone": int(d), "hazard_id": mm["hid"], "logged_by_drone": int(mm["owner"]),
                    "north": float(mm["n"]), "east": float(mm["e"]), "t": _tplus(ln.ts, t0)})
            if (mm := STATS_RE.search(ln.msg)):
                cams.append(float(mm.group(1)))
                infs.append(float(mm.group(2)))
        if cams:
            drones.setdefault(d, {})["camera_fps_range"] = [min(cams), max(cams)]
            drones[d]["inferred_fps_range"] = [min(infs), max(infs)]

    # --- hazard counts per drone, sightings, tracks ---------------------
    for h in hazards:
        dd = drones.setdefault(str(h["drone"]), {})
        dd.setdefault("own_hazards", []).append(h["id"])
    for p in sorted(mdir.glob("sightings_d*.csv")):
        d = re.search(r"_d(\d+)", p.name).group(1)
        s = pd.read_csv(p)
        drones.setdefault(d, {})["sighting_boxes"] = len(s)
        drones[d]["below_threshold_boxes"] = int((s.outcome == "below").sum())
    for p in sorted(mdir.glob("survey_track_d*_*.csv")):
        d = re.search(r"_d(\d+)_", p.name).group(1)
        t = pd.read_csv(p)
        dist = float((t[["x_ned", "y_ned"]].diff().pow(2).sum(axis=1) ** 0.5).sum())
        drones.setdefault(d, {}).update({
            "track_samples": len(t), "track_duration_s": _r(t.t_s.iloc[-1] - t.t_s.iloc[0], 1),
            "track_distance_m": _r(dist, 1), "max_altitude_m": _r(-t.z_ned.min(), 1)})
    for v in drones.values():
        v["n_own_hazards"] = len(v.get("own_hazards", []))
    f["drones"] = dict(sorted(drones.items()))

    # --- takeovers --------------------------------------------------------
    events = []
    for d, v in f["drones"].items():
        took = v.get("took_over")
        if took and took.lower() != "none":
            band = re.search(r"band (\d+)", took)
            cause = next((x["msg"] for x in takeover_lines if x["drone"] == int(d)), None)
            events.append({"failed_drone": int(band.group(1)) if band else None, "cause": cause,
                           "takeover_drone": int(d), "lanes": took, "time": v.get("verify_t")})
    f["takeover_events"] = events
    f["takeover_log_lines"] = takeover_lines

    # --- coverage ---------------------------------------------------------
    done = [v for v in f["drones"].values() if "own_lanes" in v]
    all_done = bool(done) and all(v["own_lanes"].split("/")[0] == v["own_lanes"].split("/")[1] for v in done)
    f["all_bands_complete"] = all_done and len(done) == f.get("n_drones")
    f["area_covered_m2"] = f.get("area_m2") if f["all_bands_complete"] else None
    f["area_covered_note"] = ("every drone's VERIFY reports all own lanes flown, so the whole run.json "
                              "area was covered" if f["all_bands_complete"] else
                              "not every band reports all lanes flown; coverage not derived")
    return f
