"""Eval questions built FROM facts.py. Question wording is a template; every
expected answer is read from the computed facts, never typed by hand.
Unanswerable questions are only kept if no record mentions the quantity."""
from __future__ import annotations

import re
from dataclasses import asdict, dataclass, field

from .records import Record


@dataclass
class EvalQuestion:
    id: str
    type: str                       # numeric | entity | explanatory | unanswerable
    question: str
    expected: str | float | None
    tolerance: float = 0.0
    accept: list[str] = field(default_factory=list)    # regexes; any match = correct (entity)
    reject: list[str] = field(default_factory=list)    # regexes; any match = wrong (entity)
    support: list[str] = field(default_factory=list)   # regexes a cited record should contain
    facts: list[str] = field(default_factory=list)     # ground truth shown to the judge
    source: str = ""                                    # the fact it was built from

    def to_dict(self) -> dict:
        return asdict(self)


def drone_re(d) -> str:
    return rf"\bdrone[\s_#-]*{int(d)}\b"


def num_re(x) -> str:
    s = f"{x:g}" if isinstance(x, float) else str(x)
    return rf"(?<![\d.]){re.escape(s)}(?![\d])"


UNANSWERABLE = [
    ("u_battery_voltage", "What was the battery voltage of drone 1 at landing?", r"volt"),
    ("u_wind", "What was the wind speed during the mission?", r"\bwind\b"),
    ("u_gps_sats", "How many GPS satellites did drone 2 have during the survey?", r"satellite"),
    ("u_cpu_temp", "What was the CPU temperature of drone 0's onboard computer?", r"temperat"),
]


def build_eval_set(facts: dict, records: list[Record]) -> list[EvalQuestion]:
    qs: list[EvalQuestion] = []
    drones = facts.get("drones", {})
    truth = facts.get("truth")

    # ---- numeric ---------------------------------------------------------
    if facts.get("n_map_hazards") is not None:
        n = facts["n_map_hazards"]
        qs.append(EvalQuestion("n_map_hazards", "numeric",
                               "How many unique hazards are in the final hazard map?",
                               n, support=[num_re(n)], source="n_map_hazards"))
    if drones:
        d, v = max(drones.items(), key=lambda kv: (kv[1].get("n_own_hazards", 0), -int(kv[0])))
        qs.append(EvalQuestion("n_own_hazards", "numeric",
                               f"How many hazards did drone {d} detect itself (its own detections)?",
                               v["n_own_hazards"], support=[num_re(v["n_own_hazards"])],
                               source=f"drones.{d}.n_own_hazards"))
    if truth:
        qs.append(EvalQuestion("n_truth", "numeric",
                               "How many ground-truth targets were placed for this mission?",
                               truth["targets_truth"], support=[num_re(truth["targets_truth"])],
                               source="truth.targets_truth"))
    tracked = [k for k, v in drones.items() if "track_samples" in v]
    if tracked:
        d = "1" if "1" in tracked else tracked[0]
        s = drones[d]["track_samples"]
        qs.append(EvalQuestion("n_track_samples", "numeric",
                               f"How many samples does drone {d}'s flown survey track contain?",
                               s, support=[num_re(s)], source=f"drones.{d}.track_samples"))
    if facts.get("lane_spacing"):
        ls = facts["lane_spacing"]["lane_spacing_m"]
        qs.append(EvalQuestion("n_lane_spacing", "numeric",
                               "What lane spacing did the survey use, in metres?",
                               ls, tolerance=0.005, support=[num_re(ls)], source="lane_spacing.lane_spacing_m"))

    # ---- entity / event --------------------------------------------------
    if facts.get("first_hazard"):
        fh = facts["first_hazard"]
        qs.append(EvalQuestion("e_first_hazard", "entity",
                               "Which drone detected the first hazard of the mission?",
                               f"drone {fh['drone']}", accept=[drone_re(fh["drone"])],
                               support=[re.escape(fh["id"])], source="first_hazard"))
    if facts.get("landing_order"):
        last = facts["landing_order"][-1]
        qs.append(EvalQuestion("e_landed_last", "entity", "Which drone landed last?",
                               f"drone {last}", accept=[drone_re(last)],
                               support=[rf"drone {last}: LANDED"], source="landing_order"))
    events = facts.get("takeover_events", [])
    if events:
        e = events[0]
        qs.append(EvalQuestion("e_takeover", "entity",
                               "Which drone took over another drone's lanes during this mission?",
                               f"drone {e['takeover_drone']}", accept=[drone_re(e["takeover_drone"])],
                               support=[r"took over"], source="takeover_events"))
    elif any("took_over" in v for v in drones.values()):
        qs.append(EvalQuestion("e_takeover", "entity",
                               "Which drone took over another drone's lanes during this mission?",
                               "none",
                               accept=[r"\bnone\b", r"\bno drone\b", r"\bno (lane )?takeovers?\b",
                                       r"\bdid not\b", r"\bdidn't\b", r"\bnot take over\b", r"\bno lanes\b",
                                       r"\bno other drone\b", r"\bneither\b"],
                               reject=[r"\bdrone[\s_]*\d\s+took over\b(?! *: *none)"],
                               support=[r"took over: none"], source="drones.*.took_over"))
    if facts.get("suppressed_duplicates"):
        sd = facts["suppressed_duplicates"][0]
        qs.append(EvalQuestion("e_dedup", "entity",
                               f"Which drone saw the person already logged as hazard {sd['hazard_id']} "
                               f"but did not log it again?",
                               f"drone {sd['seen_by_drone']}", accept=[drone_re(sd["seen_by_drone"])],
                               support=[r"already logged"], source="suppressed_duplicates"))

    # ---- explanatory (LLM judge) -----------------------------------------
    if not events and drones:
        fl = [f"takeover was enabled on every drone: "
              f"{all(v.get('takeover_enabled') for v in drones.values())}"]
        for d, v in drones.items():
            fl.append(f"drone {d}: VERIFY {v.get('verify')}, own lanes {v.get('own_lanes')}, took over: "
                      f"{v.get('took_over')}, returned={v.get('returned')}, why={v.get('why')}, "
                      f"RETURNING at T+{v.get('states', {}).get('RETURNING')}s after finishing its lanes")
        fl.append("no drone returned early or left lanes unflown, so no band was orphaned and none "
                  "needed taking over")
        qs.append(EvalQuestion("x_no_takeover", "explanatory",
                               "Why was there no lane takeover in this mission?", None,
                               support=[r"took over: none", r"8/8", r"own lanes"], facts=fl,
                               source="drones.*.{verify,own_lanes,took_over,takeover_enabled}"))
    if drones and facts.get("area"):
        a = facts["area"]
        fl = [f"survey area north {a['x_min']}-{a['x_max']} m, east {a['y_min']}-{a['y_max']} m, "
              f"altitude {facts.get('altitude_m')} m, {facts.get('n_drones')} drones"]
        fl += [f"drone {d} flew band {v.get('band')} = shared east {v.get('band_east')}, "
               f"{v.get('lanes_total')} lanes" for d, v in drones.items() if "band" in v]
        qs.append(EvalQuestion("x_area_split", "explanatory",
                               "How was the survey area divided among the drones?", None,
                               support=[r"shared east", r"band \d"], facts=fl, source="area, drones.*.band"))
    if facts.get("lane_spacing"):
        ls = facts["lane_spacing"]
        fl = [f"detection swath {ls['detection_swath_m']} m (camera footprint {ls['camera_footprint_m']} m) "
              f"at {ls['altitude_m']} m altitude", f"{ls['sidelap_pct']}% sidelap",
              f"lane spacing = {ls['lane_spacing_m']} m (the swath reduced by the sidelap)"]
        qs.append(EvalQuestion("x_lane_spacing", "explanatory",
                               "How was the survey's lane spacing derived?", None,
                               support=[r"detection swath"], facts=fl, source="lane_spacing"))

    # ---- unanswerable ----------------------------------------------------
    corpus = "\n".join(r.text for r in records).lower()
    for qid, question, absent in UNANSWERABLE:
        if re.search(absent, corpus):
            continue                       # the data does mention it: not unanswerable here
        qs.append(EvalQuestion(qid, "unanswerable", question, None,
                               source=f"no record matches /{absent}/"))
    return qs
