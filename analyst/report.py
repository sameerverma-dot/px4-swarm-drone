"""Structured mission report.

Split on purpose: every number comes from facts.py (deterministic); the LLM
writes only `summary` and `anomalies`, from the facts plus retrieved records.
The narrative's citations must be provided record ids, and every number it
writes must appear in the facts or records (retry once, else flagged).
"""
from __future__ import annotations

import json
import re
from pathlib import Path

from pydantic import BaseModel, ValidationError

from . import config
from .facts import compute_facts
from .ingest import load_mission
from .llm import LLMError, Provider, get_provider
from .prompts import REPORT_SYSTEM, report_user
from .retrieve import Index


class GeotagError(BaseModel):
    min: float
    max: float
    mean: float


class TakeoverEvent(BaseModel):
    failed_drone: int | None
    cause: str | None
    takeover_drone: int
    lanes: str
    time: float | None


class MissionReport(BaseModel):
    mission_id: str
    n_drones: int
    area_covered_m2: float | None
    targets_found: int | None
    targets_truth: int | None
    false_positives: int | None
    duplicates: int | None
    geotag_error_m: GeotagError | None
    takeover_events: list[TakeoverEvent]
    anomalies: list[str]
    summary: str


class Narrative(BaseModel):
    summary: str
    anomalies: list[str] = []
    citations: list[str] = []


# Queries used to pull evidence for the narrative (anomalies, outcomes).
REPORT_QUERIES = [
    "takeover took over orphan silent abort failsafe",
    "duplicate already logged not logged again",
    "VERIFY PASS returned home mission complete",
    "swarm status battery batt landed",
    "hazard new map has",
    "separation yield warning error",
]

NUM_RE = re.compile(r"(?<![\w.\-])-?\d+(?:\.\d+)?")


def _norm(x: str) -> str:
    return ("%f" % float(x)).rstrip("0").rstrip(".")


def unsupported_numbers(text: str, sources: list[str]) -> list[str]:
    allowed = {_norm(n) for s in sources for n in NUM_RE.findall(s)}
    return sorted({n for n in NUM_RE.findall(text) if _norm(n) not in allowed})


def numbers_report(facts: dict) -> dict:
    """The deterministic part of MissionReport, straight from facts.py."""
    truth = facts.get("truth") or {}
    return {
        "mission_id": facts["mission_id"],
        "n_drones": facts["n_drones"],
        "area_covered_m2": facts.get("area_covered_m2"),
        "targets_found": truth.get("targets_found"),
        "targets_truth": truth.get("targets_truth"),
        "false_positives": truth.get("false_positives"),
        "duplicates": truth.get("duplicates"),
        "geotag_error_m": truth.get("geotag_error_m"),
        "takeover_events": facts["takeover_events"],
    }


def generate_report(mission: str, provider: Provider | str | None = None,
                    records=None) -> tuple[MissionReport, dict]:
    facts = compute_facts(mission)
    records = records if records is not None else load_mission(mission)
    provider = provider if isinstance(provider, Provider) else get_provider(provider)
    idx = Index(records)
    evidence, seen = [], set()
    for q in REPORT_QUERIES:
        for r, _ in idx.search(q, 4):
            if r.id not in seen:
                seen.add(r.id)
                evidence.append(r)
    lines = [r.prompt_line() for r in evidence]
    facts_for_llm = {k: v for k, v in facts.items() if k not in ("t0_epoch",)}
    sources = [json.dumps(facts_for_llm, default=str)] + lines

    meta = {"provider": provider.name, "model": provider.model, "evidence_ids": [r.id for r in evidence],
            "narrative_valid": False, "unsupported_numbers": [], "attempts": 0, "error": None,
            "latency_s": 0.0}
    narrative, feedback = None, None
    for attempt in (1, 2):
        meta["attempts"] = attempt
        try:
            raw = provider.generate_json(REPORT_SYSTEM, report_user(facts_for_llm, lines, feedback),
                                         task="report")
        except LLMError as e:
            meta["error"] = str(e)
            break
        meta["latency_s"] += raw.pop("_latency_s", 0.0)
        try:
            narrative = Narrative.model_validate(raw)
        except ValidationError as e:
            feedback = f"reply did not match the JSON schema ({e.errors()[0]['msg']})"
            continue
        bad_cites = [c for c in narrative.citations if c not in seen]
        bad_nums = unsupported_numbers(" ".join([narrative.summary, *narrative.anomalies]), sources)
        meta["unsupported_numbers"] = bad_nums
        if not bad_cites and not bad_nums:
            meta["narrative_valid"] = True
            break
        feedback = "; ".join(filter(None, [
            f"citations {bad_cites} are not provided record ids" if bad_cites else "",
            f"numbers {bad_nums} do not appear in the FACTS or records" if bad_nums else ""]))
        meta["error"] = feedback
    if meta["narrative_valid"]:
        meta["error"] = None
    nums = numbers_report(facts)
    report = MissionReport(**nums,
                           summary=narrative.summary if narrative else "(narrative unavailable)",
                           anomalies=narrative.anomalies if narrative else [])
    meta["citations"] = narrative.citations if narrative else []
    return report, meta


def _fmt(v, unit=""):
    return "n/a" if v is None else f"{v}{unit}"


def render_markdown(report: MissionReport, facts: dict, meta: dict, records_by_id: dict) -> str:
    t = facts.get("truth") or {}
    ge = report.geotag_error_m
    L = [f"# Mission report: {report.mission_id}", "",
         f"Generated by `python -m analyst report` · narrative by `{meta['provider']}` "
         f"(`{meta['model']}`) · narrative checks: "
         f"{'passed' if meta['narrative_valid'] else 'FAILED - ' + str(meta.get('error'))}", "",
         "Numbers in the tables come from `facts.py` (deterministic, computed from the data files). "
         "Only **Summary** and **Anomalies** are LLM-written.", "",
         "## Summary", "", report.summary, "",
         "## Key numbers", "",
         "| metric | value |", "|---|---|",
         f"| drones | {report.n_drones} |",
         f"| area covered | {_fmt(report.area_covered_m2, ' m²')} ({facts.get('area_covered_note', '')}) |",
         f"| targets found / truth | {_fmt(report.targets_found)} / {_fmt(report.targets_truth)} "
         f"(one-to-one match within {t.get('match_radius_m', '?')} m) |",
         f"| false positives | {_fmt(report.false_positives)} |",
         f"| duplicates in final map | {_fmt(report.duplicates)} |",
         f"| duplicates suppressed onboard | {len(facts.get('suppressed_duplicates', []))} |",
         f"| geotag error min / max / mean | "
         f"{f'{ge.min} / {ge.max} / {ge.mean} m' if ge else 'n/a (no truth)'} |",
         f"| unique hazards (ground log) / in final map | {facts.get('n_unique_hazards')} / "
         f"{facts.get('n_map_hazards')} |",
         f"| lane spacing | {_fmt((facts.get('lane_spacing') or {}).get('lane_spacing_m'), ' m')} |",
         f"| mission duration | {_fmt(facts.get('mission_duration_s'), ' s')} "
         f"({facts.get('mission_duration_note', '')}) |",
         f"| landing order | {', '.join(f'drone {d}' for d in facts.get('landing_order', []))} |", "",
         "## Per drone", "",
         "| drone | band (east) | own lanes | VERIFY | took over | own hazards | track samples | "
         "track distance | landed |", "|---|---|---|---|---|---|---|---|---|"]
    for d, v in facts["drones"].items():
        be = v.get("band_east")
        L.append(f"| {d} | {v.get('band', '?')} ({be[0]:g}-{be[1]:g} m) | {v.get('own_lanes', '?')} | "
                 f"{v.get('verify', '?')} | {v.get('took_over', '?')} | {v.get('n_own_hazards', 0)} | "
                 f"{v.get('track_samples', '?')} | {_fmt(v.get('track_distance_m'), ' m')} | "
                 f"T+{v.get('states', {}).get('LANDED', '?')}s |" if be else f"| {d} | ? |")
    L += ["", "## Takeover events", ""]
    if report.takeover_events:
        L += ["| failed drone | takeover drone | lanes | cause | time |", "|---|---|---|---|---|"]
        L += [f"| {e.failed_drone} | {e.takeover_drone} | {e.lanes} | {e.cause or '-'} | "
              f"{_fmt(e.time, ' s')} |" for e in report.takeover_events]
    else:
        L.append("None. Every drone's VERIFY line reports `took over: none`.")
    if t.get("matches"):
        L += ["", "## Truth matching", "", "| target | truth N, E | hazard | drone | error |",
              "|---|---|---|---|---|"]
        L += [f"| {m['target']} | {m['truth_north']:g}, {m['truth_east']:g} | {m['hazard']} | {m['drone']} | "
              f"{m['error_m']} m |" for m in t["matches"]]
        if t.get("missed_targets"):
            L.append(f"\nMissed: {', '.join(t['missed_targets'])}")
    L += ["", "## Anomalies (LLM)", ""]
    L += [f"- {a}" for a in report.anomalies] or ["- none reported"]
    if meta.get("citations"):
        L += ["", "## Records cited by the narrative", ""]
        for c in meta["citations"]:
            r = records_by_id.get(c)
            L.append(f"- `[{c}]` {r.text if r else '(unknown id)'}")
    if meta.get("unsupported_numbers"):
        L += ["", f"> Unsupported numbers in narrative: {meta['unsupported_numbers']}"]
    return "\n".join(L) + "\n"


def write_report(mission: str, provider: Provider | str | None = None,
                 out_dir: Path | None = None) -> tuple[Path, MissionReport, dict]:
    records = load_mission(mission)
    report, meta = generate_report(mission, provider, records)
    out_dir = Path(out_dir or config.RESULTS_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)
    md = render_markdown(report, compute_facts(mission), meta, {r.id: r for r in records})
    path = out_dir / "sample_report.md"
    path.write_text(md)
    (out_dir / "sample_report.json").write_text(json.dumps(
        {"report": report.model_dump(), "meta": meta}, indent=2, default=str))
    return path, report, meta
