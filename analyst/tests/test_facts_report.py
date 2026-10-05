import csv

import pandas as pd

from analyst.config import mission_dir
from analyst.facts import compute_facts, match_truth
from analyst.llm import MockProvider
from analyst.report import MissionReport, generate_report, unsupported_numbers


def _rows(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def test_facts_match_hand_checked_csv_values(mission):
    f = compute_facts(mission)
    d = mission_dir(mission)
    gh = _rows(d / "ground_hazards.csv")
    assert f["n_unique_hazards"] == len({r["hazard_id"] for r in gh}) == 11
    assert f["hazards_per_drone"] == {"0": 5, "1": 3, "2": 3}
    assert f["drones"]["1"]["track_samples"] == len(_rows(next(d.glob("survey_track_d1_*.csv")))) == 6536
    assert f["truth"]["targets_truth"] == len(_rows(d / "targets.csv")) == 11
    assert f["n_drones"] == 3 and f["area_m2"] == 30.0 * 90.0


def test_facts_from_logs(mission):
    f = compute_facts(mission)
    assert f["lane_spacing"]["lane_spacing_m"] == 3.99            # survey_node_*.log
    assert [v["took_over"] for v in f["drones"].values()] == ["none"] * 3
    assert f["takeover_events"] == []
    assert f["landing_order"] == [0, 1, 2]                        # ground_station.log LANDED lines
    assert f["suppressed_duplicates"][0]["hazard_id"] == "d1-1"
    assert f["suppressed_duplicates"][0]["seen_by_drone"] == 0


def test_geotag_error_matches_hand_computation(mission):
    t = compute_facts(mission)["truth"]
    # sw4 (16, 12) vs final map #7 (16.02, 11.91): hypot(0.02, 0.09) = 0.092
    assert next(m for m in t["matches"] if m["target"] == "sw4")["error_m"] == 0.09
    assert t["geotag_error_m"] == {"min": 0.09, "max": 0.29, "mean": 0.18}
    assert t["targets_found"] == 11 and t["false_positives"] == 0 and t["duplicates"] == 0


def test_match_truth_classifies_duplicates_and_false_positives():
    truth = pd.DataFrame({"name": ["a", "b", "c"], "north": [0.0, 10.0, 50.0], "east": [0.0, 0.0, 0.0]})
    hz = [{"id": "h1", "drone": 0, "north": 0.2, "east": 0.0},
          {"id": "h2", "drone": 1, "north": 0.9, "east": 0.0},     # second flag on a -> duplicate
          {"id": "h3", "drone": 0, "north": 10.0, "east": 0.3},
          {"id": "h4", "drone": 2, "north": 30.0, "east": 0.0}]    # nothing near -> false positive
    m = match_truth(hz, truth, 5.0)
    assert m["targets_found"] == 2 and m["missed_targets"] == ["c"]
    assert m["duplicate_hazards"] == ["h2"] and m["false_positive_hazards"] == ["h4"]


def test_report_validates_and_numbers_equal_facts(mission, records):
    f = compute_facts(mission)
    reply = {"summary": "Three drones flew; 11 of 11 targets found.", "anomalies": [],
             "citations": ["survey_node_0:L72"]}
    rep, meta = generate_report(mission, MockProvider([reply]), records)
    assert isinstance(rep, MissionReport) and meta["narrative_valid"]
    MissionReport.model_validate(rep.model_dump())
    assert rep.n_drones == f["n_drones"]
    assert rep.targets_found == f["truth"]["targets_found"]
    assert rep.false_positives == f["truth"]["false_positives"]
    assert rep.geotag_error_m.model_dump() == f["truth"]["geotag_error_m"]
    assert rep.area_covered_m2 == f["area_covered_m2"]
    assert rep.takeover_events == []


def test_report_flags_invented_numbers(mission, records):
    bad = {"summary": "Mean geotag error was 0.42 m.", "anomalies": [], "citations": []}
    rep, meta = generate_report(mission, MockProvider([bad, bad]), records)
    assert not meta["narrative_valid"] and meta["unsupported_numbers"] == ["0.42"]
    assert meta["attempts"] == 2
    assert rep.geotag_error_m.mean == 0.18           # the number in the report is still the fact


def test_unsupported_numbers_ignores_ids():
    assert unsupported_numbers("hazard d1-1 at T+22.1s, 11 found", ["T+22.1s", "11"]) == []
