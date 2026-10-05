import re

from analyst.evalset import build_eval_set
from analyst.evaluate import (numbers_in, run_eval, score_entity, score_numeric, summarise)
from analyst.facts import compute_facts
from analyst.llm import MockProvider


def test_numeric_parse():
    assert numbers_in("drone 1's track has 6,536 samples") == [1.0, 6536.0]
    assert numbers_in("hazard d1-2 at T+22.1s") == [22.1]
    assert score_numeric("The lane spacing was 3.99 m.", 3.99, 0.005)
    assert not score_numeric("about 4 m", 3.99, 0.005)
    assert score_numeric("There are 11 unique hazards.", 11)


def test_entity_match(mission, records):
    acc = [r"\bdrone[\s_#-]*2\b"]
    assert score_entity("Drone 2 landed last.", acc)
    assert score_entity("drone_2", acc)
    assert not score_entity("Drone 1 landed last at T+140.8s.", acc)
    q = next(q for q in build_eval_set(compute_facts(mission), records) if q.id == "e_takeover")
    assert score_entity("No drone took over lanes; all show 'took over: none'.", q.accept, q.reject)
    assert score_entity("None of the drones did.", q.accept, q.reject)
    assert not score_entity("Drone 1 took over drone 2's lanes.", q.accept, q.reject)


def test_eval_set_built_from_facts(mission, records):
    f = compute_facts(mission)
    qs = build_eval_set(f, records)
    assert 12 <= len(qs) <= 16
    types = {q.type for q in qs}
    assert types == {"numeric", "entity", "explanatory", "unanswerable"}
    assert 3 <= sum(q.type == "unanswerable" for q in qs) <= 4
    by = {q.id: q for q in qs}
    assert by["n_map_hazards"].expected == f["n_map_hazards"]
    assert by["n_track_samples"].expected == f["drones"]["1"]["track_samples"]
    assert by["e_landed_last"].expected == f"drone {f['landing_order'][-1]}"
    # unanswerable questions really are absent from the corpus
    corpus = " ".join(r.text for r in records).lower()
    assert "volt" not in corpus and "satellite" not in corpus


def _oracle(facts):
    """A scripted 'LLM' that answers every eval question correctly from facts."""
    def responder(system, user, task):
        if task == "judge":
            return {"score": 5, "reason": "matches facts"}
        ids = re.findall(r"^\[([^\]]+)\]", user, re.M)
        q = user.rsplit("Question: ", 1)[-1].split("\n")[0]
        if any(w in q for w in ("voltage", "wind", "satellites", "temperature")):
            return {"answer": "Not in the records.", "citations": [], "answerable": False}
        answers = {
            "unique hazards": f"{facts['n_map_hazards']} hazards.",
            "detect itself": "5 own detections.",
            "ground-truth targets": "11 targets.",
            "samples": f"{facts['drones']['1']['track_samples']} samples.",
            "lane spacing did": "3.99 m.",
            "first hazard": "Drone 0.",
            "landed last": "Drone 2.",
            "took over another": "None; every drone reports took over: none.",
            "already logged": "Drone 0.",
        }
        ans = next((v for k, v in answers.items() if k in q), "Explained from the records.")
        return {"answer": ans, "citations": ids[:1], "answerable": True}
    return responder


def test_run_eval_scoring_with_oracle_and_unanswerable_logic(mission, tmp_path):
    f = compute_facts(mission)
    out = run_eval(mission, provider=MockProvider(_oracle(f)), out_dir=tmp_path)
    s = out["summary"]
    assert s["accuracy"] == 1.0, [r["id"] for r in out["results"] if not r["correct"]]
    assert s["refusal_rate_unanswerable"] == 1.0
    assert s["citation_validity_rate"] == 1.0
    assert (tmp_path / "eval_table_mock.md").exists() and (tmp_path / "eval_results_mock.json").exists()


def test_unanswerable_answered_is_wrong_and_refusal_of_answerable_is_wrong():
    rows = [
        {"type": "unanswerable", "correct": False, "qa_valid": True, "answerable": True,
         "latency_s": 1.0, "citations_valid": True, "citation_support": None},
        {"type": "numeric", "correct": False, "qa_valid": True, "answerable": False,
         "latency_s": 3.0, "citations_valid": None, "citation_support": None},
    ]
    s = summarise(rows)
    assert s["refusal_rate_unanswerable"] == 0.0
    assert s["false_refusal_rate_answerable"] == 1.0
    assert s["mean_latency_s"] == 2.0
