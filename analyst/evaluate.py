"""Run the eval set, score it, write results/eval_results.json + eval_table.md.

Deterministic checks carry most of the weight (numeric parse, entity match,
refusal, citation validity/support); the LLM judge is used only for the
explanatory questions.
"""
from __future__ import annotations

import datetime as dt
import json
import re
import time
from pathlib import Path

from pydantic import BaseModel, ValidationError, field_validator

from . import config
from .evalset import EvalQuestion, build_eval_set
from .facts import compute_facts
from .ingest import load_mission
from .llm import LLMError, Provider, get_provider
from .prompts import JUDGE_SYSTEM, judge_user
from .qa import Analyst, Answer

TYPES = ["numeric", "entity", "explanatory", "unanswerable"]
JUDGE_PASS = 4


class JudgeOutput(BaseModel):
    score: int
    reason: str = ""

    @field_validator("score")
    @classmethod
    def _range(cls, v):
        if not 1 <= v <= 5:
            raise ValueError("score must be 1-5")
        return v


def numbers_in(text: str) -> list[float]:
    t = re.sub(r"(?<=\d),(?=\d{3}\b)", "", text)          # 6,536 -> 6536
    t = re.sub(r"\bd\d+-\d+\b", " ", t)                     # hazard ids are not numbers
    return [float(x) for x in re.findall(r"(?<![\w.])-?\d+(?:\.\d+)?", t)]


def score_numeric(answer: str, expected: float, tol: float = 0.0) -> bool:
    return any(abs(x - float(expected)) <= tol + 1e-9 for x in numbers_in(answer))


def score_entity(answer: str, accept: list[str], reject: list[str] = ()) -> bool:
    a = answer.lower()
    if any(re.search(p, a, re.I) for p in reject):
        return False
    return any(re.search(p, a, re.I) for p in accept)


def citations_valid(ans: Answer, corpus_ids: set[str]) -> bool | None:
    """None when there is nothing to check (a refusal with no citations)."""
    if not ans.citations:
        return False if ans.answerable else None
    return all(c in corpus_ids and c in ans.retrieved_ids for c in ans.citations)


def citation_support(ans: Answer, q: EvalQuestion, by_id: dict) -> bool | None:
    if not q.support or not ans.answerable or not ans.citations:
        return None
    texts = [by_id[c].text for c in ans.citations if c in by_id]
    return any(re.search(p, t) for p in q.support for t in texts)


def judge(provider: Provider, q: EvalQuestion, answer: str) -> tuple[int | None, str, float]:
    try:
        raw = provider.generate_json(JUDGE_SYSTEM, judge_user(q.question, q.facts, answer), task="judge")
    except LLMError as e:
        return None, f"judge error: {e}", 0.0
    lat = raw.pop("_latency_s", 0.0)
    try:
        out = JudgeOutput.model_validate(raw)
    except ValidationError as e:
        return None, f"judge reply invalid: {e.errors()[0]['msg']}", lat
    return out.score, out.reason, lat


def score_question(q: EvalQuestion, ans: Answer, judge_provider: Provider, corpus_ids, by_id) -> dict:
    r = {"id": q.id, "type": q.type, "question": q.question, "expected": q.expected,
         "answer": ans.answer, "answerable": ans.answerable, "citations": ans.citations,
         "retrieved_ids": ans.retrieved_ids, "qa_valid": ans.valid, "qa_error": ans.error,
         "attempts": ans.attempts, "latency_s": ans.latency_s,
         "citations_valid": citations_valid(ans, corpus_ids),
         "citation_support": citation_support(ans, q, by_id),
         "judge_score": None, "judge_reason": None, "judge_latency_s": None}
    ok = ans.valid
    if q.type == "numeric":
        r["correct"] = bool(ok and ans.answerable and score_numeric(ans.answer, q.expected, q.tolerance))
    elif q.type == "entity":
        r["correct"] = bool(ok and ans.answerable and score_entity(ans.answer, q.accept, q.reject))
    elif q.type == "explanatory":
        if ok and ans.answerable:
            s, reason, lat = judge(judge_provider, q, ans.answer)
            r.update(judge_score=s, judge_reason=reason, judge_latency_s=lat)
            r["correct"] = s is not None and s >= JUDGE_PASS
        else:
            r.update(judge_reason="not judged: refused or invalid answer")
            r["correct"] = False
    elif q.type == "unanswerable":
        r["correct"] = bool(ok and not ans.answerable)
    return r


def summarise(rows: list[dict]) -> dict:
    def rate(xs):
        xs = [x for x in xs if x is not None]
        return (sum(bool(x) for x in xs) / len(xs)) if xs else None
    by_type = {t: {"n": sum(r["type"] == t for r in rows),
                   "correct": sum(r["correct"] for r in rows if r["type"] == t),
                   "accuracy": rate([r["correct"] for r in rows if r["type"] == t])}
               for t in TYPES}
    unans = [r for r in rows if r["type"] == "unanswerable"]
    answerable_qs = [r for r in rows if r["type"] != "unanswerable"]
    lat = [r["latency_s"] for r in rows if r["latency_s"] is not None]
    return {
        "n": len(rows),
        "correct": sum(r["correct"] for r in rows),
        "accuracy": rate([r["correct"] for r in rows]),
        "by_type": by_type,
        "refusal_rate_unanswerable": rate([r["qa_valid"] and not r["answerable"] for r in unans]),
        "refused_unanswerable": sum(bool(r["qa_valid"] and not r["answerable"]) for r in unans),
        "n_unanswerable": len(unans),
        "false_refusal_rate_answerable": rate([not r["answerable"] for r in answerable_qs]),
        "citation_validity_rate": rate([r["citations_valid"] for r in rows]),
        "n_citation_checked": sum(r["citations_valid"] is not None for r in rows),
        "citation_support_rate": rate([r["citation_support"] for r in rows]),
        "n_support_checked": sum(r["citation_support"] is not None for r in rows),
        "mean_latency_s": (sum(lat) / len(lat)) if lat else None,
        "qa_invalid": sum(not r["qa_valid"] for r in rows),
    }


def _pct(x):
    return "n/a" if x is None else f"{100 * x:.0f}%"


def render_table(out: dict) -> str:
    s, m = out["summary"], out["meta"]
    lat = "n/a" if s["mean_latency_s"] is None else f"{s['mean_latency_s']:.2f} s"
    L = [f"# Eval results: {m['mission']}", "",
         f"Provider `{m['provider']}` (`{m['model']}`), judge `{m['judge_provider']}` (`{m['judge_model']}`), "
         f"top_k={m['top_k']}, {m['n_questions']} questions, run {m['run_at']}.", ""]
    if m["provider"] == "mock":
        L += ["> **Offline mock run** - a naive extractive baseline that answers with the top "
              "retrieved record. It checks the harness, not an LLM. Use `--provider gemini` for real numbers.", ""]
    L += ["| metric | value |", "|---|---|",
          f"| overall accuracy | {_pct(s['accuracy'])} ({s['correct']}/{s['n']}) |"]
    for t in TYPES:
        b = s["by_type"][t]
        if b["n"]:
            L.append(f"| accuracy: {t} | {_pct(b['accuracy'])} ({b['correct']}/{b['n']}) |")
    L += [f"| hallucination-refusal rate (unanswerable correctly refused) | "
          f"{_pct(s['refusal_rate_unanswerable'])} ({s['refused_unanswerable']}/{s['n_unanswerable']}) |",
          f"| false-refusal rate (answerable questions refused) | {_pct(s['false_refusal_rate_answerable'])} |",
          f"| citation validity (cited ids exist and were retrieved) | {_pct(s['citation_validity_rate'])} "
          f"(n={s['n_citation_checked']}) |",
          f"| citation support (cited records contain the expected fact) | {_pct(s['citation_support_rate'])} "
          f"(n={s['n_support_checked']}) |",
          f"| answers rejected by the citation check after retry | {s['qa_invalid']} |",
          f"| mean QA latency per question | {lat} |", "",
          "## Per question", "",
          "| id | type | expected | correct | answerable | cites valid | support | judge | latency | answer |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    yn = {True: "yes", False: "no", None: "-"}
    for r in out["results"]:
        ans = r["answer"].replace("|", "/").replace("\n", " ")
        ans = ans if len(ans) <= 110 else ans[:107] + "..."
        L.append(f"| {r['id']} | {r['type']} | {r['expected'] if r['expected'] is not None else '-'} | "
                 f"{yn[r['correct']]} | {yn[r['answerable']]} | {yn[r['citations_valid']]} | "
                 f"{yn[r['citation_support']]} | {r['judge_score'] if r['judge_score'] is not None else '-'} | "
                 f"{r['latency_s']:.2f}s | {ans} |")
    return "\n".join(L) + "\n"


def run_eval(mission: str, provider: Provider | str | None = None,
             judge_provider: Provider | str | None = None, top_k: int = config.TOP_K,
             limit: int | None = None, out_dir: Path | None = None) -> dict:
    records = load_mission(mission)
    facts = compute_facts(mission)
    provider = provider if isinstance(provider, Provider) else get_provider(provider)
    if judge_provider is None or judge_provider == provider.name:
        jprov = provider
    else:
        jprov = judge_provider if isinstance(judge_provider, Provider) else get_provider(judge_provider)
    analyst = Analyst(mission, provider=provider, records=records)
    by_id = analyst.index.by_id
    corpus_ids = set(by_id)
    qs = build_eval_set(facts, records)[:limit] if limit else build_eval_set(facts, records)
    rows = []
    t_start = time.monotonic()
    for q in qs:
        ans = analyst.ask(q.question, top_k=top_k)
        if ans.error and not ans.raw:
            # No reply at all: the provider itself failed (bad key, unknown model, quota).
            # Stop instead of scoring every question as wrong and writing a 0% table.
            raise SystemExit(f"eval aborted at {q.id}: provider error, nothing written.\n  {ans.error}")
        rows.append(score_question(q, ans, jprov, corpus_ids, by_id))
        r = rows[-1]
        why = ""
        if not r["correct"]:
            why = f"  <- {r['qa_error'] or r['judge_reason'] or r['answer']}"[:160]
        print(f"  [{len(rows)}/{len(qs)}] {q.id:<18} {'PASS' if r['correct'] else 'FAIL'}{why}", flush=True)
    out = {"meta": {"mission": mission, "provider": provider.name, "model": provider.model,
                    "judge_provider": jprov.name, "judge_model": jprov.model, "top_k": top_k,
                    "n_questions": len(qs), "run_at": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d %H:%M UTC"),
                    "wall_time_s": round(time.monotonic() - t_start, 1)},
           "summary": summarise(rows),
           "questions": [q.to_dict() for q in qs],
           "results": rows}
    out_dir = Path(out_dir or config.RESULTS_DIR)
    out_dir.mkdir(parents=True, exist_ok=True)
    suffix = "" if provider.name != "mock" else "_mock"
    jp, tp = out_dir / f"eval_results{suffix}.json", out_dir / f"eval_table{suffix}.md"
    jp.write_text(json.dumps(out, indent=2, default=str))
    tp.write_text(render_table(out))
    out["json_path"], out["table_path"] = str(jp), str(tp)
    return out
