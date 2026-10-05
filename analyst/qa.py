"""Grounded Q&A: retrieve -> prompt -> JSON -> schema + citation check.

A citation must be the id of a record that was actually retrieved for this
question. A reply that breaks that (or cites nothing while claiming an answer)
is retried once with the reason; a second failure marks the answer invalid.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field

from pydantic import BaseModel, ValidationError, field_validator

from . import config
from .ingest import load_mission
from .llm import LLMError, Provider, get_provider
from .prompts import QA_SYSTEM, qa_user
from .records import Record
from .retrieve import Index


class QAOutput(BaseModel):
    answer: str
    citations: list[str] = []
    answerable: bool

    @field_validator("citations", mode="before")
    @classmethod
    def _strip(cls, v):
        if v is None:
            return []
        return [str(c).strip().strip("[]").strip() for c in v]


@dataclass
class Answer:
    question: str
    answer: str
    citations: list[str]
    answerable: bool
    valid: bool
    retrieved_ids: list[str]
    attempts: int
    latency_s: float
    error: str | None = None
    raw: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return asdict(self)


def citation_problem(out: QAOutput, retrieved: set[str]) -> str | None:
    bad = [c for c in out.citations if c not in retrieved]
    if bad:
        return f"citations {bad} are not ids of the provided records"
    if out.answerable and not out.citations:
        return "an answerable reply must cite at least one provided record id"
    return None


class Analyst:
    def __init__(self, mission: str, provider: Provider | str | None = None,
                 records: list[Record] | None = None):
        self.mission = mission
        self.records = records if records is not None else load_mission(mission)
        self.index = Index(self.records)
        self.provider = provider if isinstance(provider, Provider) else get_provider(provider)

    def retrieve(self, question: str, top_k: int = config.TOP_K) -> list[Record]:
        return [r for r, _ in self.index.search(question, top_k, with_summaries=True)]

    def ask(self, question: str, top_k: int = config.TOP_K) -> Answer:
        hits = self.retrieve(question, top_k)
        lines = [r.prompt_line() for r in hits]
        retrieved = [r.id for r in hits]
        feedback, latency, raws, last = None, 0.0, [], None
        for attempt in (1, 2):
            try:
                raw = self.provider.generate_json(QA_SYSTEM, qa_user(question, lines, feedback), task="qa")
            except LLMError as e:
                return Answer(question, "", [], False, False, retrieved, attempt, latency, error=str(e),
                              raw=raws)
            latency += raw.pop("_latency_s", 0.0)
            raws.append(raw)
            try:
                out = QAOutput.model_validate(raw)
            except ValidationError as e:
                feedback = f"reply did not match the JSON schema ({e.errors()[0]['msg']})"
                continue
            last = out
            feedback = citation_problem(out, set(retrieved))
            if feedback is None:
                return Answer(question, out.answer, out.citations, out.answerable, True, retrieved,
                              attempt, round(latency, 3), raw=raws)
        return Answer(question, last.answer if last else "", last.citations if last else [],
                      last.answerable if last else False, False, retrieved, 2, round(latency, 3),
                      error=feedback, raw=raws)
