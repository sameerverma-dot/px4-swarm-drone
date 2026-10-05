"""BM25 retrieval over Record.text plus field tokens.

Lexical, not embeddings: mission logs are full of exact tokens (drone ids,
hazard ids like d1-2, event names, timestamps) where exact match beats
semantic similarity. Deterministic, testable, no API needed.
"""
from __future__ import annotations

import re

from rank_bm25 import BM25Okapi

from .records import Record

STOPWORDS = set("""
a an the of in on at to for from by with and or is was were be been are did does do
which what who whom whose when where why how many much this that these those it its
any all each every there their they them during than then into as about
""".split())

TOKEN_RE = re.compile(r"[a-z0-9]+(?:[.\-][a-z0-9]+)*")


def normalise(text: str) -> str:
    t = text.lower()
    t = re.sub(r"\bdrone[\s_#-]*(\d+)\b", r"drone\1", t)     # "drone 2" / "drone_2" -> drone2
    t = re.sub(r"\btake[\s-]?overs?\b|\btook over\b|\btaking over\b", "takeover", t)
    return t


def stem(tok: str) -> str:
    """Plural -> singular for plain words (drones -> drone, batteries -> battery).
    Ids and numbers are left alone. Applied to queries and records alike."""
    if not tok.isalpha() or len(tok) <= 3:
        return tok
    if tok.endswith("ies") and len(tok) > 4:
        return tok[:-3] + "y"
    if tok.endswith("s") and not tok.endswith("ss"):
        return tok[:-1]
    return tok


def tokenize(text: str) -> list[str]:
    toks = []
    for tok in TOKEN_RE.findall(normalise(text)):
        if tok in STOPWORDS:
            continue
        toks.append(stem(tok))
        m = re.fullmatch(r"(d\d+)-\d+", tok)                 # hazard id d1-2 also matches "d1"
        if m:
            toks.append(m.group(1))
        if re.fullmatch(r"drone\d+", tok):                   # "drone 2" also matches plain "drone"
            toks.append("drone")
    return toks


def _field_tokens(value) -> list[str]:
    if isinstance(value, str):
        return [value]
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return []                                             # numbers are already in the text
    return []


def index_text(r: Record) -> str:
    parts = [r.text, r.source_file.rsplit("/", 1)[-1].rsplit(".", 1)[0]]
    if r.drone_id is not None:
        parts.append(f"drone{r.drone_id}")
    for k, v in r.fields.items():
        if k in ("event", "topic", "hazard_id", "took_over", "status", "outcome"):
            parts += _field_tokens(v)
    return " ".join(p.replace("_", " ") for p in parts)


class Index:
    def __init__(self, records: list[Record]):
        if not records:
            raise ValueError("no records to index")
        self.records = records
        self.by_id = {r.id: r for r in records}
        self._bm25 = BM25Okapi([tokenize(index_text(r)) for r in records])

    def search(self, query: str, k: int = 8, with_summaries: bool = False) -> list[tuple[Record, float]]:
        """Top-k records by BM25. With `with_summaries`, also append the file
        summary record (id '<file>:summary') of each file the hits come from,
        so count questions see the per-file totals, not just k rows."""
        q = tokenize(query)
        if not q:
            return []
        scores = self._bm25.get_scores(q)
        order = sorted(range(len(scores)), key=lambda i: (-scores[i], i))
        hits = [(self.records[i], float(scores[i])) for i in order[:k] if scores[i] > 0]
        if with_summaries:
            have = {r.id for r, _ in hits}
            for r, _ in list(hits):
                sid = r.id.split(":", 1)[0] + ":summary"
                if sid not in have and sid in self.by_id:
                    have.add(sid)
                    hits.append((self.by_id[sid], 0.0))
        return hits
