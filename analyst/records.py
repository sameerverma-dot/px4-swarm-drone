"""The unit of evidence: one CSV row, one log line, or one per-file /
per-segment summary. Record.text is what retrieval and the LLM see."""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True)
class Record:
    id: str                    # stable, citable: "<source>:<locator>"
    source_file: str           # path relative to the mission folder
    row_or_line: str           # "row 4", "line 12", "lane 3", "summary"
    mission_id: str
    drone_id: int | None
    t: float | None            # seconds since mission T0 (launch start)
    text: str
    fields: dict = field(default_factory=dict, compare=False, hash=False)

    def prompt_line(self) -> str:
        return f"[{self.id}] {self.text}"
