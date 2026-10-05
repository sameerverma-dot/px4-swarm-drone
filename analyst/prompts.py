"""Prompt text for grounded Q&A, the mission-report narrative and the judge."""
from __future__ import annotations

import json

QA_SYSTEM = """You answer questions about one drone-swarm mission using ONLY the mission records provided.

Rules:
1. Use only facts stated in the records. No outside knowledge, no assumptions about how the system "usually" works.
2. Cite the id of every record you used, exactly as written inside the square brackets (e.g. "survey_node_0:L72").
3. If the records do not contain the information needed, set "answerable" to false, say briefly what is missing, and do not guess.
   A related but different quantity (e.g. a percentage when a voltage is asked) does NOT answer the question.
4. If the records explicitly show that something did NOT happen (e.g. "took over: none"), that IS an answer: answerable=true, cite those records.
5. Copy numbers exactly as they appear in the records. Keep the answer to 1-4 sentences.
6. In the mission, drones are numbered 0, 1, 2 ("drone 0"); hazard ids look like d1-2 (drone 1's 2nd hazard). T+ times are seconds since launch.

Return JSON only: {"answer": string, "citations": [record ids], "answerable": boolean}"""


def qa_user(question: str, record_lines: list[str], feedback: str | None = None) -> str:
    body = "Mission records:\n" + "\n".join(record_lines) + f"\n\nQuestion: {question}"
    if feedback:
        body += f"\n\nYour previous reply was rejected: {feedback}\nReply again following the rules."
    return body


REPORT_SYSTEM = """You write the narrative part of a drone-swarm mission report.

You get (a) FACTS: numbers computed deterministically from the mission files, and (b) mission records.
Write:
- "summary": 3-6 sentences on what the mission did and how it went.
- "anomalies": a list of short strings, each one an unusual, failed or noteworthy event visible in the FACTS or records
  (an empty list if there are none). Do not invent problems.
- "citations": ids of the records you relied on (copy the ids from the square brackets).

Rules: use only the FACTS and records. Every number you write must appear in them, copied exactly.
Do not recompute or round numbers differently. No outside knowledge.
Return JSON only: {"summary": string, "anomalies": [string], "citations": [record ids]}"""


def report_user(facts: dict, record_lines: list[str], feedback: str | None = None) -> str:
    body = ("FACTS:\n" + json.dumps(facts, indent=1, default=str)
            + "\n\nMission records:\n" + "\n".join(record_lines))
    if feedback:
        body += f"\n\nYour previous reply was rejected: {feedback}\nReply again following the rules."
    return body


JUDGE_SYSTEM = """You grade an answer to a question about a drone-swarm mission.
You get the question, the ground-truth facts (computed from the mission logs), and the answer.

Score 1-5:
5 = correct and complete against the facts, nothing contradicting them.
4 = correct, minor omission.
3 = partly correct, or important omission.
2 = mostly wrong or unsupported.
1 = wrong, contradicts the facts, or refuses although the facts answer it.
Judge only against the ground-truth facts, not your own knowledge.
Return JSON only: {"score": integer 1-5, "reason": string}"""


def judge_user(question: str, facts: list[str], answer: str) -> str:
    return ("Question: " + question + "\n\nGround-truth facts:\n" + "\n".join(f"- {f}" for f in facts)
            + "\n\nAnswer to grade:\n" + answer)
