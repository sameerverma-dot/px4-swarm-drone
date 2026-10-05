from analyst.llm import MockProvider, parse_json
from analyst.qa import Analyst

Q = "Which drone took over lanes, and why?"


def make(records, mission, responses):
    mock = MockProvider(responses)
    return Analyst(mission, provider=mock, records=records), mock


def test_valid_citation_accepted(records, mission):
    good = {"answer": "No drone took over lanes.", "citations": ["survey_node_0:L72"], "answerable": True}
    an, mock = make(records, mission, [good])
    ans = an.ask(Q)
    assert ans.valid and ans.attempts == 1 and ans.citations == ["survey_node_0:L72"]
    assert "survey_node_0:L72" in ans.retrieved_ids


def test_citation_outside_retrieved_set_retried_then_invalid(records, mission):
    bad = {"answer": "drone 1", "citations": ["made_up:L1"], "answerable": True}
    an, mock = make(records, mission, [bad, bad])
    ans = an.ask(Q)
    assert not ans.valid and ans.attempts == 2
    assert "made_up:L1" in ans.error
    assert len(mock.calls) == 2 and "rejected" in mock.calls[1]["user"]


def test_real_record_not_retrieved_is_still_rejected(records, mission):
    # map:1 exists in the corpus but is not retrieved for this question
    bad = {"answer": "x", "citations": ["map:1"], "answerable": True}
    good = {"answer": "none", "citations": ["survey_node_1:L72"], "answerable": True}
    an, _ = make(records, mission, [bad, good])
    ans = an.ask(Q)
    assert "map:1" not in ans.retrieved_ids
    assert ans.valid and ans.attempts == 2


def test_answerable_without_citation_rejected(records, mission):
    nocite = {"answer": "drone 0", "citations": [], "answerable": True}
    an, _ = make(records, mission, [nocite, nocite])
    assert not an.ask(Q).valid


def test_refusal_without_citations_is_valid(records, mission):
    refuse = {"answer": "Not in the records.", "citations": [], "answerable": False}
    an, _ = make(records, mission, [refuse])
    ans = an.ask("What was the battery voltage of drone 1 at landing?")
    assert ans.valid and ans.answerable is False


def test_schema_violation_retried(records, mission):
    an, _ = make(records, mission, [{"text": "oops"},
                                    {"answer": "a", "citations": ["survey_node_2:L72"], "answerable": True}])
    ans = an.ask(Q)
    assert ans.valid and ans.attempts == 2


def test_parse_json_tolerates_fences():
    assert parse_json('```json\n{"a": 1}\n```') == {"a": 1}
    assert parse_json('Sure: {"a": 2} ok') == {"a": 2}
