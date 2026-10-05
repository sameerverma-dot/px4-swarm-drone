import pytest

from analyst import llm
from analyst.llm import LLMError, Provider


class Scripted(Provider):
    name = "scripted"

    def __init__(self, outcomes):
        super().__init__("scripted-model")
        self.outcomes = list(outcomes)

    def _raw(self, system, user):
        o = self.outcomes.pop(0)
        if isinstance(o, Exception):
            raise o
        return o


@pytest.fixture(autouse=True)
def no_sleep(monkeypatch):
    monkeypatch.setattr(llm.time, "sleep", lambda s: None)


def test_overload_is_retried_until_success():
    p = Scripted([RuntimeError("503 UNAVAILABLE high demand"), RuntimeError("429 rate limit"), '{"ok": 1}'])
    out = p.generate_json("s", "u")
    assert out["ok"] == 1 and p.outcomes == []


def test_non_transient_error_fails_fast():
    p = Scripted([RuntimeError("400 API key not valid"), '{"ok": 1}'])
    with pytest.raises(LLMError, match="API key not valid"):
        p.generate_json("s", "u")
    assert p.outcomes == ['{"ok": 1}']


def test_gives_up_after_max_retries(monkeypatch):
    monkeypatch.setattr(llm.config, "MAX_RETRIES", 3)
    p = Scripted([RuntimeError("503 UNAVAILABLE")] * 3)
    with pytest.raises(LLMError, match="after 3 attempts"):
        p.generate_json("s", "u")
