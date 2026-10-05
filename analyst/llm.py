"""Provider interface: Gemini | Groq | Mock. Every call asks for JSON and
returns a parsed dict. Transient failures are retried with backoff.

API keys come only from the environment (GEMINI_API_KEY / GROQ_API_KEY) and
are never logged or printed.
"""
from __future__ import annotations

import json
import os
import re
import sys
import time
from typing import Callable

from . import config


class LLMError(RuntimeError):
    pass


def parse_json(text: str) -> dict:
    """Parse a JSON object, tolerating ```json fences around it."""
    t = text.strip()
    m = re.search(r"```(?:json)?\s*(.*?)```", t, re.S)
    if m:
        t = m.group(1).strip()
    try:
        out = json.loads(t)
    except json.JSONDecodeError:
        i, j = t.find("{"), t.rfind("}")
        if i < 0 or j <= i:
            raise LLMError(f"response is not JSON: {text[:200]!r}")
        out = json.loads(t[i:j + 1])
    if not isinstance(out, dict):
        raise LLMError(f"expected a JSON object, got {type(out).__name__}")
    return out


class Provider:
    name = "base"

    def __init__(self, model: str):
        self.model = model
        self._last_call = 0.0
        self.min_interval_s = 0.0

    def _raw(self, system: str, user: str) -> str:          # pragma: no cover - interface
        raise NotImplementedError

    def generate_json(self, system: str, user: str, task: str = "qa") -> dict:
        """One JSON completion. Returns the parsed dict plus '_latency_s'
        (time spent in the API call, excluding rate-limit spacing)."""
        last_err = None
        for attempt in range(config.MAX_RETRIES):
            wait = self.min_interval_s - (time.monotonic() - self._last_call)
            if wait > 0:
                time.sleep(wait)
            t0 = time.monotonic()
            try:
                text = self._raw(system, user)
                self._last_call = time.monotonic()
                out = parse_json(text)
                out["_latency_s"] = round(self._last_call - t0, 3)
                return out
            except LLMError as e:                            # bad JSON: retry soon
                self._last_call = time.monotonic()
                last_err, delay = e, 2.0
            except Exception as e:                           # network / overload / rate limit
                self._last_call = time.monotonic()
                last_err = e
                if not _transient(e):
                    raise LLMError(f"{self.name} call failed: {_safe(e)}") from None
                wait_s = _server_retry_after(e)
                if wait_s is not None and wait_s > 120:
                    # e.g. a daily free-tier quota: "Please retry in 7h22m". Retrying now is pointless.
                    raise LLMError(f"{self.name} quota exhausted for model {self.model} (server says retry "
                                   f"in {wait_s / 3600:.1f} h): {_safe(e)}") from None
                delay = min(config.RETRY_BASE_S * 2 ** attempt, 60.0)
            if attempt + 1 < config.MAX_RETRIES:
                print(f"    {self.name}: {_safe(last_err)[:70]}... retry {attempt + 2}/"
                      f"{config.MAX_RETRIES} in {delay:.0f}s", file=sys.stderr, flush=True)
                time.sleep(delay)
        raise LLMError(f"{self.name} failed after {config.MAX_RETRIES} attempts: {_safe(last_err)}")


def _server_retry_after(e: Exception) -> float | None:
    """Seconds from a 'Please retry in 7h22m20.9s' style hint in the error, if any."""
    m = re.search(r"retry in (?:(\d+)h)?(?:(\d+)m)?(?:([\d.]+)s)?", str(e))
    if not m or not any(m.groups()):
        return None
    h, mi, se = (float(g) if g else 0.0 for g in m.groups())
    return h * 3600 + mi * 60 + se


def _transient(e: Exception) -> bool:
    s = str(e).lower()
    return any(k in s for k in ("429", "rate", "quota", "timeout", "timed out", "unavailable",
                                "503", "500", "502", "overloaded", "connection", "resource_exhausted"))


def _safe(e) -> str:
    """Error text with anything key-like scrubbed."""
    s = str(e)
    for env in config.API_KEY_ENV.values():
        k = os.environ.get(env)
        if k:
            s = s.replace(k, "***")
    return s[:500]


class GeminiProvider(Provider):
    name = "gemini"

    def __init__(self, model: str | None = None):
        super().__init__(model or config.MODELS["gemini"])
        key = os.environ.get("GEMINI_API_KEY")
        if not key:
            raise LLMError("GEMINI_API_KEY is not set in the environment")
        from google import genai
        from google.genai import types
        self._types = types
        self._client = genai.Client(api_key=key)
        self.min_interval_s = config.MIN_CALL_INTERVAL_S

    def _raw(self, system: str, user: str) -> str:
        cfg = self._types.GenerateContentConfig(
            system_instruction=system, response_mime_type="application/json", temperature=0.0,
            automatic_function_calling=self._types.AutomaticFunctionCallingConfig(disable=True))
        resp = self._client.models.generate_content(model=self.model, contents=user, config=cfg)
        return resp.text or ""


class GroqProvider(Provider):
    name = "groq"

    def __init__(self, model: str | None = None):
        super().__init__(model or config.MODELS["groq"])
        key = os.environ.get("GROQ_API_KEY")
        if not key:
            raise LLMError("GROQ_API_KEY is not set in the environment")
        from groq import Groq
        self._client = Groq(api_key=key)
        self.min_interval_s = config.MIN_CALL_INTERVAL_S

    def _raw(self, system: str, user: str) -> str:
        resp = self._client.chat.completions.create(
            model=self.model, temperature=0.0, response_format={"type": "json_object"},
            messages=[{"role": "system", "content": system}, {"role": "user", "content": user}])
        return resp.choices[0].message.content or ""


Responder = Callable[[str, str, str], "dict | str"]


class MockProvider(Provider):
    """Offline provider. With no responder it behaves as a naive extractive
    baseline (answers with the top retrieved record). Tests pass a responder
    function, or a list of canned responses consumed in order."""
    name = "mock"

    def __init__(self, responder: Responder | list | None = None):
        super().__init__("mock")
        self.calls: list[dict] = []
        if isinstance(responder, list):
            queue = list(responder)
            self._responder = lambda s, u, t: queue.pop(0)
        else:
            self._responder = responder or default_mock_responder

    def generate_json(self, system: str, user: str, task: str = "qa") -> dict:
        self.calls.append({"system": system, "user": user, "task": task})
        out = self._responder(system, user, task)
        out = parse_json(out) if isinstance(out, str) else dict(out)
        out["_latency_s"] = 0.0
        return out


RECORD_LINE_RE = re.compile(r"^\[([^\]]+)\] (.*)$", re.M)


def default_mock_responder(system: str, user: str, task: str) -> dict:
    if task == "judge":
        return {"score": 3, "reason": "mock judge: fixed score"}
    if task == "report":
        return {"summary": "Mock narrative: run with a live provider for a written summary.",
                "anomalies": [], "citations": []}
    recs = RECORD_LINE_RE.findall(user)
    if not recs:
        return {"answer": "The provided records do not contain this.", "citations": [],
                "answerable": False}
    rid, text = recs[0]
    return {"answer": text, "citations": [rid], "answerable": True}


def get_provider(name: str | None = None, **kw) -> Provider:
    name = (name or config.DEFAULT_PROVIDER).lower()
    if name == "mock":
        return MockProvider(**kw)
    if name == "gemini":
        return GeminiProvider(**kw)
    if name == "groq":
        return GroqProvider(**kw)
    raise LLMError(f"unknown provider {name!r}")
