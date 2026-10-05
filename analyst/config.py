"""Paths, provider and retrieval settings. Keys are read from the
environment only (GEMINI_API_KEY / GROQ_API_KEY) and never printed."""
from __future__ import annotations

import os
from pathlib import Path

PKG_DIR = Path(__file__).resolve().parent
DATA_DIR = Path(os.environ.get("ANALYST_DATA_DIR", PKG_DIR / "data"))
RESULTS_DIR = Path(os.environ.get("ANALYST_RESULTS_DIR", PKG_DIR / "results"))

DEFAULT_PROVIDER = os.environ.get("ANALYST_PROVIDER", "mock")
MODELS = {
    "gemini": os.environ.get("ANALYST_GEMINI_MODEL", "gemini-2.5-flash"),
    "groq": os.environ.get("ANALYST_GROQ_MODEL", "llama-3.3-70b-versatile"),
    "mock": "mock",
}
API_KEY_ENV = {"gemini": "GEMINI_API_KEY", "groq": "GROQ_API_KEY"}
# Free tiers rate-limit per minute; space live calls out (seconds).
MIN_CALL_INTERVAL_S = float(os.environ.get("ANALYST_MIN_CALL_INTERVAL_S", "4.5"))
MAX_RETRIES = 4

TOP_K = int(os.environ.get("ANALYST_TOP_K", "8"))

# A truth target with no hazard within this radius is MISSED; same default as
# src/perception/perception/hazard_map.py --miss-radius.
MATCH_RADIUS_M = 5.0


def missions() -> list[str]:
    if not DATA_DIR.is_dir():
        return []
    return sorted(p.name for p in DATA_DIR.iterdir() if p.is_dir())


def resolve_mission(mission: str | None) -> str:
    found = missions()
    if mission:
        if mission not in found:
            raise SystemExit(f"mission '{mission}' not found under {DATA_DIR} (have: {found})")
        return mission
    if len(found) == 1:
        return found[0]
    raise SystemExit(f"pass --mission; found {found or 'no missions'} under {DATA_DIR}")


def mission_dir(mission: str) -> Path:
    return DATA_DIR / mission
