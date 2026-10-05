import pytest

from analyst.config import missions
from analyst.ingest import load_mission
from analyst.retrieve import Index

MISSION = "swarm_20261002_180220"


@pytest.fixture(scope="session")
def mission():
    if MISSION not in missions():
        pytest.skip(f"mission data {MISSION} not present")
    return MISSION


@pytest.fixture(scope="session")
def records(mission):
    return load_mission(mission)


@pytest.fixture(scope="session")
def index(records):
    return Index(records)
