from analyst.config import mission_dir
from analyst.ingest import file_kind, inventory, parse_log


def test_every_present_file_kind_produces_records(mission, records):
    sources = {r.source_file for r in records}
    for row in inventory(mission):
        if row["kind"] != "other":
            assert row["file"] in sources, f"{row['file']} produced no records"


def test_record_count_and_unique_ids(records):
    assert len(records) > 0
    ids = [r.id for r in records]
    assert len(ids) == len(set(ids))


def test_tracks_are_downsampled_per_lane(records):
    lanes = [r for r in records if r.fields.get("event") == "lane"]
    assert len(lanes) == 24                      # 3 drones x 8 lanes
    assert len(records) < 1000                   # not one record per 50 Hz sample


def test_lane_times_come_from_survey_log(records):
    # survey_node_0.log: "reached wp 2/17" at 1790944359.603, launch T0 1790944340.710
    lane0 = next(r for r in records if r.id == "track_d0:lane0")
    assert abs(lane0.t - (1790944359.603229238 - 1790944340.7104430)) < 0.02


def test_status_block_is_one_record(records):
    status = [r for r in records if r.fields.get("event") == "swarm_status"]
    assert status and all(len(r.fields["rows"]) == 3 for r in status)


def test_log_parser_handles_both_formats(mission):
    d = mission_dir(mission) / "ros_log"
    assert parse_log(d / "launch.log", "launch.log")[0].ts == 1790944340.7104430
    assert parse_log(d / "survey_node_0.log", "s")[0].node == "survey_node_0"
    assert file_kind("ros_log/x.log") == "log" and file_kind("hazards_d2.csv") == "drone_hazards"


def test_bm25_drone_specific_query(index):
    top = index.search("how many samples did drone 1 track contain", k=3)[0][0]
    assert top.drone_id == 1 and "6536 samples" in top.text
    hits = index.search("hazards reported by drone 2", k=3)
    assert all(r.drone_id == 2 for r, _ in hits)


def test_bm25_hazard_id_and_takeover(index):
    top = index.search("already logged d1-1 not logged again", k=1)[0][0]
    assert top.id == "detector_node_0:L47"
    top3 = [r.id for r, _ in index.search("which drone took over lanes", k=3)]
    assert all(i.endswith(":L72") and i.startswith("survey_node_") for i in top3)
