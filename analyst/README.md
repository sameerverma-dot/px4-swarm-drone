# LLM Mission Analyst (SWARN drone swarm)

A retrieval-grounded LLM layer over the swarm's **saved mission outputs**: hazard CSVs, the final GeoJSON
map, flown tracks, camera sightings, ground truth and the ROS node logs. It does three things:

1. **`ask`**: answers questions from the logs with **cited record ids**, and refuses when the logs
   don't contain the answer.
2. **`report`**: writes a schema-validated `MissionReport`. All numbers are deterministic; only the
   summary and anomalies are written by the LLM.
3. **`eval`**: an eval set generated from computed facts, scored by deterministic checks plus an
   LLM judge.

It is pure Python on files only: no ROS, Gazebo, PX4 or GPU. All tests run offline against a mock provider.

## Quickstart

```bash
pip install -r analyst/requirements.txt
python -m analyst inventory                       # files, columns, row counts, records
python -m analyst ask "Which drone took over lanes, and why?" --provider gemini
python -m analyst report --provider gemini        # -> analyst/results/sample_report.md
python -m analyst eval   --provider gemini        # -> analyst/results/eval_table.md
pytest analyst/tests                              # offline, mock provider, no key needed
```

`--mission <id>` picks a folder under `analyst/data/`. It's optional when there is only one.
Keys are read only from the environment: `GEMINI_API_KEY` (default live provider) or `GROQ_API_KEY`.
Other settings are environment variables too: `ANALYST_GEMINI_MODEL` (default `gemini-2.5-flash`),
`ANALYST_TOP_K` (8) and `ANALYST_MIN_CALL_INTERVAL_S` (4.5 s spacing for free-tier rate limits).

## Data → records

Mission `swarm_20261002_180220`: 3 drones, area N 0–30 m × E 0–90 m, 10 m altitude, 11 ground-truth targets.
`ingest.py` turns 22 files into **363 citable records**:

| source | records | how |
|---|---|---|
| `ground_hazards.csv` | 39 | one per row (first report / refinement vN) |
| `hazards_d{0,1,2}.csv` | 3 | one summary per drone's onboard list (own vs. received from peers) |
| `hazard_map.geojson` | 12 | one per final-map hazard + a file summary |
| `targets.csv` | 12 | one per truth target + a file summary |
| `sightings_d*.csv` (335 rows) | 15 | one per (drone, hazard) + below-threshold summaries |
| `survey_track_d*.csv` (19,666 samples at 50 Hz) | 30 | **downsampled**: per-lane segments (distance, speed, times) + summary + return leg |
| `ros_log/*.log` (558 lines) | 251 | one per meaningful line; status tables grouped into one record; gate and per-waypoint lines summarised |
| `run.json` | 1 | mission config |

Record ids are stable and citable, e.g. `survey_node_0:L72` (log line 72), `ground_hazards:r4` (CSV row 4),
`track_d1:lane3`, `map:summary`. Record text is a short sentence, and every number in it is read from the file.
Lane times come straight from the survey log's `reached wp` lines.

Not ingested: `hazard_map.png` (image), and `docs/PROGRESS.md`. PROGRESS.md narrates many flights, so its
numbers (e.g. flight A's 4/4 targets) would contaminate answers about *this* mission. `facts.py`
independently reproduces its §12.3 "final" row: 11/11 targets, 0 duplicates, 0.09–0.29 m.

## Design choices

**BM25, not embeddings.** The logs are full of exact tokens where lexical match beats semantic similarity:
drone ids, hazard ids like `d1-2`, event names (`VERIFY`, `RETURNING`), and timestamps. BM25 is deterministic,
unit-testable and needs no API. The tokenizer normalises `drone 2` / `drone_2` to one token and maps
`took over` / `take over` to `takeover`. Each record is also indexed with its event-type field tokens, so
"which drone took over lanes" lands on the three `VERIFY ... took over: none` lines. Embeddings (or hybrid
BM25 + embeddings) are a possible extension for paraphrased questions.

**Grounded answers with a citation check.** The model must return `{answer, citations, answerable}`
(validated with pydantic). Any citation that isn't the id of a record retrieved *for this question* is
rejected. That includes real record ids that weren't retrieved, and answers that claim to be answerable
but cite nothing. The model is retried once with the reason; a second failure marks the answer invalid.

**Facts vs. narrative split.** `facts.py` computes every number with pandas and regex log parsing:
one-to-one truth matching within 5 m (the same miss radius as `hazard_map.py`), false positives,
duplicates, geotag error, lanes, VERIFY / takeover status, landing order and suppressed duplicates.
The `MissionReport` numbers come only from there. The LLM writes only `summary` and `anomalies`, from
the facts plus retrieved records. A deterministic guard checks that every number in the narrative appears
in the facts or records. Invented numbers trigger a retry, and if they persist the report is flagged.

**Eval built from facts, not hand-typed answers.** `evalset.py` turns computed facts into 16 questions:

| type | n | scoring |
|---|---|---|
| numeric (e.g. hazards in the final map, lane spacing) | 5 | parse numbers from the answer, exact match (or tolerance) |
| entity / event (e.g. which drone landed last; which drone took over lanes → **none**) | 4 | expected entity regex present, contradicting phrasing rejected |
| explanatory (e.g. why no takeover happened) | 3 | LLM-as-judge, 1–5 rubric against the facts, pass ≥ 4 |
| unanswerable (battery *voltage*, wind, GPS satellites, CPU temperature) | 4 | pass only if `answerable == false` |

An unanswerable question is kept only if no record mentions the quantity. The battery-voltage question is
a deliberate trap: battery **%** is logged (it sits at 50% for most of the flight) but voltage is not, so
answering "50%" is a hallucination. Every answer is also scored for **citation validity** (cited ids exist
and were retrieved) and **citation support** (the cited records contain the expected number or entity).

**Judge bias.** The judge uses the same model family as the answerer, which is a known self-preference
bias. That's why the judge grades only the 3 explanatory questions. The other 13 are scored deterministically,
along with citation validity and support for all 16.

**This mission had no takeover.** All three drones log `own lanes 8/8 | took over: none`. So
`takeover_events` is `[]`, and the takeover question's ground truth is "none". The takeover-event parser
(VERIFY `took over:` field plus takeover-type log lines) has not been exercised on a mission with a real
takeover.

## Results

<!-- RESULTS:START -->
**Live run: pending.** No API key was available in the build session. Run
`python -m analyst eval --provider gemini` to write `results/eval_table.md`.

Offline baseline (`results/eval_table_mock.md`): the mock provider just answers with the top retrieved
record. It scores **31% (5/16)**: 4/5 numeric, 1/4 entity, 0/3 explanatory, 0/4 unanswerable refused,
100% citation validity. This is the floor the LLM has to beat, and a check that the harness doesn't
reward answering everything.
<!-- RESULTS:END -->

## Layout

```
analyst/
  __main__.py   CLI: inventory | search | ask | report | eval
  config.py     paths, provider/model, top_k, match radius
  records.py    Record(id, source_file, row_or_line, mission_id, drone_id, t, text, fields)
  ingest.py     files -> records; log parsing; track -> lane segments; inventory
  retrieve.py   BM25 index over record text + field tokens
  llm.py        Gemini | Groq | Mock providers, JSON mode, retries, rate-limit spacing
  prompts.py    grounded-QA, report-narrative and judge prompts
  qa.py         ask(): retrieve -> JSON -> schema + citation check (retry once)
  facts.py      deterministic mission facts (ground truth)
  report.py     MissionReport schema, narrative checks, Markdown render
  evalset.py    eval questions generated from facts
  evaluate.py   scoring, eval_results.json, eval_table.md
  tests/        27 offline tests (mock provider)
  results/      sample_report.md, eval_table*.md, eval_results*.json
```

## Limitations

- One mission. The eval has 16 questions, so one question is about 6 points of accuracy.
- Entity scoring is regex-based. It is lenient: an answer that mentions the right drone among others passes.
- `top_k = 8`: questions that need many records at once (e.g. "list all 11 hazards") rely on the per-file
  summary records.
- The narrative number guard checks that a number *appears* in the sources, not that it's used correctly.
