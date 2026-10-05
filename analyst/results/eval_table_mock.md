# Eval results: swarm_20261002_180220

Provider `mock` (`mock`), judge `mock` (`mock`), top_k=8, 16 questions, run 2026-10-05 16:05 UTC.

> **Offline mock run** - a naive extractive baseline that answers with the top retrieved record. It checks the harness, not an LLM. Use `--provider gemini` for real numbers.

| metric | value |
|---|---|
| overall accuracy | 31% (5/16) |
| accuracy: numeric | 80% (4/5) |
| accuracy: entity | 25% (1/4) |
| accuracy: explanatory | 0% (0/3) |
| accuracy: unanswerable | 0% (0/4) |
| hallucination-refusal rate (unanswerable correctly refused) | 0% (0/4) |
| false-refusal rate (answerable questions refused) | 0% |
| citation validity (cited ids exist and were retrieved) | 100% (n=16) |
| citation support (cited records contain the expected fact) | 58% (n=12) |
| answers rejected by the citation check after retry | 0 |
| mean QA latency per question | 0.00 s |

## Per question

| id | type | expected | correct | answerable | cites valid | support | judge | latency | answer |
|---|---|---|---|---|---|---|---|---|---|
| n_map_hazards | numeric | 11 | yes | yes | yes | yes | - | 0.00s | final hazard map (hazard_map.geojson): 11 hazards in total; by reporting drone: drone 0: 5, drone 1: 3, dro... |
| n_own_hazards | numeric | 5 | yes | yes | yes | yes | - | 0.00s | drone 0 onboard hazard list (hazards_d0.csv): 11 unique hazards in 39 rows; own detections (5): d0-1, d0-2,... |
| n_truth | numeric | 11 | no | yes | yes | no | - | 0.00s | ground-truth target sw0 placed at N=6 E=7.5 |
| n_track_samples | numeric | 6536 | yes | yes | yes | yes | - | 0.00s | drone 1 flown track (survey_track_d1_1790944481.csv): 6536 samples over 131.0 s (~50 Hz), 393.9 m flown hor... |
| n_lane_spacing | numeric | 3.99 | yes | yes | yes | yes | - | 0.00s | survey_node_0 T+8.7s: lane_spacing derived: detection swath 4.99 m (camera footprint 23.71 m) at 10.0 m alt... |
| e_first_hazard | entity | drone 0 | yes | yes | yes | no | - | 0.00s | final hazard map (hazard_map.geojson): 11 hazards in total; by reporting drone: drone 0: 5, drone 1: 3, dro... |
| e_landed_last | entity | drone 2 | no | yes | yes | no | - | 0.00s | ground_station T+139.0s: drone 0: LANDED |
| e_takeover | entity | none | no | yes | yes | no | - | 0.00s | final hazard map (hazard_map.geojson): 11 hazards in total; by reporting drone: drone 0: 5, drone 1: 3, dro... |
| e_dedup | entity | drone 0 | no | yes | yes | yes | - | 0.00s | detector_node_0 T+112.7s: saw person at N=22.1 E=29.8 - already logged by drone 1 as d1-1, not logged again |
| x_no_takeover | explanatory | - | no | yes | yes | yes | 3 | 0.00s | survey_node_0 T+138.7s: VERIFY PASS / drone 0 / own lanes 8/8 / took over: none / waypoints 17/17 / returne... |
| x_area_split | explanatory | - | no | yes | yes | no | 3 | 0.00s | mission config (run.json): num_drones=3, x_min=0.0, x_max=30.0, y_min=0.0, y_max=90.0, altitude=10.0, start... |
| x_lane_spacing | explanatory | - | no | yes | yes | yes | 3 | 0.00s | survey_node_0 T+8.7s: lane_spacing derived: detection swath 4.99 m (camera footprint 23.71 m) at 10.0 m alt... |
| u_battery_voltage | unanswerable | - | no | yes | yes | - | - | 0.00s | ground_station T+10.3s: drone 1: SURVEY |
| u_wind | unanswerable | - | no | yes | yes | - | - | 0.00s | survey_node_0 T+121.5s: RTL: mission complete |
| u_gps_sats | unanswerable | - | no | yes | yes | - | - | 0.00s | ground_station T+10.0s: drone 2: SURVEY |
| u_cpu_temp | unanswerable | - | no | yes | yes | - | - | 0.00s | drone 0 onboard hazard list (hazards_d0.csv): 11 unique hazards in 39 rows; own detections (5): d0-1, d0-2,... |
