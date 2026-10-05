# Eval results: swarm_20261002_180220

Provider `groq` (`openai/gpt-oss-120b`), judge `groq` (`openai/gpt-oss-120b`), top_k=8, 16 questions, run 2026-10-05 16:47 UTC.

| metric | value |
|---|---|
| overall accuracy | 75% (12/16) |
| accuracy: numeric | 80% (4/5) |
| accuracy: entity | 50% (2/4) |
| accuracy: explanatory | 67% (2/3) |
| accuracy: unanswerable | 100% (4/4) |
| hallucination-refusal rate (unanswerable correctly refused) | 100% (4/4) |
| false-refusal rate (answerable questions refused) | 25% |
| citation validity (cited ids exist and were retrieved) | 100% (n=9) |
| citation support (cited records contain the expected fact) | 100% (n=9) |
| answers rejected by the citation check after retry | 0 |
| mean QA latency per question | 0.97 s |

## Per question

| id | type | expected | correct | answerable | cites valid | support | judge | latency | answer |
|---|---|---|---|---|---|---|---|---|---|
| n_map_hazards | numeric | 11 | yes | yes | yes | yes | - | 0.91s | There are 11 unique hazards in the final hazard map. |
| n_own_hazards | numeric | 5 | yes | yes | yes | yes | - | 0.71s | 5 |
| n_truth | numeric | 11 | no | yes | yes | yes | - | 1.02s | 8 ground-truth targets were placed. |
| n_track_samples | numeric | 6536 | yes | yes | yes | yes | - | 0.82s | Drone 1's flown survey track contains 6536 samples. |
| n_lane_spacing | numeric | 3.99 | yes | yes | yes | yes | - | 0.74s | 3.99 m |
| e_first_hazard | entity | drone 0 | no | no | - | - | - | 0.35s |  |
| e_landed_last | entity | drone 2 | yes | yes | yes | yes | - | 0.85s | Drone 2 landed last at T+143.5 s. |
| e_takeover | entity | none | yes | yes | yes | yes | - | 0.92s | No drone took over another drone's lanes; each drone’s record shows took over: none. |
| e_dedup | entity | drone 0 | no | no | - | - | - | 2.29s |  |
| x_no_takeover | explanatory | - | yes | yes | yes | yes | 5 | 1.09s | There was no lane takeover because each drone completed all of its own lanes (8/8), so no takeover was needed. |
| x_area_split | explanatory | - | no | no | - | - | - | 1.13s | The records do not specify how the total survey area was partitioned among the three drones. |
| x_lane_spacing | explanatory | - | yes | yes | yes | yes | 5 | 1.06s | The lane spacing was derived from a detection swath of 4.99 m (camera footprint 23.71 m) at 10.0 m altitude... |
| u_battery_voltage | unanswerable | - | yes | no | - | - | - | 0.77s | Battery voltage at landing not recorded in the provided records. |
| u_wind | unanswerable | - | yes | no | - | - | - | 0.95s | Wind speed not recorded in the mission records. |
| u_gps_sats | unanswerable | - | yes | no | - | - | - | 0.61s | The records do not state how many GPS satellites drone 2 had during the survey. |
| u_cpu_temp | unanswerable | - | yes | no | - | - | - | 1.23s | CPU temperature not provided in the records. |
