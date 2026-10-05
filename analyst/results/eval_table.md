# Eval results: swarm_20261002_180220

Provider `groq` (`openai/gpt-oss-120b`), judge `groq` (`openai/gpt-oss-120b`), top_k=8, 16 questions, run 2026-10-05 16:55 UTC.

| metric | value |
|---|---|
| overall accuracy | 88% (14/16) |
| accuracy: numeric | 100% (5/5) |
| accuracy: entity | 75% (3/4) |
| accuracy: explanatory | 67% (2/3) |
| accuracy: unanswerable | 100% (4/4) |
| hallucination-refusal rate (unanswerable correctly refused) | 100% (4/4) |
| false-refusal rate (answerable questions refused) | 17% |
| citation validity (cited ids exist and were retrieved) | 100% (n=10) |
| citation support (cited records contain the expected fact) | 100% (n=10) |
| answers rejected by the citation check after retry | 0 |
| mean QA latency per question | 1.56 s |

## Per question

| id | type | expected | correct | answerable | cites valid | support | judge | latency | answer |
|---|---|---|---|---|---|---|---|---|---|
| n_map_hazards | numeric | 11 | yes | yes | yes | yes | - | 1.03s | 11 |
| n_own_hazards | numeric | 5 | yes | yes | yes | yes | - | 1.01s | Drone 0 detected 5 hazards itself. |
| n_truth | numeric | 11 | yes | yes | yes | yes | - | 0.82s | 11 ground-truth targets were placed. |
| n_track_samples | numeric | 6536 | yes | yes | yes | yes | - | 2.36s | Drone 1's flown survey track contains 6536 samples. |
| n_lane_spacing | numeric | 3.99 | yes | yes | yes | yes | - | 2.87s | 3.99 metres |
| e_first_hazard | entity | drone 0 | no | no | - | - | - | 0.72s |  |
| e_landed_last | entity | drone 2 | yes | yes | yes | yes | - | 0.93s | Drone 2 landed last at T+143.5 s. |
| e_takeover | entity | none | yes | yes | yes | yes | - | 0.72s | No drone took over another drone's lanes during this mission. |
| e_dedup | entity | drone 0 | yes | yes | yes | yes | - | 2.77s | Drone 0 saw the person already logged as hazard d1-1 but did not log it again. |
| x_no_takeover | explanatory | - | yes | yes | yes | yes | 5 | 1.02s | No lane takeover occurred because each drone completed all of its assigned lanes (8/8) and the verification... |
| x_area_split | explanatory | - | no | no | - | - | - | 2.87s | The records do not specify how the survey area was divided among the drones. |
| x_lane_spacing | explanatory | - | yes | yes | yes | yes | 5 | 1.03s | The lane spacing was derived from a detection swath of 4.99 m (camera footprint 23.71 m) at 10.0 m altitude... |
| u_battery_voltage | unanswerable | - | yes | no | - | - | - | 1.23s | Battery voltage at landing not recorded in the provided records. |
| u_wind | unanswerable | - | yes | no | - | - | - | 1.74s | Wind speed not recorded in the mission records. |
| u_gps_sats | unanswerable | - | yes | no | - | - | - | 2.15s | The records do not state the number of GPS satellites for drone 2 during the survey. |
| u_cpu_temp | unanswerable | - | yes | no | - | - | - | 1.74s | CPU temperature not recorded in the provided mission records. |
