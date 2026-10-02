# Autonomous Swarm Drone System — Landmine Detection & Mapping (Phase I)

Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali

A swarm of drones (2–5) autonomously surveys an area, detects suspected
landmines from a downward camera, and produces a geotagged hazard map.

**Status:** a **decentralised 3-drone swarm** is flight-verified in simulation
(2 Oct): every drone runs its own survey, detector and logs and needs no ground
station; drones exchange heartbeats and hazard reports peer to peer; a drone
that returns early or goes silent has its lanes taken over by a neighbour;
detections are de-duplicated drone-to-drone; peers keep separation. Geotags land
0.09–0.29 m from truth (11 of 11 targets, calibrated from flight data).
Remaining on software: landmine weights in place of the COCO `person` stand-in,
and terrain following. **Hardware (Track B) is 50 % of the grade and is at
zero** — the project's largest risk. Details in `docs/PROGRESS.md` §11–12.

The governing constraints: detection needs ~24 px on target, which caps flight
altitude at **5.6 m** (640 px capture) or **11.2 m** (1280 px); and the detector
finds a target reliably only within ~2.5 m of the track at 10 m (measured), so
lanes are 4 m apart even though the camera sees 24 m across. Altitude and the
model's swath set lane spacing, which sets flight time, which sets what the
swarm buys — read `docs/SWARM_PLAN.md` and `docs/SYSTEM_GUIDE.md` §3.6 before
changing any of them.

---

## Run it

**One drone:**

```bash
# Terminal 1 — the simulator (plain terminal, NOT inside tmux)
bash ~/px4_ros_ws/start_px4_sim.sh gz_x500_mono_cam_down

# Terminal 2 — everything else, one line at a time
cd ~/px4_ros_ws && source install/setup.bash
bash tools/check_system.sh                  # six layers, PASS/FAIL — want 0 failed
bash tools/add_target.sh                    # the target is NOT in the world file
ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=10.0
ros2 run perception hazard_map --area 0,30,0,20 --truth 10,15
```

**The swarm (3 drones by default):**

```bash
# Terminal 1
bash ~/px4_ros_ws/tools/start_px4_swarm.sh --num-drones 3 --y-min 0 --y-max 90

# Terminal 2 — one line at a time
cd ~/px4_ros_ws && source install/setup.bash
bash tools/check_system.sh                  # probes EVERY drone — want 0 failed
NUM_DRONES=3 Y_MIN=0 Y_MAX=90 bash tools/add_swarm_targets.sh   # one target PER BAND
ros2 launch survey swarm_mission.launch.py num_drones:=3 y_min:=0.0 y_max:=90.0
ros2 run perception hazard_map --truth "15,15;15,45;15,75"      # newest run, no other args
```

Each drone runs its own camera bridge, detector and survey node
(`onboard.launch.py` — what each onboard computer would run). They talk only to
each other, over `/swarm/heartbeat` and `/swarm/hazards`. The ground station
that `swarm_mission` also starts is a passive monitor: kill it mid-flight and
nothing changes. Each run writes everything to one folder,
`~/maps/swarm_<stamp>/`.

What the swarm does on its own:

| Capability | What happens |
|---|---|
| Onboard autonomy | Flies its band, detects, logs, returns — no ground link needed |
| Band takeover | A peer that returns early, or goes silent past its own projected finish, has its unfinished lanes flown by the nearest free drone |
| Shared hazard list | Detections are broadcast drone-to-drone; an object a peer already logged (e.g. on a band boundary) is not logged again |
| Separation | If a peer comes within 8 m / 5 m, the drone without right of way (working > waiting > lower id; a drone under PX4 control always has it) moves away horizontally at its own altitude; it passes over or under only a peer that holds its altitude and stays in the way |
| Geotagging | Attitude-aware projection, frames from turns and lane ends excluded; 0.09–0.29 m against 11 known targets, a pair 2 m apart kept as two (`docs/PROGRESS.md` §12) |

Try the failure cases (fault injection, per drone):

```bash
ros2 launch survey swarm_mission.launch.py abort_after_lanes:="-1;-1;1"   # drone 2 goes home after 1 lane
ros2 launch survey swarm_mission.launch.py start_delay:="0;100;0"         # drone 1 late -> separation yield
```

`num_drones`, `y_min` and `y_max` must be identical for `start_px4_swarm.sh`,
`add_swarm_targets.sh` and `swarm_mission.launch.py` — they compute the band
geometry independently. Full runbooks: `docs/CHEATSHEET.md` §5.

The survey nodes exit on their own after `VERIFY`; the detectors keep running
(Ctrl-C the launch when done).

Stop everything: `bash ~/px4_ros_ws/tools/stop_sim.sh` — **not** `tmux
kill-server`, which leaves PX4 and Gazebo running.

---

## Where things are

```
├── start_px4_sim.sh   THE entry point — kept at root on purpose
├── docs/              everything to read
├── tools/             everything to run that isn't a ROS node
├── experiments/       one-off measurements, kept for the report
├── src/               the ROS 2 packages — survey/ and perception/
└── build/ install/ log/    generated by colcon; never edit, never commit
```

Outputs land in `~/maps/`: `survey_track_*.csv` (flown path),
`hazard_points.csv` (detections), `hazard_map_*.png` + `.geojson` (the
deliverable).

---

## Which doc do I want?

| I want to… | Read |
|---|---|
| remember where something is, or which terminal to run it in | `docs/CHEATSHEET.md` |
| run the stack, or look up a launch argument | `docs/STACK_README.md` |
| understand **how** any of this works | `docs/SYSTEM_GUIDE.md` |
| know what's done, what's broken, what's next | `docs/PROGRESS.md` |
| pick up where the last session stopped | `docs/NEXT_SESSION.md` |
| decide the swarm design — altitudes, drone count, resolution | `docs/SWARM_PLAN.md` |
| check deliverables and the grading split | `docs/PHASE1_ROADMAP.md` |
| debug a camera that shows a topic but no data | `docs/CAMERA_DIAGNOSTIC.md` |

New here, or coming back after a break? `docs/SYSTEM_GUIDE.md` top to bottom
once, then `docs/CHEATSHEET.md` as a reference.

---

## Three things that will bite you

1. **Spawn targets after every sim start** — `tools/add_target.sh` for one
   drone, `tools/add_swarm_targets.sh` for a swarm. The target is not in the
   world file; a model inserted through the Gazebo GUI dies with that sim
   process. One flight logged `VERIFY PASS`, flew perfect lanes over the target
   position, and found nothing, because there was nothing there. The 12 Sep
   2-drone flight hit the swarm version of this: one target, in one band, so
   half the swarm was never tested. Use the swarm spawner.
2. **A listed topic is not a published topic.** `ros2 topic list` counts
   subscribers too. Check with
   `ros2 topic echo <topic> --qos-reliability best_effort --once` — `echo`, not
   `hz`, because in Humble only `echo` accepts that flag, and `/fmu/out/*`
   publishes BEST_EFFORT, so a default RELIABLE subscription receives nothing
   and a live topic looks dead. Note also that this build publishes on the
   **versioned** names (`/fmu/out/vehicle_local_position_v1`).
3. **Rebuild after editing.** `tools/check_system.sh` reports a stale build; a
   stale build is the most common reason a fix appears not to work.
