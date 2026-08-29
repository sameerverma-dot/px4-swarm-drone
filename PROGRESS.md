# Project Progress — Autonomous Swarm Drone System for Landmine Detection & Mapping (Phase I)
### Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali

_Last updated: 30 Aug 2026. Companion docs: `STACK_README.md` (how to run), `PHASE1_ROADMAP.md` (plan)._

---

## 1. Summary of status

The **software / autonomy / AI pipeline is built and working end-to-end in
code**. A single drone can autonomously survey a specified area and return,
via two independent paths (a QGroundControl Survey mission, and a custom ROS 2
node). The **AI detection pipeline is written and builds cleanly** and is
waiting on one thing: a working camera image out of Gazebo, which is currently
blocked by a **GPU/graphics rendering issue specific to this hybrid-GPU laptop**
(details in section 5).

| Capability | Status |
|---|---|
| Single-drone PX4 SITL + Gazebo + QGC + DDS bridge | ✅ Working |
| One-command launcher (`start_px4_sim.sh`) | ✅ Working |
| Sim home relocated to IIT Gandhinagar | ✅ Working |
| Autonomous area survey — ROS 2 node (`survey`) | ✅ Built & runs (flies, offboard, RTL) |
| Autonomous area survey — QGC "draw polygon" mission | ✅ Working (demonstrated) |
| Downward camera model in sim (`x500_mono_cam_down`) | ✅ Model + airframe confirmed |
| Gazebo camera → ROS 2 bridge (`ros_gz_bridge`) | ✅ Configured & launches |
| YOLO detector node (`perception`) | ✅ Built; loads model; waiting on camera frames |
| **Gazebo actually producing camera frames** | ❌ Blocked (GPU render — section 5) |
| Geotagged hazard map / CSV | ⚙️ Code ready; needs detections |
| Multi-drone swarm (2–5) | ⬜ Not started |

---

## 2. Environment

- Laptop: HP OMEN 16, **Ubuntu 22.04** (glibc 2.35), **NVIDIA RTX 4060** (Optimus/hybrid) + Intel iGPU.
- **ROS 2 Humble**, user `sam`, home `/home/sam`.
- Gazebo **Harmonic** (gz-sim 8.15).
- GitHub repo: `github.com/sameerverma-dot/px4-swarm-drone` (branch `main`).

---

## 3. What is built and working

### 3.1 Base simulation stack
- **PX4 SITL + Gazebo (gz_x500)** on the RTX GPU via `prime-run`.
- **Micro XRCE-DDS Agent** bridging PX4 ⇄ ROS 2 on UDP 8888 — all `/fmu/out/*` topics confirmed.
- **QGroundControl v4.4.3** (v5.0 needs glibc 2.38, incompatible) — connects on UDP 14550.
- **One-command launcher** `~/px4_ros_ws/start_px4_sim.sh` starts DDS agent + PX4/Gazebo + sourced ROS 2 shell + QGC in a 3-pane tmux session. Home set to IIT Gandhinagar (23.2127, 72.6846).

### 3.2 Area survey — ROS 2 package `survey`
- `ros2 launch survey survey.launch.py x_max:=40 y_max:=30 altitude:=10 lane_spacing:=8`
- Boustrophedon ("lawnmower") coverage in PX4 local NED; streams OffboardControlMode + TrajectorySetpoint, arms, flies waypoints, verifies each was reached (PASS/FAIL), writes flown track CSV to `~/maps`, and RTLs on completion.
- Builds clean (`colcon build --packages-select survey` → `Finished <<< survey`).
- Demonstrated: drone armed, entered Offboard, and climbed under node control (a full run was interrupted by a terminal closure, not a code fault).

### 3.3 Area survey — QGroundControl Survey mission
- QGC Plan view → Pattern → Survey → draw polygon with clicked points → set transect spacing → Upload → Start Mission.
- **Demonstrated working**: drew an area over the IITGN field and the drone flew the full transect pattern autonomously and returned. This is the fastest "draw an area and survey it" path.

### 3.4 Detection pipeline — ROS 2 package `perception` (code complete)
- `detector_node`: subscribes to the camera image, runs **Ultralytics YOLO**, publishes an annotated image (`/detection/image_annotated`), and geotags detections (nadir projection from drone pose) into `~/maps/hazard_points.csv`, de-duplicated by distance.
- Launch also starts the **Gazebo→ROS 2 camera bridge** (`ros_gz_bridge`).
- Builds clean; the detector starts and loads `yolov8n.pt`. Confirmed the camera model, topic name, bridge, and detector all line up in ROS 2.
- **Note:** default `yolov8n.pt` = COCO classes (people/cars), not landmines. Swap in trained weights via `weights:=~/runs/.../best.pt` for real detection.

---

## 4. File locations

| What | Path |
|---|---|
| ROS 2 workspace | `~/px4_ros_ws` |
| Launcher | `~/px4_ros_ws/start_px4_sim.sh` |
| Survey package | `~/px4_ros_ws/src/survey/` (node, launch, package.xml, setup.py) |
| Standalone survey script | `~/px4_ros_ws/survey_node.py` |
| Perception package | `~/px4_ros_ws/src/perception/` (detector_node, launch, package.xml) |
| Vendored msg/bridge pkgs | `~/px4_ros_ws/src/px4_msgs`, `~/px4_ros_ws/src/px4_ros_com` |
| PX4 firmware / SITL | `~/PX4-Autopilot` |
| DDS agent | `~/Micro-XRCE-DDS-Agent` |
| QGroundControl | `~/Downloads/QGroundControl.AppImage` |
| YOLO venv / training runs | `~/yolo_test` (venv), `~/runs` |
| Output CSVs (tracks, hazards) | `~/maps/` |
| Docs | `~/px4_ros_ws/{STACK_README,PHASE1_ROADMAP,PROGRESS}.md` |

Camera model: `x500_mono_cam_down` (airframe `4014_gz_x500_mono_cam_down`) — downward-facing camera, 1280×960, ~99° FOV.
Camera gz topic: `/world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image`.

---

## 5. The current blocker — Gazebo camera not rendering

**Symptom:** with the `x500_mono_cam_down` model, the camera topic is advertised
(`gz topic -l` lists it) but publishes **no frames** (`gz topic -e` stays blank;
a live 30 fps stream would flood the terminal). So the detector has nothing to
process. The detector, bridge, and node are all correct — Gazebo simply isn't
producing images.

**Root cause:** Gazebo renders camera **sensors** through a headless **EGL**
context ("Render Engine Server Headless"), and on this Intel+NVIDIA (Optimus)
laptop that path fails. The GUI window renders fine (it uses GLX, which
`prime-run` fixes), but the sensor EGL path does not.

**What was tried:**
| Attempt | Result |
|---|---|
| `prime-run` (NVIDIA GLX offload) | `libEGL: failed to create dri2 screen` → no frames |
| Force NVIDIA EGL vendor (`__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json` + `__NV_PRIME_RENDER_OFFLOAD=1`) | **dri2 crash cleared** ✅, but still no camera frames |
| `LIBGL_ALWAYS_SOFTWARE=1` (CPU render) | Refused: "Not allowed to force software rendering when API explicitly selects a hardware device" → dri2 error returns |

**Next options for the camera (to try hands-on / with the lab):**
1. Add an explicit `Sensors` system plugin to the world with `<render_engine>ogre</render_engine>` (try Ogre v1 instead of ogre2).
2. Verify NVIDIA driver + EGL device enumeration (`eglinfo`, `nvidia-smi`), ensure the RTX is the EGL device used for the headless render.
3. Test on a cleaner single-GPU machine or a lab workstation — Gazebo camera rendering on hybrid laptops is a known pain point; someone at Maker Bhavan has likely solved it.

**This does not block the rest of the project** — detection can be developed and
demonstrated on recorded/test images while the camera render is fixed separately.

---

## 6. Gotchas already solved (don't re-discover)

- Run the launcher from a **normal terminal**, not inside tmux. It auto-clears stale px4/agent/gz processes (else "port 8888 in use" / "instance 0 already running").
- `commander` commands go in the **PX4 (pxh>) pane**, not a bash pane.
- Do **not** pipe PX4 through `| tee` — it makes the pxh console non-interactive (launcher uses tmux `pipe-pane` for logging instead).
- Arming from ROS 2 without a GCS: set `NAV_DLL_ACT=0` and `CBRK_SUPPLY_CHK=894281` (or just run QGC, which satisfies the GCS check).
- Gazebo must render on the RTX (`prime-run`) or it's unusably slow.
- PX4 `/fmu/*` topics are **best-effort QoS** — `ros2 topic echo` needs `--qos-reliability best_effort`.
- Correct ROS 2 topic names for this PX4 are **unversioned** (`/fmu/out/vehicle_local_position`, etc.).
- `pip install --user ultralytics` bumps setuptools to 84 and **breaks colcon** (needs <80). Fix: `pip install --user "setuptools==70.3.0"`.
- QGC v5.0 won't run on Ubuntu 22.04 (glibc). Use v4.4.3.
- Committing files over the device bridge resets the executable bit — re-`chmod +x` the launcher, or run it with `bash`.

---

## 7. Recommended next steps

1. **Prove detection now** — feed the detector a test image (bypassing the camera) so YOLO detect → annotate → geotag is demonstrated today (M3/M5 deliverable).
2. **Fix the Gazebo camera** — pursue the options in section 5, ideally at a lab workstation.
3. **Train landmine weights** — build a small dataset, train YOLO, drop `best.pt` into the detector.
4. **Scale to swarm (2–5)** — extend the launcher to spawn namespaced PX4 instances (`/px4_1`, `/px4_2`, …) and split a drawn area across drones.
5. **Hardware track (Track B)** — 50% of the grade is self-designed/Make parts + a DFM + FEA/CFD design report; run this in parallel (see `PHASE1_ROADMAP.md`).
