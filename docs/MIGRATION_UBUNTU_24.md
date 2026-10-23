# Migration guide — Ubuntu 22.04 + ROS 2 Humble → Ubuntu 24.04 + ROS 2 Jazzy

*For the `px4_ros_ws` landmine-survey swarm project (Maker Bhavan, IIT Gandhinagar).
Written 23 Oct 2026, to be pasted into a fresh Claude chat that will guide the
migration step by step.*

---

## 0. Read this first (for the assistant)

You are helping Sam move a **working** ROS 2 / PX4 / Gazebo drone-swarm project
from Ubuntu 22.04 + ROS 2 Humble to Ubuntu 24.04 + ROS 2 Jazzy. The project is
known-good on 22.04; the goal is to reach the **same verified results** on 24.04,
not to add features.

### The project in one paragraph

Three simulated PX4 drones (Gazebo Harmonic, `gz_x500_mono_cam_down`, 1280×960
downward camera) each run their own ROS 2 stack — `survey_node` (lawnmower lanes,
offboard control, takeover of a peer's band, separation) and `detector_node`
(YOLOv8 on GPU, geotags detections into a shared hazard list) — and coordinate
peer-to-peer over `/swarm/heartbeat` and `/swarm/hazards` (`swarm_msgs`). PX4 ↔ ROS 2
is the uXRCE-DDS bridge (Micro XRCE-DDS Agent on UDP 8888). An `analyst/` package
is a separate RAG "mission debrief" tool with its own venv.

### Paths — keep these IDENTICAL on the new install

The launch scripts hard-code them. User name must stay **`sam`**.

| Path | What |
|---|---|
| `~/px4_ros_ws` | the git repo (ROS 2 workspace) — `github.com/sameerverma-dot/px4-swarm-drone`, branch `main` |
| `~/PX4-Autopilot` | PX4 firmware + SITL (**has a local edit**, see §2) |
| `~/Micro-XRCE-DDS-Agent` | the DDS agent, built from source |
| `~/maps` | all flight outputs (`swarm_<stamp>/` run folders) |
| `~/runs`, `~/yolo_test` | YOLO training runs / experiments |
| `~/.gz/fuel` | cached Gazebo Fuel models (the "Standing person" target) |
| `~/Downloads/QGroundControl*.AppImage` | QGC (optional) |

### Hard-won lessons — do not re-learn these

1. **`GZ_IP=127.0.0.1`** must be set for every `gz` command and every standalone
   PX4 instance. Without it gz-transport *discovers* topics but delivers **no data**
   (a drone boots with "No valid data from Accel 0"). `tools/start_px4_swarm.sh`
   already sets it.
2. **PX4 publishes on versioned topics** (`/fmu/out/vehicle_local_position_v1`,
   `vehicle_status_v4`). Unversioned names carry nothing.
3. **A listed topic is not a published topic.** Check data, not names:
   `ros2 topic echo /fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --once`
   (`/fmu/out/*` is BEST_EFFORT; a RELIABLE subscriber hears nothing).
4. **`px4_msgs` must match the PX4 firmware commit**, or topics go silent.
5. **`setuptools` must be < 80** for `colcon build` of the Python packages.
6. **pytest + ROS `launch_testing` plugin clash**: run tests as
   `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest …`.
7. **Stop the sim with `bash ~/px4_ros_ws/tools/stop_sim.sh`**, not `tmux kill-server`
   (that leaves PX4 and Gazebo running).
8. **One mission launch at a time** — two launches share `/swarm/hazards`.

### Rules for this migration

- Change **one thing at a time**. Keep the **same PX4 commit** as on 22.04; upgrading
  PX4 is a separate, later step.
- Work on a branch: `git checkout -b jazzy-migration`. Merge to `main` only after
  every gate in §10 passes.
- **Never read, print, paste or commit `~/.git-credentials`** or any token.
- Never `git push --force` to `main`.
- When something fails, ask for the exact error text before guessing.

### Docs inside the repo worth reading once the repo is cloned

`docs/NEXT_SESSION.md` (current state + pre-flight checklist), `docs/SYSTEM_GUIDE.md`
(how it works, §3.6 run command and measured numbers), `docs/CHEATSHEET.md`
(every command and where to run it), `docs/PROGRESS.md` §11–12 (the 3-drone swarm
and its calibration results).

---

## 1. Choose the install method

| Method | Verdict |
|---|---|
| **In-place `do-release-upgrade`** | **Don't.** ROS Humble packages have no 24.04 builds; the upgrade disables the ROS repo and leaves a half-broken ROS install to clean up. |
| **Clean install over 22.04** | Fine **only after** the backup in §2 is copied off this disk and checked. Simplest. |
| **Dual boot (24.04 on a new partition)** | Safest: 22.04 stays as a known-good fallback to compare against. Needs ~100 GB free. |

Everything in git is safe on GitHub. What is **not** in git and would be lost:
flight data (`~/maps` — the calibration evidence for the report), trained weights
(`~/runs`), the PX4 camera edit, the exact `px4_msgs`/`px4_ros_com` commits, the Fuel
cache, and your GitHub login.

---

## 2. Phase 0 — back up (on 22.04, before anything else)

```bash
# where things stand
df -h / /home
lsblk -o NAME,SIZE,FSTYPE,MOUNTPOINT
nvidia-smi | head -15

# make sure all code is on GitHub
cd ~/px4_ros_ws
git fetch origin && git status && git branch -vv     # want: clean, no [ahead]

mkdir -p ~/migration_backup && cd ~/migration_backup

# PX4: exact commit + the local camera edit (mono_cam model.sdf -> 1280x960)
cd ~/PX4-Autopilot
git rev-parse HEAD                      > ~/migration_backup/px4_commit.txt
git diff                                > ~/migration_backup/px4_local_changes.patch
git status --short                      > ~/migration_backup/px4_status.txt
git ls-files --others --exclude-standard > ~/migration_backup/px4_untracked.txt
git submodule status                    > ~/migration_backup/px4_submodules.txt

# vendored ROS packages (gitignored in the repo) - exact commits
(cd ~/px4_ros_ws/src/px4_msgs    && git rev-parse HEAD) > ~/migration_backup/px4_msgs_commit.txt
(cd ~/px4_ros_ws/src/px4_ros_com && git rev-parse HEAD) > ~/migration_backup/px4_ros_com_commit.txt

# DDS agent version
(cd ~/Micro-XRCE-DDS-Agent && git describe --tags --always) > ~/migration_backup/xrce_agent_version.txt

# Python packages and ROS packages you have now (for reference)
pip freeze --user           > ~/migration_backup/pip_user.txt
dpkg -l | grep ros-humble   > ~/migration_backup/ros_humble_packages.txt
cp ~/.bashrc                  ~/migration_backup/bashrc_22.04
cp ~/px4_ros_ws/yolov8n.pt    ~/migration_backup/

# check the patch really contains the camera change
grep -n "width\|height" ~/migration_backup/px4_local_changes.patch

du -sh ~/maps ~/runs ~/yolo_test ~/.gz/fuel ~/migration_backup
```

Then copy **`~/migration_backup`, `~/maps`, `~/runs`, `~/yolo_test`, `~/.gz/fuel`**
to a USB drive or cloud storage, and open a couple of files from the copy to prove
it is readable. If `px4_untracked.txt` lists anything you created (models,
worlds), copy those files too.

**GitHub login:** your token lives in `~/.git-credentials`. On the new system you
will authenticate again (new personal access token, or SSH key). Do not copy the
token into any chat.

---

## 3. Phase 1 — install Ubuntu 24.04 LTS

1. Download the latest **Ubuntu 24.04.x LTS Desktop** ISO, write it to a USB stick
   (Startup Disk Creator or balenaEtcher).
2. Boot from USB (HP OMEN: **F9** boot menu). Keep Secure Boot on.
3. Installer choices:
   - Dual boot: "Install alongside" or manual partitioning into the free space.
     Clean install: "Erase disk" **only after §2 is verified**.
   - **Tick "Install third-party software for graphics and Wi-Fi"** (NVIDIA driver).
   - User name **`sam`**.
4. If asked to set a **Secure Boot / MOK password**, remember it — on the next
   reboot choose **Enroll MOK** and enter it, or the NVIDIA driver will not load.
5. After first boot:

```bash
sudo apt update && sudo apt full-upgrade -y
nvidia-smi                       # must show the RTX 4060
# if nvidia-smi fails:
sudo ubuntu-drivers install && sudo reboot
prime-select query               # 'on-demand' is fine
```

6. Reinstall the **Claude desktop app** and reconnect folders if you want Claude
   to check files directly.

---

## 4. Phase 2 — base tools + ROS 2 Jazzy

```bash
sudo apt install -y git curl build-essential cmake tmux python3-pip python3-venv \
                    software-properties-common
sudo add-apt-repository -y universe

# locale (ROS needs UTF-8)
locale        # if not UTF-8:
sudo apt install -y locales && sudo locale-gen en_US en_US.UTF-8 && \
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
```

Add the ROS 2 apt source **using the current method from the official Jazzy
install page** (docs.ros.org → Jazzy → Installation → Ubuntu (deb packages)). As
of 2025–26 it is the `ros2-apt-source` package:

```bash
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
```

*(If the official page shows a different method, follow the page.)*

```bash
sudo apt install -y ros-jazzy-desktop ros-dev-tools \
                    ros-jazzy-ros-gz        # Gazebo Harmonic + ros_gz_bridge for Jazzy
sudo rosdep init || true
rosdep update

echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source ~/.bashrc
echo $ROS_DISTRO                 # jazzy
ros2 run demo_nodes_cpp talker   # Ctrl-C after a few lines
```

---

## 5. Phase 3 — PX4 at the SAME commit

```bash
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd ~/PX4-Autopilot
git checkout $(cat <backup>/px4_commit.txt)      # was 9eafff1a4c (v1.18.0-beta1-314, main)
git submodule update --init --recursive

bash ./Tools/setup/ubuntu.sh     # installs Gazebo Harmonic + PX4 deps
# sim only and short on disk?  bash ./Tools/setup/ubuntu.sh --no-nuttx
#   (skips the ARM toolchain; you will want it later to build Pixhawk firmware)
sudo reboot
```

If `ubuntu.sh` stops with **"externally-managed-environment"** (Python 3.12 /
PEP 668), report the exact output before forcing anything.

Re-apply the camera edit, then build and fly one drone by hand:

```bash
cd ~/PX4-Autopilot
git apply --check <backup>/px4_local_changes.patch && git apply <backup>/px4_local_changes.patch
git diff --stat                                     # mono_cam/model.sdf changed
grep -n "<width>\|<height>" Tools/simulation/gz/models/mono_cam/model.sdf   # 1280 / 960

make px4_sitl gz_x500_mono_cam_down                 # first build takes a while
# in the pxh> console:  commander takeoff   then   commander land
```

---

## 6. Phase 4 — Micro XRCE-DDS Agent

Use the same version as before (`xrce_agent_version.txt`; PX4 docs use the v2.4.x
line):

```bash
cd ~
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
git checkout $(cat <backup>/xrce_agent_version.txt)
mkdir build && cd build && cmake .. && make -j$(nproc)
sudo make install && sudo ldconfig /usr/local/lib/
MicroXRCEAgent --help | head -3
```

The build downloads Fast-DDS during `cmake`; a network error there is a download
problem, not a code problem.

---

## 7. Phase 5 — QGroundControl (optional)

24.04 has a new enough glibc for **QGC v5**.

```bash
sudo usermod -a -G dialout $USER
sudo apt remove -y modemmanager
sudo apt install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl libfuse2t64
# download QGroundControl*.AppImage into ~/Downloads, then:
chmod +x ~/Downloads/QGroundControl*.AppImage
```

(`libfuse2t64` is 24.04's name for `libfuse2`; AppImages need it.) Log out/in once
for the `dialout` group.

---

## 8. Phase 6 — the workspace

```bash
cd ~
git clone https://github.com/sameerverma-dot/px4-swarm-drone.git px4_ros_ws
cd ~/px4_ros_ws
git checkout -b jazzy-migration

# vendored packages, at the recorded commits
cd ~/px4_ros_ws/src
git clone https://github.com/PX4/px4_msgs.git    && (cd px4_msgs    && git checkout $(cat <backup>/px4_msgs_commit.txt))
git clone https://github.com/PX4/px4_ros_com.git && (cd px4_ros_com && git checkout $(cat <backup>/px4_ros_com_commit.txt))

# anything hard-coded to the old system?
cd ~/px4_ros_ws
grep -rn "humble\|python3\.10" --include=*.sh --include=*.py --include=*.xml --include=*.md . \
     | grep -v "^./src/px4_" | grep -v "^./build\|^./install\|^./log"
```

Fix every code hit (`/opt/ros/humble` → `/opt/ros/jazzy`, `python3.10` paths →
`python3.12`, or better, derive them at runtime). Doc hits can wait until §11.

```bash
cd ~/px4_ros_ws
rosdep install --from-paths src --ignore-src -r -y
cp <backup>/yolov8n.pt ~/px4_ros_ws/
# restore data
cp -r <backup-drive>/maps ~/maps
cp -r <backup-drive>/fuel ~/.gz/fuel      # or let Gazebo re-download the model
```

**Do not `colcon build` yet** — set up Python first (§9), otherwise the setuptools
version may be wrong.

---

## 9. Phase 7 — Python ML stack (the likeliest snag)

Ubuntu 24.04 refuses plain `pip install` into the system Python (PEP 668). ROS
nodes run under `/usr/bin/python3`, so the detector's `ultralytics` and `torch`
must be visible to that interpreter **without** replacing the system numpy that
ROS Jazzy's packages are built against.

Approach: one venv that can see the system packages, put on `PYTHONPATH` after
ROS is sourced.

```bash
python3 -m venv --system-site-packages ~/ros_venv
source ~/ros_venv/bin/activate
python -c "import numpy; print(numpy.__version__)"     # system numpy, 1.26.x on 24.04

# keep numpy at the system's 1.x and setuptools < 80
pip install "numpy<2" "setuptools<80" torch torchvision ultralytics matplotlib
deactivate

echo 'export PYTHONPATH="$HOME/ros_venv/lib/python3.12/site-packages:$PYTHONPATH"' >> ~/.bashrc
source ~/.bashrc

# verify under the SYSTEM interpreter, which is what ROS nodes use
/usr/bin/python3 -c "import rclpy, numpy, cv2, torch, ultralytics; print(numpy.__version__, torch.__version__, torch.cuda.is_available())"
```

Want: numpy 1.26.x, `True` for CUDA. If pip refuses the combination (a package
demanding numpy ≥ 2), stop and report the exact error — do not drop the pin
blindly, because a numpy 2 on `PYTHONPATH` can break ROS's compiled message
packages.

Now build:

```bash
cd ~/px4_ros_ws
python3 -c "import setuptools; print(setuptools.__version__)"   # < 80
colcon build
source install/setup.bash
```

`px4_msgs` takes several minutes. Then the analyst tool, in its own venv:

```bash
cd ~/px4_ros_ws
python3 -m venv analyst/.venv
source analyst/.venv/bin/activate
pip install -r analyst/requirements.txt
deactivate
```

---

## 10. Phase 8 — verification gates (all must pass before merging)

These are the results the system already achieves on 22.04.

| # | Gate | Command | Pass |
|---|---|---|---|
| 1 | Swarm unit tests | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/survey/test src/perception/test/test_hazard_registry.py -q` | **44 passed** |
| 2 | Analyst tests | `source analyst/.venv/bin/activate && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest analyst/tests -q; deactivate` | **35 passed** |
| 3 | Single drone stack | T1: `bash ~/px4_ros_ws/start_px4_sim.sh gz_x500_mono_cam_down` · T2: `bash tools/check_system.sh` | **0 FAIL** |
| 4 | Single-drone mission | `bash tools/add_target.sh` then `ros2 launch survey mission.launch.py x_max:=30.0 y_max:=20.0 altitude:=10.0` | `VERIFY PASS`, target found, ≲1 m |
| 5 | 3-drone stack | `bash tools/stop_sim.sh` · `NUM_DRONES=3 Y_MIN=0 Y_MAX=90 bash tools/start_px4_swarm.sh` · `bash tools/check_system.sh` | every drone `sensors OK`, **0 FAIL** |
| 6 | 3-drone mission | `NUM_DRONES=3 Y_MIN=0 Y_MAX=90 bash tools/add_swarm_targets.sh` · `ros2 launch survey swarm_mission.launch.py num_drones:=3 x_max:=30.0 y_min:=0.0 y_max:=90.0 altitude:=10.0` | three `VERIFY PASS`, one hazard per target, 0 duplicates |
| 7 | Calibration flight | the 11-target layout from `docs/PROGRESS.md` §12.3 (run command in `docs/SYSTEM_GUIDE.md` §3.6) | **11/11, 0 duplicates, 0.09–0.29 m** (22.04 baseline) |

Run each line on its own; `ros2 launch` blocks until the flight ends. Before gate
5–6 also confirm instance 1 actually publishes:
`ros2 topic echo /px4_1/fmu/out/vehicle_local_position_v1 --qos-reliability best_effort --once`.

If gate 7's errors are clearly worse than 0.3 m, the likeliest causes are timing
(`pose_lag_s`) or frame rate — compare the detector's 5-second stats line
(`camera … fps, inferred … fps, pose … Hz`) with the 22.04 values in PROGRESS §12.

---

## 11. Phase 9 — record it and merge

1. Update docs: `docs/STACK_README.md` and `docs/CHEATSHEET.md` (Jazzy, Python venv,
   `PYTHONPATH` line, QGC v5), `README.md` status, and a new `docs/PROGRESS.md`
   section "Ubuntu 24.04 / Jazzy migration" with the gate results and anything
   that had to change.
2. Add `analyst/.venv/` to `.gitignore` if not already there.
3. Commit and push the branch, open a pull request, merge into `main` once all
   gates pass:

```bash
cd ~/px4_ros_ws
git status                         # read the list: no .venv, no tokens, no build/
git add -A
git commit -m "Migrate to Ubuntu 24.04 + ROS 2 Jazzy"
git push -u origin jazzy-migration
```

4. Run `bash tools/make_working_doc.sh` once (it reinstalls the post-commit hook
   that refreshes `working_doc/` on every commit).

---

## 12. Troubleshooting — symptoms seen in this project

| Symptom | Likely cause | Check / fix |
|---|---|---|
| `bash: install/setup.bash: No such file` | not in `~/px4_ros_ws` | `cd ~/px4_ros_ws` first |
| `message type 'px4_msgs/...' is invalid` | workspace not sourced, or `px4_msgs` not built | `source install/setup.bash`; rebuild `px4_msgs` |
| Topic listed but `echo --once` hangs | wrong QoS, or no data behind the writer | use `--qos-reliability best_effort`; check PX4 log for sensor failures |
| A drone: "No valid data from Accel 0", "Arming denied" | `GZ_IP` not set on that PX4 instance | launcher sets `GZ_IP=127.0.0.1`; check `gz topic -e -t /world/default/model/x500_mono_cam_down_1/link/base_link/sensor/imu_sensor/imu -n 1` with `GZ_IP=127.0.0.1` |
| Camera topic exists, no frames in ROS | `GZ_IP` mismatch on the bridge | `docs/CAMERA_DIAGNOSTIC.md` |
| `colcon build` Python package errors | setuptools ≥ 80 visible | `python3 -c "import setuptools; print(setuptools.__version__)"`; pin `<80` in `~/ros_venv` |
| pytest `PluginValidationError ... launch_testing` | pytest too new for ROS plugin | `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` |
| `ModuleNotFoundError: ultralytics/torch` in a node | `PYTHONPATH` line missing in that terminal | `source ~/.bashrc`; re-check §9 import line |
| numpy / `_ARRAY_API` / "compiled using NumPy 1.x" errors | numpy 2 shadowing system numpy | `pip install "numpy<2"` in `~/ros_venv` |
| PX4 and Gazebo still running after stopping | `tmux kill-server` used | `bash tools/stop_sim.sh` |
| Mission fails arm/offboard on one drone | that instance blind or not booted | the PX4 log in `log/swarm_launch_*/px4_sitl_<i>.log` |
| `ros2 topic hz ... --qos-reliability` "unrecognized arguments" | flag only exists on `echo` in some distros | use `ros2 topic echo … --once` |

---

## 13. Rollback

- **Dual boot:** reboot into 22.04 — nothing there was touched.
- **Clean install:** reinstall 22.04, ROS Humble, and restore from §2's backup; the
  code comes back from GitHub `main`, which is unchanged until the migration branch
  is merged.
