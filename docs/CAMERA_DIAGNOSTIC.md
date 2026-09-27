# Gazebo camera "no frames" — SOLVED

**Verdict: the camera was never broken.** It rendered correctly in every run.
The frames never reached external subscribers (`gz topic -e`, `ros_gz_bridge`)
because of a **gz-transport network-address mismatch**, not a GPU/render fault.

An earlier version of this document blamed EGL/GPU rendering. **That was
wrong.** The server logs disprove it. Corrected analysis below.

---

## 1. Proof the camera works

From `~/.gz/sim/log/<run>/server_console.log` — present in **every** run that
used `x500_mono_cam_down` (verified in the 00:49 and 01:16 runs):

```
[Sensors.cc:349]     Initializing render context
[Sensors.cc:391]     Rendering Thread initialized              <-- render context OK
[CameraSensor.cc:405] Camera images for [...camera] advertised on [...]
[CameraSensor.cc:504] Enabling camera sensor: '...camera' data generation.   <-- RENDERING
[GstCameraSystem.cpp:281] Camera info: 1280x960               <-- real frames read
[GstCameraSystem.cpp:475] GStreamer pipeline started, streaming to 127.0.0.1:5600
```

PX4's in-process `GstCameraSystem` **successfully consumed the camera frames
and streamed them as H.264 to UDP 5600** for 15+ minutes. Rendering, the
Sensors system, the model and the sensor are all fine.

### Why the earlier EGL theory was wrong

`~/.gz/rendering/ogre2.log` shows Ogre probing all 4 EGL devices at startup:

```
Found Num EGL Devices: 4
 #0 /dev/dri/card2 (NVIDIA)      -> Created GL 4.5 context      OK
 #1 /dev/dri/card1               -> Created GL 4.5 context      OK
 #2 /dev/dri/card2               -> eglInitialize failed         <-- the scary line
 #3 EGL_MESA_device_software     -> Created GL 4.5 context      OK
GL_RENDERER = NVIDIA GeForce RTX 4060 Laptop GPU
```

The `eglInitialize failed` / `failed to create dri2 screen` messages come from
Ogre **enumerating and testing every device**. Three of four succeeded, one
failed, and Ogre proceeded normally on a working one. Those warnings are
**noise, not the fault** — a textbook red herring.

---

## 2. The actual root cause

PX4 starts the Gazebo server with `GZ_IP=127.0.0.1`. It is visible in
`px4_sitl.log`, in the build rule PX4 uses to launch:

```
cmake -E env PX4_SIM_MODEL=gz_x500 GZ_IP=127.0.0.1 .../bin/px4
```

gz-transport works in two stages:

| Stage | Mechanism | With mismatched GZ_IP |
|---|---|---|
| **Discovery** | UDP multicast (239.255.0.7:11317) | ✅ works — so `gz topic -l` **lists** the topic |
| **Data** | direct ZeroMQ connection to the publisher's *advertised address* | ❌ fails — publisher advertises `127.0.0.1`, a subscriber without `GZ_IP` resolves a different interface |

So an external process **sees the topic and receives nothing** — silently, with
no error on either side. That is exactly the observed symptom.

This also explains why PX4's own `GstCameraSystem` worked perfectly: it runs
**inside the server process**, so it inherits `GZ_IP=127.0.0.1` and never
crosses the boundary.

### Every observation now fits

| Observation | Explained by |
|---|---|
| `gz topic -l` lists the camera topic | multicast discovery works |
| `gz topic -e` prints nothing | data connection never established |
| `ros2 topic hz` on the bridged topic: nothing | same — bridge had no `GZ_IP` |
| GStreamer streamed video fine | in-process, same env |
| GUI renders fine | irrelevant, but also fine |
| dri2 / eglInitialize warnings | device-probe noise, not fatal |
| IMU / GPS / baro reach PX4 fine | in-process too |

---

## 3. The fix

**Any process that subscribes to a Gazebo topic must use the same `GZ_IP`.**

### Quick test (one line, proves it immediately)

With the `gz_x500_mono_cam_down` sim running:

```bash
GZ_IP=127.0.0.1 gz topic -e -t /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image
```

Expect an immediate flood of image data (1280×960 → ~3.7 MB per message, so
`Ctrl-C` quickly). Without `GZ_IP` it stays blank; with it, data flows.

### Permanent fix — already applied

`~/px4_ros_ws/src/perception/launch/perception.launch.py` now sets
`additional_env={'GZ_IP': '127.0.0.1'}` on **both** the `ros_gz_bridge` and the
detector node, and the broken topic remap has been removed (the detector now
subscribes to exactly the name the bridge publishes).

```bash
cd ~/px4_ros_ws && colcon build --packages-select perception && source install/setup.bash
ros2 launch perception perception.launch.py
```

### For any ad-hoc shell

```bash
export GZ_IP=127.0.0.1     # put this in ~/.bashrc if you use gz CLI often
```

---

## 4. Bonus: see the camera right now, with zero setup

PX4 is *already* streaming the camera as H.264 to `udp://127.0.0.1:5600`.
QGroundControl can display it:

**QGC → Application Settings → Video → Source = "UDP h.264 Video Stream",
Port = 5600.**

The live downward camera view appears in QGC's video widget. Useful for the
demo, and independent of ROS 2.

---

## 5. Lessons for the project log

1. **A loud warning is not automatically the cause.** The `dri2` message was
   prominent and plausible, and cost hours. The quiet `server_console.log` had
   the truth all along.
2. **`~/.gz/sim/log/<timestamp>/server_console.log` is the highest-value file**
   when anything Gazebo-side misbehaves. Check it *first*. `~/.gz/rendering/ogre2.log`
   is the render engine's own log.
3. **"Topic listed but no data" in gz-transport ⇒ suspect `GZ_IP`/`GZ_PARTITION`
   mismatch**, not rendering. Discovery and data transport are separate paths.
4. PX4 ships its own server plugin config at
   `~/PX4-Autopilot/src/modules/simulation/gz_bridge/server.config` (not
   `/usr/share/gz/...`, not `~/.gz/sim/8/`). That file is where the sensor
   render engine (`ogre2`) is set, if it ever *does* need changing.
