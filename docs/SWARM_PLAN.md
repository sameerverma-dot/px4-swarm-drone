# Swarm plan — the numbers before the code

*6 Sep 2026. Written while the machine was offline, from measured parameters
only. Reproduce with `experiments/coverage_model.py`.*

Every input here came from this project's own flights or experiments — no
guessed constants except one, which is flagged.

---

## 1. Detection sets the ceiling, not the airframe

A person is ~0.5 m across seen from directly above, and YOLO needs ~24 px on
target (measured, `experiments/px_sweep.py`). That fixes a hard maximum altitude:

| capture | max altitude | footprint | lane spacing (30 % sidelap) |
|---|---|---|---|
| 640 px | **5.6 m** | 13.3 × 10.0 m | 9.3 m |
| 1280 px | **11.2 m** | 26.7 × 20.0 m | 18.7 m |

The 5 Sep flight at 5.0 m sat just under the 640 ceiling. The 3 Sep flight at
10 m was double it, and found nothing — not a bug, arithmetic.

**Everything downstream follows from this number.** Altitude sets lane spacing,
lane spacing sets flight time, flight time sets how much the swarm buys.

---

## 2. One shared detector across 3 drones is fine

I previously worried that splitting one detector across N drones would starve
them. **The numbers do not support that worry.**

| drones | imgsz | FPS each | frame gap | footprint depth | ratio |
|---|---|---|---|---|---|
| 1 | 640 | 11.0 | 0.35 m | 10.0 m | 0.03 |
| 3 | 640 | 3.7 | 1.04 m | 10.0 m | 0.10 |
| 3 | 1280 | 1.0 | 3.96 m | 20.0 m | 0.20 |

Even the worst case has **5× redundancy** along-track. Inference throughput is
nowhere near binding. So the shared-detector design is chosen on its merits —
one model in memory, no GPU contention, and correct deduplication because one
node owns one hazard list — not as a compromise.

---

## 3. Resolution is worth about one extra drone

Minutes to clear 1 hectare:

| | 1 drone | 2 drones | 3 drones |
|---|---|---|---|
| 640 px @ 5.6 m | 6.1 | 3.5 | 2.5 |
| 1280 px @ 11.2 m | 3.8 | 2.1 | 1.6 |

**One 1280 drone (3.8 min) ≈ two 640 drones (3.5 min)**, within 9 %. Not more —
I over-claimed that in an earlier draft and the arithmetic corrected me. Three
640 drones (2.5 min) still beat one 1280 drone.

But the two are not equal in *cost*: the resolution change is two numbers in
`mono_cam/model.sdf` plus an `imgsz` parameter. The second drone is days of
work. And they compose — 3 drones at 1280 is 1.6 min, 3.8× better than where
you are now.

**Sequencing consequence: change the camera BEFORE building the swarm.** The
swarm's area-splitting depends on lane spacing, which depends on altitude,
which depends on resolution. Do it after and you re-tune everything.

Area per 15-minute sortie: **3.2 ha** per drone at 640, **6.4 ha** at 1280.

---

## 4. Your test arena is too small to demo the swarm

At 1280/11.2 m, a 30 × 20 m area needs only 2 lanes — so 2 drones and 3 drones
finish at the same time. The parallelism has nothing to divide.

For a demo that actually *shows* the swarm working, either:

- fly at 640/5.6 m, where 3 drones do show a clear speedup over 1, or
- enlarge the survey area to at least ~60 × 60 m.

Worth deciding before you build, because it changes what you demonstrate.

---

## 5. The speed knob, priced

| speed | geotag error | 30×20 time | ha / sortie |
|---|---|---|---|
| 3.8 m/s | 0.75 m | 46 s | 3.2 |
| 6.0 m/s | ~1.6 m | 32 s | 5.0 |
| 9.2 m/s | ~4.0 m | 23 s | 7.7 |

Geotag error is latency × speed, so it scales linearly. 3.8 m/s was chosen for
accuracy. If a demo needs to be shorter, this is the knob — and now the cost is
stated rather than discovered afterwards.

---

## 6. One honest caveat

The coverage model has a single fitted constant: `turn_penalty_s = 2.3`,
tuned so the model reproduces the 5 Sep flight's 45 s. It therefore matches
that flight exactly, **which proves nothing** — it is a fit, not a prediction.

Lane geometry and path length are independent of that fit and are sound. To
actually validate, fly a *different* area (40 × 30, say) and compare against the
model. Until then, treat all times here as ±20 %.

---

## 7. Recommended order

1. **Camera to 1280 + `imgsz` to match.** Cheapest large win, and it must
   precede the swarm so the geometry is settled. Verify a real detection still
   lands near the known target.
2. **Second PX4 instance**, `-i 1`. Verify the `/px4_1/...` namespace with
   `ros2 topic hz` — do not assume it.
3. **Namespace-parameterise `survey_node`**, split the area along East.
4. **One detector, N camera topics, one hazard list.**
5. **Third drone** — should be a loop change only.
6. **Validate the coverage model** on a differently-sized area while you are
   flying anyway. Free data, and it turns section 6's caveat into a result.

Hardware (Track B) runs in parallel throughout — it is calendar time you do not
control, and it is still 50 % of the grade.
