# Phase I Plan — Autonomous Swarm Drone System for Landmine Detection & Mapping
### Maker Bhavan Project Course · IIT Gandhinagar · Mentor: Aniruddh Mali · 14 weeks · team of 6

> Aligned to the official course brief: deliverables, milestones (M1–M7), and
> assessment weights. This replaces the earlier sim-only roadmap.

## The key realization: this is TWO tracks, not one

The simulation stack you've built (PX4 + ROS 2 + Gazebo + QGC) is excellent —
but it only serves *one half* of how you're graded. Read the assessment split:

| Weight | Component | Served by |
|-------:|-----------|-----------|
| 35% | Functional POC of swarm drone system | Software/autonomy + (likely) real flight |
| 25% | Design report — design principles, **DFM, FEA/CFD** | **Hardware/design track** |
| 25% | Use of **self-designed and Make parts** | **Hardware/fabrication track** |
| 10% | Creativity and novelty | Both |
| 5%  | Project management, planning, execution | Both |

**50% of the grade (the two 25% blocks) is physical design and fabrication** —
self-designed parts, digital fabrication, and a design report with DFM and
FEA/CFD analysis. The simulation earns you the 35% POC and de-risks the autonomy,
but it does **not** earn the fabrication/design half. You need a parallel
hardware track running from the start. (Your `~/CFDPython` folder is the seed of
the CFD requirement — that's the right instinct.)

So the project splits into:

- **Track A — Software / Autonomy / AI** (what we've been building): swarm
  control, mission planning, perception, hazard mapping. Sim-first.
- **Track B — Hardware / Design / Fabrication**: drone frame/mounts designed and
  Make-fabricated, flight-controller assembly, payload mounts, plus the DFM +
  FEA/CFD design report.

With 6 people, run both tracks in parallel.

## Official deliverables → your status

| Deliverable | Status | Track |
|-------------|--------|-------|
| Functional multi-drone swarm platform (2–5) | Single-drone sim working; scale to N next | A (+B for real) |
| Mission planner with coordinated waypoints | Not started; `offboard_control` is the template | A |
| Sensor payload integration (camera and/or metal/GPR simulator) | Not started; camera model + GPR *simulator* both allowed | A/B |
| Prototype AI pipeline (suspected-landmine regions) | YOLO experiments exist (`~/yolo_test`, `~/runs`) | A |
| Geotagged hazard map + mission report | Not started | A |
| Arena demo | Not started | A+B |
| **Design report (DFM, FEA/CFD)** | CFD started (`~/CFDPython`) | **B** |
| **Self-designed & Make parts** | Not started | **B** |

## Note on the sensor payload

The brief allows "camera **and/or** metal detection/GPR **simulator**." A real
GPR is out of scope/expensive, so a **simulated** metal/GPR sensor plus a
downward camera is a fully compliant, sensible choice — and it fits the sim
pipeline you already have.

## Milestones (from the brief) and where the sim fits

- **M1 (W1–2)** Literature review, mission definition, architecture, **BOM** —
  includes the hardware bill of materials (Track B starts here).
- **M2 (W3–4)** Drone platform assembly + communication setup — physical build +
  your comms stack (the DDS/ROS 2 bridge you have).
- **M3 (W5–6)** Sensor payload + AI framework integration — camera/GPR-sim + YOLO.
- **M4 (W7–8)** Swarm coordination + autonomous navigation — multi-drone + planner.
- **M5 (W9–11)** Detection workflow, mapping + validation — YOLO → geolocation → hazard map.
- **M6 (W12–13)** Field testing + optimization — arena, tuning, FEA/CFD refinement.
- **M7 (W14)** Final demo, technical presentation, grading.

The simulation lets Track A run *ahead* of the physical build: you can develop
and validate swarm autonomy, perception, and mapping in Gazebo while Track B
designs and fabricates, then converge for the arena demo.

## Suggested team-of-6 split

- **Autonomy (2):** multi-vehicle PX4/ROS 2, mission planner, swarm coordination.
- **Perception/AI (1–2):** YOLO node, camera/GPR-sim, pixel→world geolocation, hazard map.
- **Hardware/fabrication (2):** frame & mount design (CAD), Make-part fabrication,
  drone assembly, flight-controller wiring.
- **Systems/design report (shared):** BOM, DFM writeup, FEA/CFD analysis,
  integration, documentation, project management.

## Where the simulation work maps

Everything built so far (single-drone PX4+ROS2+Gazebo+QGC, the launcher, home at
IIT Gandhinagar) is the **Track A foundation** and the de-risking bench for
Track B. It directly supports M2's comms setup and everything in M3–M5.

## Recommended immediate sequence (Track A)

1. **Understand the current single drone** — coordinate frames (NED vs ENU), key
   topics, read the `offboard_control` node. (Frames matter for geolocation.)
2. **De-risk perception** — camera on the single drone + YOLO node + pixel→world
   geolocation. This is M3/M5's core and the riskiest unknown.
3. **Scale to 2 drones** — namespaced multi-vehicle sim (M4 foundation).
4. **Coordinated mission planner** — area split + coverage paths for N drones.
5. **Hazard map** — aggregate geotagged detections + export + report.

## Decision to confirm with your mentor

Whether the **arena demo** must use physically-built drones, a simulation, or a
hybrid (e.g. 1–2 real drones + a larger sim swarm). This affects how much of the
2–5 count is physical vs simulated. The autonomy/AI you develop in sim transfers
to real drones either way, and it de-risks the physical build — so Track A is
worth doing regardless of the answer. Ask early; it sets the Track B scope.
