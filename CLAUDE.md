# CLAUDE.md
---

## 1. What this project is

This is a fork of **zjz0001/LIO-Drone-250**, which is itself a stack built on
top of:

- **FAST-LIO2** — LiDAR-inertial odometry (publishes `nav_msgs/Odometry`).
- **mavros** — bridge between ROS and PX4 over MAVLink.
- **PX4** — flight controller firmware running EKF2 as the state estimator.
- **`geometric_controller`** — Jaeyoung Lim–style geometric tracking controller
  that publishes attitude+thrust setpoints to `/mavros/setpoint_raw/attitude`.
- **ego-planner** — local planner. Out of scope for the current work.

The goal of this fork is to make it work on the user's drone, which uses
different hardware than the original.

---

## 2. Scope of the current work

**Hardware and FAST-LIO are NOT in scope.** The user has already:

- Integrated a **Hesai JT128** LiDAR with its SDK + ROS driver.
- Integrated a **Microstrain 3DM-CV7** IMU with its SDK + ROS driver.
- Patched / configured FAST-LIO2 to consume these sensors and publish a
  stable `nav_msgs/Odometry` for the drone body.

**Do not touch the sensor drivers, sensor configuration, or FAST-LIO source.**
They are working. Treat FAST-LIO as a black box that produces an Odometry
topic. The work begins at that topic and ends at the PX4 flight controller.

The scope of this project is everything **downstream of the FAST-LIO Odometry
topic**:

- The bridge node that forwards FAST-LIO → mavros.
- mavros configuration.
- The `geometric_controller` node, its parameters, and its launch files.
- PX4 EKF2 parameters (set in QGroundControl, mentioned here for reference).

---

## 3. Environment (verify if uncertain)

- **OS:** Ubuntu 20.04
- **ROS:** ROS 1 Noetic
- **PX4:** flashed and working, talks to mavros over USB/TELEM
- **FAST-LIO2:** **already running, stable, publishes `nav_msgs/Odometry`**
  for the drone body.
  - **IMPORTANT: the published Odometry contains pose only. The `twist` field
    (velocity) is empty / zero.** This is the single most important
    constraint in this project. Any change must account for it.
  - The exact topic name should be confirmed with `rostopic list` while
    FAST-LIO is running, not assumed.

---

## 4. Working style — read this before writing any code

The user does **not** vibe-code and does **not** want code written from
scratch. The user's workflow is:

> *"I will mainly use Claude Code to modify the repo to make it work for my drone."*

This means:

1. **The repo is the source of truth.** When in doubt about how something
   works, `grep` and read the existing code. Do not invent behavior.
2. **Prefer the smallest diff that makes the thing work.** A 3-line change is
   better than a 30-line refactor. Do not "improve" code that already works.
3. **No scaffolding, no new packages, no new abstractions** unless explicitly
   asked. If a tiny relay node is needed, write a tiny relay node — not a
   class hierarchy.
4. **Do not delete or rename existing files** unless explicitly asked.
5. **If you are unsure what a node does, read the source and the launch file
   before proposing any change.** Saying "I think this node probably does X"
   is not acceptable. Either show the relevant code lines, or say "I need to
   read X to know."
6. **Explicitly distinguish what you know from what you are guessing.** If
   you haven't read the relevant file, say so, and read it before proceeding.

---

## 5. Use of context7

The user has the **context7** MCP plugin installed. Use it as follows.

### When to use context7

Look up current, authoritative documentation for **external libraries and
APIs**, especially:

- `mavros` plugin names, parameters, and topic conventions
  (`vision_pose`, `odometry`, `setpoint_raw`, `px4_config.yaml`).
- PX4 parameter names and meanings (`EKF2_AID_MASK`, `EKF2_EV_CTRL`,
  `EKF2_HGT_MODE`, `EKF2_EV_DELAY`, `MAV_ODOM_LP`, etc. — these change
  between PX4 versions).
- ROS message type fields (`nav_msgs/Odometry`, `geometry_msgs/PoseStamped`,
  `mavros_msgs/...`).
- FAST-LIO published topics and message types (treat its API as external).
- Any third-party ROS package the repo depends on.

### When NOT to use context7

- To learn what *this repo* does. For that, read the local source.
- For PX4 firmware questions where the answer depends on the specific PX4
  version flashed on the user's drone — ask the user the version first.
- For trivia that does not affect a code change. Stay focused.

### How to use it well

- Cite the specific page or section returned by context7 when its result
  drove a code decision. One-line citation is enough.
- If context7's information conflicts with what is in the local repo,
  **the local repo wins** unless the user says otherwise — the repo is
  what runs on the user's drone.
- Do not bury the user in documentation. Summarize the relevant fact in
  one or two sentences and proceed.

---

## 6. The known data-flow (verify before relying on it)

Per the upstream README, the runtime is started in this order:

1. `roslaunch geometric_controller takeoff_px4.launch` — starts mavros.
2. Lidar driver launch (user's hardware-specific, already working).
3. FAST-LIO launch (user's hardware-specific, already working).
4. `roslaunch geometric_controller takeoff_vrpn.launch` — **bridge node**
   (despite the name, this is the FAST-LIO → mavros relay, not actually VRPN).
5. `roslaunch geometric_controller takeoff_group.launch` — geometric
   controller.

The intended dataflow is:

```
FAST-LIO2 ──Odometry──► [bridge] ──/mavros/vision_pose/pose──► mavros ──► PX4 EKF2
                                                                              │
                                                                              ▼
geometric_controller ──/mavros/setpoint_raw/attitude──► mavros ──► PX4
                          ▲
                          │ (state feedback — see Section 7, OPEN QUESTION)
```

---

## 7. OPEN QUESTION — do not assume, verify before changing anything

**Where does `geometric_controller` get its state feedback (position AND
velocity) from?** Two possibilities:

- **(A) PX4-fused path:** subscribes to `/mavros/local_position/pose` and
  `/mavros/local_position/velocity_local`. Velocity comes from EKF2's
  internal differentiation + IMU fusion. **Pose-only FAST-LIO output is
  sufficient.**
- **(B) Direct path:** subscribes to FAST-LIO's Odometry directly,
  reading both pose and twist. **Requires FAST-LIO to publish velocity,
  which currently it does not.**

**Step zero of any controller-related task is to determine which pattern
this repo uses.** Run:

```bash
grep -rn "subscribe\|Subscriber" src/geometric_controller/src/
grep -rn "mavros/local_position\|Odometry" src/geometric_controller/
cat src/geometric_controller/launch/takeoff_group.launch
cat src/geometric_controller/launch/takeoff_vrpn.launch
```

Report findings before proposing changes. The fix depends on the answer:

- If **(A)**: probably nothing to do on the controller side. Focus on the
  bridge node and PX4 EKF2 parameters.
- If **(B)** and FAST-LIO velocity is zero: either (i) retarget the
  controller to subscribe to `/mavros/local_position/*` (preferred — does
  not require touching FAST-LIO, which is out of scope), or (ii) add a
  small node that consumes the pose-only Odometry, numerically
  differentiates position, and republishes a full Odometry. Ask the user
  which they prefer before doing either.

---

## 8. Things that are NOT in scope right now

Do not modify, refactor, or "clean up" any of these unless explicitly asked:

- Sensor drivers (Hesai LiDAR SDK, Microstrain IMU SDK).
- FAST-LIO2 source. It is working. Do not touch it.
- `ego-planner` (planner is a later concern).
- Any RViz config, visualization, or tooling.
- PX4 firmware source.
- The build system (CMakeLists.txt, package.xml) unless a change is
  unavoidable for a required task.

If you find a bug outside the current task, **mention it once, do not fix
it unless asked.**

---

## 9. Things that probably ARE in scope

- The bridge node started by `takeoff_vrpn.launch` (topic names, message
  type conversion, frame_id).
- `geometric_controller` parameters (gains, mass, hover thrust, init
  position).
- Topic remappings inside launch files.
- mavros config (`px4_config.yaml`, `px4_pluginlists.yaml`) — for example
  to enable/disable specific mavros plugins.
- PX4 EKF2 parameters (these live in QGroundControl, not in the repo, but
  the user may ask for guidance — see Section 10).

---

## 10. PX4 EKF2 parameters reference (for guidance, not for the repo)

When the user asks about PX4-side configuration for the
`/mavros/vision_pose/pose` path, the typical settings are listed below.
**Verify the exact names against the user's PX4 version using context7
before quoting them**, because they changed around PX4 1.14.

- `EKF2_AID_MASK = 24` — vision position + vision yaw, GPS off
  (for PX4 ≤ 1.13; PX4 ≥ 1.14 uses `EKF2_EV_CTRL` instead).
- `EKF2_HGT_MODE` — set to vision for height (value depends on PX4 version).
- `EKF2_EV_DELAY` — measured FAST-LIO latency (typically 30–80 ms).
- `MAV_ODOM_LP = 1` — makes PX4 echo received external pose back as
  MAVLink ODOMETRY for verification in QGC MAVLink Inspector.

Do **not** edit these in the repo; they are flight-controller parameters
set via QGC. Mention them when relevant.

---

## 11. The before-every-change checklist

Before proposing any code change, you must be able to answer:

1. **What file(s) am I changing?** (Exact paths.)
2. **What do those files currently do?** (Show the relevant lines, not a
   guess.)
3. **What is the smallest possible diff?** (Lines added/removed, not "I'll
   refactor X.")
4. **What could break?** (List the things downstream of this change.)
5. **How will the user verify it worked?** (Concrete `rostopic echo` /
   `rostopic hz` / `rqt_graph` command.)

If you cannot answer all five, stop and read more code, or ask the user.

---

## 12. Verification commands the user already knows and trusts

Use these when proposing a sanity-check step:

```bash
# Is FAST-LIO publishing? (Confirm the actual topic name first.)
rostopic list | grep -i odom
rostopic hz <fast_lio_odom_topic>
rostopic echo -n1 <fast_lio_odom_topic>

# Is the bridge alive and feeding mavros?
rostopic hz /mavros/vision_pose/pose
rostopic echo -n1 /mavros/vision_pose/pose

# Is PX4 EKF2 actually fusing it?
rostopic hz /mavros/local_position/pose
rostopic echo -n1 /mavros/local_position/pose

# Is the controller sending setpoints?
rostopic hz /mavros/setpoint_raw/attitude

# Full graph
rqt_graph
```

---

## 13. Commit hygiene

- One logical change per commit.
- Commit message format: `<area>: <one-line summary>`, e.g.
  `bridge: convert Odometry to PoseStamped for vision_pose`.
- Do not commit build artifacts, `devel/`, `build/`, or editor files.
- Do not auto-commit. Show the diff first; the user will commit.

---

## 14. When in doubt

Ask. A one-line clarifying question is much cheaper than a 200-line diff
in the wrong direction.