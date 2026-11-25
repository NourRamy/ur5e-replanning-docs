# Wed 25 Nov lab, 09:00–18:30: human-aware moving replanning on the real UR5e (the only lab day this week)

**Frozen build:** SC3 **v2.43**, md5 **e184273a** (`experiments/candidates/strategy_b_detour_a2s_g1_sc3.py`); `run/b_take.sh`
refuses any other file and refuses every configuration that did not pass the simulator safety rule.
Freeze check (45 files): `md5sum -c --quiet experiments/lab/FREEZE_LAB_TUE24.md5 && echo FREEZE OK`.
Report: `test_results/lab_20261124/REPLANNING_HUMAN_REPORT.md`.

| Short | Command (T10) | What it is | Simulator evidence (25 Nov night) |
|---|---|---|---|
| **E2F** (main) | `HUMAN=1 FIX=1 bash run/b_take.sh` | human mode + Fix A + Fix B | 0 contacts (40 normal + 18 live-like); live-like 4 moving detours / 13 eligible conflicts, 11 stops (Exp1 33) |
| E2 | `HUMAN=1 bash run/b_take.sh` | human mode without the fixes | 0 contacts; 0 moving detours |
| E1 | `NOSTOP=0 CANCEL=1 bash run/b_take.sh` | Experiment 1, stop-and-replan baseline | 0 contacts; 0 moving detours |
| not allowed | `MPC=1`, `MPCSTOP=1`, `FIX=1` without `HUMAN=1` | NOT VALIDATED (b_take refuses) | E3 had 1 moving contact live-like; the others were not run |

Fallback at any time: E2F → E2 → E1 (all three are in the same frozen file; no file is changed in the lab).

Rules for every take:
- e-stop in your hand; the UR safety system stays as configured; speed as in b_take (velocity scaling 0.1 unless `SPEED`);
- **wait 10 s after a take** (MoveIt is busy right after a take); "check: no answer" → wait 10 s, run again;
- arm stops high and `move_to_start` says INVALID with nobody near → bring it down from the pendant, then Play;
- **first time of every scenario: do the human action slowly**; the sudden-reach case reached 0.3–1.1 cm in the simulator
  in every configuration (protective stop, braking) - keep the hand flat and slow until it has been seen once;
- every take: **T4** `cd ~/ur_workspace && bash run/b_rec.sh <name>` · **T10** the command · Ctrl+C T4 ·
  **T7** `python3 experiments/lab/session/verify_switch.py` (prints MOVING SWITCH / sent at rest / stop for every new
  path, from the bag's joint speeds) · send me T10 + the verify output.

## A. Start-up, no arm motion (09:00–09:45)
1. USB: Ethernet adapter as Tuesday; **Kinect 1 alone on USB-A 1; Kinect 2 UNPLUGGED for every arm motion** (18 Nov: K2
   on the USB bus crashed the UR driver; the lean stack starts Kinect 1 only, `kinect2:=false`). K2 only in block G.
   **T7**: `nmcli con up "UR robot (hub)"` and `ping -c 2 192.168.1.102` → 2 received.
2. Pendant on, brakes released. **T1**: `cd ~/ur_workspace && bash run/lab_1_driver.sh` → pendant **Play**.
3. Table empty, nobody in view. **T2**: `cd ~/ur_workspace && bash run/lab_2_stack_lean.sh` → "You can start planning";
   then close the RViz window.
4. **T7** (every T7 command starts with `cd ~/ur_workspace && source /opt/ros/humble/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=0 &&`):
   - `md5sum -c --quiet experiments/lab/FREEZE_LAB_TUE24.md5 && echo FREEZE OK`
   - `python3 experiments/calibration/roll_fix/kinect_level_fixed.py` (≤ 0.35° is fine)
   - `python3 experiments/install/live_counter.py --seconds 30 --expect empty` → PASS
   - `python3 experiments/lab/virtual_limits.py add --frame --pad 0.05 --top 1.74` (frame padding)
5. C1 as Tuesday: **T7** `bash run/c_step.sh stop_prod`, then **T6** `bash run/c_step.sh c2` (leave T6 open). Tracker:
   production (the queue-depth-1 tracker gave no gain under load: no `t_step`).
6. **Person check, no motion** (T7, you in front of the table):
   - stand still at the table edge 30 s: `python3 experiments/install/live_counter.py --seconds 30 --expect present --label person_still` → PASS
   - move a hand slowly over the table 30 s: same with `--label hand_moving` → PASS
   - `python3 experiments/install/live_counter.py --topic /perception/objects/tracks --seconds 20 --expect present --label tracks` → PASS
   - while you stand at the table edge, the person is in MoveIt's scene:
     `ros2 service call /get_planning_scene moveit_msgs/srv/GetPlanningScene "{components: {components: 8}}" | grep -o "id='dynamic[^']*'"`
     → at least one `dynamic_object_…`; step away 3 s, run again → none (removed)
7. **T7**: `bash run/c_step.sh status`, `bash experiments/lab/session/session_snapshot.sh wed25_start`.
8. Arm to START: **T7** `echo GO | python3 experiments/lab/move_to_start_checked.py go`.
→ send me the outputs of 4, 6, 7.

**Runtime configuration check (during the first take B1, T7):** `ros2 param get /strategy_b_local v3_splice_lead` → 1.0;
`ros2 param get /strategy_b_local sc3v2_h_horizon` → 2.5; `ros2 param get /strategy_b_local sc3v2_h_cred` → True;
after the take: `grep -E "HUMAN MODE|v2.43 test|margins" $(ls -td experiments/replan/bench/out/real_sb4static_* | head -1)/logs/SB_node.log`
→ "growth 2.5 s", "Fix A … ON, Fix B … ON", "clear_req 0.0 cm, contact_dist 1.0 cm".

## B. No-obstacle baseline + box (09:45–10:05)
| # | Name | Command | What | Expected |
|---|---|---|---|---|
| 1 | w_free_e2f | E2F | nobody near | 2 legs, ~6.4 s per leg, 0 stops |
| 2 | w_box_e2f | E2F | green box still on the path middle; stay away | over the box (ready-made route), ~7 s per leg, 0 stops |

## C. M4: moving detour with a person beside the path (10:05–11:45) - the main experiment
The simulator's moving switches came from **a hand passing beside the path, seen ~2.5 s before the arm gets there**.
Action "pass beside": during a leg, about 1 s after the arm starts, move your hand (or walk) **along** the path at
~25 cm from it, at a calm pace, from the goal side toward the start side; do not enter the path.
**First take `m4_e2f_1` alone, then STOP and check before anything else** (5 min): verify_switch output; the take's
SB_node.log (H7_REPLAN / CANDIDATE / REPLACE / stops; I read it with you); the person in the tracks and the scene
(SB_stop_objects); SC3's clearance; controller (no CTRL_REJECTED). Only then continue.
Order per round: **E2F, E2, E1** (same action each time). 4 rounds = 12 takes. Names `m4_e2f_1`, `m4_e2_1`, `m4_e1_1`, …
Expected E2F: H7_REPLAN in the log, a LOCAL / GLOBAL path sent while moving, verify_switch → **MOVING SWITCH**,
0 stops. E2 / E1: a stop or a restart (that is the comparison).
If E2F shows no MOVING SWITCH after 2 rounds: tell me (I read the log while you continue with D).

## D. Crossing and approach (11:45–12:45)
Per scenario 3 takes, order **E2F, E2, E1**:
| Scenario | Names | What you do | Expected (E2F) |
|---|---|---|---|
| H4 walk across (class A) | h4_* | walk across between the table and the arm's path at normal pace | detour while moving if one is free, else stop and restart < 1.5 s |
| H2 still hand beside (class B) | h2_* | hand held still ~10 cm beside the path | passes without stopping (Fix B: a still hand does not grow) |
| H5 reach toward the arm (class B) | h5_* | reach slowly toward the moving gripper from the side, stop 10 cm before it | slows / stops before your hand, never touches |

## Lunch + endurance (12:45–13:30)
`LEGS=40 HUMAN=1 FIX=1 bash run/b_take.sh` (name `w_endur_e2f`), nobody near at first, then walk in now and then.

## E. Protective response, class C (13:30–14:30) - slowly first
Per scenario 3 takes, order **E2F, E2, E1**:
| Scenario | Names | What you do | Expected |
|---|---|---|---|
| H1 late hand | h1_* | hand INTO the path ~0.5 s before the arm gets there (first take slowly) | protective stop at once (controller cancel), moving again < 1.5 s after the hand leaves |
| H3 hand on the path, then away | h3_* | hand still ON the path ~3 s mid-leg, then away | holds, goes < 1.5 s after the hand leaves |

## F. Repeatability + other start/goal (14:30–16:30)
- M4 "pass beside" 5 more takes with **E2F** only (`m4r_e2f_1..5`), then 3 with E2 for comparison.
- If time: 1–2 other start / goal pairs (as in the plan), M4 + H1 with E2F.
- Repeat any take that failed for a technical reason (no answer, driver).
- Fix H: only if the report (section 7) lists it as validated, with its own command; otherwise nothing else.

## G. Hand-E MoveIt model check, then blocks + box camera check + force sensor (16:30–17:45)
**G1 Hand-E model (for Saturday; only after every replanning take; Kinect 2 still UNPLUGGED):**
1. **T2** Ctrl+C (lean stack), wait until it is down; **T2** `cd ~/ur_workspace && bash run/lab_2_stack_lean_hande.sh` →
   "You can start planning"; close RViz. (MoveIt model = Hand-E; the driver, perception and SC3 are unchanged.)
2. **T7** `python3 experiments/lab/virtual_limits.py add --frame --pad 0.05 --top 1.74`, then
   `python3 experiments/lab/virtual_limits.py check --frame --pad 0.05 --top 1.74` → "state VALID".
3. **T7** `ros2 param get /move_group robot_description_semantic | grep -o "hand_e[a-z_]*" | sort -u` → `hand_e_base_link`.
4. Nobody near: **T4** `bash run/b_rec.sh g_hande_free` · **T10** `HUMAN=1 FIX=1 bash run/b_take.sh` (2 legs) · Ctrl+C T4 ·
   verify_switch. Expected: as B1 (~6.4 s per leg, 0 stops).
5. Back to the normal stack for the rest of G: T2 Ctrl+C → `bash run/lab_2_stack_lean.sh`.
(Home test 25 Nov 02:12, mock hardware: Hand-E model loads, 1228 / 1257 replayed lab states valid = the 23 Nov result.)

**G2** As in `LAB_WED_20261125_PLAN.md` (blocks + box recordings, photos, FT check), arm not moving. Kinect 2 may be
plugged in here only.

## If something goes wrong (no code changes on the arm; every failed take is kept)
| What happens | What we do |
|---|---|
| Kinect does not see the person | `live_counter --expect present` again; check Kinect 1 on USB-A 1, stack T2 output; re-run `kinect_level_fixed.py` |
| Person seen but the region looks wrong / late | keep the take; I read SB_stop_objects / SB_monitor; continue with E2 meanwhile |
| MoveIt does not get the person | `c_step.sh status`; `get_planning_scene` check from A6; restart T6 (`c_step.sh c2`) |
| "check: no answer" / MoveIt busy | wait 10 s, run again (known since 23 Nov) |
| No detour found (RRT none) | expected sometimes: stop + restart is the correct fallback; keep the take |
| Detour arrives too late | protective stop is the correct response; keep the take |
| Controller rejects a trajectory (CTRL_REJECTED) | stop takes, send me the log; do not bypass anything |
| Arm stops unexpectedly / protective stop on the pendant | e-stop state + pendant message first; only then `move_to_start_checked.py`; never restart blind |
| Any contact | stop the block, send me the take; continue with E1 only until we have looked |
| Driver lost (USB / network) | `nmcli con up "UR robot (hub)"`, T1 again, Play; K2 stays unplugged |

## Evidence per take (automatic, nothing to type besides the take)
b_rec bag (joint states, detections, tracks, scene), SB_node.log + SB_*.jsonl (decisions, candidates, trajectories),
meta.txt (configuration + md5 via the freeze), verify_switch output (MOVING SWITCH from the measured joint speeds).
Separation on the real robot = SC3's clearance in the log (no ground truth) - reported as such.

## How we judge the day
- **Level 1 (core):** detection, tracking, prediction, scene updates and protective stops work on the real arm.
- **Level 2 (main goal):** at least one **MOVING SWITCH** (verify_switch, measured joint speeds) with no contact.
- **Level 3 (strong):** repeatable moving switches over several takes, with latency and clearance numbers.
One take that does not switch is information, not failure: the log says which stage stopped it.

## H. Wrap-up (17:30–18:30)
T6 Ctrl+C → `bash run/c_step.sh back`; `mkdir -p test_results/lab_20261125 && python3 test_results/lab_20261124/analysis/take_metrics.py 20261125 test_results/lab_20261125/take_metrics.md`;
session snapshot `wed25_end`; Kingston copy; git push (approved); shutdown. Keep every failed take (nothing is deleted).
