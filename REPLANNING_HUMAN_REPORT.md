# Human-aware replanning: state on the night of 24 Nov 2026

Installed for Wednesday: SC3 **v2.43**, md5 **e184273a** (`experiments/candidates/strategy_b_detour_a2s_g1_sc3.py`; all
switches off = v2.42b; `run/b_take.sh` checks it and refuses unvalidated configurations; freeze
`experiments/lab/FREEZE_LAB_TUE24.md5`, 45 files). Recommended configuration **E2F = `HUMAN=1 FIX=1`**. Test copies: v2.42c (MPC stops when no speed is safe),
v2.43test (fixes A and B, section 6). All simulator results: harness on domain 88, fake hardware, scripted people.
Evaluation rules: `test_results/EVALUATION_PROTOCOL_REPLANNING.md`.

## 0. The five questions

| Question | Answer |
|---|---|
| **Bottom line (25 Nov 01:15)** | **E2F (Fixes A+B) is the only configuration with verified moving detour switches with people: live-like 4 detours / 13 eligible conflicts (31 %) + 1 more switch, 0 contacts, 11 stops (Exp1 33). Simulator only; the lab validates it.** |
| Did v2.42b remove the five-contact failure of v2.42? | **Yes in the normal runs** (person_sweep 5 → 0 contacts, 40 runs: 0 contacts). **Live-like: 1 contact with the robot moving** (−0.2 cm, person_reach_prox; the hand was seen when it was already on the path, the stop was sent in 19 ms and the arm was braking) |
| Does v2.42c (stop when no speed is safe) improve contacts? | Not measured: stopped to make room for the human-mode fixes (E3 is behind E2 anyway) |
| Why does MoveIt reject the start state? | It never reaches MoveIt: SC3 refuses to plan because the splice state lies inside the person's **planning region**. 0 % real geometry. 41 % predicted growth of a person measured at 0.00 m/s, 38 % the horizon loop never tries the exact splice time, 21 % the person's arm is at the splice point now and gone when the robot arrives (the region grows around the current position and does not move with the person). Section 4 |
| Can the robot switch to a detour while moving? | **Yes, with people, in E2F under live-like conditions** (hand passing beside the path: conflict seen 2.5 s ahead, GLOBAL detour found in 0.47 s, switched at 0.37 rad/s, arm never below 0.16 rad/s, 16 cm separation). Box rules: 1–4 of 5 slow crossings in every version (timing variation, bisect 6.5). |
| Which configuration goes to Wednesday? | **E2F**, fallback E2, then E1 (section 7) |

## 1. Intended, implemented, demonstrated

Success criterion (user, 24 Nov): when a safe detour exists, the robot generates it and switches to it while moving;
when none exists, it slows down or stops. It never moves just to avoid a stop.

| Part | Intended | Implemented | Demonstrated (simulator unless said) |
|---|---|---|---|
| Human detection and tracking | depth-based detection, Kalman tracking, velocity | yes (production since 4 Nov) | yes, live; tracking error 0.5–2.3 cm in the harness |
| Prediction | time-indexed occupancy of the person | region grown by speed × time (up to 2.5 s), symmetric around the current position; 0.25 m/s minimum for objects not yet still | conservative; too large for standing people and in the wrong place for people moving away (section 4) |
| Event-triggered monitoring | check the running path every 50 ms | yes (H7): ≥ 0.6 s ahead → replan while moving, closer → stop at once | yes; stop sent in 3–43 ms (live 24 Nov) |
| Detour planning while moving | RRT-Connect from the splice state, ready-made routes as fallback | yes (splice 1.0 s ahead for RRT-Connect, 0.3 s for the routes) | with boxes yes; with people: E2 almost never (start refused, section 4), E2F 4 / 13 eligible live-like |
| Full check of the new trajectory | time-indexed check of the whole spliced trajectory | yes (every candidate, before sending) | yes |
| Moving trajectory switch | join the new path at the splice point, keep moving | yes | **E2F live-like: 4 detours + 1 more switch, verified from joint speeds; E2 / Exp1: 0; box rules v2.36: 4/5** |
| Speed MPC | slow down along the same path when that keeps the separation | v2.42b | works (slows 100 → 15 %, speeds up again); normal runs: 13 of 35 conflicts solved by slowing; live-like: slower than Experiment 1, 1 moving contact |
| Stop when no safe motion | protective stop, then restart | yes (controller cancel; fast restart ~1 s) | yes |

## 2. Algorithm (v2.42b human mode; v2.43 changes in section 6)

Every monitor cycle (50 ms):
1. **People present?** Any object ≥ 30 cm tall, or not still (still = within 3 cm and under 6 cm/s for 1 s), seen in the
   last 2 s → person rules. Only still boxes → box rules (ready-made route over the box, unchanged from v2.36).
2. **Region of each object at time t** (`reach_at`): its measured box, grown along its direction of motion by
   1.1 × v × min(t − t_meas + 0.1 s, 2.5 s), v = max(current speed, 2 s peak, 0.25 m/s), plus 1 cm/s sideways; people +1 cm.
   A still object keeps its own size.
3. **Monitor (H7).** Time to the first conflict on the running path. None → continue (MPC may change the speed).
   ≥ 0.6 s → keep moving, start a replanning cycle. < 0.6 s or the person at the arm → cancel the controller goal at once.
4. **Replanning cycle.** Splice state 1.0 s ahead on the running path (`v3_splice_lead`). Planning region = the regions at the splice time
   + horizon H, H stepped down by 0.25 s until the splice state is outside. RRT-Connect (MoveIt, 6 attempts, shortest)
   to points after the conflict (LOCAL) and to the goal (GLOBAL); if it finds nothing, the six ready-made routes.
   Each candidate is re-timed, joined at the splice state, and the whole trajectory is checked against the regions at the
   times the arm will be there. Quality filter: length ≤ 2 × direct + 1 rad, wrist 3 ≤ direct + 0.5 rad.
5. **Speed MPC (E3 only).** Every 100 ms: speeds {1, 0.7, 0.5, 0.3, 0.15} along the same path over 1.5 s; keep those
   with distance ≥ 1 cm + v_tool × (0.2 + 0.3) s at every sample; pick the one minimising (1 − v)² + 0.3 (v − v_prev)²;
   speed-up at most +0.3 per 0.5 s; no safe speed → no change, step 3 decides (v2.42b; v2.42 slowed to 15 % and parked the
   arm in the sweep). Sent as a re-timed copy of the same path.
6. **At rest.** Straight path first (checked with 0.5 cm extra), then RRT-Connect, then the routes.

Margins unchanged: boxes 0 cm, people 1 cm, contact alarm 1 cm, MoveIt pad 0. This is an SSM-inspired research system,
not a certified safety function; the UR safety system and the e-stop are the independent safety layer.

## 3. Simulator results

### 3.1 Normal conditions (40 runs per configuration: 8 scenarios × 5 seeds)
| | Exp1 stop-and-replan | v2.41 | v2.42 | v2.42b |
|---|---|---|---|---|
| reached | 39 | 39 | 40 | 39 |
| contacts robot moving / standing | 0 / 0 | 0 / 0 | 0 / **5** | 0 / 0 |
| closest median (min) | 6.0 (0.3) cm | 6.1 (0.3) cm | 7.3 (−3.8) cm | 6.0 (0.3) cm |
| time to goal median / p90 / max | 10.9 / 18.9 / 34.3 s | 11.5 / 13.0 / 21.0 s | 12.0 / 13.2 / 21.7 s | 11.4 / 12.0 / 16.0 s |
| stops | 154 | 57 | 53 | 57 |
| moving detour switches (joint speeds) | 3 | 0 | 1 | 0 |

Statistics against Exp1 (95 % bootstrap interval, Mann-Whitney p): v2.41 and v2.42b stops per run −2.4 [−5.1, −0.6]
(a real difference); time to goal +0.5 to +0.6 s, p 0.41–0.46 (no difference); closest distance equal.
The two unreached runs (one v2.41, one v2.42b) are a still box on the path, held for 40 s by the start-state refusal.

### 3.2 Live-like (18 person runs: 0.25 s detection delay, 4 % dropped frames, 4 CPU hogs)
| | Exp1 | v2.36 (bank) | v2.41 | v2.42 | v2.42b |
|---|---|---|---|---|---|
| contacts robot moving / standing | 0 / 0 | 0 / 0 | 0 / 0 | 1 / 1 | **1** / 0 |
| closest median (min) | 7.8 (0.1) cm | 11.6 (2.4) cm | 7.8 (0.4) cm | 8.8 (−4.0) cm | 8.3 (−0.2) cm |
| time to goal median / p90 | **7.4 / 11.0 s** | 9.2 / 11.0 s | 8.8 / 13.2 s | 11.9 / 15.3 s | 11.5 / 12.8 s |
| stops | 33 | 13 | 36 | 27 | 36 |

Against Exp1: v2.41 slower (+1.4 s median, p 0.03; interval crosses 0), stops equal; v2.42b slower (+4.0 s, p 0.001).
**Under live-like load the human mode is not better than stop-and-replan.** v2.36 keeps the most distance because its
ready-made route lifts the arm at every start (the behaviour removed after the 24 Nov lab).

### 3.3 Per-conflict indicators (protocol)
Eligible conflicts (seen while moving, ≥ 0.6 s ahead), normal runs: Exp1 16 → 16 stops; v2.41 29 → 22 stops, 7 cleared
without a stop; v2.42b 35 → 22 stops, 13 cleared (MPC). Successful replanning (detour while moving): 0 % in every
version with people. Reaction (hand starts → first conflict line): 0.5–0.6 s median, 0.8 s max (normal); 0.5–0.9 s
median, 1.1–1.5 s max (live-like). Planning and executable latency: no moving detours to measure with people.
Unnecessary replanning %: not computed (needs the planned trajectories against the true future person motion).

## 4. Why the planning start state is refused (diagnostic copy, logging only, 18 person runs, 73 refusals)

| Cause | Share | What the log shows |
|---|---|---|
| Real geometry (person at the splice state) | **0 %** | true person 9–25 cm from it |
| Predicted growth of a person who stands still | 41 % | measured speed 0.00 m/s, not yet "still" (the torso centre jitters > 3 cm) → grown at 0.25 m/s for up to 2.5 s (≈ 69 cm); start 17 cm from the measured box |
| Horizon loop skips the splice time | 38 % | start clear by 3.7 cm at the splice time; the 0.25 s steps stop at H mod 0.25 > 0 |
| Person's arm at the splice state now, gone when the robot arrives | 21 % | moving arm 0.30–0.46 m/s; tracker within 0.5–2.3 cm of the truth; the region grows symmetrically around the current position and does not move with the person |

In the box tests the refusals are real geometry, but only after the stop (the arm stands at the box).

## 5. Splice lead - first test INVALID (corrected 25 Nov 00:10)
The RRT-Connect splice in SC3 is **1.0 s ahead** (`v3_splice_lead`, set in `StrategyBReplan.__init__` after the parents'
`splice_lead` 0.6 s), not 0.6 s as stated earlier tonight. The test that varied `splice_lead` (0.2 / 0.3 / 0.45 / 0.6 s)
therefore did not change the splice; its differences are run-to-run variation (only the planning budget changed). Result
withdrawn. Consequence for the diagnosis: with the replan trigger at 0.6 s ahead and the splice at 1.0 s ahead, a
conflict first seen 0.6–1.0 s ahead puts the splice point past the conflict -> "start inside the region".
Corrected test (section 6.4): 1.0 s is the best of 0.6 / 0.8 / 1.0 (0.6 s gave 2 moving contacts).

## 6. Human-mode fixes (v2.43 test copy)
- **Fix A** (`sc3v2_h_occ_zero`): when every stepped horizon fails, use the planning region at exactly the splice time.
  Only the region MoveIt plans around changes; the full timed check of the whole spliced trajectory still decides.
- **Fix B** (`sc3v2_h_cred`): an object that is not still but has no credible measured motion (current and 2 s peak
  speed < 6 cm/s, the existing jitter threshold) keeps its own size + measurement uncertainty (1 cm/s sideways, people
  +1 cm) instead of the 0.25 m/s growth. The first credible speed brings the full growth back; it decays after the 2 s
  speed memory. Fix B changes the monitor too (it uses the same regions).
- Not done tonight: moving the region with the person (directional prediction) - Thursday candidate.

All in one file, **v2.43** (md5 e184273a, all switches off = v2.42b + the DIAG logging), tested on that exact file.

### 6.1 Isolated M4 (11 runs: slow crossing box seeds 1–5, box appearing on the path 1–3, box crossing at 0.5 m/s 1–3)
| | human mode (none) | Fix A | Fix B | A + B | box rules |
|---|---|---|---|---|---|
| conflicts seen ≥ 0.6 s ahead (planning can start) | 4 of 10 | 4 of 11 | 9 of 12 | 8 of 12 | – |
| moving detour switches (joint speeds) | 0 | 1 | 0 | 1 | 0 |
| RRT-Connect paths found / none / skipped | 17 / 10 / 12 | 12 / 10 / 11 | 19 / 9 / 5 | 34 / 16 / 13 | – |
| planning starts refused | 10 | 4 | 26 | 10 | – |
| contacts | 0 | 0 | 0 | 0 | 0 |
| time to goal median / max | 10.2 / 18.5 s | 9.9 / 25.2 s | 9.9 / 13.0 s | 9.8 / 18.1 s | 9.3 / 14.2 s |

Fix B does what it is meant to (conflicts seen early enough to plan; twice as many RRT paths), but the paths found around
a moving obstacle's grown region are long (10–19 rad of joint motion against 2.7–3.3 rad direct, wrist 3 up to 6 rad),
and the quality filter refuses them; the arm stops and restarts. Fix B also switches the growth on at once when the
speed becomes credible, so a conflict can appear suddenly 0.16 s ahead. **M4 with people / moving obstacles is not
demonstrated**; the remaining obstacles are the size and placement of the moving region (directional prediction) and
the detour quality, not the switching mechanism.

**Box rules with tonight's file: 0 of 5 slow crossings switched while moving; v2.36 last night: 4 of 5** (same scenario,
seeds and settings). The v2.36 log: RRT detour found in 124 ms, switched while moving (arm ≥ 0.15 rad/s). Tonight: the
ready-made-route rebuild ran first (429 ms) and used the planning budget ("LOCAL_SKIPPED budget 52 ms") → stop. A bisect
v2.36 / v2.40 / v2.41 / v2.42b / v2.43 (5 runs each) runs after the safety batches: section 6.3.

### 6.2 Safety and benefit of E2F (A + B) - same seeds as the baselines
| | Exp1 normal | E2 normal | **E2F normal** | Exp1 live-like | E2 live-like | **E2F live-like** |
|---|---|---|---|---|---|---|
| contacts moving / standing | 0 / 0 | 0 / 0 | **0 / 0** | 0 / 0 | 0 / 0 | **0 / 0** |
| stops | 154 | 57 | **39** | 33 | 36 | **11** |
| time median / p90 / max | 10.9 / 18.9 / 34.3 | 11.5 / 13.0 / 21.0 | **10.6 / 12.3 / 18.4** | 7.4 / 11.0 / 11.9 | 8.8 / 13.2 / 14.8 | **7.4 / 10.0 / 10.7** |
| closest median (min) | 6.0 (0.3) | 6.1 (0.3) | 6.2 (0.3) | 7.8 (0.1) | 7.8 (0.4) | 7.6 (0.1) |
| eligible → detour / original kept / cleared / stop | 0/0/0/16 | 0/0/7/22 | 0/1/1/14 | 0/0/0/24 | 0/–/5/18 | **4 / 2 / 6 / 1** |
| verified moving detour switches | 3 | 0 | 0 | 0 | 0 | **5** |
E2F vs Exp1: normal time −0.37 s [−0.89, −0.17], p 0.001, stops −2.9 per run [−5.5, −1.1]; live-like stops −1.2 per run
[−1.7, −0.7], time equal (p 0.17), separation equal. E2F live-like latencies (4 detours): conflict → detour found 0.58 s,
→ sent 0.02–0.03 s; hand starts → conflict 0.73 s median, 1.09 s max. (The earlier "6 / 13" counted two re-sends of the
re-checked original path as detours; corrected to 4 / 13.)
Closest distances ≤ 2 cm occur only in the sudden-reach scenario (person_reach*), the same in every configuration
including Exp1 (0.3–1.1 cm): the arm moves at ~0.33 rad/s, the stop is sent in 4–18 ms, the closest point comes while
braking. A limitation of the protective response with a 1 cm alarm and 0 cm path margin, not of the fixes.
**Not validated (refused by b_take):** E3 (MPC, 1 moving contact live-like), E3S / E3F (not run: their slot went to Fix H).

### 6.3 Fix H (v2.45, md5 10aee331): moving replanning through the blended handover - NOT adopted
Root cause found tonight: the junction-safe blended handover (strategy_b_blend, 23 Oct) runs only when the running
trajectory is a SAFE_STOP cut (`strategy_b_event._extend`); H7 and the no-stop layer keep the arm on DIRECT / BANK, so
moving replanning never used it (junction refusals: 15 of 34 RRT paths). Fix H routes H7 through it (H5 kept).
| | E2F | E2F + H | H alone (no A+B) |
|---|---|---|---|
| isolated M4 (11): junction refusals / moving switches | 15 / 1 | **1** / 0 | – / 0 |
| normal (40): contacts moving / standing, stops, switches | 0/0, 39, 0 | 0/0, 42, 0 | 0/**1**, 61, 3 |
| live-like (18): contacts, stops, moving detours | 0/0, 11, **5** | 0/0, 9, **0** | – |
Fix H removes the junction refusals, but the blended cycle sends one LOCAL request to rejoin states after the conflict
(54 times all of them inside the person's region -> no request) and has no GLOBAL route to the goal; all of E2F's real
detours were GLOBAL. Safe, but it loses the moving switches. Fix H alone fails the safety rule (1 standing contact).
Fix J (Hermite connection, v2.44) was rejected offline: the path retimer limits acceleration only along the path, so the
connection's bend asked for 1.2–3.3 rad/s² against 0.5 - an infeasible trajectory.

### 6.4 Splice-lead audit
Effective RRT-Connect splice 1.0 s: `strategy_b_local` sets `splice_lead` 0.6 -> `strategy_b_adaptive` max(lead, budget +
0.1) -> `strategy_b_replan` sets `v3_splice_lead` 1.0 after `super().__init__()` (wins); no later assignment, no override
in the runners or b_take; runtime log "takeover 1.00 s ahead", actual takeover 1.02–1.11 s (snaps to the next trajectory
sample). Routes: `sc3v2_bank_lead` 0.6 s, `sc3v2_bank_kick_lead` 0.3 s. Corrected comparison on E2F
(`v3_splice_lead`, same seeds): live-like 18 runs - 1.0 s (installed): 0 contacts, 4 detours / 13 eligible, 5 moving
switches; 0.8 s: 0 contacts, 3 / 14, 4 switches; **0.6 s: 2 contacts with the robot moving**, 0 / 14, 1 switch. Isolated M4:
0 switches at 0.6 and 0.8 s. A shorter lead leaves less planning time before the splice; **1.0 s stays** (frozen build
unchanged).

### 6.5 Box-rules bisect (25 Nov 01:20–01:36): no regression - timing variation
Slow crossing box, seeds 1–5, route bank on, human mode off; moving switches from the joint speeds:
v2.36 2 (previous night 4), v2.40 1, v2.41 2, v2.42b 1, **v2.43 2** (earlier tonight 0); 0 contacts in all. The same
version gives 0–4 of 5 depending on whether the ready-made-route rebuild or the RRT answer comes first inside the planning
budget. The installed v2.43 still switches while moving with the box rules.

## 7. Wednesday configuration
- Build: SC3 v2.43, md5 e184273a, freeze OK (45 files). Runbook: `test_results/lab_plans/LAB_WED_20261125_RUNBOOK.md`.
- Main: **E2F** `HUMAN=1 FIX=1 bash run/b_take.sh`; comparison E2 `HUMAN=1 bash run/b_take.sh` and E1
  `NOSTOP=0 CANCEL=1 bash run/b_take.sh`; after every take `python3 experiments/lab/session/verify_switch.py`.
- Fallback: E2F -> E2 -> E1, same file, no change in the lab. Kinect 1 only for motion (K2 unplugged).
- Must NOT be used on the arm: `MPC=1`, `MPCSTOP=1`, `FIX=1` without `HUMAN=1` (b_take refuses), v2.44 (Fix J), v2.45
  (Fix H) - neither is installed.
- Risks: real planning time (Tuesday: 3–6 × the simulator; a detour must be ready before the splice 1.0 s ahead), real
  torso jitter above the 6 cm/s credible-speed threshold (then E2F behaves like E2), small simulator sample (4 detours,
  one scenario), sudden reach 0.3–1.1 cm in the simulator (do H1 slowly first).

## 8. Hand-E MoveIt option (25 Nov 02:30, user "OK Hand-E option")
`dynamic_obstacles.launch.py` gets `gripper:=2f85|hande` (default 2f85 = unchanged); `hande` loads the 23 Nov candidate
`ur5e_robot_hande.urdf.xacro` / `.srdf.xacro` into MoveIt only (driver, perception and SC3 unchanged; SC3 already uses its
own Hand-E model since v2.19). Home test (domain 88, mock, `experiments/candidates/hande_moveit/launch_test.sh`): both start
in 8 s; default SRDF without Hand-E, `hande` with `hand_e_base_link`; replay of the 1257 lab stop/hold states: 2f85 1227
valid, hande 1228 (= the 23 Nov results). Installed (one file copied; no rebuild); lab script `run/lab_2_stack_lean_hande.sh`;
used only in runbook block G1 after the replanning takes (no-motion checks, then one free leg).
