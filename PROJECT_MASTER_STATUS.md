# PROJECT MASTER STATUS (last update: 25 Nov 2026 01:20, home)

Thesis: dynamic path planning for an industrial robot with ROS 2 Humble and MoveIt 2, on a UR5e (inverted mount) with
a Robotiq 2F-85. Deadline 5 Dec 2026 (moved from 20 Nov, user 15 Nov). This file summarises the whole project; the details are in the daily reports
(`daily_progress/`) and the documents named below. Moved to the workspace root 10 Nov (user OK);
the user asked for the workspace root: move it with the user's OK.

## 0. NOW (25 Nov 2026 01:20): lab Wed 25 Nov = the only replanning lab day
- Project end MAX 15 Dec 2026. Core concept = replanning with a human in the workspace.
- Installed SC3 **v2.43** (md5 e184273a, freeze 41 files). Main config **E2F** `HUMAN=1 FIX=1 bash run/b_take.sh`
  (human mode + Fix A + Fix B): simulator 0 contacts (40 + 18), live-like 4 moving detours / 13 eligible conflicts,
  11 stops vs Exp1 33. Fallback E2 (`HUMAN=1`), E1 (`NOSTOP=0 CANCEL=1`). b_take refuses MPC / MPCSTOP / FIX alone.
- Runbook `test_results/lab_plans/LAB_WED_20261125_RUNBOOK.md`; report `test_results/lab_20261124/REPLANNING_HUMAN_REPORT.md`;
  protocol `test_results/EVALUATION_PROTOCOL_REPLANNING.md`; after each take `experiments/lab/session/verify_switch.py`.
- Kinect 1 only for motion (K2 unplugged). Not adopted: Fix H (v2.45), Fix J (v2.44), MPC variants.
- Pick & place = blocks into an open box (static, camera-found); camera check + force sensor Wed (no motion).
- **Lab roadmap (user 25 Nov):** Wed 25 Nov replanning · Sat 28 Nov pick & place · Tue 1 Dec integration (no
  marathon: fix only what Saturday showed) · Wed 2 Dec final experiments + paper. E2F stays the protected baseline.

## 1. Architecture (production, frozen 4 Nov)
Kinect RGB-D (`src/kinect_camera` kinect_node) → depth obstacle detector (`src/object_localization`
depth_obstacle_detector: crop, ignore zones, self-filter, clustering `cluster_mode: xy`) → tracker
(`src/object_tracking` object_tracker, constant-velocity Kalman, prediction; occlusion hold v4.1 on:
`occlusion_hold: true`, `occl_freeze: true`) → `dynamic_object_msgs/DynamicObjectArray` → scene_updater
(`src/dynamic_planning`, `mode: swept`) → MoveIt planning scene → planning and event-triggered replanning.
Frozen production additions: package A (per-link self-filter radii), package B (detector cap 20 Hz), occlusion hold
v4.1, camera collision objects (`experiments/lab/camera_collision.py`) and frame padding
(`experiments/lab/virtual_limits.py --frame --pad 0.05 --top 1.59`), both mandatory before any arm motion.
Protected-file check: `experiments/candidates/dual_kinect/tests/check_kinect1_untouched.sh` (16 files).

## 2. Planner and replanning
- Planners compared: RRT, RRT-Connect, RRT*, Informed RRT* (simulation). Physical runs use RRT-Connect.
- **Frozen SC3** `experiments/candidates/strategy_b_detour_a2s_g1_sc3.py`, md5 `1b38d729…` (milestone 24 Oct).
- **SC3 rule changed (user, 20 Nov 2026): SC3 is no longer frozen in place.** Baseline = git tag `sc3-baseline-1b38d729`
  + read-only copy `experiments/candidates/strategy_b_detour_a2s_g1_sc3_baseline_1b38d729.py` (md5 1b38d729; the
  harness "baseline" mode runs it). The original file becomes SC3 v2 (version line + change log at the top, one commit
  per edit; no edit before the 20 Nov ROUTE-FAST batch has finished). Every harness / lab run logs the md5 of all planner
  files loaded (`experiments/replan/harness/planner_md5.py`); ROUTE / ROUTE-PAD / ROUTE-FAST import SC3, so SC3 v2 edits
  change them too. Unchanged: no escape recovery, Rule E, lab use only with the user's approval.
- Escape recovery: used ONLY in the 5 Oct simulation comparison (130 runs); DISABLED for physical experiments
  (exact-mesh study: escape moves could carry the arm toward the obstacle).
- Planning harness (domain 88, mock arm): Stage 2 baseline (`experiments/replan/harness/STAGE2_REPORT.md`), Stage 3
  (`STAGE3_REPORT.md`): H with 3 s horizon FAILED (contact), dropped. H 5 s + `h_estop` fix under test: 482 clean
  runs kept (1 invalid + 12 partial excluded); remaining runs follow `STAGE3X_RUN_PLAN.md` case A / B / M, chosen from
  Tuesday's e2 braking measurement; nothing starts before the user's OK. Stage 4 on hold until H 5 s with the fix passes.
- Real run 140445 (−0.12 cm contact, 28 Oct): the arm was parked; the virtual box moved into it; not the e-stop.

## 3. Perception
- Kinect 1 = baseline and fallback (must stay working). Kinect 2 calibrated (grid fit, `WED_DIR=…wednesday_20261104_grid`).
- Package G dual fusion (best2, GS + split_reseat + k2_release): offline OK, OFF (`package_g.sh`; G-ON only typed).
- Package S (sigma cap 0.05 m): built, OFF. Packages H, I: candidates, not installed.
- Principle (user 10 Nov night): **quality-based dual-Kinect perception with single-camera fault tolerance** — both cameras
  equal; agreeing reliable views fused; the better view used per object when quality differs; either camera carries
  perception if the other fails; re-admission only after stable streams. Package G (best2) implements this in part; gaps
  and the 11 Nov tests (D1 stationary, D2 selection / fusion, D3 dropout + recovery, D5 motion + CPU) are listed in
  `experiments/lab/session/LAB_11NOV_CHECKLIST.md`. G stays OFF; production = Kinect 1.
- 10 Nov: hub link for the robot PASS; K2 pipeline + motion overloaded the laptop (K2 depth 20.9 Hz, K1 gap 1.7 s).

## 4. Human detection (new, experimental, not connected to tracker / scene_updater / MoveIt)
- YOLO26n-pose exported to OpenVINO (`experiments/yolo/pose/`), offline ≈ 70 ms per frame, 7.9 frames/s (9 Nov).
- Skeleton `experiments/human/pose_skeleton.py`: keypoints → registered depth (median window, no depth = no point)
  → world via existing TF → body parts (sizes UNTUNED in `experiments/human/config/body_parts.yaml`). Offline-tested on
  K1 bag replays only. No tracking yet. Shoulder / face depth often missing (9 Nov).
- Fail-safe requirements (planned, not implemented): hold people through short occlusions, conservative uncertain
  parts, moving depth objects near the robot = possible person, no blind continuation on stale data, missing keypoints
  never free space.
- Articulated person simulation: planned Thu/Fri. MoveIt human mode: 12–16 Nov.

Dev (9 Nov, offline): skeleton v2 (shoulder 3D 51/54 vs 19/57); labeller PERSON/BOX/UNKNOWN + POSSIBLE_HUMAN fail-safe and a synthetic person in `experiments/dev/` (`test_results/dev_20261109/DEV_REPORT.md`). Lab freeze `experiments/lab_freeze_20261110/` (verify_freeze.sh). Labelling rule (CLASS + MOTION axes, UNKNOWN stays an obstacle, POSSIBLE_HUMAN, class memory): `experiments/docs/OBSTACLE_CLASS_MOTION_RULE.md`; focus: PERSON and BOX well known. Draft 11 Nov checklist `experiments/lab/session/LAB_11NOV_CHECKLIST.md`.

## 5. Lab box (obstacle only): for 10 Nov the GREEN BOX 20.5 × 21 × 7.5 cm (name per user 10 Nov; the cardboard box comes later)
Readout `experiments/lab/session/obstacle_readout.py` (ID, centre, W×D×H, static/moving, held, source, BOX label);
profile `experiments/config/box_profile.yaml`, tape values to be entered 10 Nov. No container functions.

## 6a. Goal (user 9 Nov 23:05)
Finish camera vision PERCEPTION on Tue 10 Nov; build the human-and-box candidate Tue night (plan `experiments/docs/HUMAN_BOX_CANDIDATE_PLAN.md`); first robot test Wed 11 Nov; 17–18 Nov fixes + video. Obstacles: PERSON (body parts, camera-measured sizes, pose_skeleton_v3) + BOX (green box 20.5 × 21 × 7.5 cm; camera size, tape = ground truth); UNKNOWN stays an obstacle; POSSIBLE_HUMAN fail-safe. Lab freeze: 731 files, FREEZE_MD5.txt 6b791d43….

## 6. Tomorrow: lab Tue 10 Nov 13:00–17:00
Checklist `experiments/lab/session/LAB_10NOV_CAMERA_DAY_CHECKLIST.md` (FINAL SCHEDULE section). Take log
`test_results/lab_20261110/TAKES.md`, problem log `test_results/PROBLEM_LOG_1011.md`.
**Disk:** caches deleted 9 Nov (19 GB free); 50.2 GB backup originals + old ~/.ros/log being copied to USB (Kingston, `Nour/`), deletion only after the user's "delete" → about 69 GB; short-take plan needs ≥ 41 GB at start (`test_results/lab_20261110/DISK_CLEANUP_PROPOSAL.md`). Obstacle mode: production has no mode switch (N/A); behaviour = all objects.

## 6c. USB backup (10 Nov 11:37)
Bag originals 50.2 GB on Kingston `Nour/ur_bags_originals_20261019`: verified 0 failed at 00:29; the drive was later unplugged without unmount → re-read sha256 before any deletion. Code + results copy NOT complete (no manifest). Nothing deleted from the backup.


## 6d. Lab 10 Nov results (details: daily_progress/2026-11-10.md)
- K2: no recalibration (user); marks within 1.1–1.2 cm of the real marks; pitch 0.45° from reference. K1: box on P08 within 1.5 cm, no recalibration.
- f: robot link through the hub PASS (gap 0.40 s vs 0.77 s direct); Kinect 2 through the hub FAIL (load: 20.9 Hz depth, K1 gap 1.7 s; one USB dropout). Wed 11 Nov (corrected by the user 23:30): both Kinects connected all day (K1 USB-A; K2 + Ethernet on the USB-C hub); dual operation validated stationary first, during motion only if the CPU check D5 passes; G stays OFF.
- e2: 12 clean stops; h_estop travel 0.008 / 0.035 rad (0.1 / 0.2), 6× shorter than sc3_estop at 0.1; effective decel 5.8–6.6 rad/s². **Harness setting B (5 rad/s²) proposed by the user**, to confirm on the .npy / bags (none may be < 5).
- Green box (name, not "tall box") = 20.5 × 21 × 7.5 cm; camera 19.3 × 20.9 × 8.5.
- PERSON A: 10 K1 + 2 K2 takes; live skeleton exceeded the CPU limit (177 %, 84–89 % all cores) → tonight's item 1 = CPU fix.
- Freeze nov10_freeze VERIFIED; bags in the archive ur_bags_lab (bag_check ALL OK); off-laptop copy pending.
- Tonight (user's list): 1 CPU fix · 2 LAB_11NOV checklist + run_11nov.sh · 3 person simulation · 4 human-and-box candidate · 5 Kinect URDF candidate · 6 harness Stage 3x case B overnight.

## 6e. Night 10 → 11 Nov (details: daily_progress/2026-11-11.md, test_results/FINAL_REPORT_NIGHT_20261111.md)
- Wed 11 Nov lab: checklist v4 + run_11nov.sh v4, freeze `experiments/lab_freeze_20261111` (753 files). Package G GATED (user 00:40): typed G-ON only after D1, D2, D3 (incl. K1 dropout), D5 with fusion pass.
- **Recovery gate fails on real K1 data → `recovery_gate_s 0.0` at D1 (user decides).** Thursday option: LOST 2 s / gap 1.5 s / stable 10 s (start parameters).
- G dry run on domain 88: switch ON / OFF verified, K1-silent and watchdog auto-OFF PASS; 3 G bugs fixed (fusion start crash, fusion JSON crash, flaky param reads).
- Harness case B: fix (h_estop) 0 contacts / 400 standard runs; no fix 1 / 80; sweep contacts only at leads ≤ 1.0 s; summarize.py done (STAGE3XB_SUMMARY_TABLE.txt).
- Skeleton v4 + TA v3 next to the replayed stack: max core 46–48 %. Candidate ladder (additive): L3–L5 PASS, L1 / L2 FAIL on MoveIt planning. J: xacro identical to the runtime objects.
- Two PERSON bags with truncated split file 0 (beside, hand_near_flat; archive identical).

## 6b. Open decisions
- Box held by a person: part of the person or a separate BOX (carrybox take skipped 10 Nov; open).
- Calibration-tilt fix (box height under-measured): production change, only with the user's approval.
- Recovery gate thresholds (Thursday); record_light.sh; deleting 3.7 GB unpacked .db3; Kingston copy of 10 Nov bags.

## 7. Standing rules
Domain 88 for tests (86 = the user's simulation, 0 = lab); stop only own processes; nothing writes into the archive;
Kinect 1 pipeline untouched; robot model frozen; propose, then wait; long runs: lid action "nothing" + inhibitor,
confirm first; daily progress document + this file before closing the laptop.

## 8. Remaining thesis work
Evaluation chapter numbers (Ch. 4 [TODO]/[FILL] items), Overleaf compile check, per-day attendance list (Section 3.18),
e2 limitation note (only shoulder_pan and wrist_3 measured; noted in `thesis/rewrite_20261105/REVIEW_NOTES.md`).

## 11 Nov 2026 (lab day + evening)
Details: test_results/lab_20261111/DAY_SUMMARY_20261111.md.
- **Dual failover live:** K2 drops → K1 only; K2 driver-only restart → paired again. G stayed OFF (the watchdog demo script printed FAIL on its parameter check).
- **S7 detour:** HOLD when START or GOAL is inside the box region. P03m12 reached twice, run 2 with 0 holds. Next planner goal: fastest time with zero stops, replanning from the current point.
- **Person:** live skeleton facing test, 6 one-person takes, person sim (camera-measured body, xyz labels, human look). New live avatar node v2 (display only), not yet live-tested.
- **URDF (user request):** Kinect 1 and 2 boxes moved to the calibrated poses. Collision unchanged; backup backups/urdf_before_kinect_real_20261111/.
- 12 Nov (home): 3 cm contact rule (user). Padding-only SC3-PAD batch 68 of 110 (nominal stopped by a harness start check). Re-score of SC3 / PAD / H: person_reach fails for SC3 and PAD; H fix 400 of 400 pass 3 cm; P02 = goal infeasible under the margins; 0.06 m/s threshold UNTESTED on real data. Doc: experiments/replan/harness/STEP1_PAD_RESULTS_20261112.md. Git: noreply, reports only. Details: daily_progress/2026-11-12.md.
- 15 Nov (home): deadline 5 Dec. Gemini audit verified (experiments/docs/GEMINI_AUDIT_VERIFICATION_20261115.md). Week-1 checks + N1 benchmark: experiments/replan/harness/RESULTS_20261115_WEEK1_AND_N1.md (#2 rejected by G4; G2 load contact -1.9 cm; N1 fixes person_reach, breaks person_sweep). Details: daily_progress/2026-11-15.md.
- 15 Nov late: E2 noise 1.5 mm -> static_on_path 10/10 with the real tracker. Proximity-triggered person cases (hand starts by the arm's real position): SC3 and N1 both < 3 cm in person_reach_prox (arm moving), both pass sweep, N1 +5.5 s; N1's earlier reach gain was a timing confound. detour_feasibility.py: no horizontal detour pair outside a person's reach envelope. DECISION (user): Option C = frozen SC3 stays the primary replanner, N1/N1b shelved, these are Chapter 4 findings. After the 17 Nov lab: verify the scorer's "unnecessary" stop classification (all stops counted unnecessary in the _prox runs).
- 17 Nov (lab, camera day): K1 + K2 recalibrated (box centre error 0.4-1.2 cm), flat-top box rule, still margin 1.5 cm, pin ignore zones REMOVED
  (restore before pick-and-place). Evening: head-height candidate, 18 Nov checklist. Details: daily_progress/2026-11-17.md.
- **18 Nov (lab, real UR5e, G ON):** static S1 / S2 8/8 runs CLEAR PASS (SC3 + N1); S3 goal pocket blocks every variant (GOAL 2.3 cm from
  the box as MoveIt has it). Person head: fixed crown 1.80 m (K1 cannot see a head at the back edge). NEW candidates (user decisions):
  **SC3-ROUTE / ROUTE-PAD** = fastest-not-shortest side choice: 7.7 / 8.2 s vs frozen SC3 12.8 / 11.3 s, 0 holds; mid-leg switch, lanes,
  escape written; virtual-box dynamic test (virtual_box_appear.py): detection 0.07-0.09 s, lane decision 1.6-2.4 s = next bottleneck.
  K2 USB drops crash the UR driver (robot Ethernet on the same hub). Freeze: test_results/session_20261118_184049_lab18_integrated.
  Runtime-only: still margin 1.0 cm (user), person_figure stopped. Person matrix + occlusion NOT run. Details: daily_progress/2026-11-18.md,
  test_results/lab_20261118/NOTES.md.

- 2026-11-20 night: latency step 2 (telemetry, A1 + C1 candidates, A3 cause, dual-Kinect K1-K5), all UNTESTED live, see test_results/latency_baseline_20261120/STAGE_A1_C1_REPORT_20261120.md and experiments/docs/REPLAN_LATENCY_CHECKLIST.md
- 21 Nov: Stage A finish A9-A14 (candidates default off, UNTESTED live; lab block L1-L7 in REPLAN_LATENCY_CHECKLIST.md); report test_results/stage_a_20261121/
- 21 Nov: Stage A decisions recorded (A1+A1b lab L5 order prod -> A1 -> A1+A1b; skeleton 2 Hz; A9 off until L6; A12 dropped; A11 off = thesis limitation); Stage A FROZEN: experiments/candidates/stage_a/FREEZE_STAGE_A_20261121.md5
- 21 Nov: Stage B measured (report test_results/stage_b_20261121/); B10 absorbed-detection finding; proposals pending
- 21 Nov: Stage B candidate tracker built (B10a/B10b/B2/B8 switches), B6 launch fix; report test_results/stage_b_20261121/


## 20 Nov 2026 evening: Stage A / B / K status (details: test_results/REPORT_STAGES_A_B_K_20261120.md)
- Stage A frozen (21 Nov manifest). Lab block L1–L8 for Tuesday.
- Stage B candidate tracker `experiments/candidates/stage_b/object_tracker_stage_b.py` (all off = installed, verified 73/73):
  - Tuesday candidates: **B10 (a) + B11** (harness 40 / 40, 0 contacts, no stop rise; offline near-arm uncovered
    1,837 → 263).
  - Dropped / not recommended: B1, B3, B4 (no gain), B10 (b), b10_arm_skip. B2 optional.
- Det-mode harness (first tracker-in-the-loop harness): SC3 baseline 0 contacts in all B arms. The < 3 cm failures in
  person_reach / _prox are an SC3 property.
- SC3-ROUTE-FAST: 3 contacts in 56 person runs → not for persons. Stage D parked (D8 hybrid, D9 RECOVER timing bug in the
  shared retime(), also in the baseline).
- K: K1–K5 measured; K6 point fusion designed, prototype written, items 2–4 not run.

## 20 Nov 2026 night: A0 closed (home), Stage C accepted (user)
- **A0 evidence** `test_results/stage_a_evidence/A0/EVIDENCE_A0.md`: bag numbers reproduced exactly; replay steps partly
  (explained); production unchanged; telemetry detector = production detections (328/328, 271/271). Live part = lab L1a.
- **Stage C accepted by the user (20 Nov):**
  - C0 bags: track → scene msg 12–38 ms median; /monitored_planning_scene is a 4 Hz display stream, not a delay.
  - C1 `scene_updater_head_event.py` (event_publish, default off): new object 45 → 11 ms (harness). Live: Tuesday.
  - C2 harness: MoveIt apply < 9 ms median (upper bound), swept vs inflated +0.5 ms → no change.
  - C3 session_snapshot.sh running-value check (+6 values).
  - C4 `scene_updater_c4.py` (c4_stamp_topic, default off): smoke PASS (101/101 scene msgs described, 0 errors). Live: Tuesday.
  - C5 future work (Rule E).
- Lab tools (user OK): profile T + /experiment/scene_stamps + /experiment/tracker_stamps; SC3 lab runner switch
  `C4_CANDIDATE=1` (default = production scene_updater). Re-frozen: `experiments/lab/FREEZE_LAB_TOOLS_20261120b.md5`
  (88 files incl. the SC3 runner and the Tuesday candidates).
- **Lab split (user 20 Nov): Monday 23 Nov = A + B, Tuesday 24 Nov = K + C.** Tuesday checklist started:
  `experiments/lab/session/LAB_TUE_20261124_K_C_CHECKLIST.md` (C block done, K block after K6).
- Next (user order): A1 evidence package → K6 items 2–4 → Monday + Tuesday checklists, dry-run on 88, freeze, git,
  Kingston copy by Sunday evening.
- Open: delete the stray decompressed .db3 in bags_lab/E_20261111_141605_lab11_s7k1_P02_1 (needs OK); harness runner does
  not abort when the scene_updater dies (proposal, needs OK).
- **LAB MOVED (user, 20 Nov 23:30): Sat 21 Nov 08:30-12:00, Stages A + B + K calibration take.** Checklist
  `experiments/lab/session/LAB_SAT_20261121_A_B_CHECKLIST.md`, runner `experiments/lab/session/sat21.sh` (one command per
  node swap; md5 + switches logged), freeze `experiments/lab/FREEZE_LAB_SAT21.md5` (112 files, `sat21.sh check`). Dry-run
  on domain 88: `test_results/lab_20261121/dryrun_20261120.md` (3 runner bugs found and fixed: swap_node patterns for the
  candidates, K2 detector / observer excluded by node name, SC3 runner accepts the Stage B tracker).
- K6 items 2-3: prototype + calibration check -> STOP rule (4 Nov box top 2.3-2.9 cm, centre 7.5-8 cm; old calibrations).
  User: option 1 -> calibration take in the lab (box P03 / P05 / P09), `k6_calib_take.sh` on the spot; <= 1.5 cm -> K6
  observer; > 1.5 cm -> K2 recalibration (grid fit, backup, user OK). The 7.5-8 cm matches the K4 9.6 cm source-switch jumps
  (best2 / Package G). Report `test_results/k6_20261120/K6_ITEM3_CALIBRATION_CHECK.md`. Item 4 + B7 after the take.
- AFTER the Sat 21 Nov lab (user note): check BIOS Thunderbolt settings (Thunderbolt Adapter Configuration / security level) = likely reason the USB-C port gives only USB 2 (no Thunderbolt controller in lspci on 19 Nov). Laptop USB-A 2 is broken.


## 21 Nov 2026 (Sat lab): Stage A (details: daily_progress/2026-11-21.md, test_results/lab_20261121/NOTES.md)
- A0-A9 + A14 done (A8, A9 offline from lab bags). Headline: A1 frame->detection 55 -> 15 ms; A1b 29 Hz; A4 FK=TF within
  0.2 mm at speed; A5 K2 on the hub 10 min clean; A8 K1 latency 25-39 ms; A9 +3 cm hand near the arm, 0 false obstacles.
- Skeleton 2 Hz fails CPU with motion -> off. K1 re-levelled live (reverts on stack restart).
- Decided (user): A-d = A1 + A1b; A9 = separate switch a9_keep_foreign.
- **13:43 INSTALLED A1 + A1b in production (user override of the Kinect 1 rule): detector bc74b432, YAML a0a3fbf9** (package experiments/install/stage_a_a1b, rollback script, backup backups/stage_a_a1b_20261121_134338). A9-A13 off. SC3 baseline 1b38d729 and tracker unchanged.
- **Rule change (user, 21 Nov): the 18 Nov "no hand near the moving arm" rule is REMOVED**; person tests during motion allowed (speed 0.1, e-stop held, user starts motion).
- Open: A10-A13 (4 PERSON1 takes pending), K take, L8, B10a + B11, freeze check.

## 22 Nov 2026 night (home): SC3 v2.1 → v2.3, item 4 (details: test_results/OVERNIGHT_REPORT_20261122.md, daily_progress/2026-11-22.md)
- SC3 v2.1 (a24e682b) fails in the harness: the in-process joint-state reader starves (37 of 80 runs lost) → not for the robot.
- SC3 v2.3 (`experiments/candidates/strategy_b_detour_a2s_g1_sc3.py` md5 b21aaab9 + `sc3v2_state_reader.py` b20ea462):
  switches `sc3v2_js_process`, `sc3v2_js_reliability` (best_effort), `sc3v2_retime_fix` (D9) added, all default off.
  SC3 + all switches (S-on3): 80/80 reached, 96 stops 0/0/0, 0 contacts; min clearance 0.36 → 0.28 cm (user decides).
- D9 fix: ROUTE-FAST reached 77.5 → 92.5 %. ROUTE-FAST still not for persons (R0: stop path 3.3 s, GIL).
- Run 140445 contact: arm parked, the box moved into it (stale state not the cause), see BLOCK2_ITEM4_REPORT.md.
- Kingston has the full 21 Nov lab data (verified). Production unchanged; Kinect 1 untouched.


## 22 Nov 2026 day (home): Sunday blocks 1–5 + Stage D + Stage E (test_results/SUNDAY_REPORT_20261122.md)
- SC3 file = v2.4 (md5 81bd8f11): v2.3 + logging switches (off by default). Monday: S-on3 set + logging switches,
  gated on the user's call on 1a (one plant-stall speed jump). FREEZE_LAB_MON23.md5 made.
- D items: candidates `sc3v2_d_items.py` (D1, D2, D4), `sc3v2_d4v2.py`; none adopted (no gain or slower). M4 no-stop
  candidate `sc3v2_nostop.py` PARKED, not harness-tested.
- Open: over-speed 0.40–0.74 rad/s over 50 ms in a few runs of every batch incl. the baseline.

## 22 Nov 2026 evening: user decisions applied
- **1a PASSES** (plant-stall rule: no SC3 command ±100 ms + stall in the plant log + same in switches-off batches) →
  SC3 v2.3 (S-on3) runs Monday. Scorer reports plant stalls in their own column.
- **Over-speed cause found** (read-only): new-path join entry from the commanded point (real 0.40–0.44 rad/s), retime
  corners, one-point stop timing; stale cuts already fixed by fresh_cut. Fixes F1 / F2 proposed, not built.
  Monday: any joint > 0.4 rad/s → stop the block (speed_watch.py, T9).
- Thesis numbers: stop 110 / 201 / 231 ms vs new path 1235 / 5508 / 10143 ms (`test_results/THESIS_NUMBERS.md`).
  Planning time = Stage M (after A–R). D4 → v3 (new-path validation only), low priority after the over-speed fixes.

## 22 Nov 2026 night → 23 Nov 01:30 (home)
- SC3 file = **v2.5 (md5 8f7c3a69)**: v2.4 + F1 corner_stretch / F2 entry_clamp switches (off). Overnight F1 + F2
  test **FAILED** the strict rule (1 jump, 14 over-speed episodes not plant), though it reduced over-speed a lot →
  **Monday runs S-on3 alone**. `test_results/sunday_20261122/f1f2/F1F2_RESULT.md`.
- Monday 23 Nov = **Stage B fully live** (plan `test_results/lab_plans/LAB_MON_20261123_PLAN.md`); speed watch
  ALERT > 0.4 / STOP > 0.5; K calibration → Tue 24 Nov 13:00. Freeze 139 files OK.
- Kingston: 21 Nov copy wrongly deleted by me, being restored; then the 109 21 Nov bags leave the laptop.

## 23 Nov 2026 (Mon lab): A + B + C live (details: daily_progress/2026-11-23.md, test_results/lab_20261123/C_RESULTS.md)
- SC3 file now **v2.27 (md5 24cae9bf)**, edited live v2.6 → v2.27 with the user's go-ahead (one file, every version committed;
  `run/b_take.sh` checks the md5). Margins: path 0 cm, contact alarm 1 cm, MoveIt pad 0 (user). No retreat, lift / forward
  recovery ≤ 3 s, no-stop (defer 0.6 s) with an immediate emergency stop when it gives up or the detour search fails.
- **B1 v2.26 touched the user's hand** (path 1.6 cm from a still hand, the deferred stop came 0.8 s late) → fixed in v2.27;
  C-3 on v2.27: 6.4 / 6.4 s, 0 stops with a person walking in. v2.27 is NOT harness-tested yet.
- Stage B tracker INSTALLED 13:31 (user). Tracker delay detection → track 75–126 ms median: the largest stage.
- Stage C live: C1+C4 track → scene 34 → 23 ms, 0 extra stops (C1 = candidate, user decides); C3 values = YAML;
  **C5: scene_updater stale_timeout 0.5 → 0.6 s INSTALLED** (track gaps up to 0.55 s; backup
  backups/scene_updater_yaml_20261123_150439). Thesis text still says 0.5 s.
- Open: **instant replanning** (ready-made routes, home), Hand-E in MoveIt (URDF/SRDF, sim first), read all bags,
  v2.27 harness Rule E, freeze update (FREEZE_LAB_MON23 outdated).
- **23 Nov night (home):**
  - B review `test_results/lab_20261123/B_RESULTS.md` (MoveIt final-check rejects 96 of 491 requests; the gripper
    model in MoveIt is not the cause: Hand-E vs 2F-85 replay 1228 / 1227 valid).
  - Real-robot D / E: stop decision 8 ms median, send → arm slowing 26 ms, new path → moving 95 ms.
  - **User removed Rule E** (no pass/fail gate; numbers still reported).
  - SC3 versions:
    - **v2.28** (811bf5c9): corner stretch on. Harness vs S-on3: 80/80 reached, holds 462 → 120, 2 speed jumps
      (after 2–5 ms splice segments).
    - **v2.29** (b2af8c69, take script): 1 cm from people (≥ 0.30 m tall) / 0 cm from boxes, < 10 ms points dropped,
      clean exit. Harness running.
    - **v2.30** route bank (Stage M) written as a patch, switch default off; overnight harness test next.
  - Tuesday plan: `test_results/lab_plans/LAB_TUE_20261124_PLAN.md`.
- **24 Nov 03:45:** SC3 **v2.36** (de7d93d3), route bank at rest (`BANK=1`): harness 80 paired runs vs S-on3 — holds
  462 → **3**, 80/80, 0 contacts, closest 0.28 → **4.07 cm**, mean time to goal **−2.04 s** (boxes −6 to −13 s, people
  +0.3 to +1.8 s). Exit segfault fixed (v2.31). Freeze FREEZE_LAB_TUE24.md5. Morning report
  `test_results/lab_20261123/MORNING_REPORT_20261124.md`. Tuesday: v2.36 with BANK=1.
