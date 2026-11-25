# Replanning Decision Time: Problems and Solutions Checklist

Source: the PDF "Replanning Decision Time: Problems and Solutions Checklist" (19 Nov 2026), copied here with the same IDs so
later steps can reference it. Only the Status column (and the notes under it) changes as work is done; the text of the
problems and solutions is the PDF's. Status updates are made ONLY for Stage A, the added Stage K (dual Kinect) and the B0 note; every other row keeps the PDF's status.
Status of Stages D and E updated 22 Nov 2026 evening with the user's decisions (D0, D5, D9, E0, E3 sim done; D1, D2, D3, E2 not adopted; D4 open (v3); D6, E4 future work; D7 target not met; no-stop parked until A–R are done).
Status as of 21 Nov 2026 (Stage A finish: test_results/stage_a_20261121/STAGE_A_FINISH_REPORT_20261121.md). Evidence for Stage A: `test_results/latency_baseline_20261120/STAGE_A_REPORT.md` and
`test_results/latency_baseline_20261120/STAGE_A1_C1_REPORT_20261120.md`.

## Goal and rules
Target: from the moment an obstacle appears, the robot starts reacting in 200 ms or less, measured end to end and proven
per stage. Each stage is worked on separately, measured before and after, and compared with the frozen SC3 baseline.

The decision chain has six stages. A delay in an early stage multiplies: a faster detector also shortens the tracker's
3-hit wait.
1. A. Camera + detection: Kinect frame to detected obstacle.
2. B. Tracking: detection to confirmed, moving track with speed.
3. C. Scene update: track to MoveIt's planning scene.
4. D. Decision: lookahead sees the conflict, SC3 decides to continue, detour or stop.
5. E. New path: planning and checking the detour.
6. F. Braking: the robot physically slows down.

Rules for every item:
- Production stays unchanged. Every change is a candidate copy, default off, installed only after approval.
- SC3 RULE (CHANGED by the user, 20 Nov 2026): SC3 is no longer frozen in place.
  - Baseline = git tag `sc3-baseline-1b38d729` + read-only copy
    `experiments/candidates/strategy_b_detour_a2s_g1_sc3_baseline_1b38d729.py` (md5 1b38d729). The harness "baseline"
    mode runs that copy and checks its md5. "Frozen SC3" in older rows of this checklist = this baseline.
  - The original file `strategy_b_detour_a2s_g1_sc3.py` becomes SC3 v2: version line + change log at the top, every
    edit committed separately. No SC3 edit before the B10 ROUTE-FAST harness batch of 20 Nov has finished.
  - Every harness and lab run logs the md5 of the planner files actually loaded (`experiments/replan/harness/planner_md5.py`:
    the node file + every local module it imports). ROUTE / ROUTE-PAD / ROUTE-FAST and the other SC3 variants import
    `strategy_b_detour_a2s_g1_sc3.py`: editing SC3 v2 changes them too.
  - Unchanged: no escape recovery, Rule E, lab use of any SC3 version only with the user's approval.
- Changes that make the robot react earlier are the safe direction. Changes that remove obstacles earlier or shrink
  margins fall under Rule E and need worst-case evidence, or go to future work.
- Measure first, change second. No estimate replaces a measured number.
- Each candidate is tested alone (ablation), then combined, on the same harness scenarios and seeds (setting B).

Columns: Gain is the expected effect (to be confirmed by measurement). Where says whether the work can be done at home on
bags and in the harness, or needs the lab.

## Stage A: camera and detection
The biggest lever: detector rate also sets the tracker's wait (3 hits take about 200 ms at 15 Hz, about 100 ms at 30 Hz).
For a moving person, A9 and A10 matter most.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| A0 | Stage delays not measured | Measure from bags: depth stamp vs receipt, detector time per step, stamp age, TF waits, CPU | Real numbers for every decision below | None (read-only) | Home | Done: bags (20 Nov) + per-step telemetry on replay (20 Nov) |
| A1 | Detector delivers about 15 Hz (cap 20 Hz), camera gives 30 Hz | Profile per step, speed up the slowest part, raise the cap towards 30 Hz if CPU allows | About 30 ms per frame, plus about 100 ms less tracking wait | Low to medium: more CPU | Home, final CPU check in lab | **Done 21 Nov: INSTALLED** (A1 + A1b, detector bc74b432, YAML a0a3fbf9); live 55 → 15 ms frame → detection. test_results/lab_20261121/STAGE_A_BEFORE_AFTER.md |
| A2 | Old frames queue up when the detector is slower than the camera | Queue depth 1 and sensor-data QoS: always the newest frame | Removes backlog, can be 50 to 100+ ms | Low | Home, quick lab check | **Done (verified)**: already KEEP_LAST depth 1 in production; no backlog in the bags |
| A3 | K1 detection pauses over 0.5 s about every 5 s (11 Nov, cause unknown) | Find the cause (background update, TF wait, periodic task) and fix it | Removes the worst delays | Low once the cause is known | Home if visible in bags | Closed live 21 Nov (0 pauses > 0.5 s); 11 Nov cause known; fix never tested (only matters if both cameras record raw depth at once). STAGE_A_BEFORE_AFTER.md |
| A4 | Self-filter can wait up to 0.15 s for robot TF | Measure how often; use the latest fresh TF straight away (age check exists) | Up to 150 ms in bad cases | Low to medium: TF accuracy | Home | **Done 21 Nov** (live at speed: FK vs TF 0.04 / 0.10 / 0.16 mm, 0 late); analysis file pending. STAGE_A_BEFORE_AFTER.md |
| A5 | K2 USB rate drops and dropouts | UGREEN gigabit hub, K2 colour stream off | Stable K2 rate | Low | Lab (10-minute test) | **Done 21 Nov** (10 min, 80/80 legs, 0 ping loss, 0 USB errors) with K2 colour OFF; colour on lost USB packets -> stays off. STAGE_A_BEFORE_AFTER.md |
| A6 | CPU competition from other processes | No ZSTD during live runs, RAM upgrade, optional higher detector priority | Fewer late frames | Low | Home and lab | **Done 21 Nov** (CPU rule met with the skeleton off). STAGE_A_BEFORE_AFTER.md |
| A7 | IR dot patterns of the two Kinect V1s may interfere in the overlap | Compare K1 missing-depth pixels with K2 on vs off; angle cameras if needed | Fewer missed frames in the overlap | Low | Lab (5 minutes) | **Done 21 Nov** (K1 valid pixels 73.2 -> 69.1 % with K2 on, harmless). STAGE_A_BEFORE_AFTER.md |
| A8 | Camera-internal latency (exposure, USB, driver), about 30 to 60 ms | Cannot be changed on Kinect V1: measure and report as a hardware limit | Known number for the thesis | None | Estimate at home, exact in lab | **Done 21 Nov** (K1 25–39 ms; K2 ~18 ms low confidence). STAGE_A_BEFORE_AFTER.md |
| A9 | A hand near the arm is erased by the self-filter (radii 10.5 to 15 cm) | Measure how often; tighter radii where the model is accurate, or keep points that moved in from outside | Hand stays visible until close | Medium: robot parts seen as obstacles | Home (PERSON takes) | **Closed 21 Nov: measured, stays off** (hand 3.2 vs 6.0 cm still arm, 0 false obstacles); install decision later (test_results/stage_a_evidence/A9/) |
| A10 | Body and arm form one blob; its centre moves slowly, so hand speed is underestimated | Report the point nearest the robot (leading edge) and its speed, or use body-part columns | Real hand speed, probably the largest gain for a person | Medium: tracker must handle it | Home (reaching takes) | Closed 21 Nov, stays off (K2 low confidence). STAGE_A_BEFORE_AFTER.md |
| A11 | A person is only seen once inside the workspace crop | Watch zone outside the crop: warning only, nothing added to MoveIt | About 100 to 300 ms earlier warning | Low | Home (full depth image in bags) | Closed 21 Nov, off (thesis limitation). STAGE_A_BEFORE_AFTER.md |
| A12 | Fast hands lose depth for a frame | Measure missing frames; smaller minimum cluster size only near the robot | Fewer missed frames | Medium: noise blobs | Home (fasthand takes) | Closed 21 Nov: DROPPED (adds false objects). STAGE_A_BEFORE_AFTER.md |
| A13 | A flat hand just above the table falls inside the background margin | Measure detection rate; smaller margin only near the robot if needed | Flat hands seen earlier | Medium: table noise | Home (hand_near_flat takes) | Closed 21 Nov, no fix (small sample, 33 frames live). STAGE_A_BEFORE_AFTER.md |
| A14 | Skeleton runs at 2 Hz | Skeleton classifies the person; the fast depth path handles reaction; raise the rate only if CPU allows | Clear roles | Low | Home, live CPU check | Closed 21 Nov: skeleton stays off (2 Hz fails the CPU rule while the arm moves). STAGE_A_BEFORE_AFTER.md |

### LAB BLOCK "Finish Stage A" (~60 min, added 21 Nov 2026; ROS_DOMAIN_ID=0; robot only at velocity scaling 0.1, user starts every motion)
Order chosen so the robot-free items come first and the CPU log covers everything. Record profile T (no images, no ZSTD) unless a
step says otherwise. Every candidate is started from its own switch; production YAML and the frozen SC3 stay as installed.
User decisions 21 Nov: detector switch order in L5 = PRODUCTION first (reference), then A1, then A1 + A1b, each with the CPU log
running; production is the fallback at any problem. Skeleton at 2 Hz; 3 Hz only if the L1 log of the FULL stack (MoveIt, tracker,
UR driver, detector, skeleton) shows the busiest core p95 < 75 %. A9 off except in L6; A10, A11, A12, A13 off for the whole block.
The hub plug-flip was NOT done at home (hub not plugged in on 21 Nov): L2 tests both USB-C orientations FIRST.
**SAFETY NOTE for every test with a person (added 21 Nov, until B10 is validated live):** the installed tracker DROPS
detections that lie inside another track's extent sphere (B10: a hand next to the body can be missing from MoveIt for ~0.7 s,
up to 0.62 m outside the person's box). Robot at velocity scaling 0.1 only, e-stop held in hand by the operator, no fast
reaches towards the arm, the person stays outside the arm's reach unless a step explicitly needs a hand near the arm (L6:
slow approach, arm at 0.1).

| # | Min | Item | What to do | Pass / record |
|---|---|---|---|---|
| L1 | 5 | A6 CPU log (start, keeps running) | `hw_monitor.sh` (or `cpu_sampler.py`) at 1 Hz: per-core %, MHz, package temperature, for the whole block, full stack incl. MoveIt, tracker, UR driver, skeleton 2 Hz | file per step; no core > 85 % for > 1 s in L5 / L6; busiest core p95 < 75 % = skeleton 3 Hz allowed |
| L1a | 8 | **A0 live** (added 20 Nov, evidence `test_results/stage_a_evidence/A0/`) | Full stack running (MoveIt, tracker, UR driver, detector, skeleton 2 Hz) + the L1 CPU log. (1) T3: `DETECTOR_CANDIDATE=experiments/candidates/telemetry/depth_obstacle_detector_timing.py experiments/lab/swap_node.sh candidate detector` (telemetry copy = production output field by field, 328/328 + 271/271 frames on 20 Nov; overhead 0.12-0.15 ms/frame). (2) T4: `experiments/lab/session/record_profile.sh T a0_live` (no images, no ZSTD). (3) 60 s empty table, arm still; 60 s arm moving at 0.1 (user starts the motion); 60 s person walking / reaching beside the table (safety note above). (4) Stop the recording (Ctrl+C in T4), then Ctrl+C in T3 (stops the timing copy; `swap_node.sh installed` does NOT find the timing copy by itself), then `experiments/lab/swap_node.sh installed detector`; check `ros2 node list` shows one /depth_obstacle_detector. Analysis at home: `python3 experiments/lab/session/latency_join.py <bag copy>` (frame -> detection), `python3 test_results/stage_a_evidence/A0/bag_timing_to_jsonl.py <bag copy> <out>` then `python3 experiments/candidates/telemetry/replay_analyse.py steps <out>` (per step), `python3 test_results/stage_a_evidence/A0/rate_scan.py <bag copy>` (rates) | PASS if, per phase: live frame -> detection median within 20 % of the replay median (replay 51-66 ms, live bags 56-66 ms); live TF wait, self-filter and total medians within 20 % of the replay (20.4 / 22-24 / 43-49 ms); telemetry overhead median < 0.3 ms; detector rate within 20 % of 14.5 Hz; CPU rule of L1 (no core > 85 % for > 1 s). Any fail -> back to the installed detector, record why |
| L2 | 10 | A5 hub test | FIRST the plug-flip (not done at home): `lsusb -t` + `ethtool --show-eee` in both USB-C orientations, keep the better one; then K2 through the UGREEN hub, K2 colour off, robot moving at 0.1 for 10 min | K2 depth >= 24 Hz all 10 min, no `-71` in `dmesg`, no UR link drop |
| L3 | 5 | A7 K1 depth, K2 off / on / off | empty table, 60 s each, K1 depth recorded (profile with K1 depth, not T) | K1 valid points per frame and top-surface noise std per phase (as 17 Nov A/B/A) |
| L4 | 10 | A8 camera latency | LED / phone stopwatch filmed by K1 colour + a second camera, or the arm moving: K1 depth stamp vs joint_states when the arm starts | camera-internal latency (ms), median of >= 10 events; report as a hardware limit |
| L5 | 15 | A1 / A1b live | three runs, same person walk / reach, CPU log on: (a) PRODUCTION detector (reference), (b) Stage A candidate with A1 only (fast_arm_pose, fast_boxes, cap 20), (c) A1 + A1b (+ raw_depth, every frame); full stack + skeleton 2 Hz; back to production on any problem | detector rate and latency from `/experiment/detector_timing`; CPU rule (L1); same detections as production on a recorded bag |
| L6 | 10 | A4 / A9 take | hand held next to the MOVING arm (0.1), 3 approaches per side (forearm, wrist, gripper), K1 depth recorded; first with A9 OFF, then the same approaches with A9 ON (a9_keep_foreign) | A4: FK vs TF capsule error at speed; A9: hand-to-arm distance where the hand disappears, production vs candidate (offline afterwards) |
| L8 | 5 | (added 21 Nov, Stage B) detector publish -> tracker receive | profile T; installed tracker started through `object_tracker_timing.py` (A3 wrapper: installed class unchanged + `/experiment/tracker_timing`) or the Stage B candidate with only `b8_stamp_topic` on; detector = the timing copy (`/experiment/detector_timing`); 2 min person walk / reach, Package G off, then on | per detection stamp: detector `end` -> tracker `recv` (same machine clock), K1 path and fused path; this replaces the derived "~150 ms" (see B0 row) |
| L7 | 5 | spare / repeat | repeat the failed step or an empty-table recording with the arm moving (A9 false points) | |

## Stage K: dual Kinect (added 20 Nov 2026 by the user; read-only measurement first; production and Package G unchanged)
Data: 18 Nov person bag G_20261118_133426_lab18_person_C (Package G ON, 514 s) and 4 Nov K1+K2 bag KK_20261104_173503
(K2 depth images). Details: `test_results/latency_baseline_20261120/STAGE_A1_C1_REPORT_20261120.md` section K.

| ID | Problem | Measured | Adds delay? | Proposed fix (not built) | Status |
|---|---|---|---|---|---|
| K1 | K2 detector rate 8.2 Hz vs K1 12-14 Hz; per-step times unknown | Telemetry on the K2 detector (replay): TF wait 20 / 61 ms dominates like K1; total 30 / 69 ms; 8.3 Hz = its 10 Hz cap on a 30 Hz camera. A1 candidate on K2: latency 50 -> 20 ms median, same detections; cap 30 -> 18.5 Hz | Yes: ~30 ms TF wait per frame + the cap | A1 candidate (own FK) for K2 too; K2 cap 10 -> 15-30 Hz only if CPU allows (lab CPU check) | Measured; candidate replay-tested (UNTESTED live). DECIDED 21 Nov: A1 on the K2 detector / higher K2 cap = lab CPU check Tuesday. |
| K2 | Fusion pairing wait | No wait: one fused output per K1 frame (+2.8 ms), partner = nearest K2 frame already received if within 0.1 s; that K2 frame is 52 / 89 / 99 ms older than K1; 16 % of fused frames without K2 | Not for K1 objects; K2 information is ~50 ms older | Keep; option: also emit on a K2 frame that carries a K2-only object | Measured. DECIDED 21 Nov: no separate K2-only emit; handled in K6 (K2 points merged where K1 does not see, K2 frame <= 100 ms old). |
| K3 | Objects seen by one camera only | Passed through (K2-only 98, K1-only 585, both 93 output objects); 2 of 19 entering objects seen by K2 0.45-0.76 s before K1, fused output 0.41-0.72 s earlier than K1 | No (earlier), but K2-only objects wait for the next K1 frame | Same as K2 | Measured |
| K4 | ID switches / centre jumps at source switch; cam2 parameters | 2 id switches in 8.6 min; centre jump at a camera-source switch 9.6 / 14.0 / 15.0 cm vs 1.0 / 9.7 / 15.0 cm without; cam2_position / look_at / roll reach the fusion (17 Nov values); cam1_* NOT passed: code defaults differ from the 17 Nov K1 calibration by 1.6-2 cm, look-at 3.3 cm, roll 0.4 deg | Fake speed from jumps (more swept volume / stops) | Pass cam1_* from the K1 calibration in the launch; source-switch hysteresis / blend (B6) | Measured |
| K5 | K1 depth with K2 on vs off | 17 Nov A/B/A (existing notes): K1 top noise std 0.61 / 0.25 / 0.57 cm, valid points 35 / 70 / 38 per frame, table std 0.65 / 0.30 / 0.64 cm; K1 depth rate 22.6 Hz with K2 on the same USB bus (28.5 before) | Fewer points, more noise; rate drop when sharing USB | Lab A/B with the new hub layout (A7) | From existing data; lab test later |

## Stage B: tracking
The tracker waits twice on purpose: 3 hits to confirm a new object is real, then 0.1 s to confirm it is moving. Both
protect against Kinect noise (a still box jitters about 2 cm). The fixes skip the waits only for movement far larger than
noise.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| B0 | Tracking delays not measured | From bags: first detection, first confirmed track, first reported_moving, per event (median, p95, max) | Real numbers | None (read-only) | Home | Measured (21 Nov, offline bench, installed tracker, 13 takes, no transport delays): first detection -> confirmed median 278 (prod) / 242 (A1) / 176 ms (A1+A1b); -> moving 446 / 380 / 342 ms; paired A1b vs prod: 276 -> 111 ms confirm, 443 -> 292 moving. 33 % of new objects wait 705 ms median because their detection is ABSORBED by a confirmed track's extent sphere (dropped, not added to any box: sticks out up to 0.62 m, 672 cases > 5 cm within 0.5 m of the arm) - see report. NOTE on the "~150 ms transport delay" (20 Nov): NOT a direct measurement. Derived on G_20261118_133426_lab18_person_C (Package G on) as 354 ms (first detection stamp -> confirming FUSED message received by the BAG RECORDER, record time) minus 202 ms (3 hits at cap 20); it lumps detector TF wait + processing (~55 ms production), fusion, transport and recorder delivery. Direct numbers in the same bag: K1 stamp -> K1 detection at the recorder 59 / 120 ms, fused 98 / 189 ms (median / p95). The tracker's own receive time was never recorded -> lab step L8 |
| B1 | 3 hits needed before a new object is sent on | Confirm after 2 hits when the jump between frames is far above noise | About 50 to 70 ms | Low to medium: noise blob sent as object (extra stop, safe) | Home | DROPPED 20 Nov: det-mode harness B1+B2 vs B2 (40 + 40 runs) not clearly better (same reach / clearance / time, cross_slow stops 4 -> 8); code kept (b1_two_hit, off). |
| B2 | 0.1 s wait before an object counts as moving, so no swept volume yet | Skip the wait when the speed is clearly high (well above noise and statistically significant) | About 100 ms | Low to medium: false moving objects, more stops | Home | BUILT 21 Nov (b2_fast_moving, default off, UNTESTED live). Det-mode harness 20 Nov (40 runs): no measurable gain vs the plain tracker (same reach / clearance / time to goal, 0 contacts); optional, no lab switch recommended. |
| B3 | Speed estimate converges slowly on a new track | Start the speed from the first two positions; higher process noise for young tracks | Earlier, more accurate speed | Medium: noisy speed early | Home | DROPPED 21 Nov (what-if: no effect) |
| B4 | Body-centre speed is too low for a reaching hand (see A10) | Track the leading edge or body-part columns, not just the blob centre | Swept volume follows the hand, not the body | Medium | Home | BUILT + HARNESS-TESTED 20 Nov (b4_leading_edge + b4_require_moving, default off, UNTESTED live). First version: fake leads on still box / torso (static 0/5, 6 extra timeouts) -> gate added. Gated: 40/40, 0 contacts, no rise in stops, but NO clearance gain (person_reach min 0.3 cm as without) -> fails the >= 3 cm rule -> NOT in the Tuesday plan. Open option: det-mode 'blob' test (hand + body one detection). |
| B5 | Dual mode: fusion waits to pair K1 and K2 detections | Measure the pairing wait; shorten the pairing window if large. Keep pairing (no alternating cameras) | Up to about 50 ms | Low | Home | Measured (K2, 20 Nov): no pairing wait (+2.8 ms); no fix needed |
| B6 | K1/K2 offset 2.6 to 3.7 cm makes the centre jump when the source switches, giving fake speed | First check cam2 parameters are passed correctly (look_at, roll); fix the launch. Bias trim only as a validated calibration step | Fewer fake movements and false stops | Low (launch fix), medium (trim) | Home, check in lab | DONE 21 Nov in the experimental dual launch (dual_kinect_v2.launch.py: cam1_position / look_at / roll = K1 calibration of 17 Nov, passed to the fusion; verified the node reads them; backup .bak_20261121_cam1). best2 uses the poses for camera choice: re-measure the K4 source-switch jump on the next dual lab bag |
| B7 | Occlusion hold may freeze a moving hand when the arm blocks K1's view | Check that moving objects are never held; with dual mode, let K2's clear view keep the track updating | Hand keeps its real position | Medium | Home | Measured 21 Nov: 18 SHORT holds (<= 0.5 s), 0 LONG; 220 moving tracks published frozen 0.67 s median after losing measurements (mostly id churn). Re-measured with B10 (a): identical (B10 only adds published geometry) |
| B8 | Tracker overwrites the measurement timestamp | Keep the camera stamp in the output for latency logging | Exact latency per stage for the thesis | None (telemetry) | Home | BUILT 21 Nov (b8_stamp_topic, telemetry only, default off): /experiment/tracker_stamps per publish (publish time, last input stamp, per id last measurement stamp); tracks identical on/off (965 ticks); measurement age median 35 ms, p95 0.66 s |
| B10 | (added 21 Nov) Detections inside a confirmed track's extent SPHERE are dropped: no new track, not added to the absorbing box | Keep the rule (no duplicate ids) but add the absorbed detection's space to the absorbing track's published geometry (extra part), and/or test the box instead of the sphere | Hand next to the body in MoveIt at once (~0.7 s earlier for 33 % of new objects) | Low: only adds space (Rule E safe direction) | Home | (a) BUILT 21 Nov (object_tracker_stage_b.py b10_absorb_parts, default off, UNTESTED live): uncovered detection space > 3 cm 11.9 % -> 2.4 % (absorbed type 10.2 -> 1.0 %), detections > 10 % uncovered near the arm 1837 -> 370, published volume +22 %, ids unchanged, CPU +0.2 ms p95; t3 arm leftovers +25 % volume. (b) b10_box_test: 7.0 % alone, ids 160 -> 228, no gain with (a) -> not recommended. Harness false-stop test needs a track-replay mode (not built) 20 Nov: harness B10 (a) (SC3) 40/40, 0 contacts; MoveIt 'resting arm in collision' = always the hand track, never a B10 part (probe, 22 runs); static-box stop rise = variance (0 parts on the box in 5 reruns). b10_arm_skip built, 0 parts skipped at 0.15 m on the real takes (B10 (a) attaches no arm leftovers) -> not recommended. B10 (a) + B11 -> Tuesday candidate. |
| B11 | (added 21 Nov) Own confirmed tracks lag the detection: published columns / smoothed box smaller or shifted vs the newest detection (167 of the 370 near-arm cases left after B10 (a); hand held near the arm take 160 of 370) | For MOVING objects publish the union of the raw (newest) detection box and the smoothed shape (only adds space) | Track shape follows the hand without lag | Low (adds space; volume to be measured) | Home | BUILT + OFFLINE TESTED 20 Nov (b11_raw_union, default off, UNTESTED live): near-arm shape-lag cases 167 -> 60 with B10 (a) + B11, near-arm uncovered 370 -> 263, volume +33 % over B10 (a); remaining: almost-still hands (B11 acts on moving tracks only). Needs the det-mode false-stop check before a lab switch. Det-mode harness 20 Nov (B10 (a) + B11, 40 runs): 40/40 reached, 0 contacts, stops as with B10 off (static box 0) -> candidate for a lab switch together with B10 (a). |
| B9 | Speed lingers after a sudden stop or turn | Measure only. Damping removes swept volume earlier, so it falls under Rule E | Fewer unnecessary stops | Rule E | Home | Future work |

## Stage C: scene update
The scene updater receives tracks about 30 times per second but sends them to MoveIt on a fixed 10 Hz timer, adding up
to 100 ms.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| C0 | Scene delay not measured | From bags: track publish time vs first planning-scene message containing the object and its swept volume | Real numbers | None (read-only) | Home | Done 20 Nov (bags, test_results/stage_c_20261120/C0/C0_SCENE_DELAY.md): track -> scene msg 12-38 ms median (p95 <= 50, max 118); new object track -> scene_updater add 40 / 105 ms; depth -> scene 109-209 ms median. /monitored_planning_scene is a 4 Hz display stream (MoveIt default publish_planning_scene_hz), NOT the apply time: MoveIt's internal apply time -> C2 (harness) |
| C1 | Fixed 10 Hz timer | Send immediately when a new or fast object appears or its speed jumps; throttle to 20 to 30 Hz | Up to 100 ms (about 50 ms on average) | Low: more messages to MoveIt | Home | Candidate built + harness-tested (20-21 Nov night, UNTESTED live): experiments/candidates/scene_updater_head_event.py, switch event_publish (default off): new id / speed change > 0.15 m/s sent at once, max 25 Hz, 10 Hz timer kept. Harness: new object track -> /planning_scene 45.5 / 86.9 -> 10.8 / 13.6 ms; track -> SC3 OBJECT_SEEN 53-66 -> 11-21 ms; CPU unchanged; 9/9 reached both. Bags (C0): 40 / 105 ms on a new object = the gain |
| C2 | MoveIt may need time to apply many swept copies | Measure the time move_group takes to apply each update; reduce copies only if it is significant | Unknown until measured | Medium: fewer copies cover less | Home | DONE 20 Nov (harness, measurement only, test_results/stage_c_20261120/C2/C2_APPLY_TIME.md): 16 runs, 615 diffs, 0 timeouts. Apply time < one /get_planning_scene round trip (visible in the first poll every time): upper bound swept 8.7 / 16.5 / 30.0 ms (med/p95/max, up to 12 primitives), inflated 8.2 / 15.8 / 27.0 ms. Swept copies cost ~0.5 ms -> not significant, no change |
| C3 | Code defaults differ from the YAML values (margins, tracker mode) | Startup check that the YAML was loaded; do not edit production defaults | Prevents silent wrong settings | None | Home | DONE 20 Nov (user-approved): session_snapshot.sh key_params check +6 values (still_safety_margin 0.005, still_sigma 0.01, table_clip_z 0.71, smooth_release_frames 3, occl_freeze true = occlusion_hold class, velocity_chi2 1.0); tested on the 18 Nov lab dumps: flags the 18 Nov override still_safety_margin 0.0 as DIFFERS, rest OK. Lab tools frozen: experiments/lab/FREEZE_LAB_TOOLS_20261120.md5 (81 files, verify OK). Production unchanged. Finding: occlusion_hold is not in the running tracker's param dump (read once to choose the class) -> checked through occl_freeze. |
| C4 | No end-to-end latency log | Log camera-to-scene time per object | Thesis evidence for every stage | None (telemetry) | Home | Candidate built + harness smoke PASS 20 Nov (UNTESTED live): experiments/candidates/scene_updater_c4.py, switch c4_stamp_topic (default off) on top of the C1 candidate; /experiment/scene_stamps per scene message: id, op, track_pub, cam (B8), det_end (A0 timing detector), ms per stage. Smoke: 101/101 scene messages described in order, 0 errors, cam matched 143/143; track->scene 34 ms median, new object 5-14 ms, cam->scene 131 ms (test_results/stage_c_20261120/C4/C4_SMOKE.md). Tuesday C block with C1 |
| C5 | Stale objects stay 0.5 s after they leave | Do not shorten (removes obstacles earlier, Rule E). Report as a design choice | n/a | Rule E | n/a | Future work |

## Stage D: decision (lookahead and SC3)
Today SC3 reacts when a conflict is found on the near trajectory, and checks each detour state through a ROS service call
(3.4 to 5.9 ms per state). Under the 3 cm rule, SC3 fails person_reach 10 of 10, so this stage decides the main thesis
result.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| D0 | Decision delay not measured | From harness logs: scene update, first lookahead conflict, SC3 decision, first command change | Real numbers | None (read-only) | Home | **Done** 22 Nov (harness): conflict → stop trajectory sent **30.6 / 89 / 99 ms** (S-on3, median / p95 / max); camera chain **detection → stop sent 110 / 201 / 231 ms** (B 7, C 28, D1 37, D2 26 ms median; `test_results/sunday_20261122/b4a/BLOCK4A_HEADLINE.md`). For comparison, the new-path decision (E0) is 1235 / 5508 / 10143 ms. `test_results/overnight_20261122/r0/R0_DECISION_TIME.md` |
| D1 | Lookahead check rate unknown | Measure the check period; raise the rate if checks are cheap | Earlier conflict detection | Low: CPU | Home | **Not adopted** 22 Nov (harness, 80 runs, switch `sc3v2_d1_period`, `sc3v2_d_items.py`): 25 ms makes the period 51.7 → 26.6 ms, but conflict → stop goes 30.6 → 42.1 ms median (p95 89 → 80), time to goal +0.11 s, 1 timeout. Keep 50 ms (`test_results/sunday_20261122/d_items/D_ITEMS_SUNDAY.md`) |
| D2 | SC3 reacts only on a hard conflict | Use the existing early-warning distance (soft clearance) to start planning the detour before the conflict | Detour ready sooner | Low to medium: more replans | Home | **Not adopted** 22 Nov: `sc3v2_d2_early` (v2 also on full trajectories). 4 cm (80 runs) and 10 cm (40 runs): Rule E pass, time to goal −0.03 / +0.06 s, warning lead 15 ms median → safe, no gain |
| D3 | No reaction until a detour or stop is decided | Slow down immediately on warning while the detour is planned in parallel | Reaction becomes almost instant | Medium: unnecessary slow-downs | Home, then lab | **Not adopted** (not built) 22 Nov: warnings come 15–132 ms (median) before the hard conflict, so a slow-down has almost no time to act. Goes to Stage M (M4) |
| D4 | Service call per detour state (3.4 to 5.9 ms each) | Check states in-process in C++ (existing plan_check) in a candidate copy of SC3 | Much faster checks on long detours | Medium: new code path | Home | **Open: D4 v3** (user 22 Nov evening). Built 22 Nov (`sc3v2_d4_cpp_check`): in shadow mode C++ agrees 166/166, per check 34.7 → 21.6 ms median. Used in the stop path (v1 polling, v2 event wait), conflict → stop got slower, 30.6 → 42–45 ms median → **keep the C++ check out of the stop path**. Next (low priority, after the over-speed fixes): v3 uses it only to validate new paths during planning |
| D5 | P02 goal is 5.7 cm from the box, inside SC3's 6 cm margins | Report as "goal infeasible", separate from timeouts; use valid marks; never lower margins to make it pass | Honest results | None | Home | **Done** 22 Nov: the scorer reports P02 timeouts as goal infeasible, and setup errors separately |
| D6 | Time-matched checking (checkHandover) could replace swept volumes | Future work: assumes constant velocity, unsafe for a person who changes direction | Fewer unnecessary stops | Rule E | n/a | **Future work** |
| D8 | SC3-ROUTE-FAST: lanes never helped with persons (0 splices), lane attempt costs ~0.5 s per failed decision and starves SC3's global plan on a static box (3 + 3 timeouts in the 20 Nov det-mode batch) | SC3-HYBRID (proposed 20 Nov): SC3 keeps its planning time first, lanes in parallel / leftover time only, only for still or slow objects, lanes cached per still box, early drop of lanes whose splice point passes; each part behind a switch, all off = SC3 v2 | Box detours without starving SC3 | Medium: new candidate | Home | Design written 22 Nov (`test_results/overnight_20261122/D_ITEMS.md`), not built: R0 shows ROUTE-FAST's lane work starves the monitor (stop path 3.3 s median vs SC3 44 ms). SAFETY NOTE 20 Nov stands: ROUTE-FAST not for persons (item 4: 2 contacts in 43 runs even with the v2.1 switches) D7 split 22 Nov (R-on3): lane splice 585 / 602 / 605 ms, screen 206 ms median, 88 % of lane decisions fall back to SC3 planning (`test_results/sunday_20261122/d_items/d7_split.txt`); screen-in-a-process built in the parked M4 candidate `sc3v2_nostop.py` |
| D9 | RECOVER timing bug in the shared retime() (`strategy_b_local.py`): a 2-point rest-to-rest path is timed 2*ds/1e-6 s (0.02 rad -> 39,300 s); the arm counts as moving for hours and the planner never plans again. Seen in 17 of 21 ROUTE-FAST timeouts (20 Nov) and in 3 older harness runs (5, 11, 15 Nov); also present in the SC3 baseline 1b38d729 | Fix later in SC3 v2 (own commit): rest-to-rest segment = minimum-time move 2*sqrt(ds/a_max); changes every SC3 variant that imports SC3 | Liveness after a recovery | Low | Home | **Done** 22 Nov: SC3 v2.2 switch `sc3v2_retime_fix` (in the v2.3 S-on3 set). Unit tests ALL PASS (`test_results/overnight_20261122/d9/`); RECOVER 39,300 s → ~0.5 s; ROUTE-FAST reached 77.5 → 92.5 % |

**Real robot, 23 Nov lab (read-only from the take logs, `test_results/lab_20261123/analysis/d0_real.txt`):** SC3 decision
line → stop sent 8 / 216 / 1124 ms (median / p95 / max, n 130, versions before v2.27; the tail = the last resort going
through SC3's cut rules, removed in v2.27). v2.27's immediate stop never fired live → Tuesday D0 takes. D4: only worth it
inside the route-bank refresh (background); the refresh time is now logged (BANK_REFRESH), D4 there only if it is slow.


**Real robot, 24 Nov lab (SC3 v2.37, `test_results/lab_20261124/analysis/problems_24nov.txt`):** D0 live: SC3 decision →
stop sent 3 ms median for last-resort stops (n 14). **The no-stop deferral is unsafe near a real hand: 2 contacts** (bank:
defer 3.3 s while route builds took 1.3–3.6 s; MAXDEFER 1.0: a flickering hand detection restarted the 1 s timer).
D1 (25 ms) live: no visible gain (2 takes). D2 / D3: not run live (harness 24 Nov: safe, no gain, `V237_RESULT.md`).
D4: not possible live (no C++ plan_check in the lab stack). → **SC3 v2.38 HUMAN MODE** (`sc3v2_human_mode`, b_take
`HUMAN=1`): people / moving objects → SC3's immediate decision (no deferral); still boxes keep the deferral + bank.


## Stage E: new path
RRTConnect plans in about 20 to 60 ms (5 Oct evidence); the planner stays. The time goes into checking, smoothing and
handing the new trajectory to the robot.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| E0 | New-path time not split | From harness logs: planning, checking, time parameterisation, sending to the controller | Real numbers | None (read-only) | Home | **Done** 22 Nov (harness). **New-path decision: conflict → first new path sent 1235 / 5508 / 10143 ms** (S-on3, median / p95 / max), mostly MoveIt: one request 141 / 450 / 1605 ms; smoothing 70 / 80 / 120 ms; validation 15 / 48 / 134 ms per cycle; send → accepted 16 / 38 / 113 ms. Thesis numbers next to the stop decision (detection → stop sent 110 / 201 / 231 ms): `test_results/THESIS_NUMBERS.md`. **Planning time belongs to Stage M** (ready-made detours R3, planner budget): not worked on before A–R are done (user 22 Nov evening) |
| E1 | Planner choice | Keep RRTConnect; RRT* and Informed RRT* use the full time budget | Already the fastest option | None | Done | Done |
| E2 | Planning time limit and attempts may waste time | Measure; set the budget to what detours actually need | Shorter worst case | Low | Home | **Not adopted** 22 Nov (R2, `planning_time` 0.5 s, 80 runs): conflict → new path p95 5508 → 3776 ms, no gain in time to goal, 1 timeout → inconclusive (`test_results/sunday_20261122/b4b/R2_RESULT.md`). Planner budget → Stage M |
| E3 | Controller handover delay unknown | Measure time from new trajectory sent to the robot following it | Known number, possibly reducible | Low | Home (sim), lab (real UR) | **Done (sim)** 22 Nov, harness: send → accepted 16 / 38 / 113 ms; send → arm slowing 57 / 97 / 167 ms (stops); → arm moving 64 / 77 / 915 ms (from rest) (`test_results/sunday_20261122/e_items/`). Real UR: Wednesday cancel test |
| E4 | Detours planned only after a conflict | Plan alternatives in the background so one is ready | Detour ready almost instantly | Medium: CPU, complexity | n/a | **Future work** (Stage M, M4) |

**Real robot, 23 Nov lab (`test_results/lab_20261123/analysis/e3_real.txt`):** send → controller accepted 20 / 64 / 157 ms
(n 466), send → arm slowing (stops) 26 / 32 / 34 ms (n 52), send → arm moving from rest 95 / 170 / 556 ms (n 161). E4 =
the route bank (SC3 v2.30 `sc3v2_route_bank`, Stage M), harness-tested 23–24 Nov night.


**Real robot, 24 Nov lab:** E0 conflict → new path sent: 3.5 s median (n 23), stop → moving again 2.7 / 7.1 / 16.4 s
(median / p90 / max, n 58); chosen paths up to 13.7–20 rad (direct ~3.5). Cause: a moving object's region grew with its
PEAK speed × the whole path duration (a hand that once moved at 0.6 m/s: 4.1 m half-width at +6 s) → start state "inside
the planning region" with the hand 12–15 cm away. E2 (planning 0.5 s) live: no gain (5.4 s median, 2 takes). E4 (route
bank) live: **box on the path 7.0 s per leg, 0 stops (no bank: 31 / 20 s)**; with a hand: lifted at every free start
(fixed in v2.38 H4). → v2.38 H2 (growth 0.8 s at the current speed), H3 (fast restart, direct first), H5 (path quality).


## Stage F: robot and braking
Braking is physics, measured on 10 Nov: h_estop stops in 0.12 s at speed 0.1 (0.235 s at 0.2), sc3_estop in 0.35 s,
sc3_safe in 0.82 s. Faster stopping only comes from choosing the faster stop type.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| F1 | sc3_estop needs 0.35 s, h_estop only 0.12 s | Package H (h_estop): 0 contacts in 400 harness runs with the fix; re-score under the 3 cm rule, then a live test | About 0.23 s shorter stop | Medium: needs live validation | Home (re-score), lab (live) | Home re-score done 12 Nov (400 of 400 pass the 3 cm rule, lowest 4.3 cm; daily_progress/2026-11-12.md); live test open |
| F2 | Higher speed means longer braking | Keep 0.1 for tests; report decision time and stop distance per speed | Clear trade-off for the thesis | None | Home, lab | Table written 22 Nov from the 10 Nov e2 real-robot stops + harness (`test_results/overnight_20261122/E_F_ITEMS.md`); cancel stop on the UR open (Tuesday X-1/X-2) |
| F3 | Robot link through a USB hub | UGREEN gigabit hub, EEE off, never touch the hub during motion | Stable link, no dropped commands | Low | Lab | In progress |
| F4 | Escape recovery could speed up evasion | Never on the real robot: escape moves can carry the arm towards the obstacle | n/a | Excluded | n/a | Future work |

**Real robot, 23 Nov lab (`test_results/lab_20261123/analysis/f1_stops_real.txt`):** SC3 EMERGENCY_STOP at 0.31 rad/s:
send → still 339 ms, joint travel after the send **44.8 mrad** (its one target point is 0.15 s × speed ahead); controller
cancel (X-1 / X-2, 0.31–0.63 rad/s): send → still 227 ms, travel **2.9 mrad** (max 5.5), peak decel 15–22 rad/s², no
protective stop. → F1 candidate `sc3v2_estop_cancel` (SC3 v2.30, default off: an EMERGENCY_STOP cancels the controller
goal). F2: cancel numbers above complete the braking table. F3: K2 on the hub crashed the UR driver twice on 23 Nov
(and stopped the robot program once) → K2 on the laptop's USB-C port from 24 Nov.


**Real robot, 24 Nov lab:** F1 `sc3v2_estop_cancel` live: every emergency stop as a controller cancel, sent in 4–43 ms
(6 takes, no protective stop). F2 at speed 0.2 (box pushed in): cancel 4–43 ms; wrist_3 up to 0.64 rad/s with back-and-forth
between replans (→ v2.38 H5 wrist check). F3: K2 on USB-C all day, no UR driver drop; the robot's USB Ethernet adapter is
USB 2 (cdc_ncm 480M) - all on bus 01; after re-plugging: `nmcli con up "UR robot (hub)"`.


## Stage R: replanning decision time (added 21 Nov 2026 night by the user)
R0 measures first; the order of R1–R7 follows what R0 shows. Findings so far: `test_results/sc3_safe_stop_20261121/`
(SAFE_STOP_SNAPBACK_REPORT.md, PART2B_REPORT.md) and `test_results/R_ITEMS_NOTES.md`.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| R0 | Where the decision time goes is not split | Per part, median / p95 / max, SC3 v2.1 switches off vs on, harness: state wait, scene update, checks, planning, smoothing / retiming, send, controller start. **Found so far (21 Nov):** arm-state queue wait in SC3 (50-deep FIFO in a shared executor) 104 ms median / 458 ms p95 / 1.7 s max on the robot (21 Nov afternoon); monitor checks took 682 ms before the 14:15:57 SAFE_STOP (1.4–2.2 s at 14:02, up to 3.9 s in ROUTE-FAST harness runs); GIL: in-process planning / checking delays the arm state (17 → 135 ms p95 with a dedicated reader thread, harness test) | The priority order for R1–R7 | None (read-only) | Home | Done 22 Nov for item 4 (R-off / R-on / S-off), `test_results/overnight_20261122/r0/R0_DECISION_TIME.md`; v2.2 batches added in the overnight report |
| R1 | Detour states checked one ROS service call each (3.4–5.9 ms) | C++ plan_check in-process (= D4) | Much faster checks | Medium: new code path | Home | = D4: **open, D4 v3** (only for validating new paths during planning; never in the stop path) |
| R2 | Smoothing time and planning budget unbounded | Bound the smoothing time and the planning budget to what detours need (with E2) | Shorter worst case | Low | Home | = E2: **not adopted** 22 Nov (inconclusive); planner budget → Stage M |
| R3 | Detours planned only after a conflict | Plan detours in the background (= E4) | Detour ready almost at once | Medium: CPU, GIL | Home | = E4: **Stage M** (ready-made detours), after A–R are done |
| R4 | No reaction until a detour or stop is decided | Slow down first, decide while slower (= D3) | Instant reaction | Medium: unnecessary slow-downs | Home, then lab | = D3: **not adopted** (not built), Stage M (M4) |
| R5 | The plan starts from where the arm is when planning starts | Plan from the predicted future state at the switch-over time | No catch-up at the handover | Medium | Home | Already in SC3 (plans from the switch point ahead: strategy_b_adaptive.extend, C1); no build needed (22 Nov) |
| R6 | Lanes cost time and starve SC3's own plan | Hybrid lanes (= D8 SC3-HYBRID) | Lanes without starving SC3 | Medium | Home | Not started |
| R7 | Arm state delayed by the GIL even with a dedicated reader thread (p95 135 ms under load) | Joint-state reader in a **separate process** + shared memory (design: `test_results/R_ITEMS_NOTES.md`; switch `sc3v2_js_process`) | State age p95 ≤ 15 ms whatever SC3 does | Low to medium: process lifecycle | Home | Built 22 Nov (item 4 made it urgent: v2.1 reader starved, 37 of 80 runs lost): SC3 v2.2 switch `sc3v2_js_process` (default off), `experiments/candidates/sc3v2_state_reader.py`; freshness test p95 8.3 ms vs 137 ms; harness batches i4b_* (overnight report) |

Note (21 Nov): the user's overnight list mentions **D7**; there is no D7 row in Stage D (D6 is followed by D8).
**D7 status (user 22 Nov evening): target not met.** Lane decision < 0.3 s → lane splice 585 / 602 / 605 ms, and 88 % of lane decisions fall back to SC3 planning (`test_results/sunday_20261122/d_items/d7_split.txt`). **No-stop candidate** (`sc3v2_nostop.py`, M4): parked until A–R are done.

## Hardware and laptop
**Standing lab start-up rule (all future labs, user 21 Nov):** before any robot motion and before each motion block, show
CPU / load / swap (`uptime; free -m; vmstat 1 5`); Firefox closed; skeleton and person_figure off; **STOP if swap is in
use** (no motion until it is 0). Reason: on 18 Nov at 17:19:21 the UR driver's command stream stalled under CPU 97 %, load
26–28 and 1.3 GB swap; the robot froze ~170 ms, then jumped 0.15 rad at ~3 rad/s (PART2B_REPORT.md item 5).

The laptop removes stalls, not design delays: more RAM makes timing consistent (lower p95), but does not shorten stages
A to F.

| ID | Problem | Solution | Gain | Risk | Where | Status |
|---|---|---|---|---|---|---|
| H1 | One 8 GB stick, single-channel; swap freezes | Add 8 GB DDR4-2400 SO-DIMM in slot B: 16 GB dual-channel; check with free -h | No swap stalls, faster memory | None | Home | In progress |
| H2 | K2 and robot Ethernet share one USB 2.0 path | UGREEN USB-C hub with gigabit Ethernet (USB 3 path) and USB 3.0 ports; new interface: robot IP + EEE off | K2 and robot link no longer compete | Low | Home setup, lab test | In progress |
| H3 | Possible heat throttling of the i5-8365U | Log CPU MHz and temperature in a live run; cooling pad only if clocks drop | Sustained CPU speed | None | Lab | Not started |
| H4 | Updates could change kernel or drivers before the deadline | Automatic updates off until 5 Dec | No surprises | None | Home | Not started |
| H5 | Avoidable load during live runs | No ZSTD recording, K2 colour off, Firefox closed, on charger, Performance mode | Fewer CPU peaks | None | Lab | In progress |

## Order of work and testing
Work stage by stage, measuring before changing; about 80 % runs at home on bags and in the harness, one lab session
confirms it live.
1. Measure everything first: A0, B0, C0, D0, E0 in one pass from existing bags and harness logs. Result: a table of
   median, p95 and max per stage for person_reach, fast_crossing_box and the lab box cases.
2. Pick the biggest stage from that table and start there. Expected order: A2, A3, A1, A10, then B1, B2, C1, then D2 to D4.
3. One candidate at a time: build it as a copy, test on the same bags, then the same harness scenarios and seeds
   (setting B) as the frozen SC3 baseline.
4. Report per candidate: latency per stage, contacts (must be 0), minimum clearance (3 cm rule), emergency stops,
   unnecessary stops, goal completion, CPU.
5. Combine the candidates that helped, run the full harness again.
6. Lab session: hub test, IR interference, camera latency, CPU and temperature log, then the combined candidate live at
   speed 0.1 with explicit approval and the e-stop held.

Never done for speed:
- Lowering margins to make a case pass (P02)
- Removing obstacles earlier (stale timeout, speed damping) without worst-case evidence
- Feeding K1 and K2 into the tracker alternately without pairing
- Escape recovery on the real robot
- Editing production files or the frozen SC3
