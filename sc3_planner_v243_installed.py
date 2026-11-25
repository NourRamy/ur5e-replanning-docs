"""SC3 v2 (version line; rule of 20 Nov 2026: SC3 is no longer frozen in place).
Baseline = git tag sc3-baseline-1b38d729 + read-only copy strategy_b_detour_a2s_g1_sc3_baseline_1b38d729.py (md5 1b38d729).
Every edit of this file is committed separately and listed here. ROUTE / ROUTE-PAD / ROUTE-FAST, SC3-PAD and the other SC3
variants import this file: an edit here changes them too. Unchanged rules: no escape recovery, Rule E, lab use only with
the user's approval. Every harness / lab run logs the loaded files' md5 (experiments/replan/harness/planner_md5.py).
CHANGE LOG
  v2.0  20 Nov 2026  version line + change log only; behaviour identical to the baseline 1b38d729.
  v2.1  21 Nov 2026  SAFE_STOP snap-back fixes, two switches, BOTH DEFAULT OFF (off = behaviour identical to v2.0):
        sc3v2_fresh_cut  the SAFE_STOP cut is re-spliced from the running trajectory at the moment it is sent (fresh
                         time.monotonic(), same resting point), then re-checked (path vs the reach region, resting point
                         wait-safe); passed / not clear / re-base slower than sc3v2_cut_rebase_max -> no cut (the caller
                         holds: EMERGENCY_STOP). Cause (21 Nov 14:15:57, a9_moving_on leg 3): the cut was built for the
                         monitor iteration's start time, the iteration took 682 ms, the cut was sent unstamped 645 ms
                         later -> the controller pulled the arm back 0.19 rad at up to 2.6 rad/s.
        sc3v2_js_latest  /joint_states read by a dedicated node + thread (own executor), KEEP_LAST depth 1, speed qd from
                         the header stamps. Cause: the shared executor delivered joint states 0.1-1.7 s late (queue
                         depth 50, FIFO, starved callbacks) -> every check and the EMERGENCY_STOP target used old states.
        Evidence and tests: test_results/sc3_safe_stop_20261121/.
  v2.2  22 Nov 2026  two more switches, BOTH DEFAULT OFF (off = behaviour identical to v2.1):
        sc3v2_js_process /joint_states read in a SEPARATE PROCESS (experiments/candidates/sc3v2_state_reader.py, R7):
                         the newest state goes into shared memory (seqlock); self.q / qd / q_t / q_stamp read it on
                         every access. Reader process gone or its heartbeat > 50 ms old -> the state counts as stale
                         (state_age = inf, STATE_READER_DOWN) and the state guard applies. Takes precedence over
                         sc3v2_js_latest. Cause: item 4 (21 Nov night harness, ROUTE-FAST + v2.1 switches) - the in-process
                         js_latest thread got states 100-780 ms after their stamp under SC3's own load (GIL); the guard
                         refused every send and 19 of 40 runs could not return to START.
        sc3v2_retime_fix D9: a rest-to-rest segment (both ends at speed 0, i.e. a 2-point path from rest) is timed
                         T = max(sqrt(6 ds / amax), 1.5 ds / vmax) (minimum-time cubic: peak accel 6 ds / T^2 <= amax, peak
                         speed 1.5 ds / T <= vmax) instead of 2 ds / 1e-6 s (0.02 rad -> 40,000 s, the RECOVER timeouts).
                         Every other segment is timed exactly as before. Unit test: test_results/overnight_20261122/d9/.
  v2.3  22 Nov 2026  one parameter, default = v2.2 behaviour:
        sc3v2_js_reliability  QoS of the js_process reader: 'reliable' (default, v2.2) | 'best_effort'. Cause: item 4b
                         R-on2 - in 9 of 40 runs the reader's first state was 378-387 ms old (~48 messages at 125 Hz =
                         the plant's RELIABLE depth-50 queue) and stayed that old; the guard refused, the arm could not
                         return to START. Test: test_results/sc3_safe_stop_20261121/harness/reader_qos_test.py.
  v2.4  22 Nov 2026  two LOGGING switches, both default off (off = v2.3 behaviour; logging never takes part in a decision):
        sc3v2_log_stop_object  every stop / hold event (EMERGENCY_STOP, SAFE_STOP, HOLD, STOP, WAIT, NO_SAFE_RESPONSE,
                         STALE_STATE) writes one record to logs/SB_stop_objects.jsonl: the event text, the arm state used,
                         every object SC3 holds then (id, track class / confidence / track age / time since its last real
                         measurement, centre, size, number of parts, observation age, speed SC3 uses) and its distance to
                         the arm, nearest first. The snapshot is taken in the event call (copies only); distances are
                         computed in a background thread. Point count and K1 / K2 are NOT in the track message
                         (DynamicObject.msg is camera-free by design): join offline with /perception/objects/detections_3d.
                         Cause: Block 6 (22 Nov) - 8 of 9 real stops could not be classified, no object was logged.
        sc3v2_region_age_fix   SB_region.jsonl objects get t_obs_at_call / age_at_call / p_at_call: the observation as it
                         was when occupancy() was CALLED. Cause: the parent reads self.objs after the call returned (the
                         call takes up to hundreds of ms), so age_at_check was negative in 53 of 60 records of 21 Nov - not
                         a clock-domain mismatch (all 53 are within the call's own duration, check in
                         test_results/sunday_20261122/b2/). The old fields are kept unchanged.
  v2.5  22 Nov 2026  two speed switches, both default off (off = v2.4 behaviour; send() passes the same objects on):
        sc3v2_corner_stretch  F1: before sending, every segment between two sent points is checked with the cubic
                         Hermite a spline controller (and the harness plant) interpolates; a segment whose peak joint
                         speed is above vmax is stretched (x peak / vmax, at most x1.5 per pass, <= 20 passes) until
                         every segment peaks <= 1.005 vmax. Positions and point velocities unchanged; only later.
                         Cause: retime() limits path speed / acceleration only; at a corner where a joint reverses the
                         interior velocity v (u[k-1] + u[k]) / 2 no longer matches the positions and the cubic overshoots
                         (up to 1.33 vmax in the harness, 1.22 vmax real). Offline: +5-8 ms per changed trajectory.
        sc3v2_entry_clamp     F2: before sending, the entry segment from the measured arm (q + qd x state age, qd) to the
                         first sent point is checked with the same cubic; above vmax the whole trajectory is delayed by
                         the smallest d (2 ms steps) that brings it to <= 1.005 vmax (one-point STOP / EMERGENCY_STOP:
                         the same rule lengthens T; the stop point does not move). d > sc3v2_entry_clamp_max (0.25 s)
                         -> not delayed, ENTRY_STALE logged (a stale start is fresh_cut's job, not hidden by a delay).
                         Limit per joint = 1.005 x max(vmax, |qd_j| now): a joint already slightly above vmax may keep
                         its speed (a cubic cannot start below it), every other joint stays <= vmax.
                         Cause: a new path starts at the old trajectory's COMMANDED point; the real arm lags its command
                         by ~13-19 ms, the entry needs 1.4-1.5 vmax (real UR 0.40-0.44 rad/s at 0.314 in 5 of 32 runs).
                         Offline: +16-38 ms per join. Evidence: test_results/sunday_20261122/overspeed/.
  v2.6  23 Nov 2026  one safety switch, default off (off = v2.5 behaviour):
        sc3v2_monitor_safe    the monitor can no longer die silently. Cause: harness f12_base person_reach_prox_s2
                         (-0.99 cm): with js_process every read of self.q / q_t / q_stamp re-reads shared memory; one read
                         returned None (seqlock busy under load), the monitor-log line raised TypeError and the monitor
                         thread ended -> no stop for the rest of the run (5 of 1,061 js_process harness runs).
                         (1) the monitor takes q, q_t and q_stamp from ONE shared-memory read per pass; no state ->
                         the pass is skipped and counted as missed. Every other state read (all threads) retries a busy
                         read twice (1 ms) and then returns the last good state, which keeps its own stamp, so
                         state_age / state_guard still see its true age (never None once a state has arrived).
                         (2) any exception in a pass is logged with its traceback (MONITOR_EXCEPTION) and the loop goes on.
                         (3) watchdog thread (every 25 ms): monitor thread not alive, or >= 3 consecutive missed passes,
                         or one pass running > sc3v2_monitor_stuck_s (1.0 s; longest normal pass seen 0.54 s) ->
                         MONITOR_FAILSAFE: cancel the controller goal (the JTC holds at its measured position), and send()
                         refuses every new trajectory until the monitor is healthy again (alive, 3 good passes in a row).
                         Evidence: test_results/sunday_20261122/contact_f12base/. 23 Nov 09:30: the watchdog stands
                         down when rclpy is shutting down (3 false MONITOR_FAILSAFE at node shutdown in the high-load batch).
  v2.7  23 Nov 2026  (lab, user: "keep it 0 always ... change in the code itself") new DEFAULTS, launch values still win:
        clear_req 0.0 (was 0.02), lat_const 0.0 (was 0.02), speed_memory 2.0 s (was 10: one fast reach kept the region
        growing 10 s and swallowed path + goal), sc3v2_monitor_stuck_s 3.0 (was 1.0: false trip at 11:18 under lab load).
        NOT harness-tested. 0 cm also applies with a person near (user's decision 11:45).
  v2.9  23 Nov 2026  (lab 12:02, user: "remove that fail safe"): the LONG-PASS failsafe trigger is OFF by default
        (sc3v2_monitor_stuck_s 0; it fired at 11:18 and 12:00 while the monitor was busy building a stop, 1-3 s under
        lab load). KEPT: monitor thread dead -> cancel (the 22-23 Nov -0.99 cm contact), and >= 3 consecutive failed
        state reads -> cancel. Per-pass exception catch kept.
  v2.10 23 Nov 2026  (lab 12:05, user: "it should be REPLANNING"): the no-stop monitor decides alone. A clear check
        returns 'clear' instead of handing over to SC3's monitor_check, which re-checked a moment later, found the hand
        that had just arrived and built a SAFE_STOP cut (12:00 leg 2, 1.66 s ahead) instead of a deferred replan. SC3's
        own stop rules run only for NS_LAST_RESORT (contact now, conflict < 1.2 s ahead, no detour within 3 s).
  v2.11 23 Nov 2026  (lab 12:08): sc3v2_ns_defer default 1.2 -> 0.6 s. Live 12:04 the hand was first in the way
        0.9-1.0 s ahead (< 1.2) -> last-resort stops twice; LOCAL plans take 160-290 ms, the emergency stop ~0.3 s.
  v2.12 23 Nov 2026  (lab 12:10, user: "a crowd ... instead of waiting we move to other routes"): live 12:07 leg 2
        (77 s): GLOBAL skipped 19x 'no time left' (the blocked-path re-checks used the whole 2 s rest budget), 4 RETREATs.
        Defaults: budget_rest / plan_budget_rest 2 -> 6 s (GLOBAL gets its turn); sc3v2_no_retreat on (escape() refuses:
        the arm holds and replans, never backs away). Launch values still win.
  v2.13 23 Nov 2026  (lab 12:15, user: hand left in the path, the arm waited 33 holds 'START_STATE contact ... no recovery
        with growing distance'): sc3v2_lift_recover (default on): from contact, first try lifting tool0 6 / 10 / 15 cm
        straight up (SC3-POCKET's IK, orientation kept); accepted only if the lifted pose is MoveIt-valid and the distance
        to the hand grows at every step to >= release_dist. Then the normal planners (GLOBAL included) go on from there.
        Fallback: SC3's own forward / back recovery.
  v2.14 23 Nov 2026  (lab 12:24, user: "fastest, not shortest; minimal time to replan and decide"): planning_attempts
        default 6 -> 2 (MoveIt ran 6 parallel attempts and returned the SHORTEST, waiting for all; candidates are already
        selected by arrival time). Live 12:21: blockage -> GLOBAL chosen 30 s, 23 found plans rejected by SC3's reach check,
        16 by MoveIt's post-processing check (next: plan against the reach region; raw path when smoothing breaks it).
  v2.15 23 Nov 2026  v2.14 reverted (planning_attempts back to 6): live 12:25 legs 90 s / 57 s, MoveIt's final check
        rejected 50 found plans (16 with 6 attempts).
  v2.16 23 Nov 2026  (lab 12:50, user: "do whatever changes ... make all the problems solved"), from the B1-B8 v2.15
        logs (test_results/lab_20261123/analysis/takes_v215.txt): 30 of 35 emergency stops were 'box at the arm now'
        (paths planned to 0 cm, contact alarm at 1 cm -> SC3 drove into its own alarm; B4 a loop of 21 stops / 16 lifts)
        -> clear_req 0.01, contact_dist 0.005 (paths keep >= 1 cm, the alarm only at real touching). Plans rejected
        'inside the planning region' (B5 34 s blocked -> go) -> plan_reach_horizon 3.0 -> 1.5 s, wait_safe_horizon 2.0 -> 1.0.
        B9 checked: tracker speed linger median 0.00 s, max 0.06 s over 31 stops (not a cause).
  v2.17 23 Nov 2026  (lab 12:58): v2.16 live B4 / crowd: 25 / 19 'box at the arm now' e-stops in a RECOVER -> e-stop ->
        RECOVER loop: the monitor's contact check stopped each lift (the move away from the hand) half-way. The no-stop
        monitor now lets a RECOVER trajectory run to its end ('recovering').
  v2.18 23 Nov 2026  (lab 13:18, B8 static box leg 1: 84 s, 84 holds, 'every target inside the planning region' with
        region 5-8.7 s): reach_at grew a STILL box by 1.1 x its jitter speed (p90 of a few cm/s) x 8 s + lat_rate x 8 s =
        20-40 cm. sc3v2_still_obj_speed 0.06 m/s / sc3v2_still_obj_time 1.0 s: an object whose 0.5 s speeds all stayed
        below 6 cm/s for 1 s is STILL: its region = own size + lat_const (no growth); it grows again from the first
        faster sample (<= 0.5 s). Applies to boxes and to a person standing still.
  v2.19 23 Nov 2026  (lab 13:28, user "OK both" for the Hand-E model): SC3's own geometry = Robotiq Hand-E: TIP
        0.165 -> 0.157 m, gripper capsule 0.06 -> 0.045 m (forearm / wrists 0.06 unchanged); module-level, this process
        only. MoveIt's URDF / SRDF (still 2F-85) are prepared separately and NOT switched live.
  v2.20 23 Nov 2026  (lab 13:40, B4 v2.19: the arm held 48 s after the box was removed): SC3 kept a GHOST box
        (dynamic_object_25, no update for 29-36 s) whose REMOVE diff it had missed; the planning scene no longer had it.
        sc3v2_obj_max_age 3.0 s: an object not refreshed for 3 s is dropped (OBJECT_EXPIRED). A real still box is
        refreshed every <= 1.06 s (B8 v2.18 region log), so it is never dropped.
  v2.21 23 Nov 2026  (lab, B4 v2.20 leg 2: 16 s HOLD beside a still box, fingertips 4.5 cm above it): the batched checks
        (strategy_b_fastval) still used the 2F-85 TIP 0.165 + R 0.06 (imported by value; v2.19 never reached them) and
        every capsule cap reached R past the fingertips: 22.5 cm modelled vs 15.7 cm real. One Hand-E model now in both
        paths: gripper capsule 0.045 ending at the fingertips + fingertip ring (12 x 1 cm spheres at 2.5 cm).
        AND the v2.18 still test never fired for a truly still box (refreshed every ~1.06 s: never 2 speed samples in
        1 s), so it grew 1 cm/s sideways x (age + horizon); now also still when it stays within 3 cm of an anchor
        position for sc3v2_still_obj_time and no recent speed sample is >= sc3v2_still_obj_speed.
  v2.22 23 Nov 2026  (lab, B4 v2.21 leg 2: node died, KeyError 'dynamic_object_66' in strategy_b_event.on_scene: the
        object list changed between listing and reading): scene callbacks and the v2.20 expiry share one lock; a
        KeyError left in on_scene is logged (SCENE_RACE), never fatal.
  v2.23 23 Nov 2026  (lab, B2 v2.22 leg 2: one monitor pass took 21.4 s and the planning cycle 18 s while a 252-part
        person (Stage B raw union) was in the scene; the arm waited 23 s): the batched distance gave up (None) when the
        box count changed between states and the per-state Python fallback took 7 s (v2.20) / 15 s (v2.21) for 60 states;
        now one vectorized pass over all (state, box) pairs, any box count, never the fallback; pairs whose bounding
        spheres are >= 30 cm apart get that lower bound (every SC3 threshold is a few cm), the rest exact. Also: a log
        line that fails because the publisher is gone at shutdown is ignored (no false NODE WATCH stop).
  v2.24 23 Nov 2026  (lab, B2 v2.23 leg 2: planning-region builds 1-11.8 s, the arm waited 19 s): the region search's
        state x box checks run as one batch per 0.25 s step (same rule and result), and single-pose checks cache the
        gripper model per pose. Distance: box arrays built once when states share a box list, matmul instead of einsum,
        exact cutoff 30 -> 15 cm (every SC3 threshold <= clear_ref 10 cm): 40 states x 252 parts 91 -> ~25 ms.
  v2.25 23 Nov 2026  (lab 14:30, user "0"): margins all 0 together: clear_req 0 (paths may touch the arm model, not
        cross it), contact_dist 0 (the contact e-stop only when the model touches: path and alarm agree, no v2.15 loop),
        plan_pad 0 (MoveIt planning region = the reach region itself). The arm model keeps its own size (forearm 6 cm,
        Hand-E 4.5 cm capsule); z_grow 1 cm (height uncertainty) unchanged.
  v2.26 23 Nov 2026  (lab, B2 v2.25 leg 2: a 'RECOVER (back)' at 8.7 cm - MoveIt called the pose invalid - moved back
        along the last path as one point 9.87 s away and could not be replaced, ~13 s lost): with no_retreat the contact
        recovery is lift (v2.13) or forward along the direct path only, never back, and never longer than 3 s.
  v2.27 23 Nov 2026  (lab 14:40, B1 v2.26 TOUCHED THE USER'S HAND: path planned 1.6 cm from a still hand, conflict seen
        2.7 s ahead and deferred, the detour search found nothing, last resort at 0.3 s ahead went to SC3's cut rules,
        stop sent 0.8 s later at 0 cm): (1) user: path 0 cm, contact alarm 1 cm; (2) the no-stop last resort (and the
        contact check) sends the emergency stop at once from the fresh state; (3) once a detour search for the running
        trajectory has ended without a new path, a deferred conflict is stopped at once (no waiting until 0.6 s ahead).
  v2.28 23 Nov 2026  (home, user "go for it" after "remove rule E"): F1 sc3v2_corner_stretch default ON. The v2.27
        harness check (80 runs vs S-on3) had 4 speed jumps from corner overshoot: every sent point <= vmax, but the cubic
        between points peaked up to 2.02 x vmax (S-on3 max 1.17 x). F1 stretches each such segment's time until its peak
        is <= 1.005 x vmax (positions unchanged, logged CORNER_STRETCH). F2 entry_clamp stays off.
  v2.29 23 Nov 2026  (home, user "do all 3", option c): path clearance 1 cm from PEOPLE, 0 cm from boxes. An object
        at least sc3v2_person_height 0.30 m tall gets an extra sc3v2_person_clear 0.01 m on its reach region (path
        check, planning region, rest checks). Contact alarm 1 cm and clear_req 0 unchanged. Also: sent points closer than
        10 ms to the previous one are dropped (v2.28 harness: 2 speed jumps right after 2-5 ms splice segments), and a
        clean exit on Ctrl+C (the harness saw a segfault at every node stop).
  v2.30 23 Nov 2026  (home, user: ready-made routes built today; Stage M): sc3v2_route_bank (default OFF) - a background
        bank of fully checked routes (via-poses over / beside the midpoint to the goal) refreshed every 1 s; at the first
        sighting of a deferred conflict the earliest-arrival route is activated at once (no MoveIt request). Also: rclpy's
        signal handler off, SC3's own exits cleanly (v2.29 still segfaulted at 21-31 % of harness run ends). F1
        sc3v2_estop_cancel (default OFF): an EMERGENCY_STOP cancels the controller goal (real UR: 2.9 vs 45 mrad travel). And the <10 ms merge
        now runs BEFORE the corner stretch (v2.29 merged after it: 2 harness jumps at 1.20-1.24 x vmax).
  v2.31 23 Nov 2026  (home): exit without the segfault - v2.30's handler closed (unmapped) the state reader's shared
        memory while the monitor thread still read it; now the children are terminated and only the name is unlinked.
  v2.32 23 Nov 2026  (home): route bank v2 - 9 more varied routes, rebuilt at the conflict from 0.3 s ahead, refresh 0.5 s
        (v2.30 batch B: 0 switches in 52 runs, mostly 'no clear route' and stale splice points).
  v2.33 23 Nov 2026  (home): route bank v3 - routes may end at a validated resting point (full to the goal was almost
        never clear against a moving person), the background bank is used if still valid (rebuild only otherwise), 6 routes
        (v2.32 batch D: 0 switches, rebuild 1.1-1.6 s).
  v2.34 24 Nov 2026  (home): the route bank also at rest - when the arm stands blocked, the bank routes from its own pose
        are tried first and the best one is sent at once (MoveIt cycle skipped); v2.33 batch F: 32 holds at rest, the bank
        was never asked there.
  v2.35 24 Nov 2026  (home): the direct path is a bank route too, so a free direct path wins the ranking (v2.34 G: the
        rest hook replaced free direct starts by lifted via-routes).
  v2.36 24 Nov 2026  (home): direct route removed from the bank again - v2.35 batch H: holds 108 vs 3 (v2.34 G), mean
        time to goal -0.70 vs -1.62 s, people clearance back to baseline (direct paths pass where people reach in).
  v2.37 24 Nov 2026  (home, before the lab; user: "D1, D2, D3, D4"): D2 early band made reachable under the no-stop
        layer (it was dead code since v2.8); D3 sc3v2_d3_slow = same path at 0.5 x vmax while the detour is decided,
        full speed again after 0.3 s clear. All default off (= v2.36).
  v2.38 24 Nov 2026  (home, after the lab; user: "replanning with humans is the core ... perfect outcome tomorrow"):
        HUMAN MODE sc3v2_human_mode (default off): people/moving -> immediate stop (no deferral); region growth only for
        0.8 s at the current speed; fast restart direct-first then bank (+3 cm); bank only when direct is blocked; path
        quality (<= 2x direct + 1 rad, no wrist_3 back-and-forth). Still boxes keep v2.36 (deferral + bank).
  v2.39 24 Nov 2026  (home): human mode growth uses max(peak, current) speed over 0.8 s (v2.38 current-only saw a
        reaching hand 0.00 s ahead); no recovery moves while people are near (H6).
  v2.43 24 Nov 2026 night (home): all switches OFF = v2.42b. sc3v2_h_occ_zero (Fix A: planning region at the splice
        time when every stepped horizon fails), sc3v2_h_cred (Fix B: no 0.25 m/s growth without a credible measured speed),
        sc3v2_mpc_none stop (v2.42c: MPC stops when no moving speed is safe); DIAG_START logging of every refused planning
        start (logging only).
  v2.40 24 Nov 2026  (home; user: "it should be REPLANNING"): H7 replan while moving when the person conflict is >= 1.0 s
        ahead (fresh ready-made routes + MoveIt detour), stop at once when it is closer - recomputed every cycle, no timer.
  v2.41 24 Nov 2026  (home; user: "replanning quickly unless it's totally blocked, then the ready-made routes"): human
        mode order straight path -> RRT-Connect replanning -> ready-made routes (sc3v2_h_bank_first restores v2.40).
  v2.42 24 Nov 2026  (home; user: "do both tonight"): predictive speed MPC (sc3v2_h_mpc, default off) - receding 1.5 s
        horizon over 5 speed levels along the current path, SSM-inspired separation constraint, cost w1(1-v)^2 + w2 dv^2,
        actuated by re-timing the same path (v2.37 ramp) only when the level changes.
  v2.8  23 Nov 2026  (lab, user: "we will continue with this always ... add it to the SC3 file"): the D-items layer
        (formerly sc3v2_d_items.py, class SC3DItems, every switch default off) and the NO-STOP layer (formerly
        sc3v2_nostop.py, class SC3NoStop) are merged into this file; main() starts SC3NoStop. NO-STOP DEFAULTS ON:
        sc3v2_ns_defer 1.2 s (a hard conflict >= 1.2 s ahead gets no cut: a detour cycle is woken at once; closer, or after
        sc3v2_ns_max_defer 3 s, or box at the arm now -> SC3's own stop, logged NS_LAST_RESORT) and sc3v2_ns_lanes on
        (lane screening in a separate process). Plain SC3 behaviour: -p sc3v2_ns_defer:=0.0 -p sc3v2_ns_lanes:=false.
        Live 11:42 (0 cm box, defer + lanes): 2 LOCAL detours, 2 'box at the arm now' stops. NOT harness-tested.

A2S-G1-SC3 - DEADLINE-AWARE DENSE CHECK (2026-10-24, candidate on top of A2S-G1-SC2; NOT the default, not merged).
SC2 (strategy_b_detour_a2s_g1_sc2.py, md5 91f40aed) and every parent unchanged; prediction, margins, lateral growth,
checks (dense_step 0.02 rad included), deadlines, speed limits, planner configuration unchanged.
Finding (person trial out_person/person_SB4_20261024_143046, SC2): MoveIt answered in time, SC2 shortened for 193 ms and
then checked 153 shortcut states densely in 713 ms - the check has no deadline (80 ms reserved) and MoveIt's validity
service costs 3.4-5.9 ms per state live (0.9 ms in the offline harness) -> the switch-over point passed, the route was
rejected (safe stop -> replan). Same check cost 0.3-1.1 s per rest-cycle answer in the shuttle run.
Change (only the shortening's time handling; everything a shortened route must pass is unchanged):
  BUDGET    the shortening window W = t_end - now (SC / SC2 t_end: threat cycle min(now + 0.3 s, takeover - 0.35 s);
            rest / extension cycle now + 0.2 s, takeover-bounded when moving) is split: search until now + 0.35 W, then
            the dense check, which must finish by t_end - dense_margin.
  COST      the planned check cost per state = 1.5 x a running estimate of the MEASURED live cost (start 6 ms: the worst
            live value seen; floor 3 ms). Updated only from completed checks.
  CAP       half the sampled pairs are local (bridging at most the waypoints a checkable shortcut can span),
            half any pair (SC). The search keeps a shortcut only if ALL of the shortened route's new-segment states (<= 0.02 rad apart) fit the
            check window at the planned cost; otherwise that shortcut is not taken (smaller shortcuts may still be).
            Nothing to shorten within the cap -> the original answer, as before.
  DEADLINE  the dense check sends a request only if it can still finish before its deadline at the planned cost; at the
            deadline an unfinished check is INCOMPLETE -> the shortened route is rejected (never used partially checked)
            and the original, already validated answer is kept (SC / SC2 fallback, unchanged). In-flight checks are
            bounded by the window (8), so an abandoned check cannot queue validity calls far beyond the deadline.
Every shortened route that is used still passes the unchanged build() (time-resolved reach check, resting-point rules,
kinematics, transition check, static validity) after the complete dense check.
    python3 experiments/candidates/strategy_b_detour_a2s_g1_sc3.py   (SB4_NODE in the runners)
"""
import json
import os
import queue
import re
import threading
import time

import numpy as np
import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from strategy_a_global import ARM, GROUP
from strategy_a_global import arm_box_distance as _v24_box_dist, fk_points as _v24_fk
from strategy_b_detour_a2s_g1_sc2 import StrategyBDetourA2SG1SC2

# v2.19 (23 Nov lab, user OK 13:28 "OK both"): SC3's own arm model uses the REAL gripper, Robotiq Hand-E (was the
# 2F-85): tool0 -> fingertips 0.157 m (adapter 0.011 + Hand-E body 0.099 + fingers 0.047; was 0.165), and the gripper
# segment (tool0 -> fingertips) a 0.045 m capsule (body 7.5 cm + finger stroke; was 0.06 like the forearm). Forearm /
# wrist capsules stay 0.06 m. In THIS process only (baseline, tracker, detector unchanged). MoveIt's URDF is unchanged.
import strategy_a_global as _sag
_sag.TIP = 0.157
_sag.R_TOOL = 0.045


def _v219_arm_box_distance(pts, box):
    c, R, h = box
    best = np.inf
    segs = list(zip(pts[1:-1], pts[2:]))
    for i, (a, b) in enumerate(segs):
        n = max(2, int(np.linalg.norm(b - a) / 0.02) + 1)
        p = (a + np.linspace(0, 1, n)[:, None] * (b - a) - c) @ R
        d = np.maximum(np.abs(p) - h, 0.0)
        r = R_TOOL if i == len(segs) - 1 else R_ARM
        best = min(best, float(np.sqrt((d * d).sum(1)).min()) - r)
    return max(best, 0.0)


_sag.arm_box_distance.__code__ = _v219_arm_box_distance.__code__   # every importer holds this same function object

# v2.21 (23 Nov lab, B4 v2.20 leg 2: 16 s of HOLD with a still box 8 cm from the arm, every path over it refused): the
# batched checks that SC3 actually uses (strategy_b_fastval fk_batch / capsule_dist: first_conflict, reach_clearance,
# clearance) had imported TIP 0.165 and R_ARM 0.06 BY VALUE, so v2.19 never reached them, and every model (old and
# v2.19) ended the gripper capsule with a round cap that reaches R past the fingertips: SC3 saw the gripper 22.5 cm below
# tool0 (real Hand-E 15.7 cm). Now ONE model in both paths: forearm .. tool0 capsules R_ARM (unchanged); gripper capsule
# R_TOOL 0.045 from tool0 to a point R_TOOL short of the fingertips, so its cap ends AT the fingertips; plus a ring of 12
# spheres r 1 cm, radius 2.5 cm, in the fingertip plane (open fingers reach 3.5 cm from the axis).
import strategy_b_fastval as _fv
_fv.TIP = 0.157
_V222_SCENE_LOCK = threading.RLock()   # v2.22: scene callbacks + object expiry, one at a time
_V226_MAX_RECOVER = 3.0               # v2.26: s, longest RECOVER move sent
_V223_EXACT = 0.15                     # v2.24: exact below 15 cm (every SC3 threshold <= clear_ref 10 cm); above, a lower bound
_V221_RING_N, _V221_RING_R, _V221_RING_S = 12, 0.025, 0.010
_V221_ANG = np.linspace(0.0, 2 * np.pi, _V221_RING_N, endpoint=False)


_V224_PARTS_CACHE = {}


def _v221_parts(pts):
    """v2.24: single states (the per-pair Python loops) are cached by their points (the same pose is checked against
    every box in turn)."""
    if len(pts) != 1:
        return _v221_parts_raw(pts)
    k = pts.tobytes()
    out = _V224_PARTS_CACHE.get(k)
    if out is None:
        if len(_V224_PARTS_CACHE) > 4096:
            _V224_PARTS_CACHE.clear()
        out = _V224_PARTS_CACHE[k] = _v221_parts_raw(pts)
    return out


def _v221_parts_raw(pts):
    """pts (N, 7, 3) -> list of (sample points (N, M, 3), radius) for forearm .. gripper + the fingertip ring."""
    parts = []
    for s in range(1, 5):                                # forearm .. wrist3 -> tool0, R_ARM as before
        a, b = pts[:, s], pts[:, s + 1]
        n = int(max(2, (np.linalg.norm(b - a, axis=1).max() / 0.02) + 1))
        u = np.linspace(0, 1, n)
        parts.append((a[:, None, :] + u[None, :, None] * (b - a)[:, None, :], _sag.R_ARM))
    a, tip = pts[:, 5], pts[:, 6]
    ax = tip - a
    ux = ax / np.maximum(np.linalg.norm(ax, axis=1, keepdims=True), 1e-9)
    end = tip - _sag.R_TOOL * ux
    n = int(max(2, (np.linalg.norm(end - a, axis=1).max() / 0.02) + 1))
    u = np.linspace(0, 1, n)
    parts.append((a[:, None, :] + u[None, :, None] * (end - a)[:, None, :], _sag.R_TOOL))
    ref = np.where(np.abs(ux[:, :1]) < 0.9, np.array([[1.0, 0.0, 0.0]]), np.array([[0.0, 1.0, 0.0]]))
    e1 = np.cross(ux, ref)
    e1 /= np.linalg.norm(e1, axis=1, keepdims=True)
    e2 = np.cross(ux, e1)
    ring = tip[:, None, :] + _V221_RING_R * (np.cos(_V221_ANG)[None, :, None] * e1[:, None, :]
                                             + np.sin(_V221_ANG)[None, :, None] * e2[:, None, :])
    parts.append((ring, _V221_RING_S))
    return parts


def _v221_capsule_dist(pts, boxes):
    """strategy_b_fastval.capsule_dist with the v2.21 Hand-E model. v2.23: every (state, box) pair in one vectorized
    pass and any box count per state (never None, so the per-state Python fallback - 7-15 s for 60 states x a 252-part
    person, the 21 s monitor freeze in B2 v2.22 - is never used)."""
    N = len(pts)
    best = np.full(N, np.inf)
    cnt = [len(b) for b in boxes]
    if N == 0 or sum(cnt) == 0:
        return best
    si = np.repeat(np.arange(N), cnt)
    if all(b is boxes[0] for b in boxes):                                  # one box list for every state: build once
        b0 = boxes[0]
        bi = np.tile(np.arange(len(b0)), N)
        C = np.array([x[0] for x in b0], float)[bi]
        R = np.array([x[1] for x in b0], float)[bi]
        H = np.array([x[2] for x in b0], float)[bi]
    else:
        C = np.array([x[0] for b in boxes for x in b], float)
        R = np.array([x[1] for b in boxes for x in b], float)
        H = np.array([x[2] for b in boxes for x in b], float)
    pair = np.full(len(si), np.inf)
    hn = np.linalg.norm(H, axis=1)
    for P, r in _v221_parts(pts):
        mid = 0.5 * (P.min(1) + P.max(1))                                   # bounding sphere of this part, per state
        rad = np.linalg.norm(P - mid[:, None, :], axis=2).max(1)
        lb = np.linalg.norm(mid[si] - C, axis=1) - rad[si] - hn - r         # lower bound of the true distance
        far = lb >= _V223_EXACT
        pair[far] = np.minimum(pair[far], lb[far])                          # far pairs: a bound >= 30 cm is enough
        near = np.flatnonzero(~far)
        for lo in range(0, len(near), 4096):                                # bounded memory
            k = near[lo:lo + 4096]
            p = np.matmul(P[si[k]] - C[k, None, :], R[k])                     # (k,m,3) @ (k,3,3)
            d = np.maximum(np.abs(p) - H[k, None, :], 0.0)
            pair[k] = np.minimum(pair[k], np.sqrt((d * d).sum(2)).min(1) - r)
    np.minimum.at(best, si, pair)
    return np.maximum(best, 0.0)


def _v221_single(pts, box):
    return float(_v221_capsule_dist(np.asarray(pts, float)[None], [[box]])[0])


def _v221_arm_box_distance(pts, box):
    return _V221_SINGLE(pts, box)


_fv.capsule_dist = _v221_capsule_dist

# v2.24 (lab, B2 v2.23 leg 2: planning-region builds took 1-11.8 s with 18-37 parts; the arm waited 19 s): the region
# search (strategy_b_replan.occupancy, H shrunk in 0.25 s steps) checked every required state x every box one pair at a
# time in Python (1.7 ms each with the v2.21 model). Same rule, same result, one batch per step (this process only).
import strategy_b_replan as _sbr


def _v224_occupancy(self, H, start, required, optional, t_ref):
    must = np.asarray([start] + list(required), float)
    Pm = _fv.fk_batch(must, self.base)
    for Hh in np.arange(H, -1e-9, -0.25):
        boxes = self.reach_at(t_ref + Hh, pad=self.clear_req + self.plan_pad)
        # as the original all(): state by state (start first), stop at the first one that touches; each state against
        # every box in one vectorized call, its gripper model cached across the steps
        if all(_v221_capsule_dist(Pm[i:i + 1], [boxes])[0] > 0.005 for i in range(len(Pm))):
            break
    else:
        return None, 0.0, [], False
    if optional and boxes:
        d = _v221_capsule_dist(_fv.fk_batch(np.asarray(optional, float), self.base), [boxes] * len(optional))
        keep = [x for x, dd in zip(optional, d) if dd > 0.005]
    else:
        keep = list(optional)
    if not boxes:
        return None, float(Hh), keep, True
    co = _sbr.CollisionObject(id='strategy_a_reach', operation=_sbr.CollisionObject.ADD)
    co.header.frame_id = 'world'
    co.pose.orientation.w = 1.0
    for c, R, h in boxes:
        co.primitives.append(_sbr.SolidPrimitive(type=_sbr.SolidPrimitive.BOX, dimensions=[float(x) for x in 2 * h]))
        ps = _sbr.Pose()
        ps.position.x, ps.position.y, ps.position.z = [float(x) for x in c]
        ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w = [float(x) for x in _sbr.mat_quat(R)]
        co.primitive_poses.append(ps)
    return co, float(Hh), keep, True


_V224_OCC_ORIG = _sbr.StrategyBReplan.occupancy
_sbr.StrategyBReplan.occupancy = _v224_occupancy
_sag._V221_SINGLE = _v221_single
_sag.arm_box_distance.__code__ = _v221_arm_box_distance.__code__

# v2.8 (23 Nov lab): imports of the merged D-items + no-stop layers (formerly sc3v2_d_items.py / sc3v2_nostop.py)
import multiprocessing as mp
from builtin_interfaces.msg import Duration, Time
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile


class _CtrlRecorder:
    """v2.1 state guard: the FollowJointTrajectory client, remembering the future of the last goal sent (to cancel it)."""

    def __init__(self, raw):
        object.__setattr__(self, '_raw', raw)
        object.__setattr__(self, 'last', None)

    def __getattr__(self, k):
        return getattr(self._raw, k)

    def send_goal_async(self, goal, *a, **kw):
        fut = self._raw.send_goal_async(goal, *a, **kw)
        object.__setattr__(self, 'last', fut)
        return fut


class _V24RegionQueue:
    """v2.4 sc3v2_region_age_fix: wraps the parent's SB_region queue; adds the observation as it was at the call."""

    def __init__(self, raw, local):
        self._raw, self._local = raw, local

    def __getattr__(self, k):
        return getattr(self._raw, k)

    def put(self, rec, *a, **kw):
        try:
            snap = getattr(self._local, 'snap', None)
            if snap is not None and isinstance(rec, dict):
                for o in rec.get('objects', []):
                    sn = snap.get(o.get('id'))
                    if sn is not None:
                        o['t_obs_at_call'] = sn[0]
                        o['age_at_call'] = round(rec['t'] - sn[0], 3)
                        o['p_at_call'] = sn[1]
                        o['obs_updated_during_call'] = bool(o.get('t_obs') != sn[0])
                rec['region_age_fix'] = 1
        except Exception:
            pass
        return self._raw.put(rec, *a, **kw)


class _JsProcState:
    """v2.2 sc3v2_js_process: the arm state attributes read the reader process's shared memory on every access (writes
    from the old callbacks are ignored). Mixed in front of the node's class only when the switch is on."""

    def _js_read(self):
        r = self.__dict__.get('_reader')
        x = None if r is None else r.read()
        if self.__dict__.get('_v26_on'):                                     # v2.6: a busy read never gives None
            if x is None and r is not None:                                  # once a state exists: retry, then the
                for _ in range(2):                                           # last good one (its own stamp, so the
                    time.sleep(0.001)                                        # age guards still see its true age)
                    x = r.read()
                    if x is not None:
                        break
                if x is None:
                    x = self.__dict__.get('_v26_last')
            if x is not None:
                self.__dict__['_v26_last'] = x
        return x

    @property
    def q(self):
        r = self._js_read()
        return None if r is None else r['q']

    @q.setter
    def q(self, v):
        pass

    @property
    def qd(self):
        r = self._js_read()
        return np.zeros(6) if r is None else r['qd']

    @qd.setter
    def qd(self, v):
        pass

    @property
    def q_t(self):
        r = self._js_read()
        return None if r is None else r['recv_ns'] / 1e9                     # time.monotonic() seconds, as on_joints

    @q_t.setter
    def q_t(self, v):
        pass

    @property
    def q_stamp(self):
        r = self._js_read()
        return None if r is None else r['stamp_ns']

    @q_stamp.setter
    def q_stamp(self, v):
        pass

    def state_age(self):
        r = self.__dict__.get('_reader')
        if r is None or not r.alive():
            if not self.__dict__.get('_reader_down_logged'):
                self.__dict__['_reader_down_logged'] = True
                self.event('STATE_READER_DOWN: the joint-state reader process is gone or silent > 50 ms - state stale')
            return float('inf')
        return super().state_age()


class StrategyBDetourA2SG1SC3(StrategyBDetourA2SG1SC2):

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.sc_search_share = p('sc_search_share', 0.35).value       # of the window for the shortcut search
        self.dense_margin = p('dense_margin', 0.01).value             # s before t_end the dense check must end
        self.dense_cost = p('dense_cost_start', 0.006).value          # s per state, running estimate (measured)
        self.dense_cost_floor = p('dense_cost_floor', 0.003).value
        self.dense_cost_factor = p('dense_cost_factor', 1.5).value
        self.dense_window = int(p('dense_window', 8).value)
        self.event(f'A2S-G1-SC3 on: dense check deadline-bound (cost {1000 * self.dense_cost:.1f} ms/state x '
                   f'{self.dense_cost_factor}, measured live), shortcuts only as large as can be checked in time')
        # v2.1 switches (default OFF = v2.0 behaviour)
        self.fresh_cut = bool(p('sc3v2_fresh_cut', False).value)
        self.cut_rebase_max = float(p('sc3v2_cut_rebase_max', 0.10).value)   # s the re-base + re-check may take
        self.js_latest = bool(p('sc3v2_js_latest', False).value)
        self.js_process = bool(p('sc3v2_js_process', False).value)          # v2.2 R7 (takes precedence)
        self.retime_fix = bool(p('sc3v2_retime_fix', False).value)          # v2.2 D9
        self.js_reliability = str(p('sc3v2_js_reliability', 'reliable').value)   # v2.3: 'reliable' | 'best_effort'
        self._js_ts = None
        if self.js_process:
            self._start_js_process()
        elif self.js_latest:
            self._start_js_latest()
        if self.retime_fix:
            self.event('SC3 v2.2: retime_fix 1 (rest-to-rest segments minimum-time, D9)')
        self.state_guard = bool(p('sc3v2_state_guard', False).value)
        self.state_max_age = float(p('sc3v2_state_max_age', 0.05).value)     # s: newest arm state older -> not used
        self.state_wait = float(p('sc3v2_state_wait', 0.06).value)           # s a stop waits for a fresh state
        self.stale_stop = str(p('sc3v2_stale_stop', 'cancel').value)         # no fresh state for a stop: 'cancel' | 'extrapolate'
        self.cut_start_tol = float(p('sc3v2_cut_start_tol', 0.02).value)     # rad: cut's first point vs the measured arm
        if self.state_guard:
            self.ctrl = _CtrlRecorder(self.ctrl)
        if self.fresh_cut or self.js_latest or self.state_guard:
            self.event(f'SC3 v2.1: fresh_cut {int(self.fresh_cut)} (re-base max {1000 * self.cut_rebase_max:.0f} ms), '
                       f'js_latest {int(self.js_latest)}, state_guard {int(self.state_guard)} (max age '
                       f'{1000 * self.state_max_age:.0f} ms, stop waits {1000 * self.state_wait:.0f} ms, then {self.stale_stop})')

        # v2.4 logging switches (default OFF = v2.3 behaviour; logging only)
        self.log_stop_object = bool(p('sc3v2_log_stop_object', False).value)
        self.region_age_fix = bool(p('sc3v2_region_age_fix', False).value)
        self._v24_tracks = {}
        if self.region_age_fix and getattr(self, '_rq', None) is not None:
            self._v24_local = threading.local()
            self._rq = _V24RegionQueue(self._rq, self._v24_local)
        if self.log_stop_object:
            d = os.environ.get('ROS_LOG_DIR')
            self._v24_log = open(os.path.join(os.path.dirname(d), 'logs', 'SB_stop_objects.jsonl'), 'a') if d else None
            self._v24_q = queue.Queue()
            threading.Thread(target=self._v24_writer, daemon=True).start()
        if self.log_stop_object or self.region_age_fix:
            self.event(f'SC3 v2.4: log_stop_object {int(self.log_stop_object)}, region_age_fix {int(self.region_age_fix)}')

        # v2.5 speed switches (default OFF = v2.4 behaviour)
        self.corner_stretch = bool(p('sc3v2_corner_stretch', True).value)      # v2.28 default ON (user 23 Nov night)
        self.entry_clamp = bool(p('sc3v2_entry_clamp', False).value)
        self.entry_clamp_max = float(p('sc3v2_entry_clamp_max', 0.25).value)   # s: larger delay needed -> ENTRY_STALE
        # v2.6 safety switch (default OFF = v2.5 behaviour)
        self.monitor_safe = bool(p('sc3v2_monitor_safe', False).value)
        self.v26_max_misses = int(p('sc3v2_monitor_max_misses', 3).value)
        self.v26_stuck = float(p('sc3v2_monitor_stuck_s', 0.0).value)   # v2.9: 0 = long-pass trigger OFF (user 12:02); was 3.0
        self._v26_failsafe, self._v26_blocked = False, 0
        if self.monitor_safe:
            self.__dict__['_v26_on'] = True
            self._v26_misses, self._v26_good, self._v26_hb, self._v26_thread = 0, 0, time.monotonic(), None
            self._v26_had_state = False
            if not isinstance(self.ctrl, _CtrlRecorder):
                self.ctrl = _CtrlRecorder(self.ctrl)                          # the failsafe needs the last goal
            self.event(f'SC3 v2.6: monitor_safe 1 (failsafe after {self.v26_max_misses} missed passes, thread death or a '
                       f'pass > {1000 * self.v26_stuck:.0f} ms)')
        # v2.7 margins (user 23 Nov 11:45, "keep it 0 always ... change in the code itself"): new DEFAULTS, a value given at
        # launch still wins. clear_req 0.02 -> 0.0, lat_const 0.02 -> 0.0, speed_memory 10 -> 2.0 s.
        ov = getattr(self, '_parameter_overrides', None) or {}
        for k, v in (('clear_req', 0.0), ('contact_dist', 0.01), ('plan_pad', 0.0),  # v2.27 user: path 0, alarm 1 cm
                     ('plan_reach_horizon', 1.5), ('wait_safe_horizon', 1.0),       # v2.16 (was 3.0 / 2.0)
                     ('lat_const', 0.0), ('speed_memory', 2.0),
                     ('budget_rest', 6.0), ('plan_budget_rest', 6.0)):          # v2.12: route planner gets its turn
            if k not in ov:
                setattr(self, k, v)
        # v2.15: v2.14's planning_attempts 2 REVERTED (live 12:25: MoveIt post-processing rejects 16 -> 50); back to 6
        self.event(f'SC3 v2.7: margins clear_req {100 * self.clear_req:.1f} cm, contact_dist {100 * self.contact_dist:.1f} cm, '
                   f'plan_pad {100 * self.plan_pad:.1f} cm, lat_const {100 * self.lat_const:.1f} cm, '
                   f'speed_memory {self.speed_memory:.1f} s; monitor long-pass trigger {"OFF" if self.v26_stuck <= 0 else "%.1f s" % self.v26_stuck}')
        # v2.12 (user 12:10, "a crowd: move to other routes instead of waiting"): no backing away
        self.no_retreat = bool(p('sc3v2_no_retreat', True).value)
        self.lift_recover = bool(p('sc3v2_lift_recover', True).value)      # v2.13
        self.still_obj_speed = float(p('sc3v2_still_obj_speed', 0.06).value)   # v2.18: m/s (harness jitter max 3.4-4 cm/s)
        self.still_obj_time = float(p('sc3v2_still_obj_time', 1.0).value)     # v2.18: s below it -> still, no growth
        self.person_clear = float(p('sc3v2_person_clear', 0.01).value)        # v2.29: m, path clearance from people
        self.person_height = float(p('sc3v2_person_height', 0.30).value)      # v2.29: m, taller object = person
        self.event(f'SC3 v2.29: person clearance {100 * self.person_clear:.1f} cm (objects >= {self.person_height:.2f} m tall)')
        self.obj_max_age = float(p('sc3v2_obj_max_age', 3.0).value)           # v2.20: s without refresh -> object dropped
        if self.obj_max_age > 0:
            threading.Thread(target=self._v220_expire, daemon=True).start()
        self.event(f'SC3 v2.12: no_retreat {int(self.no_retreat)}, rest budget {self.budget_rest:.1f} s')
        if self.corner_stretch or self.entry_clamp:
            self.event(f'SC3 v2.5: corner_stretch {int(self.corner_stretch)}, entry_clamp {int(self.entry_clamp)} '
                       f'(max delay {1000 * self.entry_clamp_max:.0f} ms), vmax {self.vmax:.3f} rad/s')

    # ------------------------------------------------------------------ v2.20: drop objects whose REMOVE was missed
    def _v220_expire(self):
        while rclpy.ok():
            time.sleep(0.2)
            try:
                now = time.monotonic()
                for oid, o in list(self.objs.items()):
                    age = now - o.get('t', now)
                    if age > self.obj_max_age:
                        with _V222_SCENE_LOCK:
                            self.objs.pop(oid, None)
                            self.motion.pop(oid, None)
                        self.event(f'OBJECT_EXPIRED {oid}: no scene update for {age:.1f} s (> {self.obj_max_age:.1f} s; '
                                   f'its REMOVE was missed)')
            except Exception as e:
                self.event(f'OBJECT_EXPIRE error {e!r}')

    # ------------------------------------------------------------------ v2.22: scene updates one at a time
    def on_scene(self, m):
        """v2.22 (B4 v2.21 crash: KeyError 'dynamic_object_66' in strategy_b_event.on_scene, the object list changed
        between listing and reading): scene callbacks and the v2.20 expiry run under one lock; a KeyError left over is
        logged, never fatal (the next scene message repairs the state)."""
        with _V222_SCENE_LOCK:
            try:
                super().on_scene(m)
            except KeyError as e:
                self.event(f'SCENE_RACE {e!r}: ignored, state repaired by the next scene message')

    # ------------------------------------------------------------------ v2.18: a still object does not grow
    def observe(self, oid, pos, now):
        super().observe(oid, pos, now)
        st = self.motion.get(oid)
        if st is None or self.still_obj_speed <= 0:
            return
        sp = st['speeds']
        recent = [v for t, v in sp if now - t <= self.still_obj_time]
        # v2.21: a truly still box is refreshed only every ~1.06 s, so it never had 2 speed samples inside 1 s and
        # was never 'still' (B4 v2.20: grew 1 cm/s sideways x up to 7 s). Also still: it stayed within 3 cm of an
        # anchor position for still_obj_time and no recent speed sample is fast.
        p = np.asarray(pos, float)
        a = st.get('anchor')
        if a is None or np.linalg.norm(p - a[0]) > 0.03:
            st['anchor'] = a = (p.copy(), now)
        by_anchor = now - a[1] >= self.still_obj_time and (not recent or max(recent) < self.still_obj_speed)
        st['still'] = by_anchor or (bool(sp) and now - sp[0][0] >= self.still_obj_time and len(recent) >= 2
                                    and max(recent) < self.still_obj_speed)

    def _v229_person(self, o):
        """v2.29: an object at least sc3v2_person_height tall (all its parts) is treated as a person."""
        lo, hi = np.inf, -np.inf
        for cl, Rl, h in o['parts']:
            R = o['R'] @ Rl
            cz = float((o['p'] + o['R'] @ cl)[2])
            ez = float((np.abs(R) @ h)[2])
            lo, hi = min(lo, cz - ez), max(hi, cz + ez)
        return hi - lo >= self.person_height

    def reach_at(self, t_abs, pad=0.0):
        """StrategyA.reach_at; v2.18: an object marked still (every 0.5 s speed < still_obj_speed for still_obj_time)
        keeps its own size + lat_const (no speed growth, no lat_rate growth). v2.29: person-like objects get an extra
        sc3v2_person_clear on every side (path >= 1 cm from people, 0 cm from boxes)."""
        if self.still_obj_speed <= 0:
            return super().reach_at(t_abs, pad)
        out = []
        for oid, o in list(self.objs.items()):
            st = self.motion.get(oid)
            dt = max(t_abs - o['t'], 0.0) + self.latency
            opad = pad + (self.person_clear if self.person_clear > 0 and self._v229_person(o) else 0.0)
            if st is not None and st.get('still'):
                g = np.array([self.lat_const, self.lat_const, self.z_grow])
                for cl, Rl, h in o['parts']:
                    R = o['R'] @ Rl
                    out.append((o['p'] + o['R'] @ cl, R, h + np.abs(R.T) @ g + opad))
                continue
            v = st['peak'] if st else 0.0
            if self.frozen is not None and oid in self.frozen:
                v = self.frozen[oid]
                if st and st['now'] > 1.3 * v:
                    v = st['now']
            along = self.reach_factor * v * dt
            lat = self.lat_const + self.lat_rate * dt
            if st is None or st['u'] is None:
                g = np.array([along + lat, along + lat, self.z_grow])
            else:
                g = np.array([abs(st['u'][0]) * along + lat, abs(st['u'][1]) * along + lat, self.z_grow])
            for cl, Rl, h in o['parts']:
                R = o['R'] @ Rl
                out.append((o['p'] + o['R'] @ cl, R, h + np.abs(R.T) @ g + opad))
        return out

    # ------------------------------------------------------------------ v2.13: step aside (lift) out of contact
    def _v213_lift(self, q, dz):
        """q with tool0 dz higher (world z), orientation kept (SC3-POCKET's lift, 18 Nov): joints 1, 2 solved,
        joint 3 keeps q1+q2+q3; base and wrists 2 / 3 unchanged. None if no solution."""
        from strategy_a_global import fk_points
        q = np.array(q, float)
        target = fk_points(q, self.base)[5] + np.array([0.0, 0.0, dz])
        s = q[1] + q[2] + q[3]
        x = q.copy()
        for _ in range(30):
            x[3] = s - x[1] - x[2]
            e = target - fk_points(x, self.base)[5]
            if np.linalg.norm(e) < 1e-4:
                return x
            J = np.zeros((3, 2))
            for k, j in enumerate((1, 2)):
                y = x.copy(); y[j] += 1e-5; y[3] = s - y[1] - y[2]
                J[:, k] = (fk_points(y, self.base)[5] - fk_points(x, self.base)[5]) / 1e-5
            d = np.linalg.lstsq(J, e, rcond=None)[0]
            x[1] += np.clip(d[0], -0.2, 0.2); x[2] += np.clip(d[1], -0.2, 0.2)
        return None

    def contact_recovery(self, q, ver):
        if getattr(self, 'lift_recover', False) and self.base is not None:
            now = time.monotonic()
            d0 = self.clearance(q, now)
            for dz in (0.06, 0.10, 0.15):
                ql = self._v213_lift(q, dz)
                if ql is None or not self.moveit_valid(ql):
                    continue
                seq = np.array([q + (ql - q) * f for f in np.linspace(0.0, 1.0, 11)])
                pos, tim, _ = self.retime(seq)
                d_prev, ok = d0, True
                for j in range(1, len(pos)):
                    d = self.clearance(pos[j], now + tim[j])
                    if d <= d_prev - 1e-4:
                        ok = False
                        break
                    d_prev = d
                if ok and d_prev >= self.release_dist:
                    self.event(f'RECOVER (lift {100 * dz:.0f} cm) from contact {100 * d0:.1f} cm -> {100 * d_prev:.1f} cm')
                    self.send(*self.retime(seq), None, False, 'RECOVER', ver=ver)
                    return
        if not getattr(self, 'no_retreat', False):
            return super().contact_recovery(q, ver)
        # v2.26 (B2 v2.25 leg 2: 'RECOVER (back)' at 8.7 cm (MoveIt called the pose invalid) went back along the last
        # path as ONE point 9.87 s away; nothing could replace it, ~13 s lost): with no_retreat, strategy_b_local's
        # recovery WITHOUT the 'back' candidate (forward = the direct path toward the goal only), and never > 3 s.
        now = time.monotonic()
        pd = self.plan(q, 'direct')
        if pd is not None:
            pos, tim, _ = self.retime(pd[0])
            d_prev = self.clearance(q, now)
            for j in self.samples(tim)[1:]:
                if tim[j] > _V226_MAX_RECOVER:
                    break
                d = self.clearance(pos[j], now + tim[j])
                if d <= d_prev - 1e-4:
                    break
                d_prev = d
                if d >= self.release_dist and self.wait_safe(pos[j], now + tim[j] + 0.4):
                    out = self.retime(pos[:j + 1])
                    if len(out[1]) >= 2 and float(out[1][-1]) <= _V226_MAX_RECOVER:
                        self.event(f'RECOVER (forward) from contact {100 * self.clearance(q, now):.1f} cm')
                        self.send(*out, None, False, 'RECOVER', ver=ver)
                        return
                    break
        self.hold(f'START_STATE contact {100 * self.clearance(q, now):.1f} cm: no lift / forward recovery (v2.26: never '
                  f'back, never > {_V226_MAX_RECOVER:.0f} s): holding and replanning')

    # ------------------------------------------------------------------ v2.12: never back away (RETREAT / ESCAPE)
    def escape(self, q, ver, why):
        if getattr(self, 'no_retreat', False):
            return False, 'v2.12 no_retreat: holding and replanning instead of backing away'
        return super().escape(q, ver, why)

    # ------------------------------------------------------------------ v2.6: monitor that cannot die silently
    def monitor(self):
        if not getattr(self, 'monitor_safe', False):
            return super().monitor()
        self._v26_thread = threading.current_thread()
        self._v26_hb = time.monotonic()
        threading.Thread(target=self.rest_watch, daemon=True).start()       # as StrategyBDetourA2S.monitor
        threading.Thread(target=self._v26_watchdog, daemon=True).start()
        last = time.monotonic()
        while rclpy.ok():
            time.sleep(0.05)
            t_in = time.monotonic()
            period, last = t_in - last, t_in
            try:
                ok = self._v26_pass(t_in, period)
            except Exception:
                import traceback
                ok = False
                self.event('MONITOR_EXCEPTION (pass skipped, monitor continues)\n' + traceback.format_exc())
            if ok:
                self._v26_misses, self._v26_good = 0, self._v26_good + 1
                self._v26_hb = time.monotonic()
            else:
                self._v26_misses, self._v26_good = self._v26_misses + 1, 0

    def _v26_snapshot(self):
        """one consistent arm state (q, qd, q_t, q_stamp) or None; a busy read is retried twice."""
        r = self.__dict__.get('_reader')
        if self.js_process and r is not None:
            for _ in range(3):
                x = r.read()
                if x is not None:
                    self.__dict__['_v26_last'] = x
                    return np.array(x['q'], float), x['recv_ns'] / 1e9, x['stamp_ns']
                time.sleep(0.002)
            return None
        with self.lock:
            q = self.__dict__.get('q')
            if q is None:
                return None
            return np.array(q, float), self.__dict__.get('q_t'), self.__dict__.get('q_stamp')

    def _v26_pass(self, t_in, period):
        """StrategyBAdaptive.monitor's loop body, on one pinned state. True = a good pass."""
        with self.slock:
            cur, ver = self.cur, self.ver
        now = time.monotonic()
        active = not (self.base is None or self.finished(cur, now) or cur['kind'] in ('EMERGENCY_STOP',))
        snap = self._v26_snapshot()
        if snap is None:
            return not self._v26_had_state                                  # before the first state: not a miss
        self._v26_had_state = True
        q, q_t, q_stamp = snap
        decision = 'idle'
        if active:
            decision = self.monitor_check(cur, ver, q, now)
        elif self.finished(cur, now):
            self.frozen = None
        if self.mon_log:
            busy = self.busy
            self.mon_log.write(json.dumps({
                't': round(now, 4), 'active': active, 'decision': decision,
                'rx_age_ms': None if q_t is None else round(1000 * (now - q_t), 1),
                'stamp_age_ms': None if not q_stamp else round((self.get_clock().now().nanoseconds - q_stamp) / 1e6, 1),
                'period_ms': round(1000 * period, 1), 'exec_ms': round(1000 * (time.monotonic() - t_in), 1),
                'busy': None if busy is None else busy[0], 'busy_for_ms': None if busy is None else round(1000 * (now - busy[1]))}) + '\n')
        return True

    def _v26_watchdog(self):
        while rclpy.ok():
            time.sleep(0.025)
            if not rclpy.ok():                                               # node shutting down: the monitor's
                return                                                       # loop ends normally, not a failure
            try:
                now = time.monotonic()
                th = self._v26_thread
                why = None
                if th is None or not th.is_alive():
                    why = 'monitor thread not alive'
                elif self._v26_misses >= self.v26_max_misses:
                    why = f'{self._v26_misses} consecutive monitor passes missed'
                elif self.v26_stuck > 0 and now - self._v26_hb > self.v26_stuck:
                    why = f'no good monitor pass for {1000 * (now - self._v26_hb):.0f} ms'
                if why and not self._v26_failsafe:
                    self._v26_trip(why)
                elif not why and self._v26_failsafe and self._v26_good >= 3:
                    self._v26_failsafe = False
                    self.event(f'MONITOR_HEALTHY again ({self._v26_good} good passes); {self._v26_blocked} trajectories '
                               f'were refused meanwhile')
                    self._v26_blocked = 0
            except Exception:
                import traceback
                self.event('MONITOR_WATCHDOG_EXCEPTION\n' + traceback.format_exc())

    def _v26_trip(self, why):
        if not rclpy.ok():
            return
        self._v26_failsafe, self._v26_good = True, 0
        t0 = time.monotonic()
        self.event(f'MONITOR_FAILSAFE t={t0:.3f} {why}: cancelling the controller goal; no new trajectory until the '
                   f'monitor is healthy')
        fut = getattr(self.ctrl, 'last', None)
        h = fut.result() if fut is not None and fut.done() else None
        if h is None or not h.accepted:
            self.event('MONITOR_FAILSAFE_HALT no accepted controller goal to cancel (arm not commanded)')
            return
        r = self.wait(h.cancel_goal_async(), 0.5)
        took = 1000 * (time.monotonic() - t0)
        if r is None:
            self.event(f'MONITOR_FAILSAFE_HALT_FAILED the cancel was not answered in 0.5 s')
            return
        try:
            q = self.q                                     # js_process + v2.6: newest, else the last good state
        except Exception:
            q = None
        got = self.slock.acquire(timeout=0.2)
        try:
            if q is not None:
                self.cur = dict(pos=np.array(q, float)[None, :], tim=np.zeros(1), vel=np.zeros((1, 6)), t0=t0, path=None,
                                full=False, kind='MONITOR_FAILSAFE')
            self.ver += 1
        finally:
            if got:
                self.slock.release()
        self.event(f'MONITOR_FAILSAFE_HALT controller goal cancelled in {took:.0f} ms (holds at its measured position)')

    # ------------------------------------------------------------------ v2.4: logging only
    _V24_STOP = re.compile(r'^(EMERGENCY_STOP|SAFE_STOP|HOLD|STOP\b|WAIT\b|NO_SAFE_RESPONSE|STALE_STATE)')

    def on_tracks(self, m):
        super().on_tracks(m)
        if getattr(self, 'log_stop_object', False):
            self._v24_tracks = {o.id: (o.class_id, round(float(o.confidence), 3), round(float(o.age), 3),
                                       round(float(o.time_since_update), 3)) for o in m.objects}

    def occupancy(self, H, start, required, optional, t_ref):
        if getattr(self, 'region_age_fix', False):
            try:
                self._v24_local.snap = {k: (o['t'], np.round(o['p'], 4).tolist()) for k, o in list(self.objs.items())}
            except Exception:
                self._v24_local.snap = None
        res = super().occupancy(H, start, required, optional, t_ref)
        if not res[3] and H > 0 and self.__dict__.get('human_mode') and self._v243('zero'):
            # v2.43 Fix A (diagnosis: 38 % of the rejections had the start clear AT the splice time; the 0.25 s steps
            # stop at H mod 0.25 > 0): the region at exactly t_ref. Only the planning region changes; every candidate
            # is still checked along its whole timed path against the full prediction before it is sent.
            r0 = super().occupancy(0.0, start, required, optional, t_ref)
            if r0[3]:
                self.event(f'V243_ZERO planning region at the splice time (horizon 0) instead of none (H {H:.2f} s)')
                return r0
        if not res[3]:
            try:
                self._diag_start(start, required, t_ref, H)
            except Exception as e:
                self.event(f'DIAG_START error {e!r}')
        return res

    def _diag_start(self, start, required, t_ref, H):
        """DIAGNOSTIC ONLY (24 Nov night, copy v242bdiag, never installed): why is the planning start state inside the
        region at every horizon? Distances (m, unpadded robot geometry) of the start state to each object's MEASURED box
        (no growth, no pad), to its region at t_ref (= what the check used at horizon 0) and to its region with the
        growth cut to the latency only; and of the robot's current state to the measured box. Nothing is decided here."""
        now = time.monotonic()
        ps = _v24_fk(np.asarray(start, float), self.base)
        pq = _v24_fk(np.asarray(self.q, float), self.base) if self.q is not None else None
        pad = self.clear_req + self.plan_pad
        objs = []
        for oid, o in list(self.objs.items()):
            st = self.motion.get(oid)
            raw = [(o['p'] + o['R'] @ cl, o['R'] @ Rl, h) for cl, Rl, h in o['parts']]
            d_raw = min(_v24_box_dist(ps, b) for b in raw) if raw else None
            d_now = min(_v24_box_dist(pq, b) for b in raw) if raw and pq is not None else None
            hh = self.h_horizon
            try:
                full = [b for b in self.reach_at(t_ref, pad=pad)]
                self.h_horizon = min(hh, self.latency + 1e-3)
                lat_only = [b for b in self.reach_at(t_ref, pad=pad)]
            finally:
                self.h_horizon = hh
            objs.append(dict(id=oid, person=bool(self._v238_is_person(oid, o)), still=bool(st and st.get('still')),
                             age=round(now - o['t'], 3), v_peak=None if not st else st.get('peak'),
                             v_now=None if not st else st.get('now'), p=np.round(o['p'], 3).tolist(),
                             d_raw=None if d_raw is None else round(float(d_raw), 4),
                             d_now_q=None if d_now is None else round(float(d_now), 4)))
        d_full = min((_v24_box_dist(ps, b) for b in full), default=None)
        d_lat = min((_v24_box_dist(ps, b) for b in lat_only), default=None)
        req_in = [round(float(min((_v24_box_dist(_v24_fk(np.asarray(x, float), self.base), b) for b in full), default=9)), 4)
                  for x in required]
        self.event('DIAG_START ' + json.dumps(dict(
            t_mono=round(now, 3), t_ref_ahead=round(t_ref - now, 3), H=round(float(H), 2), latency=self.latency,
            h_horizon=self.h_horizon, pad=pad, d_region=None if d_full is None else round(float(d_full), 4),
            d_region_latency_only=None if d_lat is None else round(float(d_lat), 4), required=req_in,
            start=np.round(np.asarray(start, float), 4).tolist(), base=None if self.base is None else np.round(np.asarray(self.base, float), 4).tolist(),
            objs=objs), default=float))

    def event(self, text):
        try:
            super().event(text)
        except Exception:                                                   # v2.23: publisher gone at shutdown (B2 v2.22
            if rclpy.ok():                                                  # rest_watch traceback after REACHED): only
                raise                                                       # then is a failed log line ignored
            return
        if not getattr(self, 'log_stop_object', False) or not self._V24_STOP.match(text):
            return
        try:                                                                # copies only; no lock, no computation
            now = time.monotonic()
            objs = []
            for k, o in list(self.objs.items()):
                st = self.motion.get(k)
                objs.append((k, np.array(o['p'], float), np.array(o['R'], float), list(o['parts']), o['t'],
                             None if st is None else (st['peak'], st['now'])))
            self._v24_q.put(dict(t=now, text=text, q=np.array(self.q, float), age=self.state_age(),
                                 base=None if self.base is None else np.array(self.base, float),
                                 tracks=dict(self._v24_tracks), objs=objs))
        except Exception as e:
            self.get_logger().info(f'v2.4 stop-object snapshot failed: {e!r}')

    def _v24_writer(self):
        while True:
            r = self._v24_q.get()
            try:
                pts = _v24_fk(r['q'], r['base']) if r['base'] is not None else None
                out = []
                for k, pc, R, parts, t_obs, mo in r['objs']:
                    d = None
                    if pts is not None and parts:
                        d = round(float(min(_v24_box_dist(pts, (pc + R @ cl, R @ Rl, h)) for cl, Rl, h in parts)), 4)
                    hs = np.array([h for _, _, h in parts]) if parts else np.zeros((1, 3))
                    tid = self.track_id(k)
                    tr = self._v24_tracks.get(tid) if tid is not None else None
                    tr = r['tracks'].get(tid, tr)
                    out.append(dict(id=k, track=None if tr is None else dict(cls=tr[0], conf=tr[1], age=tr[2], since_update=tr[3]),
                                    p=np.round(pc, 4).tolist(), n_parts=len(parts), max_half=round(float(hs.max()), 4),
                                    extent=np.round(2 * hs.max(0), 4).tolist(), obs_age=round(r['t'] - t_obs, 3),
                                    v_peak=None if mo is None else round(mo[0], 4), v_now=None if mo is None else round(mo[1], 4),
                                    d_arm=d))
                out.sort(key=lambda x: np.inf if x['d_arm'] is None else x['d_arm'])
                rec = dict(t=r['t'], event=r['text'][:200], state_age_ms=round(1000 * r['age'], 1) if np.isfinite(r['age']) else None,
                           q=np.round(r['q'], 5).tolist(), n_objects=len(out), nearest=out[0] if out else None, objects=out)
                if self._v24_log:
                    self._v24_log.write(json.dumps(rec) + '\n')
                    self._v24_log.flush()
            except Exception as e:
                self.get_logger().info(f'v2.4 stop-object log failed: {e!r}')

    # ------------------------------------------------------------------ v2.1: arm-state staleness guard
    def state_age(self):
        """s since the header stamp of the newest arm state (inf if none)."""
        qs = getattr(self, 'q_stamp', None)
        return float('inf') if not qs else (self.get_clock().now().nanoseconds - qs) / 1e9

    def wait_fresh_state(self):
        """age of the newest arm state after waiting up to state_wait for one younger than state_max_age."""
        end = time.monotonic() + self.state_wait
        age = self.state_age()
        while age > self.state_max_age and time.monotonic() < end:
            time.sleep(0.002)
            age = self.state_age()
        return age

    def send(self, pos, tim, vel, path, full, kind, ver=None):
        if getattr(self, '_v26_failsafe', False):
            self._v26_blocked += 1
            if self._v26_blocked == 1:
                self.event(f'MONITOR_FAILSAFE_BLOCKED {kind}: not sent (monitor not healthy); further refusals counted silently')
            return False
        if self.state_guard and not kind.endswith('STOP'):
            age = self.state_age()
            if age > self.state_max_age:
                self.event(f'STALE_STATE {kind}: newest arm state {1000 * age:.0f} ms old (> {1000 * self.state_max_age:.0f}'
                           f' ms) - not sent')
                return False
        if self.state_guard and kind == 'SAFE_STOP':
            age = self.state_age()
            if age > self.state_max_age:
                self.event(f'STALE_STATE SAFE_STOP: newest arm state {1000 * age:.0f} ms old - cut not sent (hold instead)')
                return False
            with self.lock:
                q = self.q.copy()
            k0 = int(np.searchsorted(tim, 0.02))
            if k0 < len(pos):
                off = float(np.abs(np.asarray(pos[k0]) - q).max())
                allowed = self.vmax * float(tim[k0]) + self.cut_start_tol
                if off > allowed:
                    self.event(f'CUT_OFF_ARM SAFE_STOP first point {off:.3f} rad from the measured arm (> {allowed:.3f})'
                               f' - not sent (hold instead)')
                    return False
        pos, tim, vel = self._v229_merge(pos, tim, vel, kind)               # v2.30: merge BEFORE the stretch
        if self.corner_stretch or self.entry_clamp:
            pos, tim, vel = np.asarray(pos, float), np.array(tim, float), np.asarray(vel, float)
            k0 = int(np.searchsorted(tim, 0.02))                             # the parent drops the points before k0
            if k0 < len(tim):
                if self.corner_stretch:
                    tim = self._v25_corner_stretch(pos, tim, vel, k0, kind)
                if self.entry_clamp:
                    tim = self._v25_entry_clamp(pos, tim, vel, k0, kind)
        return super().send(pos, tim, vel, path, full, kind, ver=ver)

    def _v229_merge(self, pos, tim, vel, kind, min_dt=0.010):
        """v2.29: drop sent points less than min_dt after the previous kept point (the last point is always kept). The
        v2.28 harness batch had 2 speed jumps (0.58-0.60 rad/s) right after 2-5 ms segments at splices, with the
        commanded curve <= 1.00 vmax: the controller / plant cannot follow a 2 ms segment smoothly."""
        tim = np.asarray(tim, float)
        if len(tim) < 3:
            return pos, tim, vel
        keep = [0]
        for i in range(1, len(tim) - 1):
            if tim[i] - tim[keep[-1]] >= min_dt:
                keep.append(i)
        if tim[-1] - tim[keep[-1]] < min_dt and len(keep) > 1:
            keep.pop()
        keep.append(len(tim) - 1)
        if len(keep) == len(tim):
            return pos, tim, vel
        return np.asarray(pos)[keep], tim[keep], np.asarray(vel)[keep]

    # ------------------------------------------------------------------ v2.5: speed (F1 corners, F2 entry)
    _V25_X = np.linspace(0.0, 1.0, 24).reshape(1, 24, 1)

    @classmethod
    def _v25_peaks(cls, a, b, va, vb, T, per_joint=False):
        """peak |joint speed| of cubic Hermite segments a -> b (rows), end velocities va, vb, durations T (n,);
        (n,) over all joints, or (n, 6) per joint."""
        x = cls._V25_X
        d00, d10, d01, d11 = 6 * x ** 2 - 6 * x, 3 * x ** 2 - 4 * x + 1, -6 * x ** 2 + 6 * x, 3 * x ** 2 - 2 * x
        T = np.maximum(np.asarray(T, float), 1e-4)[:, None, None]
        v = np.abs((d00 * a[:, None, :] + d01 * b[:, None, :]) / T + d10 * va[:, None, :] + d11 * vb[:, None, :])
        return v.max(axis=1) if per_joint else v.max(axis=(1, 2))

    def _v25_corner_stretch(self, pos, tim, vel, k0, kind):
        p, v, t = pos[k0:], vel[k0:], tim[k0:].copy()
        if len(t) < 2:
            return tim
        lim = 1.005 * self.vmax
        first = None
        for _ in range(20):
            sv = self._v25_peaks(p[:-1], p[1:], v[:-1], v[1:], np.diff(t))
            first = sv.max() if first is None else first
            if sv.max() <= lim:
                break
            f = np.minimum(np.maximum(sv / self.vmax, 1.0), 1.5)
            t = np.r_[t[0], t[0] + np.cumsum(np.diff(t) * f)]
        added = float(t[-1] - tim[-1])
        if added <= 1e-6:
            return tim
        out = tim.copy()
        out[k0:] = t
        self.event(f'CORNER_STRETCH {kind} +{1000 * added:.0f} ms: segment peak {first / self.vmax:.3f} -> '
                   f'{sv.max() / self.vmax:.3f} x vmax')
        return out

    def _v25_entry_clamp(self, pos, tim, vel, k0, kind):
        with self.lock:
            q, qd = getattr(self, 'q', None), getattr(self, 'qd', None)
            q = None if q is None else np.array(q, float)
            qd = np.zeros(6) if qd is None else np.array(qd, float)
        if q is None or q.shape != (6,):
            return tim
        age = self.state_age()
        q = q + qd * (min(age, 0.05) if np.isfinite(age) and age > 0 else 0.0)
        T0 = float(tim[k0])
        ds = np.arange(0.0, self.entry_clamp_max + 1e-9, 0.002)
        n = len(ds)
        pj = self._v25_peaks(np.repeat(q[None], n, 0), np.repeat(pos[k0][None], n, 0), np.repeat(qd[None], n, 0),
                             np.repeat(vel[k0][None], n, 0), T0 + ds, per_joint=True)
        L = 1.005 * np.maximum(self.vmax, np.abs(qd))           # per joint: a cubic cannot start below the joint's own speed
        r = (pj / L).max(axis=1)                                  # <= 1: every joint within its limit
        pk = pj.max(axis=1)
        if r[0] <= 1.0:
            return tim
        ok = np.nonzero(r <= 1.0)[0]
        if not len(ok):
            self.event(f'ENTRY_STALE {kind}: entry needs {pk[0] / self.vmax:.2f} x vmax (first point '
                       f'{float(np.abs(pos[k0] - q).max()):.4f} rad in {T0:.3f} s); > {1000 * self.entry_clamp_max:.0f} ms '
                       f'delay needed - not delayed')
            return tim
        d = float(ds[ok[0]])
        out = tim.copy()
        out[k0:] = tim[k0:] + d
        self.event(f'ENTRY_CLAMP {kind} +{1000 * d:.0f} ms: entry {pk[0] / self.vmax:.3f} -> {pk[ok[0]] / self.vmax:.3f} x vmax '
                   f'(first point {float(np.abs(pos[k0] - q).max()):.4f} rad in {T0:.3f} s)')
        return out

    def hold_here(self, why, label='EMERGENCY_STOP', ver=None):
        # v2.30 F1 sc3v2_estop_cancel (default OFF): an EMERGENCY_STOP cancels the controller goal (the JTC holds at its
        # measured position). Real UR 23 Nov: SC3's one-point stop travels 45 mrad after the send (0.15 s x speed ahead),
        # a cancel 2.9 mrad (X-1/X-2). Falls back to the one-point stop if the cancel is not answered.
        if label == 'EMERGENCY_STOP' and getattr(self, 'estop_cancel', False):
            if self.controller_halt(why, label, ver):
                return
        if not self.state_guard:
            return super().hold_here(why, label=label, ver=ver)
        age = self.wait_fresh_state()
        if age <= self.state_max_age:
            return super().hold_here(why, label=label, ver=ver)              # SC3's own stop, from a fresh state
        self.event(f'STALE_STATE {label}: newest arm state {1000 * age:.0f} ms old (> {1000 * self.state_max_age:.0f} ms)'
                   f' - not used ({why})')
        if self.stale_stop == 'cancel' and self.controller_halt(why, label, ver):
            return
        with self.lock:                                                      # last resort: the old state moved on
            q, qd = self.q.copy(), self.qd.copy()
        a = min(age, 0.5)
        q_now = q + qd * a
        target = q_now + qd * 0.15
        if self.send(np.vstack([q_now, target]), np.array([0.0, 0.3]), np.zeros((2, 6)), None, False, label, ver=ver):
            self.event(f'{label} t={time.monotonic():.3f} {why} (from the old state + qd x {1000 * a:.0f} ms)')
        else:
            self.event(f'{label}_FAILED {why}: the controller did not accept the stop')

    def controller_halt(self, why, label, ver):
        """cancel the controller's active goal: the JTC holds at ITS measured position (set_hold_position)."""
        fut = getattr(self.ctrl, 'last', None)
        with self.slock:
            if ver is not None and ver != self.ver:
                self.event(f'STALE_{label}_DROPPED {why}: the trajectory it judged (v{ver}) was replaced (now v{self.ver})')
                return True
            h = fut.result() if fut is not None and fut.done() else None
            if h is None or not h.accepted:
                self.event(f'{label}_HALT_FAILED no active controller goal to cancel')
                return False
            t0 = time.monotonic()
            r = self.wait(h.cancel_goal_async(), 0.5)
            if r is None:
                self.event(f'{label}_HALT_FAILED the cancel was not answered in 0.5 s')
                return False
            with self.lock:
                q = self.q.copy()
            self.cur = dict(pos=q[None, :], tim=np.zeros(1), vel=np.zeros((1, 6)), t0=t0, path=None, full=False, kind=label)
            self.ver += 1
        self.event(f'{label} t={t0:.3f} HALT controller goal cancelled in {1000 * (time.monotonic() - t0):.0f} ms '
                   f'(holds at its measured position) {why}')
        return True

    # ------------------------------------------------------------------ v2.1: fresh SAFE_STOP cut
    def safe_prefix(self, cur, j, now, ver):
        out = super().safe_prefix(cur, j, now, ver)
        if out is None or not self.fresh_cut:
            return out
        return self.rebase_cut(cur, out, now)

    def rebase_cut(self, cur, out, now_built):
        """the cut found for now_built, re-spliced for the time it is sent: same resting point (the running trajectory's
        own point m), splice index from the fresh time, re-checked. None = no cut (the caller holds)."""
        pos, tim, vel, path, full, kind = out
        m = next((i for i in range(len(cur['pos']) - 1, -1, -1) if np.array_equal(cur['pos'][i], pos[-1])), None)
        now2 = time.monotonic()
        age = now2 - now_built
        if m is None:
            self.event(f'CUT_REBASE_FAILED resting point not on the running trajectory (built {1000 * age:.0f} ms ago)')
            return None
        tau = now2 - cur['t0']
        i_s = min(int(np.searchsorted(cur['tim'], tau + 0.15)), len(cur['pos']) - 1)
        o = self.splice(cur, i_s, cur['pos'][i_s:m + 1], now2) if i_s < m else None
        if o is None:
            self.event(f'CUT_REBASE_PASSED the resting point is no longer ahead (built {1000 * age:.0f} ms ago): hold')
            return None
        p2, t2, v2 = o
        if self.conflict_rel(p2, t2, now2) is not None or not self.wait_safe(p2[-1], now2 + t2[-1] + self.t_slack):
            self.event(f'CUT_REBASE_UNSAFE the re-based cut is not clear (built {1000 * age:.0f} ms ago): hold')
            return None
        took = time.monotonic() - now2
        if took > self.cut_rebase_max:
            self.event(f'CUT_REBASE_SLOW re-base took {1000 * took:.0f} ms > {1000 * self.cut_rebase_max:.0f} ms: hold')
            return None
        self.event(f'CUT_REBASED built {1000 * age:.0f} ms ago, re-spliced at the send time ({1000 * took:.0f} ms)')
        return p2, t2, v2, path, full, kind

    # ------------------------------------------------------------------ v2.2: separate-process joint-state reader (R7)
    def _start_js_process(self):
        import atexit
        from sc3v2_state_reader import StateReader
        for s in list(self.subscriptions):
            if s.topic_name.lstrip('/') == 'joint_states':
                self.destroy_subscription(s)
        reader = StateReader(reliability=self.js_reliability).start()
        atexit.register(reader.close)
        self.__dict__['_reader'] = reader
        cls = self.__class__
        self.__class__ = type(cls.__name__ + 'JsProc', (_JsProcState, cls), {})
        end = time.monotonic() + 10.0
        while reader.read() is None and time.monotonic() < end:
            time.sleep(0.01)
        r = reader.read()
        self.event(f'SC3 v2.2: js_process 1 ({self.js_reliability}; reader pid {reader.proc.pid}, shm {reader.name}); first state '
                   f'{"none within 10 s" if r is None else "%.0f ms old" % ((self.get_clock().now().nanoseconds - r["stamp_ns"]) / 1e6)}')

    # ------------------------------------------------------------------ v2.2: D9 rest-to-rest retime fix
    def retime(self, pos, v0=None):
        pos2, tim, vel = super().retime(pos, v0)
        if not getattr(self, 'retime_fix', False) or len(pos2) < 2:
            return pos2, tim, vel
        ds = np.abs(np.diff(pos2, axis=0)).max(1)
        dt = np.diff(tim)
        bad = np.isclose(dt, 2 * ds / 1e-6, rtol=1e-9, atol=0.0)              # both segment ends at speed 0 (parent's 1e-6 floor)
        if not bad.any():
            return pos2, tim, vel
        dt = dt.copy()
        dt[bad] = np.maximum(np.sqrt(6 * ds[bad] / self.amax), 1.5 * ds[bad] / self.vmax)
        self.event(f'RETIME_FIX {int(bad.sum())} rest-to-rest segment(s): {float(np.diff(tim)[bad].max()):.0f} s -> '
                   f'{float(dt[bad].max()):.3f} s')
        return pos2, np.r_[0.0, np.cumsum(dt)], vel

    # ------------------------------------------------------------------ v2.1: newest joint state
    def _start_js_latest(self):
        for s in list(self.subscriptions):
            if s.topic_name.lstrip('/') == 'joint_states':
                self.destroy_subscription(s)
        self._js_node = rclpy.create_node('sc3v2_joint_states')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._js_node.create_subscription(JointState, '/joint_states', self.on_joints_latest, qos)
        self._js_exec = SingleThreadedExecutor()
        self._js_exec.add_node(self._js_node)
        threading.Thread(target=self._js_exec.spin, daemon=True).start()

    def on_joints_latest(self, m):
        """as on_joints (strategy_a_global + adaptive's stamp), but qd from the header stamps, not the receive times."""
        try:
            q = np.array([m.position[list(m.name).index(n)] for n in ARM])
        except ValueError:
            return
        ts = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = time.monotonic()
        with self.lock:
            if self.q is not None and self._js_ts is not None and ts - self._js_ts > 1e-3:
                self.qd = 0.7 * self.qd + 0.3 * (q - self.q) / (ts - self._js_ts)
            self.q, self.q_t, self._js_ts = q, t, ts
        self.q_stamp = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec

    def planned_cost(self):
        return self.dense_cost_factor * max(self.dense_cost, self.dense_cost_floor)

    # ------------------------------------------------------------------ the deadline-bound dense check
    def dense_valid(self, states, deadline):
        """True (all valid), False (one invalid), None (not finished by the deadline: INCOMPLETE)."""
        c = self.planned_cost()
        futs, t0 = [], time.monotonic()
        while True:
            now = time.monotonic()
            if any(f.done() and (f.result() is None or not f.result().valid) for f in futs):
                return False
            if len(futs) == len(states) and all(f.done() for f in futs):
                ms_per = (time.monotonic() - t0) / max(len(states), 1)
                self.dense_cost = 0.7 * self.dense_cost + 0.3 * ms_per          # measured live cost
                return True
            if now >= deadline:
                return None
            inflight = sum(not f.done() for f in futs)
            while (len(futs) < len(states) and inflight < self.dense_window
                   and time.monotonic() + c * (inflight + 1) <= deadline):
                q = states[len(futs)]
                futs.append(self.valid_cli.call_async(GetStateValidity.Request(robot_state=RobotState(
                    joint_state=JointState(name=ARM, position=[float(x) for x in q]), is_diff=True), group_name=GROUP)))
                inflight += 1
            time.sleep(0.0005)

    # ------------------------------------------------------------------ capped search + check
    def shortcut(self, cur, i_s, seq, rest, t_end):
        seq = [np.asarray(q, float) for q in seq]
        t0 = time.monotonic()
        W = t_end - t0
        st = dict(n_before=len(seq), len_before=round(float(np.abs(np.diff(seq, axis=0)).max(1).sum()), 3), evaluations=0,
                  window_ms=round(1000 * W, 1), cost_ms_per_state=round(1000 * self.planned_cost(), 2))
        if W <= 0.03 or len(seq) < 4:
            return None, dict(st, result='no time to shorten' if W <= 0.03 else 'nothing to shorten')
        t_search = t0 + self.sc_search_share * W
        deadline = t_end - self.dense_margin
        n_cap = int((deadline - t_search) / self.planned_cost())
        st['state_cap'] = n_cap
        if n_cap < 3:
            return None, dict(st, result='no time for a dense check')
        base = self._first_conflict_t(cur, i_s, seq, rest, t0)
        st['evaluations'] = 1
        if base is None:
            return None, dict(st, result='nothing to shorten')
        step = float(np.median(np.abs(np.diff(seq, axis=0)).max(1))) or 0.02
        rng = np.random.default_rng(len(seq))                          # as SC: reproducible
        length = lambda q: float(np.abs(np.diff(q, axis=0)).max(1).sum())  # noqa: E731
        orig = list(seq)
        n_new = 0
        span = max(2, int(n_cap * self.dense_step / step))             # waypoints a checkable shortcut can bridge
        while len(seq) >= 4 and time.monotonic() < t_search:
            if rng.random() < 0.5:                                      # local pair: small enough to be checked in time
                i = int(rng.integers(0, len(seq) - 2))
                j = min(len(seq) - 1, i + int(rng.integers(2, span + 1)))
            else:                                                       # any pair (SC's rule)
                i, j = sorted(int(x) for x in rng.choice(len(seq), 2, replace=False))
            if j - i < 2:
                continue
            k = max(2, int(np.ceil(np.abs(seq[j] - seq[i]).max() / step)))
            mid = [seq[i] + (seq[j] - seq[i]) * s for s in np.linspace(0, 1, k + 1)[1:-1]]
            cand = seq[:i + 1] + mid + seq[j:]
            if length(cand) >= length(seq) - 1e-6:
                continue
            n_c = len(self._new_states(orig, cand))
            if n_c > n_cap:                                             # could not be checked in time: not taken
                continue
            st['evaluations'] += 1
            r = self._first_conflict_t(cur, i_s, cand, rest, t0)
            if r is not None and r >= base - 1e-9:
                seq, n_new = cand, n_c
        st.update(n_after=len(seq), len_after=round(length(seq), 3), ms=round(1000 * (time.monotonic() - t0), 1), dense_states=n_new)
        if st['len_after'] >= st['len_before'] - 1e-6:
            return None, dict(st, result='no shorter route found')
        t1 = time.monotonic()
        states = self._new_states(orig, seq)
        ok = self.dense_valid(states, deadline) if states else True
        st.update(dense_ms=round(1000 * (time.monotonic() - t1), 1), dense_ok=ok, dense_cost_after_ms=round(1000 * self.dense_cost, 2))
        if ok is None:
            return None, dict(st, result='dense check incomplete at its deadline: shortened route rejected')
        if not ok:
            return None, dict(st, result='shortened route failed the dense table / self-collision check')
        return np.array(seq), dict(st, result='shortened')



# ====================================================================== v2.8: D-items layer (formerly sc3v2_d_items.py)
STOP_KINDS = ('SAFE_STOP', 'EMERGENCY_STOP', 'STOP', 'RETREAT')


class SC3DItems(StrategyBDetourA2SG1SC3):

    # clear_req as a property: the parents assign it once in __init__; a thread may override it for one cycle (D2)
    @property
    def clear_req(self):
        loc = self.__dict__.get('_d_local')
        o = getattr(loc, 'req', None) if loc is not None else None
        return self.__dict__['_clear_req_base'] if o is None else o

    @clear_req.setter
    def clear_req(self, v):
        self.__dict__['_clear_req_base'] = v

    def __init__(self):
        self.__dict__['_d_local'] = threading.local()
        super().__init__()
        p = self.declare_parameter
        self.d2 = bool(p('sc3v2_d2_early', False).value)
        self.d2_soft = float(p('sc3v2_d2_soft', 0.04).value)
        self.d1_period = float(p('sc3v2_d1_period', 0.05).value)
        if self.d1_period != 0.05:
            self.event(f'SC3 D-items: d1_period {1000 * self.d1_period:.0f} ms (SC3: 50 ms)')
        self.d4 = bool(p('sc3v2_d4_cpp_check', False).value)
        self.d4_shadow = bool(p('sc3v2_d4_shadow', False).value)
        self._d4_log = None
        if self.d4_shadow:
            import os
            d = os.environ.get('ROS_LOG_DIR')
            self._d4_log = open(os.path.join(os.path.dirname(d), 'logs', 'SB_d4.jsonl'), 'a') if d else None
        self._d4_reports, self._d4_alive, self._d4_id = {}, 0.0, 0
        self.d4_counts = dict(cpp=0, fallback=0, invalid=0, incomplete=0)
        if self.d4 or self.d4_shadow:
            q = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
            self._d4_pub = self.create_publisher(JointTrajectory, '/replan_moving/plan_check', q)
            self.create_subscription(Float64MultiArray, '/replan_moving/plan_report', self._d4_on_report, 10)
            self.create_subscription(Float64, '/replan_moving/lookahead_alive',
                                     lambda m: self.__dict__.__setitem__('_d4_alive', time.monotonic()), 10)
            self.event(f'SC3 D-items: d4_cpp_check {int(self.d4)}, d4_shadow {int(self.d4_shadow)} (C++ look-ahead plan_check)')
        self._d2_ver = None
        self.d2_counts = dict(soft=0, cycles=0)
        if self.d2:
            self.event(f'SC3 D-items: d2_early 1 (soft band {100 * self.d2_soft:.1f} cm)')

    # ------------------------------------------------------------------ D1: monitor period
    def monitor(self):
        if self.d1_period == 0.05:
            return super().monitor()                                         # SC3 unchanged
        threading.Thread(target=self.rest_watch, daemon=True).start()         # as strategy_b_detour_a2s.monitor
        last = time.monotonic()                                               # strategy_b_adaptive.monitor, period param
        while rclpy.ok():
            time.sleep(self.d1_period)
            t_in = time.monotonic()
            period, last = t_in - last, t_in
            with self.slock:
                cur, ver = self.cur, self.ver
            now = time.monotonic()
            active = not (self.base is None or self.finished(cur, now) or cur['kind'] in ('EMERGENCY_STOP',))
            with self.lock:
                q, q_t, q_stamp = (None, None, None) if self.q is None else (self.q.copy(), self.q_t, self.q_stamp)
            decision = 'idle'
            if active and q is not None:
                decision = self.monitor_check(cur, ver, q, now)
            elif self.finished(cur, now):
                self.frozen = None
            if self.mon_log and q is not None:
                busy = self.busy
                self.mon_log.write(json.dumps({
                    't': round(now, 4), 'active': active, 'decision': decision,
                    'rx_age_ms': round(1000 * (now - q_t), 1),
                    'stamp_age_ms': None if not q_stamp else round((self.get_clock().now().nanoseconds - q_stamp) / 1e6, 1),
                    'period_ms': round(1000 * period, 1), 'exec_ms': round(1000 * (time.monotonic() - t_in), 1),
                    'busy': None if busy is None else busy[0], 'busy_for_ms': None if busy is None else round(1000 * (now - busy[1]))}) + '\n')

    # ------------------------------------------------------------------ D2: early warning band
    def monitor_check(self, cur, ver, q, now):
        r = super().monitor_check(cur, ver, q, now)
        if not self.d2 or r != 'clear' or self.goal is None or cur.get('kind') in STOP_KINDS or self._d2_ver == ver:
            return r
        try:
            tau = now - cur['t0']
            k = min(int(np.searchsorted(cur['tim'], tau)), len(cur['pos']) - 1)
            self._d_local.req = self.__dict__['_clear_req_base'] + self.d2_soft
            try:
                j = self.first_conflict(cur['pos'], cur['tim'], now, k, self.monitor_horizon)
            finally:
                self._d_local.req = None
            if j is not None:
                self._d2_ver = ver
                self.d2_counts['soft'] += 1
                self.event(f'D2_EARLY soft conflict {float(cur["tim"][j] - cur["tim"][k]):.2f} s ahead on {cur["kind"]} '
                           f'(v{ver}, band {100 * self.d2_soft:.1f} cm): planner woken')
                self.threat_evt.set()
        except Exception as e:                                               # the early warning never blocks a stop
            self.event(f'D2 check failed: {e!r}')
        return r

    def extend(self):
        if self.d2 and self._d2_ver is not None:
            now = time.monotonic()
            with self.slock:
                cur, ver = self.cur, self.ver
            if (ver == self._d2_ver and cur is not None and cur.get('full') and cur.get('kind') not in STOP_KINDS
                    and not self.finished(cur, now) and self._expired is None and not self.moveit_busy()[0]):
                with self.lock:
                    q = self.q.copy()
                self.moving_now = True
                i_s = min(int(np.searchsorted(cur['tim'], now - cur['t0'] + self.lead)), len(cur['pos']) - 1)
                self.busy = ('extend', time.monotonic())
                try:
                    return self._extend(cur, ver, i_s, q, True, now)
                finally:
                    self.busy = None
        return super().extend()

    def _extend(self, cur, ver, i_s, q, moving, now):
        if self.d2 and moving and self._d2_ver == ver and cur.get('kind') not in STOP_KINDS:
            self.d2_counts['cycles'] += 1
            self._d_local.req = self.__dict__['_clear_req_base'] + self.d2_soft
            t0 = time.monotonic()
            try:
                self.event(f'D2_CYCLE v{ver}: extension cycle with clear_req {100 * self._d_local.req:.1f} cm')
                return super()._extend(cur, ver, i_s, q, moving, now)
            finally:
                self._d_local.req = None
                self.event(f'D2_CYCLE_DONE v{ver} in {1000 * (time.monotonic() - t0):.0f} ms (now v{self.ver})')
                if self.ver == ver:                                          # v2.27: no new path for this trajectory
                    self.__dict__['_v227_fail'] = (ver, time.monotonic())
        return super()._extend(cur, ver, i_s, q, moving, now)


    # ------------------------------------------------------------------ D4 (= R1): C++ dense check
    def _d4_on_report(self, m):
        if len(m.data) >= 7:
            self._d4_reports[int(round(m.data[0]))] = (time.monotonic(), list(m.data))

    def _d4_send(self, states):
        self._d4_id += 1
        rid = 900000 + self._d4_id                                          # never a real stamp-based id
        tr = JointTrajectory(joint_names=list(ARM))
        tr.header.stamp = Time(sec=rid, nanosec=0)
        for i, q in enumerate(states):
            tr.points.append(JointTrajectoryPoint(positions=[float(x) for x in q],
                                                  time_from_start=Duration(sec=0, nanosec=int(1e7 * (i + 1)))))
        t0 = time.monotonic()
        self._d4_pub.publish(tr)
        return rid, t0

    def static_valid(self, seq, every=6, window=8):
        if not (self.d4 or self.d4_shadow) or len(seq) == 0 or time.monotonic() - self._d4_alive > 0.5:
            if self.d4:
                self.d4_counts['fallback'] += 1
            return super().static_valid(seq, every=every, window=window)
        states = list(seq[::every]) + [seq[-1]]
        rid, t0 = self._d4_send(states)
        res = super().static_valid(seq, every=every, window=window) if self.d4_shadow else None
        t_svc = time.monotonic() - t0
        end = time.monotonic() + (0.3 if self.d4_shadow else 2.0)
        r = None
        while time.monotonic() < end and r is None:
            r = self._d4_reports.pop(rid, None)
            time.sleep(0.0005)
        cpp = None if r is None else (None if r[1][6] < 0.5 else bool(r[1][1] < 0))
        if self.d4_shadow:
            if self._d4_log:
                self._d4_log.write(json.dumps(dict(t=t0, check='static', n=len(states), svc=res, svc_ms=round(1000 * t_svc, 2),
                    cpp=cpp, cpp_rt_ms=None if r is None else round(1000 * (r[0] - t0), 2),
                    cpp_node_ms=None if r is None else round(r[1][5], 2), cpp_checked=None if r is None else int(r[1][4]),
                    agree=None if cpp is None else (res == cpp))) + '\n'); self._d4_log.flush()
            return res
        self.d4_counts['cpp'] += 1
        return bool(cpp)                                                    # unanswered / incomplete -> invalid, as SC3

    def dense_valid(self, states, deadline):
        if self.d4_shadow and states and time.monotonic() - self._d4_alive <= 0.5:
            rid, t0 = self._d4_send(states)
            res = super().dense_valid(states, deadline)                     # SC3's own check: its result is used
            t_svc = time.monotonic() - t0
            end = time.monotonic() + 0.3
            r = None
            while time.monotonic() < end and r is None:
                r = self._d4_reports.pop(rid, None)
                time.sleep(0.0005)
            cpp = None if r is None else (None if r[1][6] < 0.5 else bool(r[1][1] < 0))
            rec = dict(t=t0, check='dense', n=len(states), svc=res, svc_ms=round(1000 * t_svc, 2), cpp=cpp,
                       cpp_rt_ms=None if r is None else round(1000 * (r[0] - t0), 2),
                       cpp_node_ms=None if r is None else round(r[1][5], 2), cpp_checked=None if r is None else int(r[1][4]),
                       agree=None if (res is None or cpp is None) else (res == cpp))
            if self._d4_log:
                self._d4_log.write(__import__('json').dumps(rec) + '\n'); self._d4_log.flush()
            return res
        if not self.d4 or not states or time.monotonic() - self._d4_alive > 0.5:
            if self.d4:
                self.d4_counts['fallback'] += 1
            return super().dense_valid(states, deadline)
        rid, t0 = self._d4_send(states)
        while time.monotonic() < deadline:
            r = self._d4_reports.pop(rid, None)
            if r is not None:
                d = r[1]
                ms_per = (r[0] - t0) / len(states)
                self.dense_cost = 0.7 * self.dense_cost + 0.3 * ms_per      # measured live cost, as SC3
                self.d4_counts['cpp'] += 1
                if d[6] < 0.5:
                    self.d4_counts['incomplete'] += 1
                    return None
                if d[1] >= 0:
                    self.d4_counts['invalid'] += 1
                    return False
                return True
            time.sleep(0.0005)
        self.d4_counts['incomplete'] += 1
        return None



# ====================================================================== v2.8: no-stop layer (formerly sc3v2_nostop.py)
LIM = 2 * np.pi - 0.05


# ---------------------------------------------------------------------- lane worker (separate process, no ROS)
def _w_ik(base, q_seed, target, fk_points):
    """ROUTE's _lane_ik unchanged: joints with tool0 at target, gripper orientation of q_seed kept."""
    x = np.array(q_seed, float).copy(); s = x[1] + x[2] + x[3]; w = x[5] - x[0]
    for _ in range(60):
        x[3] = s - x[1] - x[2]; x[5] = w + x[0]
        e = target - fk_points(x, base)[5]
        if np.linalg.norm(e) < 2e-4:
            return x
        J = np.zeros((3, 3))
        for k, j in enumerate((0, 1, 2)):
            y = x.copy(); y[j] += 1e-5; y[3] = s - y[1] - y[2]; y[5] = w + y[0]
            J[:, k] = (fk_points(y, base)[5] - fk_points(x, base)[5]) / 1e-5
        x[:3] += np.clip(np.linalg.lstsq(J, e, rcond=None)[0], -0.15, 0.15)
    return None


def _w_clear(a, b, regions, base, fk_points, arm_box_distance):
    """ROUTE's route_clearance: smallest arm-region distance on the straight joint path a -> b (0.02 rad steps)."""
    n = max(2, int(np.max(np.abs(b - a)) / 0.02) + 1)
    best = np.inf
    for k in range(n + 1):
        pts = fk_points(a + (b - a) * k / n, base)
        for box in regions:
            best = min(best, arm_box_distance(pts, box))
    return best


def ns_lane_worker(conn):
    import sys, os
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from strategy_a_global import arm_box_distance, fk_points
    while True:
        try:
            req = conn.recv()
        except (EOFError, KeyboardInterrupt):
            return
        if req is None:
            return
        try:
            base, s, g, regions = req['base'], np.asarray(req['s'], float), np.asarray(req['g'], float), req['regions']
            path = np.asarray(req['path'], float) if req.get('path') is not None else None
            if path is None:                                            # rest: the straight joint path s -> g
                n = max(2, int(np.max(np.abs(g - s)) / 0.02) + 1)
                path = np.array([s + (g - s) * k / n for k in range(n + 1)])
            step = max(1, len(path) // 60)
            cl = [min(arm_box_distance(fk_points(qq, base), b) for b in regions) for qq in path[::step]]
            out = dict(id=req['id'], blocked=bool(min(cl) < req['need']), min_cl=float(min(cl)), lanes=[])
            if out['blocked']:
                qb = path[min(int(np.argmin(cl)) * step, len(path) - 1)]
                p = fk_points(qb, base)[5]
                r = np.array([p[0], p[1], 0.0]); r = r / (np.linalg.norm(r) + 1e-9)
                for name, tgt in {'inside': p - req['lane_off'] * r, 'outside': p + req['lane_off'] * r,
                                  'over': p + np.array([0, 0, req['lane_off']])}.items():
                    via = _w_ik(base, qb, tgt, fk_points)
                    if via is None or np.any(np.abs(via) > LIM):
                        continue
                    c = min(_w_clear(s, via, regions, base, fk_points, arm_box_distance),
                            _w_clear(via, g, regions, base, fk_points, arm_box_distance))
                    if c >= req['need']:
                        out['lanes'].append((float(np.max(np.abs(via - s)) + np.max(np.abs(g - via))), name, via.tolist(), float(c)))
                out['lanes'].sort()
            conn.send(out)
        except Exception as e:                                          # noqa: BLE001
            conn.send(dict(id=req.get('id'), error=repr(e), lanes=[]))


def _corners_to_pos(corners):
    pos = [np.asarray(corners[0], float)]
    for u, v in zip(corners[:-1], corners[1:]):
        u, v = np.asarray(u, float), np.asarray(v, float)
        m = max(1, int(np.max(np.abs(v - u)) / 0.02) + 1)
        pos += [u + (v - u) * k / m for k in range(1, m + 1)]
    return np.array(pos)


class SC3NoStop(SC3DItems):

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.ns_defer = float(p('sc3v2_ns_defer', 0.6).value)     # v2.11: 0.6 s (v2.8: 1.2; hands appear ~1.0 s ahead live)
        self.ns_max_defer = float(p('sc3v2_ns_max_defer', 3.0).value)
        self.ns_cycle_soft = float(p('sc3v2_ns_cycle_soft', 0.0).value)
        self.ns_lanes = bool(p('sc3v2_ns_lanes', True).value)    # v2.8 default ON (was off)
        self.ns_lane_off = float(p('sc3v2_ns_lane_off', 0.15).value)
        self.ns_lane_extra = float(p('sc3v2_ns_lane_extra', 0.005).value)
        # v2.30 route bank (default OFF)
        self.route_bank = bool(p('sc3v2_route_bank', False).value)
        self.estop_cancel = bool(p('sc3v2_estop_cancel', False).value)         # v2.30 F1, default OFF
        self.bank_period = float(p('sc3v2_bank_period', 0.5).value)            # s between refreshes (v2.32: 0.5)
        self.bank_kick_lead = float(p('sc3v2_bank_kick_lead', 0.3).value)      # v2.32: s, splice ahead at the rebuild
        self.bank_lead = float(p('sc3v2_bank_lead', 0.6).value)                # s, splice point ahead of now
        self.bank_max_age = float(p('sc3v2_bank_max_age', 1.5).value)          # s, older bank not used
        self.bank_min_ahead = float(p('sc3v2_bank_min_ahead', 0.15).value)     # s, splice point must still be ahead
        self._v230_bank, self._v230_busy, self._v230_err = None, False, False
        if self.route_bank:
            threading.Thread(target=self._v230_loop, daemon=True).start()
            self.event(f'SC3 v2.30: route bank ON (refresh {self.bank_period:.1f} s, splice {self.bank_lead:.1f} s ahead, '
                       f'used if <= {self.bank_max_age:.1f} s old)')
        self.ns_worker_timeout = float(p('sc3v2_ns_worker_timeout', 0.15).value)
        self._ns_defer = {}                                  # ver -> first deferral time
        self._ns_rid = 0
        self._ns_conn = None
        self.ns_counts = dict(defer=0, last_resort=0, lane_rest=0, lane_moving=0, worker_late=0)
        if self.ns_defer > 0:
            self.d2 = True                                   # the D2 v2 cycle machinery is used for the detour cycle
            self.d2_soft = self.ns_cycle_soft
        if self.ns_lanes:
            ctx = mp.get_context('spawn')
            a, b = ctx.Pipe()
            self._ns_proc = ctx.Process(target=ns_lane_worker, args=(b,), daemon=True)
            self._ns_proc.start()
            self._ns_conn = a
        self._v237_init()
        self._v238_init()
        if self.ns_defer > 0 or self.ns_lanes:
            self.event(f'SC3-NS on: defer {self.ns_defer:.2f} s (max {self.ns_max_defer:.1f} s, cycle soft '
                       f'{100 * self.ns_cycle_soft:.1f} cm), lanes {int(self.ns_lanes)} (offset {100 * self.ns_lane_off:.0f} cm, '
                       f'worker pid {getattr(self, "_ns_proc", None) and self._ns_proc.pid})')

    # ------------------------------------------------------------------ v2.38: HUMAN MODE
    def _v238_init(self):
        if '_v238_ready' in self.__dict__:
            return
        self.__dict__['_v238_ready'] = True
        p = self.declare_parameter
        self.human_mode = bool(p('sc3v2_human_mode', False).value)
        self.h_horizon = float(p('sc3v2_h_horizon', 0.8).value)
        self.h_vmin = float(p('sc3v2_h_vmin', 0.25).value)
        self.h_restart_margin = float(p('sc3v2_h_restart_margin', 0.03).value)
        self.h_moveit_after = float(p('sc3v2_h_moveit_after', 1.5).value)
        self.h_rest_budget = float(p('sc3v2_h_rest_budget', 1.5).value)
        self.h_len_ratio = float(p('sc3v2_h_len_ratio', 2.0).value)
        self.h_wrist_extra = float(p('sc3v2_h_wrist_extra', 0.5).value)
        self.h_relax = float(p('sc3v2_h_relax', 6.0).value)
        self.h_hold = float(p('sc3v2_h_hold', 2.0).value)
        self._v238_until = 0.0
        self._v238_pcache = {}
        self._v238_blocked = None
        self._v238_budgets = (getattr(self, 'budget_rest', 6.0), getattr(self, 'plan_budget_rest', 6.0))
        self.h_counts = dict(plain=0, fast_direct=0, fast_bank=0, quality_reject=0, moveit_skipped=0)
        if self.human_mode:
            self.event(f'SC3 v2.38 HUMAN MODE: people/moving -> immediate stop; growth {self.h_horizon:.1f} s at the current '
                       f'speed (min {self.h_vmin:.2f} m/s); fast restart direct-first (+{100 * self.h_restart_margin:.0f} cm); '
                       f'MoveIt at rest after {self.h_moveit_after:.1f} s (budget {self.h_rest_budget:.1f} s); quality x'
                       f'{self.h_len_ratio:.1f} + 1 rad, wrist +{self.h_wrist_extra:.1f} rad (relaxed after {self.h_relax:.0f} s)')

    def _v238_is_person(self, oid, o):
        k = (oid, o.get('t'))
        r = self._v238_pcache.get(k)
        if r is None:
            if len(self._v238_pcache) > 500:
                self._v238_pcache.clear()
            r = self._v238_pcache[k] = bool(self._v229_person(o))
        return r

    def _v238_humans(self):
        """True while a person-like or moving object is in the scene (held h_hold s after the last one)."""
        now = time.monotonic()
        try:
            for oid, o in list(self.objs.items()):
                st = self.motion.get(oid)
                if st is None or not st.get('still') or self._v238_is_person(oid, o):
                    self._v238_until = now + self.h_hold
                    break
        except Exception:
            self._v238_until = now + self.h_hold                        # unsure -> people rules (the safe side)
        return now < self._v238_until

    def reach_at(self, t_abs, pad=0.0):
        if not self.__dict__.get('human_mode') or self.still_obj_speed <= 0:
            return super().reach_at(t_abs, pad)
        out = []
        for oid, o in list(self.objs.items()):
            st = self.motion.get(oid)
            opad = pad + (self.person_clear if self.person_clear > 0 and self._v238_is_person(oid, o) else 0.0)
            if st is not None and st.get('still'):
                g = np.array([self.lat_const, self.lat_const, self.z_grow])
                for cl, Rl, h in o['parts']:
                    R = o['R'] @ Rl
                    out.append((o['p'] + o['R'] @ cl, R, h + np.abs(R.T) @ g + opad))
                continue
            dt = min(max(t_abs - o['t'], 0.0) + self.latency, self.h_horizon)          # H2: reaction + stop time only
            vm = max(float(st.get('peak') or 0.0) if st else 0.0,
                     float(st['now']) if st and st.get('now') is not None else 0.0)
            if self._v243('cred') and vm < self.still_obj_speed:
                # v2.43 Fix B (24 Nov night, diagnosis: 41 % of the planning-start rejections = a person measured at
                # 0.00 m/s grown at the 0.25 m/s floor because it was not yet 'still'): no credible motion measured
                # (current and 2 s peak < the still threshold) -> own size + measurement uncertainty only (sideways
                # lat_rate x dt, height z_grow, people + person_clear); the full growth returns with the first credible speed
                lat = self.lat_const + self.lat_rate * dt
                g = np.array([lat, lat, self.z_grow])
                for cl, Rl, h in o['parts']:
                    R = o['R'] @ Rl
                    out.append((o['p'] + o['R'] @ cl, R, h + np.abs(R.T) @ g + opad))
                continue
            v = max(vm, self.h_vmin)                                                        # v2.39: peak too
            along = self.reach_factor * v * dt
            lat = self.lat_const + self.lat_rate * dt
            if st is None or st.get('u') is None:
                g = np.array([along + lat, along + lat, self.z_grow])
            else:
                g = np.array([abs(st['u'][0]) * along + lat, abs(st['u'][1]) * along + lat, self.z_grow])
            for cl, Rl, h in o['parts']:
                R = o['R'] @ Rl
                out.append((o['p'] + o['R'] @ cl, R, h + np.abs(R.T) @ g + opad))
        return out

    def rank(self, cands, moving, t_s):
        if not self.__dict__.get('human_mode') or not cands or self.goal is None:
            return super().rank(cands, moving, t_s)
        tb = self._v238_blocked
        if tb is not None and time.monotonic() - tb > self.h_relax:
            return super().rank(cands, moving, t_s)
        g = np.asarray(self.goal, float)
        ok = []
        for c in cands:
            try:
                seq = np.asarray(c['seq'], float)
                ld = float(np.abs(g - seq[0]).sum())
                w = float(np.abs(np.diff(seq[:, 5])).sum()) if len(seq) > 1 else 0.0
                wd = abs(float(g[5] - seq[0][5]))
                if c['L'] > self.h_len_ratio * ld + 1.0 or w > wd + self.h_wrist_extra:
                    self.h_counts['quality_reject'] += 1
                    self.event(f'H5_QUALITY_REJECT {c.get("kind")} L={c["L"]:.2f} rad (direct {ld:.2f}), wrist_3 '
                               f'{w:.2f} rad (direct {wd:.2f})')
                    continue
            except Exception:
                pass
            ok.append(c)
        return super().rank(ok, moving, t_s) if ok else None

    def _v240_hold_near(self):
        if '_v240_hn' not in self.__dict__:
            self._v240_hn = bool(self.declare_parameter('sc3v2_h_hold_near', False).value)
        return self._v240_hn

    def contact_recovery(self, q, ver):
        if (self.__dict__.get('human_mode') and self._v240_hold_near() and self._v238_humans()):   # v2.39 H6, v2.40 off
            self.hold('H6 person near: no recovery move, holding until the path clears')
            return
        return super().contact_recovery(q, ver)

    # ------------------------------------------------------------------ v2.42: predictive speed MPC
    def _v243(self, k):
        """v2.43 switches (24 Nov night, test copy): 'zero' = sc3v2_h_occ_zero (Fix A), 'cred' = sc3v2_h_cred (Fix B)."""
        if '_v243_sw' not in self.__dict__:
            self._v243_sw = dict(zero=bool(self.declare_parameter('sc3v2_h_occ_zero', False).value),
                                 cred=bool(self.declare_parameter('sc3v2_h_cred', False).value))
            self.event(f'SC3 v2.43 test: Fix A horizon-0 planning region {"ON" if self._v243_sw["zero"] else "off"}, '
                       f'Fix B credible-motion prediction {"ON" if self._v243_sw["cred"] else "off"}')
        return self._v243_sw[k]

    def _v242_init(self):
        if '_v242_on' in self.__dict__:
            return
        p = self.declare_parameter
        self._v242_on = bool(p('sc3v2_h_mpc', False).value)
        self.mpc_h = float(p('sc3v2_mpc_horizon', 1.5).value)
        self.mpc_tr = float(p('sc3v2_mpc_tr', 0.20).value)
        self.mpc_ts = float(p('sc3v2_mpc_ts', 0.30).value)
        self.mpc_z = float(p('sc3v2_mpc_z', 0.01).value)
        self.mpc_w1, self.mpc_w2 = 1.0, 0.3
        self.mpc_none = str(p('sc3v2_mpc_none', 'h7').value)           # v2.42c: 'h7' = v2.42b (no safe speed -> the
        # 0.6 s rule decides: replan / stop); 'stop' = user 24 Nov spec (speed set incl. 0: no safe moving speed -> STOP)
        self._v242_lvl, self._v242_last, self._v242_n = {}, 0.0, 0
        self.mpc_counts = dict(slow=0, faster=0, infeasible=0, evals=0)
        if self._v242_on:
            self.event(f'SC3 v2.43 SPEED MPC on (no safe speed -> {self.mpc_none}): levels 1/0.7/0.5/0.3/0.15, horizon {self.mpc_h:.1f} s, Tr {self.mpc_tr:.2f} s, '
                       f'Ts {self.mpc_ts:.2f} s, Z {100 * self.mpc_z:.0f} cm, w1 {self.mpc_w1}, w2 {self.mpc_w2}')

    def _v242_mpc(self, cur, ver, now):
        """True if a re-timed (slower / faster) copy of the current path was sent this cycle."""
        self._v242_init()
        if not self._v242_on or not cur.get('full') or cur.get('kind') in STOP_KINDS:
            return False
        self._v242_n += 1
        if self._v242_n % 2:
            return False
        try:
            from strategy_a_global import fk_points
            tim, pos = np.asarray(cur['tim'], float), np.asarray(cur['pos'], float)
            tau = now - cur['t0']
            if tau >= tim[-1] - 0.2:
                return False
            cur_lvl = self._v242_lvl.get(ver, 1.0)
            levels = (1.0, 0.7, 0.5, 0.3, 0.15)
            ts = np.arange(0.0, self.mpc_h + 1e-9, 0.15)
            Q, T, idx = [], [], []
            for li, L in enumerate(levels):
                tt = np.minimum(tau + (L / cur_lvl) * ts, tim[-1])
                for t_rel, tp in zip(ts, tt):
                    Q.append([np.interp(tp, tim, pos[:, jj]) for jj in range(6)])
                    T.append(now + t_rel + self.t_slack)
                    idx.append(li)
            Q, idx = np.asarray(Q), np.asarray(idx)
            d = np.asarray(self.reach_dist(Q, T), float)
            self.mpc_counts['evals'] += 1
            tool = np.array([fk_points(q, self.base)[-1] for q in Q])
            best, best_cost, margins = None, None, []
            for li, L in enumerate(levels):
                m = idx == li
                tp = tool[m]
                vt = float(np.max(np.linalg.norm(np.diff(tp, axis=0), axis=1)) / 0.15) if len(tp) > 1 else 0.0
                req = self.mpc_z + vt * (self.mpc_tr + self.mpc_ts)
                margin = float(np.min(d[m] - req))
                margins.append(margin)
                if margin < 0:
                    continue
                if L > cur_lvl + 1e-6 and (L > cur_lvl + 0.3 + 1e-6 or now - self._v242_last < 0.5):
                    continue
                cost = self.mpc_w1 * (1 - L) ** 2 + self.mpc_w2 * (L - cur_lvl) ** 2
                if best_cost is None or cost < best_cost:
                    best, best_cost = L, cost
            if best is None:
                self.mpc_counts['infeasible'] += 1
                if self.mpc_none == 'stop':          # v2.42c (user spec): the fastest safe speed of {1 .. 0.15, 0} is 0
                    self.event(f'MPC_STOP v{ver}: no moving speed keeps the separation (margins '
                               f'{", ".join("%.0f" % (100 * x) for x in margins)} cm for 1/0.7/0.5/0.3/0.15) -> stop')
                    self._v242c_ret = self._v227_stop_now(ver, 'MPC: no safe moving speed')
                    return True
                return False                         # v2.42b: no speed is safe -> the 0.6 s rule decides (slowing to 15 %
                                                     # parked the arm in the sweep: 5/5 standing contacts)
            if abs(best - cur_lvl) < 1e-6:
                return False
            if best > cur_lvl and now - self._v242_last < 0.5:
                return False
            kind = 'MPC_SLOW' if best < cur_lvl else 'MPC_FAST'
            t0 = time.monotonic()
            if self._v237_send_scaled(cur, ver, now, best, kind):
                nv = self.ver
                self._v242_lvl[nv] = best
                if len(self._v242_lvl) > 200:
                    for kk in list(self._v242_lvl)[:100]:
                        self._v242_lvl.pop(kk, None)
                self._v242_last = time.monotonic()
                self.mpc_counts['slow' if best < cur_lvl else 'faster'] += 1
                self.event(f'{kind} v{ver}->v{nv}: speed {cur_lvl:.2f} -> {best:.2f} (margins '
                           f'{", ".join("%.0f" % (100 * x) for x in margins)} cm for 1/0.7/0.5/0.3/0.15), '
                           f'sent in {1000 * (time.monotonic() - t0):.0f} ms')
                return True
        except Exception as e:
            self.event(f'MPC error {e!r}')
        return False

    def _v241_bank_first(self):
        if '_v241_bf' not in self.__dict__:
            self._v241_bf = bool(self.declare_parameter('sc3v2_h_bank_first', False).value)
        return self._v241_bf

    def _v240_monitor(self, cur, ver, q, now):
        """H7: people present - replan while moving if the conflict is >= h_replan_min ahead, else stop at once."""
        if '_v240_kicked' not in self.__dict__:
            self.h_replan_min = float(self.declare_parameter('sc3v2_h_replan_min', 1.0).value)
            self._v240_kicked = None
            self.h_counts.update(replan=0, stop=0)
        if cur.get('kind') == 'RECOVER':        # v2.17 as in the NS layer: the move AWAY (distance checked to grow
            return 'recovering'                 # when it was built) is not stopped half-way
        if self.goal is None or cur.get('kind') in STOP_KINDS:
            self.h_counts['plain'] += 1
            return StrategyBDetourA2SG1SC3.monitor_check(self, cur, ver, q, now)
        if self.clearance(q, now) <= self.contact_dist:
            self.h_counts['stop'] += 1
            self.event(f'H7_STOP v{ver}: person at the arm now')
            return self._v227_stop_now(ver, 'person at the arm now')
        tau = now - cur['t0']
        k = min(int(np.searchsorted(cur['tim'], tau)), len(cur['pos']) - 1)
        j = self.first_conflict(cur['pos'], cur['tim'], now, k, self.monitor_horizon)
        if j is None:
            if self._v242_mpc(cur, ver, now) and self.__dict__.get('_v242c_ret'):   # v2.42c: MPC stopped
                r, self._v242c_ret = self._v242c_ret, None
                return r
            return 'clear'
        ahead = float(cur['tim'][j] - cur['tim'][k])
        if ahead >= self.h_replan_min:
            if self._v242_mpc(cur, ver, now):                                   # v2.42: slowed -> next cycle decides
                if self.__dict__.get('_v242c_ret'):                              # v2.42c: MPC stopped
                    r, self._v242c_ret = self._v242c_ret, None
                    return r
                return 'mpc'
            if self._v240_kicked != ver:
                self._v240_kicked = ver
                self.h_counts['replan'] += 1
                self.event(f'H7_REPLAN v{ver} {cur["kind"]}: person conflict {ahead:.2f} s ahead -> keep moving, replan now')
                if getattr(self, 'route_bank', False) and self._v241_bank_first():
                    self._v230_bank = None                                     # fresh routes only
                    self._v230_kick(cur, ver)
            fv = self.__dict__.get('_v227_fail')                               # v2.41: the RRT cycle ended, no path
            if (not self._v241_bank_first() and getattr(self, 'route_bank', False) and fv is not None and fv[0] == ver
                    and self.__dict__.get('_v241_banked') != ver):
                self._v241_banked = ver
                self.event(f'H8_BANK_FALLBACK v{ver}: RRT-Connect replanning found no path -> ready-made routes')
                self._v230_bank = None
                self._v230_kick(cur, ver)
            self._d2_ver = ver
            self.threat_evt.set()
            return 'replanning'
        self.h_counts['stop'] += 1
        self.event(f'H7_STOP v{ver} {cur["kind"]}: person conflict {ahead:.2f} s ahead (< {self.h_replan_min:.2f} s) -> stop now')
        return self._v227_stop_now(ver, f'person conflict {ahead:.2f} s ahead')

    def _v238_rest_fast(self, cur, ver, now, bank=True):
        """H3: at rest - the direct path first, then the bank routes, with h_restart_margin extra clearance."""
        g = np.asarray(self.goal, float)
        q0 = np.asarray(cur['pos'][0], float)
        end_dist = float(np.abs(q0 - g).max())
        if end_dist < 1e-3:
            return None
        t0 = time.monotonic()
        routes = [('direct', self._v230_dense([q0, g]))]
        cands = []
        self._d_local.req = self.__dict__['_clear_req_base'] + self.h_restart_margin
        try:
            cand, _ = self.build(cur, 0, routes[0][1], None, 'DIRECT_FAST', end_dist, time.monotonic(), search_rest=False)
            if cand is not None and cand['full']:
                cand['route'] = 'direct'
                cands.append(cand)
            elif bank and getattr(self, 'route_bank', False):
                for name, seq in self._v230_routes(q0, g):
                    if self.ver != ver:
                        return None
                    c2, _ = self.build(cur, 0, seq, None, 'BANK', end_dist, time.monotonic(), search_rest=False)
                    if c2 is not None and c2['full']:
                        c2['route'] = name
                        cands.append(c2)
        finally:
            self._d_local.req = None
        if not cands:
            return None
        pick = self.rank(cands, False, now)
        if pick is None:
            return None
        ms = 1000 * (time.monotonic() - t0)
        if self.activate(cur, 0, pick, ver, dict(h='rest_fast')):
            tb = self._v238_blocked
            self._v238_blocked, self.t_block = None, None
            key = 'fast_direct' if pick['route'] == 'direct' else 'fast_bank'
            self.h_counts[key] += 1
            self.event(f'H3_RESTART {pick["route"]} at rest: arrival in {pick["arrival"] - time.monotonic():.2f} s, '
                       f'built + checked in {ms:.0f} ms, blocked {0.0 if tb is None else time.monotonic() - tb:.2f} s')
            return f'ACTIVATED {pick["kind"]} (H3 fast restart)'
        return None

    # ------------------------------------------------------------------ v2.37: D2 reachable + D3 slow while deciding
    def _v237_init(self):
        if '_v237_ready' in self.__dict__:
            return
        self.__dict__['_v237_ready'] = True
        gp = self.get_parameter
        self.d2_early_on = bool(gp('sc3v2_d2_early').value)
        self.d2_band = float(gp('sc3v2_d2_soft').value)
        p = self.declare_parameter
        self.d3_slow = bool(p('sc3v2_d3_slow', False).value)
        self.d3_factor = float(p('sc3v2_d3_factor', 0.5).value)
        self.d3_resume = float(p('sc3v2_d3_resume', 0.3).value)
        self._d3_req, self._d3_slow_ver, self._d3_clear_since = None, None, None
        self.d3_counts = dict(slow=0, resume=0, slow_failed=0, early=0)
        if self.d2_early_on or self.d3_slow:
            self.event(f'SC3 v2.37: d2_early {int(self.d2_early_on)} (band {100 * self.d2_band:.1f} cm), d3_slow '
                       f'{int(self.d3_slow)} (x{self.d3_factor:.2f} vmax, resume after {self.d3_resume:.2f} s clear)')

    def _v237_clear(self, cur, ver, k, now):
        """monitor thread, path clear (hard check): D2 band check; D3 resume bookkeeping."""
        soft = False
        if self.d2_early_on and self.d2_band > 0:
            try:
                self._d_local.req = self.__dict__['_clear_req_base'] + self.d2_band
                try:
                    js = self.first_conflict(cur['pos'], cur['tim'], now, k, self.monitor_horizon)
                finally:
                    self._d_local.req = None
                soft = js is not None
                if soft and self._d2_ver != ver:
                    self._d2_ver = ver
                    self.d3_counts['early'] += 1
                    self.event(f'D2_EARLY soft conflict {float(cur["tim"][js] - cur["tim"][k]):.2f} s ahead on {cur["kind"]} '
                               f'(v{ver}, band {100 * self.d2_band:.1f} cm): planner woken')
                    if getattr(self, 'route_bank', False):
                        self._v230_kick(cur, ver)
                    if self.d3_slow and cur.get('kind') != 'D3_SLOW':
                        self._d3_req = ver
                    self.threat_evt.set()
            except Exception as e:                                           # the early warning never blocks a stop
                self.event(f'D2 check failed: {e!r}')
        if self.d3_slow and cur.get('kind') == 'D3_SLOW' and ver == self._d3_slow_ver:
            if soft:
                self._d3_clear_since = None
            elif self._d3_clear_since is None:
                self._d3_clear_since = now
            elif now - self._d3_clear_since >= self.d3_resume:
                self.threat_evt.set()                                        # the planner thread re-sends at full speed

    def _v237_retime(self, pos, v0, vcap):
        """strategy_b_local.retime with a speed cap vcap and a deceleration ramp (<= amax) from the speed of v0."""
        pos = np.asarray(pos, float)
        keep = np.r_[True, np.abs(np.diff(pos, axis=0)).max(1) > 1e-6]
        pos = pos[keep]
        if len(pos) < 2:
            return pos, np.zeros(len(pos)), np.zeros_like(pos)
        d = np.diff(pos, axis=0)
        ds = np.abs(d).max(1)
        u = d / ds[:, None]
        sv0 = float(np.clip((v0 @ d[0]) * ds[0] / (d[0] @ d[0]), 0.0, self.vmax))
        s_at = np.r_[0.0, np.cumsum(ds)]
        cap = np.maximum(vcap, np.sqrt(np.maximum(sv0 ** 2 - 2 * self.amax * s_at, 0.0)))
        v = np.minimum(cap, self.vmax)
        v[0], v[-1] = sv0, 0.0
        for k in range(len(ds)):
            v[k + 1] = min(v[k + 1], np.sqrt(v[k] ** 2 + 2 * self.amax * ds[k]))
        for k in range(len(ds) - 1, 0, -1):
            v[k] = min(v[k], np.sqrt(v[k + 1] ** 2 + 2 * self.amax * ds[k]))
        dt = 2 * ds / np.maximum(v[:-1] + v[1:], 1e-6)
        rr = (v[:-1] < 1e-6) & (v[1:] < 1e-6)                                # rest-to-rest (as the v2.2 retime fix)
        dt[rr] = np.maximum(np.sqrt(6 * ds[rr] / self.amax), 1.5 * ds[rr] / max(vcap, 1e-3))
        tim = np.r_[0.0, np.cumsum(dt)]
        vel = np.zeros_like(pos)
        vel[0] = v[0] * u[0]
        for k in range(1, len(pos) - 1):
            vel[k] = v[k] * (u[k - 1] + u[k]) / 2
        return pos, tim, vel

    def _v237_send_scaled(self, cur, ver, now, factor, kind):
        tau = now - cur['t0']
        tim = cur['tim']
        i_s = int(np.searchsorted(tim, tau + 0.10))
        if i_s >= len(tim) - 1:
            return False
        t_s = float(tim[i_s] - tau)
        keep = np.where((tim - tau > 0.02) & (np.arange(len(tim)) < i_s))[0]
        npos, ntim, nvel = self._v237_retime(cur['pos'][i_s:], cur['vel'][i_s], factor * self.vmax)
        pos = np.vstack([cur['pos'][keep], npos]) if len(keep) else npos
        tm = np.r_[tim[keep] - tau, ntim + t_s]
        vel = np.vstack([cur['vel'][keep], nvel]) if len(keep) else nvel
        return self.send(pos, tm, vel, pos.copy(), True, kind, ver=ver)

    def extend(self):
        if self.__dict__.get('d3_slow'):
            try:
                self._v237_d3()
            except Exception as e:
                self.event(f'D3 error {e!r}')
        return super().extend()

    def _v237_d3(self):
        now = time.monotonic()
        with self.slock:
            cur, ver = self.cur, self.ver
        if (cur is None or self.goal is None or not cur.get('full') or cur.get('kind') in STOP_KINDS
                or cur.get('kind') == 'RECOVER' or self.finished(cur, now)):
            self._d3_req = None
            return
        if self._d3_req == ver and cur.get('kind') != 'D3_SLOW':
            self._d3_req = None
            t0 = time.monotonic()
            if self._v237_send_scaled(cur, ver, now, self.d3_factor, 'D3_SLOW'):
                nv = self.ver
                if ver in self._ns_defer:
                    self._ns_defer[nv] = self._ns_defer.pop(ver)
                if self._d2_ver == ver:
                    self._d2_ver = nv
                self._d3_slow_ver, self._d3_clear_since = nv, None
                self.d3_counts['slow'] += 1
                self.event(f'D3_SLOW v{ver}->v{nv}: same path at x{self.d3_factor:.2f} vmax while the detour is decided '
                           f'(sent in {1000 * (time.monotonic() - t0):.0f} ms)')
            else:
                self.d3_counts['slow_failed'] += 1
                self.event(f'D3_SLOW_FAILED v{ver}: not sent')
            return
        cs = self._d3_clear_since
        if (cur.get('kind') == 'D3_SLOW' and ver == self._d3_slow_ver and cs is not None
                and now - cs >= self.d3_resume and ver not in self._ns_defer):
            if self._v237_send_scaled(cur, ver, now, 1.0, 'D3_RESUME'):
                self._d3_slow_ver, self._d3_clear_since = None, None
                if self._d2_ver == ver:
                    self._d2_ver = None
                self.d3_counts['resume'] += 1
                self.event(f'D3_RESUME v{ver}->v{self.ver}: path clear for {now - cs:.2f} s -> full speed again')

    # ------------------------------------------------------------------ NS1: defer the cut, detour first
    def monitor_check(self, cur, ver, q, now):
        self._v237_init()
        self._v238_init()
        if self.__dict__.get('human_mode') and self._v238_humans():                # v2.40 H7 (v2.38 H1)
            return self._v240_monitor(cur, ver, q, now)
        if self.ns_defer <= 0 or self.goal is None or cur.get('kind') in STOP_KINDS:
            return super().monitor_check(cur, ver, q, now)
        if cur.get('kind') == 'RECOVER':      # v2.17: a recovery is the move AWAY from contact (distance checked to grow
            return 'recovering'               # along it when it was built): no contact e-stop in the middle of it
        if self.clearance(q, now) <= self.contact_dist:      # contact imminent: SC3's own emergency, never deferred
            self.ns_counts['last_resort'] += 1
            self.event(f'NS_LAST_RESORT v{ver}: contact check (box at the arm now)')
            return self._v227_stop_now(ver, 'box at the arm now')
        tau = now - cur['t0']
        k = min(int(np.searchsorted(cur['tim'], tau)), len(cur['pos']) - 1)
        j = self.first_conflict(cur['pos'], cur['tim'], now, k, self.monitor_horizon)
        if j is None:
            self._ns_defer.pop(ver, None)
            self._v237_clear(cur, ver, k, now)                               # v2.37 (switches off: nothing)
            return 'clear'          # v2.10: decided here; never handed to SC3's own check (it re-checked a moment later,
            #                         saw the hand that had just arrived and CUT instead of replanning: lab 12:00 leg 2)
        ahead = float(cur['tim'][j] - cur['tim'][k])
        t_first = self._ns_defer.get(ver)
        fv = self.__dict__.get('_v227_fail')                                 # a search that ended after this conflict
        failed = fv is not None and fv[0] == ver and t_first is not None and fv[1] > t_first   # was first seen
        if ahead >= self.ns_defer and not failed and (t_first is None or now - t_first < self.ns_max_defer):
            if t_first is None:
                self._ns_defer[ver] = now
                self.ns_counts['defer'] += 1
                self.event(f'NS_DEFER v{ver} {cur["kind"]}: conflict {ahead:.2f} s ahead (>= {self.ns_defer:.2f}) -> no cut, '
                           f'detour cycle woken')
                if getattr(self, 'route_bank', False):
                    self._v230_kick(cur, ver)
                if self.d3_slow and cur.get('kind') != 'D3_SLOW':
                    self._d3_req = ver                                       # v2.37 D3: slow down while deciding
            self._d2_ver = ver
            self.threat_evt.set()
            return 'deferred'
        why = ('the detour search ended without a new path' if failed and ahead >= self.ns_defer
               else f'conflict {ahead:.2f} s ahead < {self.ns_defer:.2f} s' if ahead < self.ns_defer
               else f'no detour within {self.ns_max_defer:.1f} s')
        self.ns_counts['last_resort'] += 1
        self.event(f'NS_LAST_RESORT v{ver} {cur["kind"]}: {why} -> emergency stop now')
        self._ns_defer.pop(ver, None)
        return self._v227_stop_now(ver, why)

    def _v227_stop_now(self, ver, why):
        """v2.27 (B1 v2.26 touched the user's hand: the last resort went to SC3's cut rules, which used a cut built 1.3 s
        earlier, rejected it and stopped 0.8 s after the decision): the stop is sent at once from the fresh state."""
        self.hold_here(f'NS last resort: {why}', ver=ver)
        return 'emergency'

    # ------------------------------------------------------------------ v2.34: route bank at rest
    def _extend(self, cur, ver, i_s, q, moving, now):
        if self.__dict__.get('human_mode') and self.goal is not None and self.base is not None:      # v2.38 H3 / H4
            if moving:
                self._v238_blocked = None
            else:
                if self._v238_blocked is None:
                    self._v238_blocked = time.monotonic()
                bank_first = self._v241_bank_first()
                try:
                    r = self._v238_rest_fast(cur, ver, now, bank=bank_first)     # v2.41: straight path first
                    if r:
                        return r
                except Exception as e:
                    self.event(f'H3 error {e!r}')
                humans = self._v238_humans()
                if not bank_first:                                               # v2.41: RRT-Connect, then the bank
                    br, pbr = self._v238_budgets
                    self.budget_rest = min(br, self.h_rest_budget) if humans else br
                    self.plan_budget_rest = min(pbr, self.h_rest_budget) if humans else pbr
                    r = super()._extend(cur, ver, i_s, q, moving, now)
                    if isinstance(r, str) and r.startswith('ACTIVATED'):
                        self.h_counts['rrt_rest'] = self.h_counts.get('rrt_rest', 0) + 1
                        self.event(f'H8_REPLAN_RRT at rest: {r}')
                        return r
                    if self.ver != ver or self.goal is None:
                        return r
                    try:
                        r2 = self._v238_rest_fast(cur, ver, time.monotonic(), bank=True)
                        if r2:
                            self.event('H8_BANK_FALLBACK at rest: RRT-Connect found no path')
                            return r2
                    except Exception as e:
                        self.event(f'H3 error {e!r}')
                    return r
                if humans and time.monotonic() - self._v238_blocked < self.h_moveit_after:
                    self.h_counts['moveit_skipped'] += 1
                    return 'WAIT (v2.38: fast restart only, people present)'
                br, pbr = self._v238_budgets
                self.budget_rest = min(br, self.h_rest_budget) if humans else br
                self.plan_budget_rest = min(pbr, self.h_rest_budget) if humans else pbr
                return super()._extend(cur, ver, i_s, q, moving, now)          # MoveIt cycle (bank already tried)
        if getattr(self, 'route_bank', False) and not moving and self.goal is not None and self.base is not None:
            try:
                if self._v234_rest_bank(cur, ver, now):
                    return 'ACTIVATED BANK (at rest)'
            except Exception as e:
                self.event(f'BANK_REST error {e!r}')
        return super()._extend(cur, ver, i_s, q, moving, now)

    def _v234_rest_bank(self, cur, ver, now):
        g = np.asarray(self.goal, float)
        q0 = np.asarray(cur['pos'][0], float)
        end_dist = float(np.abs(q0 - g).max())
        t0, cands = time.monotonic(), []
        for name, seq in self._v230_routes(q0, g):
            if self.ver != ver:
                return False
            cand, _ = self.build(cur, 0, seq, None, 'BANK', end_dist, time.monotonic(), search_rest=True)
            if cand is not None:
                cand['route'] = name
                cands.append(cand)
        ms = 1000 * (time.monotonic() - t0)
        if not cands:
            st = self.__dict__.setdefault('_v234_none', [0, 0.0])
            st[0] += 1
            st[1] = max(st[1], ms)
            if st[0] % 10 == 1:
                self.event(f'BANK_REST none usable ({st[0]} times so far, this one {ms:.0f} ms)')
            return False
        fulls = [c for c in cands if c['full']]
        pick = self.rank(fulls, False, now) if fulls else None
        if pick is None:
            pick = min(cands, key=lambda c: float(np.abs(np.asarray(c['end']) - g).max()))
            pick['arrival'] = time.monotonic() + pick['dur']
        if self.activate(cur, 0, pick, ver, dict(bank='rest')):
            self.t_block = None
            self.event(f'BANK_SWITCH {pick["route"]} at rest ({"full" if pick["full"] else "to a resting point"}) of '
                       f'{len(cands)}: arrival in {pick["arrival"] - time.monotonic():.2f} s, built + checked in {ms:.0f} ms')
            return True
        return False

    # ------------------------------------------------------------------ v2.30: ready-made routes (route bank)
    @staticmethod
    def _v230_dense(pts, step=0.05):
        out = [np.asarray(pts[0], float)]
        for a, b in zip(pts[:-1], pts[1:]):
            a, b = np.asarray(a, float), np.asarray(b, float)
            n = max(1, int(np.ceil(np.abs(b - a).max() / step)))
            out += [a + (b - a) * k / n for k in range(1, n + 1)]
        return np.array(out)

    def _v230_routes(self, s, g):
        """v2.32: 9 routes (via-points along the joint path, lifted; two sideways; one over two via-points)."""
        s, g = np.asarray(s, float), np.asarray(g, float)
        out = []                                    # v2.36: no direct route (v2.35 H: direct taken when free, re-blocked
        #                                             soon after -> holds 3 -> 108); the lifted routes stay free
        for fr, dz in ((1 / 3, 0.15), (1 / 2, 0.30), (2 / 3, 0.15)):              # v2.33: 6 routes in total
            v = self._v213_lift(s + (g - s) * fr, dz)
            if v is not None:
                out.append((f'over{int(100 * dz)}@{fr:.2f}', self._v230_dense([s, v, g])))
        for off in (0.5, -0.5):
            m = 0.5 * (s + g)
            m[0] += off
            v = self._v213_lift(m, 0.15)
            if v is not None:
                out.append(('left' if off > 0 else 'right', self._v230_dense([s, v, g])))
        v1, v2 = self._v213_lift(s + (g - s) / 3, 0.25), self._v213_lift(s + (g - s) * 2 / 3, 0.25)
        if v1 is not None and v2 is not None:
            out.append(('over25x2', self._v230_dense([s, v1, v2, g])))
        return out

    def _v232_build(self, cur, ver, lead):
        """all bank routes from the point `lead` s ahead on cur, fully checked; (i_s, clear candidates) or None."""
        now = time.monotonic()
        i_s = min(int(np.searchsorted(cur['tim'], now - cur['t0'] + lead)), len(cur['pos']) - 1)
        if i_s >= len(cur['pos']) - 2 or self.goal is None:
            return None
        g = np.asarray(self.goal, float)
        end_dist = float(np.abs(np.asarray(cur['pos'][-1]) - g).max())
        cands = []
        for name, seq in self._v230_routes(cur['pos'][i_s], g):
            if self.ver != ver:
                return None
            cand, _ = self.build(cur, i_s, seq, None, 'BANK', end_dist, time.monotonic(), search_rest=True)
            if cand is not None:                                            # v2.33: full OR to a resting point
                cand['route'] = name
                cands.append(cand)
        return i_s, cands

    def _v230_loop(self):
        while rclpy.ok():
            time.sleep(self.bank_period)
            try:
                self._v230_refresh()
            except Exception as e:
                if not self._v230_err:
                    self._v230_err = True
                    self.event(f'BANK error {e!r}')

    def _v230_refresh(self):
        if self.__dict__.get('human_mode') and self._v238_humans():               # v2.38 H4
            self._v230_bank = None
            return
        if self.goal is None or self.base is None:
            self._v230_bank = None
            return
        with self.slock:
            cur, ver = self.cur, self.ver
        now = time.monotonic()
        if (cur is None or self.finished(cur, now) or cur.get('kind') in STOP_KINDS or cur.get('kind') == 'RECOVER'
                or not cur.get('full')):
            self._v230_bank = None
            return
        i_s = min(int(np.searchsorted(cur['tim'], now - cur['t0'] + self.bank_lead)), len(cur['pos']) - 1)
        if i_s >= len(cur['pos']) - 2:
            self._v230_bank = None
            return
        g = np.asarray(self.goal, float)
        end_dist = float(np.abs(np.asarray(cur['pos'][-1]) - g).max())
        t0, cands = time.monotonic(), []
        for name, seq in self._v230_routes(cur['pos'][i_s], g):
            if self.ver != ver:
                return
            cand, _ = self.build(cur, i_s, seq, None, 'BANK', end_dist, time.monotonic(), search_rest=True)
            if cand is not None:                                            # v2.33: full OR to a resting point
                cand['route'] = name
                cands.append(cand)
        self._v230_bank = dict(ver=ver, i_s=i_s, cands=cands, t=now, ms=1000 * (time.monotonic() - t0))
        st = self.__dict__.setdefault('_v230_stats', [])
        st.append(self._v230_bank['ms'])
        if len(st) >= 10:                                                  # refresh cost, logged every 10 refreshes
            self.event(f'BANK_REFRESH last 10: {np.median(st):.0f} ms median, {max(st):.0f} ms max; '
                       f'{len(cands)} routes clear now')
            st.clear()

    def _v230_kick(self, cur, ver):
        if self._v230_busy:
            return
        self._v230_busy = True
        threading.Thread(target=self._v230_try, args=(cur, ver), daemon=True).start()

    def _v230_try(self, cur, ver):
        try:
            # v2.33: the background bank if usable, otherwise a rebuild (fresh splice point kick_lead ahead)
            t0b = now = time.monotonic()
            b = self._v230_bank
            src = 'bank'
            if (not b or b['ver'] != ver or now - b['t'] > self.bank_max_age or not b['cands']
                    or cur['t0'] + float(cur['tim'][b['i_s']]) - now < self.bank_min_ahead):
                src = 'rebuild'
                r = self._v232_build(cur, ver, self.bank_kick_lead)
                if r is None or not r[1]:
                    self.event(f'BANK none usable for v{ver} at the conflict (rebuild {1000 * (time.monotonic() - t0b):.0f} ms)')
                    return
                b = dict(ver=ver, i_s=r[0], cands=r[1], t=now, ms=1000 * (time.monotonic() - t0b))
            i_s = b['i_s']
            t_s = cur['t0'] + float(cur['tim'][i_s])
            if t_s - time.monotonic() < self.bank_min_ahead:
                self.event(f'BANK splice point {1000 * (t_s - time.monotonic()):.0f} ms ahead ({src}): too close')
                return
            fulls = [c for c in b['cands'] if c['full']]
            pick = self.rank(fulls, True, t_s) if fulls else None
            if pick is None and b['cands']:
                g = np.asarray(self.goal, float)
                pick = min(b['cands'], key=lambda c: float(np.abs(np.asarray(c['end']) - g).max()))
                pick['arrival'] = time.monotonic() + pick['dur']
            t0 = time.monotonic()
            if pick is not None and self.activate(cur, i_s, pick, ver, dict(bank=True)):
                self.event(f'BANK_SWITCH {pick["route"]} ({"full" if pick["full"] else "to a resting point"}, {src}) of {len(b["cands"])}: '
                           f'arrival in {pick["arrival"] - time.monotonic():.2f} s, '
                           f'decision -> sent {1000 * (time.monotonic() - t0):.0f} ms (bank built {1000 * (now - b["t"]):.0f} ms before, '
                           f'in {b["ms"]:.0f} ms)')
            else:
                self.event('BANK_SWITCH not sent (re-check with the newest box data failed)')
        except Exception as e:
            self.event(f'BANK_SWITCH error {e!r}')
        finally:
            self._v230_busy = False

    # ------------------------------------------------------------------ NS2: lanes, screening in the worker process
    def _ns_ask(self, s, g, path):
        if self._ns_conn is None or self.base is None:
            return None
        regions = self.reach_at(time.monotonic())
        if not regions:
            return None
        self._ns_rid += 1
        req = dict(id=self._ns_rid, base=np.asarray(self.base, float), s=np.asarray(s, float).tolist(),
                   g=np.asarray(g, float).tolist(), path=None if path is None else np.asarray(path, float).tolist(),
                   regions=[(np.asarray(c, float), np.asarray(R, float), np.asarray(h, float)) for c, R, h in regions],
                   need=self.clear_req + self.ns_lane_extra, lane_off=self.ns_lane_off)
        t0 = time.monotonic()
        while self._ns_conn.poll(0):                         # drop late answers of earlier requests
            self._ns_conn.recv()
        self._ns_conn.send(req)
        while time.monotonic() - t0 < self.ns_worker_timeout:
            if self._ns_conn.poll(0.005):                    # blocking wait: releases the GIL
                r = self._ns_conn.recv()
                if r.get('id') == self._ns_rid:
                    r['ms'] = 1000 * (time.monotonic() - t0)
                    return r
        self.ns_counts['worker_late'] += 1
        self.event(f'NS_WORKER late (> {1000 * self.ns_worker_timeout:.0f} ms): no lane this cycle')
        return None

    def plan(self, q, mode, goal=None, planning_time=None, t_ref=None):
        if self.ns_lanes and mode == 'direct' and self.goal is not None:
            try:
                g = np.array(self.goal if goal is None else goal, float)
                r = self._ns_ask(q, g, None)
                if r is not None and r.get('blocked') and r['lanes']:
                    _, name, via, c = r['lanes'][0]
                    pos = _corners_to_pos([np.asarray(q, float), via, g])
                    if self.static_valid(pos):
                        tim = np.concatenate([[0.0], np.cumsum(np.max(np.abs(np.diff(pos, axis=0)), axis=1) / 0.3)])
                        self.ns_counts['lane_rest'] += 1
                        self.event(f'NS_LANE_REST {name}: straight path blocked ({100 * r["min_cl"]:.1f} cm) -> {name} lane, '
                                   f'clear {100 * min(c, 9.99):.1f} cm, screened in {r["ms"]:.0f} ms (worker)')
                        return self.from_positions(pos, tim)
            except Exception as e:                           # noqa: BLE001
                self.event(f'NS_LANE_REST failed ({e!r})')
        return super().plan(q, mode, goal=goal, planning_time=planning_time, t_ref=t_ref)

    def build(self, cur, i_s, seq, rest, kind, end_dist, now, **kw):
        cand, why = super().build(cur, i_s, seq, rest, kind, end_dist, now, **kw)
        if (not self.ns_lanes or kind not in ('INTENDED', 'DIRECT') or self.goal is None
                or (cand is not None and cand['full'])):
            return cand, why
        try:
            t0 = time.monotonic()
            s = np.asarray(seq[0], float); g = np.asarray(self.goal, float)
            r = self._ns_ask(s, g, np.asarray(seq, float))
            if r is None or not r['lanes']:
                return cand, why
            _, name, via, c = r['lanes'][0]                  # only the best screened lane gets SC3's full build
            pos = _corners_to_pos([s, via, g])
            c2, w2 = super().build(cur, i_s, pos, None, 'LANE_' + name.upper(), end_dist, time.monotonic(), **kw)
            if c2 is not None and c2['full']:
                self.ns_counts['lane_moving'] += 1
                self.event(f'NS_LANE_MOVING {name}: {kind} blocked -> {name} lane from the splice state, {c2["dur"]:.2f} s '
                           f'(screen {r["ms"]:.0f} ms in the worker, total {1000 * (time.monotonic() - t0):.0f} ms)')
                return c2, 'lane'
            self.event(f'NS_LANE_MOVING {name} rejected by SC3: {w2}')
        except Exception as e:                               # noqa: BLE001
            self.event(f'NS_LANE_MOVING failed ({e!r})')
        return cand, why



def main():
    # v2.30: no rclpy signal handler (it shut the context down while SC3's threads still used it: segfault at 21-31 %
    # of harness run ends, v2.29's try/finally did not help); SC3's own handler closes its child processes and exits.
    import signal
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    n = SC3NoStop()

    def _quit(*_):
        # v2.31: never unmap the shared memory here (the monitor thread may be reading it): stop the children, remove
        # the shared-memory name, exit; the OS frees the mapping.
        r = n.__dict__.get('_reader')
        for proc in (getattr(r, 'proc', None), n.__dict__.get('_ns_proc')):
            try:
                if proc is not None and proc.is_alive():
                    proc.terminate()
            except Exception:
                pass
        try:
            if r is not None and r.shm is not None:
                r.shm.unlink()
        except Exception:
            pass
        os._exit(0)

    signal.signal(signal.SIGINT, _quit)
    signal.signal(signal.SIGTERM, _quit)                                        # v2.8: the no-stop node is SC3 (defaults defer 1.2 s, lanes on)
    n.moving_now = False
    n.start()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    # v2.29: clean exit. Ctrl+C used to tear the interpreter down while SC3's own threads still called into rclpy,
    # which segfaulted at every harness run end (results unaffected). Close the reader and lane worker processes, then
    # leave without the interpreter teardown.
    try:
        ex.spin()
    except BaseException:                                   # KeyboardInterrupt / ExternalShutdownException / any
        pass
    finally:
        for attr in ('_reader', '_ns_proc'):
            obj = n.__dict__.get(attr)
            try:
                if attr == '_reader' and obj is not None:
                    obj.close()
                elif obj is not None and obj.is_alive():
                    obj.terminate()
                    obj.join(1.0)
            except Exception:
                pass
        os._exit(0)


if __name__ == '__main__':
    main()
