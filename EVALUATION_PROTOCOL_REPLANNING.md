# Evaluation protocol: human-aware replanning (fixed 24 Nov 2026)

Applies to all three experiments, in the simulator (harness, domain 88) and on the real UR5e:
- **Experiment 1**: RRT-Connect with stop-and-replan (SC3 with `NOSTOP=0 CANCEL=1`), the baseline.
- **Experiment 2**: RRT-Connect with predictive detours and moving trajectory switching (SC3 v2.41, `HUMAN=1`).
- **Experiment 3**: Experiment 2 + predictive speed MPC (v2.42, after v2.41 is validated live).
Same scenarios, start / goal configurations and settings for all three. No result is discarded: failed and stopped runs
are evidence and are reported.

## Scenario classes
| Class | Meaning | Simulator scenarios | Lab (25 Nov runbook) |
|---|---|---|---|
| A | human crosses the trajectory | cross_slow, person_walk_reach | H4 |
| B | human approaches, then moves away | hand_near, person_reach_prox | H2, H5 |
| C | sudden move into the immediate path | person_sweep, person_reach | H1, H3 (expected result: protective stop, then continue) |

## Latencies, per conflict event (each reported as median / p95 / max, never the average alone)
| Metric | From → to | Simulator source | Real-robot source |
|---|---|---|---|
| Reaction time | human movement → conflict detected | HARNESS_TRIGGER / truth → first MONITOR / H7 line | first detection of the moving object (bag) → first conflict line (approximate: the true movement start is not known live) |
| Planning latency | conflict detected → valid detour found | conflict line → CANDIDATE / BANK built | same, from SB_node.log |
| Executable time | detour found → robot executing it | candidate → REPLACE (controller accepted) → joint velocity change | same + /joint_states from the bag |

## Indicators and their denominators
| Indicator | Definition | Denominator |
|---|---|---|
| Successful replanning % | conflict events resolved by a detour while moving, with no protective stop in that event | **eligible conflict events**: conflicts with a person, detected while moving, at least the replan threshold (0.6 s) ahead |
| Planning failures | eligible events with no detour found before the stop threshold | reported as a count, separately |
| Protective stops | emergency / controller-cancel stops | count per run and per leg |
| Unnecessary replanning % | replans whose ORIGINAL trajectory would not have reached the person (simulator only: checked against the true future motion) | all replans |
| Moving detours | switches to a new trajectory while the arm moves | count per run |
| Minimum separation | smallest true distance robot ↔ person over the run (simulator); smallest SC3 clearance (real robot) | per run; contacts split into robot MOVING vs robot STANDING (joint speed > 0.02 rad/s in the 0.3 s before) |
| Task completion time | goal sent → goal reached | per leg |

## Reporting rules
- Frame → detection 55 → 15 ms (A1, installed 21 Nov) and new object → planning scene 58 → 18 ms (C1, live test version,
  not installed) are different stages and are reported separately.
- The ready-made-route result 7 s vs 31 s per leg (24 Nov) is a **box-only** experiment, never human-mode evidence.
- Perception is "depth-based human detection and tracking" (no skeleton / body parts). Separation monitoring is
  "SSM-inspired" (not an ISO-certified safety function); the UR safety system and the e-stop are the independent safety layer.

## Acceptance of v2.41 on the real robot (25 Nov)
1. Human detection and tracking reliable under the real lab load.
2. Replanning triggers when the current trajectory becomes threatened.
3. Alternative trajectories generated and validated.
4. Moving trajectory switching works on the UR5e.
5. Minimum separation kept in the tested scenarios.
6. Protective stop when no safe detour can be guaranteed (class C).
7. Every run has telemetry (SB_node.log, trajectories, T recording).
