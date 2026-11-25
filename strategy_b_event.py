"""STRATEGY B v4 - EVENT-TRIGGERED REPLANNING while a trajectory runs (Proposal B, approved 2026-10-22 night with two
safeguards; SIMULATION only). Builds on strategy_b_fastval.py (V3 + faster validation, Proposal A); V3
(strategy_b_replan.py), Strategy A and their tests are unchanged.

What changes against V3 (nothing else):
  THREAT (monitor thread; its safety logic unchanged): a conflict within monitor_horizon on the active trajectory ->
    the SAME safe cut as V3 is dispatched (identical positions, times, velocities; version-checked). NEW: the cut
    carries the threatened path after its resting point as its intended path (metadata, never sent to the
    controller) and the planner is woken at once (THREAT). Contact now / no safe cut -> hold here, as V3 (no search
    while that stop runs: it has no path).
  WAKE-UP (planner loop): a THREAT starts a decision cycle immediately (V3: only every repair_period). A cycle still
    running for an older trajectory version stops waiting for MoveIt at once and sends no new request (its result
    could not be dispatched anyway: every send is version-checked).
  THREAT CYCLE (on the cut, while it runs): takeover point = splice_lead s ahead on the cut, or its resting point if
    that comes first. The threatened path from there gets the full check only (clear again -> continue on it); no
    second resting-point search on it: the cut already IS the validated fallback. Then V3's LOCAL (one request: the
    states after the blockage + the goal, any direction, rejoining optional) and GLOBAL (one request to the goal,
    only if time is left and it could arrive earlier), full validation, ranking and activation (re-splice at
    dispatch, newest-box re-check, version-checked dispatch).
  DEADLINE: while moving, MoveIt is waited for only until the dispatch deadline minus build_reserve (the time to
    validate the candidate that arrives): the planning budget does not bound MoveIt's answer time (overhead bench:
    up to +1.7 s). A late answer is discarded. Validation is never shortened.
  Nothing valid in time -> the cut runs to its validated resting point -> V3's from-rest cycle (existing fallback).
Safeguard 1 (the cut is never assumed to stay safe): the monitor keeps re-checking whatever runs - the cut too - every
  50 ms with the newest box data and cuts again / holds when it is threatened; that newer command makes the running
  cycle stale (abandoned; its candidate can no longer be dispatched). Every replacement - including the part of the
  cut it keeps - is re-checked with the newest box data right before dispatch (V3).
Safeguard 2 (evidence of a genuine in-motion handover): logs/SB_handover.jsonl records every dispatch with its version
  and send time, every THREAT, and every activation with the replaced and new versions, the takeover time and the
  commanded speed there, the cut's resting time, the robot state at dispatch and the cycle's timings; the bench
  trace then shows whether the arm was moving at the takeover and followed the new route.
Periodic cycles: as V3 (a full trajectory is never replanned; a PART trajectory is extended; from rest).
Events added: OBJECT_SEEN, THREAT, THREAT_WAKEUP, THREAT_CYCLE, VALIDATED, CYCLE_SUPERSEDED, CYCLE_DONE.

    python3 experiments/candidates/strategy_b_event.py        (same topics as V3: /strategy_b/*)
"""
import json
import os
import threading
import time

import numpy as np
import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import CollisionObject, Constraints, JointConstraint, MoveItErrorCodes, RobotState
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from strategy_a_global import ARM, GROUP, jdist
from strategy_b_fastval import StrategyBFastVal
from strategy_b_replan import ERR


class StrategyBEvent(StrategyBFastVal):

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        # s kept before the dispatch deadline to validate a candidate that arrives (fixed before the sudden-obstacle
        # test from the live A/B bench, not tuned on its results)
        self.build_reserve = p('build_reserve', 0.15).value
        self.threat_evt = threading.Event()
        self.threat = None                               # the latest THREAT
        self._ctx = None                                 # the running decision cycle: {ver, wait_until}
        self._seen = set()
        self.hlock = threading.Lock()
        d = os.environ.get('ROS_LOG_DIR')
        self.hand_log = open(os.path.join(os.path.dirname(d), 'logs', 'SB_handover.jsonl'), 'a') if d else None

    # ------------------------------------------------------------------ logs
    def hlog(self, **kw):
        if self.hand_log:
            with self.hlock:
                self.hand_log.write(json.dumps(kw, default=lambda x: x.tolist() if hasattr(x, 'tolist') else str(x)) + '\n')
                self.hand_log.flush()

    def on_scene(self, m):
        super().on_scene(m)
        for oid in [x for x in list(self.objs) if x not in self._seen]:
            p = self.objs[oid]['p']
            self.event(f'OBJECT_SEEN {oid} t={time.monotonic():.3f} at ({p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f})')
            self.hlog(ev='object_seen', id=oid, t=time.monotonic(), p=p)
        self._seen = set(self.objs)

    def send(self, pos, tim, vel, path, full, kind, ver=None):
        """V3's send (version-checked, serialized); the dispatched version is logged atomically with it."""
        with self.slock:
            ok = super().send(pos, tim, vel, path, full, kind, ver=ver)
            if ok:
                self.hlog(ev='dispatch', ver=self.ver, based_on=ver, kind=kind, full=bool(full), t0=self.cur['t0'],
                          T=float(self.cur['tim'][-1]), has_path=self.cur['path'] is not None)
        return ok

    # ------------------------------------------------------------------ THREAT (monitor thread)
    def monitor_check(self, cur, ver, q, now):
        """V3's check, unchanged in substance; a SAFE_STOP cut carries the threatened path and wakes the planner."""
        tau = now - cur['t0']
        k = min(int(np.searchsorted(cur['tim'], tau)), len(cur['pos']) - 1)
        tim = cur['tim']
        q_cmd = cur['pos'][-1] if tau >= tim[-1] else np.array([np.interp(tau, tim, cur['pos'][:, i]) for i in range(6)])
        if jdist(q, q_cmd) > self.track_tol and self.tracking_logged != ver:
            self.tracking_logged = ver
            self.event(f'TRACKING {jdist(q, q_cmd):.3f} rad off the commanded state {cur["kind"]}')
        if self.clearance(q, now) <= self.contact_dist:
            self.hold_here('box at the arm now', ver=ver)
            return 'emergency'
        j = self.first_conflict(cur['pos'], cur['tim'], now, k, self.monitor_horizon)
        if j is None:
            return 'clear'
        ahead = float(cur['tim'][j] - cur['tim'][k])
        self.event(f'MONITOR conflict {ahead:.2f} s ahead on {cur["kind"]} (v{ver})')
        stop = self.safe_prefix(cur, j, now, ver)
        if stop is not None:
            stop = stop[:3] + (self.threatened_path(cur, stop[0], j),) + stop[4:]
        if stop is not None and self.send(*stop, ver=ver):
            self.event(f'SAFE_STOP t={time.monotonic():.3f}')
            if stop[3] is None:
                self.event('THREAT_NO_PATH the threatened trajectory has no intended path: no search while the cut runs')
            elif self.goal is not None:
                self.raise_threat(ver, cur, ahead, now)
            return 'safe_stop'
        if stop is not None and ver != self.ver:
            self.event(f'STALE_SAFE_STOP_DROPPED the trajectory it judged (v{ver}) was replaced (now v{self.ver})')
            return 'stale'
        self.hold_here('no safe stopping point before the conflict', ver=ver)
        return 'emergency'

    @staticmethod
    def threatened_path(cur, cut_pos, j):
        """the cut's intended path: its own positions, then the threatened trajectory's path after the cut's resting
        point (cur['path'] is aligned with cur['pos']); None if that point cannot be identified exactly."""
        if cur['path'] is None:
            return None
        last = cut_pos[-1]
        m = next((i for i in range(min(j, len(cur['pos'])) - 1, -1, -1) if np.array_equal(cur['pos'][i], last)), None)
        if m is None:
            return None
        return np.vstack([cut_pos, np.asarray(cur['path'])[m + 1:]])

    def raise_threat(self, ver, cur, ahead, now):
        with self.slock:
            cut = self.cur if self.ver == ver + 1 else None
        if cut is None:
            return
        self.threat = dict(cut_ver=ver + 1, threatened_ver=ver, threatened_kind=cur['kind'], t_detect=now,
                           conflict_ahead=ahead, cut_t0=cut['t0'], cut_rest=cut['t0'] + float(cut['tim'][-1]))
        self.hlog(ev='threat', **self.threat)
        self.event(f'THREAT v{ver} {cur["kind"]}: conflict {ahead:.2f} s ahead -> cut v{ver + 1} rests in '
                   f'{self.threat["cut_rest"] - time.monotonic():.2f} s; planner woken')
        self.threat_evt.set()

    # ------------------------------------------------------------------ the loop (THREAT wake-up)
    def run(self):
        """strategy_b_local.run with the THREAT wake-up; otherwise unchanged."""
        while rclpy.ok() and not (self.mg.wait_for_server(1.0) and self.ctrl.wait_for_server(1.0)
                                  and self.valid_cli.wait_for_service(1.0)):
            self.get_logger().info('waiting for move_group / controller / validity')
        while rclpy.ok() and self.base is None:
            self.init_base()
        threading.Thread(target=self.monitor, daemon=True).start()
        t_goal, last_try = None, 0.0
        while rclpy.ok():
            woke = self.threat_evt.wait(0.02)
            self.threat_evt.clear()
            with self.lock:
                ng, self.new_goal = self.new_goal, None
                q = None if self.q is None else self.q.copy()
            if self.stop_req or ng is not None:
                if not self.finished(self.cur, time.monotonic()):
                    self.hold_here('new goal / stop', 'STOP')
                if self.stop_req:
                    self.stop_req, self.goal = False, None
                    self.event('STOPPED')
            if ng is not None:
                self.goal, t_goal, self.conflict_open = ng, time.monotonic(), None
                with self.slock:
                    self.cur = None
                self.event('GOAL')
            if self.goal is None or q is None:
                continue
            if jdist(q, self.goal) < 0.01 and float(np.abs(self.qd).max()) < self.still_speed:
                self.event(f'REACHED total={time.monotonic() - t_goal:.2f}')
                self.goal = None
                continue
            if time.monotonic() - t_goal > self.goal_timeout:
                self.event(f'FAILED goal not reached in {self.goal_timeout:.0f} s')
                self.goal = None
                continue
            if woke or time.monotonic() - last_try >= self.repair_period:
                if woke and self.threat is not None:
                    self.event(f'THREAT_WAKEUP {1000 * (time.monotonic() - self.threat["t_detect"]):.0f} ms after the detection')
                last_try = time.monotonic()
                self.extend()

    # ------------------------------------------------------------------ decision cycles
    def _extend(self, cur, ver, i_s, q, moving, now):
        t_s = now + max(cur['tim'][i_s] - (now - cur['t0']), 0.0)
        self._ctx = dict(ver=ver, wait_until=(t_s - self.reserve - self.build_reserve) if moving else None)
        try:
            if moving and cur['kind'] == 'SAFE_STOP' and cur['path'] is not None:
                return self._threat_cycle(cur, ver, i_s, q, now)
            return super()._extend(cur, ver, i_s, q, moving, now)
        finally:
            self._ctx = None

    def superseded(self, ver, where):
        if self.ver == ver:
            return False
        self.event(f'CYCLE_SUPERSEDED v{ver} -> v{self.ver} {where}: abandoned')
        return True

    def wait_cycle(self, fut, timeout):
        """wait for fut, but not past the cycle's deadline and not once its trajectory was replaced."""
        ctx = self._ctx
        end = time.monotonic() + timeout
        if ctx is not None and ctx['wait_until'] is not None:
            end = min(end, ctx['wait_until'])
        while not fut.done() and time.monotonic() < end and not (ctx is not None and self.ver != ctx['ver']):
            time.sleep(0.002)
        return fut.result() if fut.done() else None

    def why_none(self, default):
        ctx = self._ctx
        if ctx is not None and self.ver != ctx['ver']:
            return 'superseded (its trajectory was replaced)'
        if ctx is not None and ctx['wait_until'] is not None and time.monotonic() >= ctx['wait_until']:
            return 'deadline (no answer in time to validate it)'
        return default

    def plan_to(self, q, targets, planning_time, co):
        """V3's request, unchanged; not sent in a superseded cycle; waiting ends at the deadline (see wait_cycle)."""
        ctx = self._ctx
        if ctx is not None and self.ver != ctx['ver']:
            return None, 'superseded (its trajectory was replaced)'
        g = MoveGroup.Goal()
        r = g.request
        r.group_name, r.planner_id = GROUP, self.planner_id
        r.allowed_planning_time, r.num_planning_attempts = float(planning_time), self.attempts
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = float(self.speed)
        r.start_state = RobotState(joint_state=JointState(name=ARM, position=[float(x) for x in q]), is_diff=True)
        r.goal_constraints = [Constraints(joint_constraints=[
            JointConstraint(joint_name=n, position=float(x), tolerance_above=0.001, tolerance_below=0.001, weight=1.0)
            for n, x in zip(ARM, t)]) for t in targets]
        o = g.planning_options
        o.plan_only = True
        o.planning_scene_diff.is_diff = True
        o.planning_scene_diff.robot_state.is_diff = True
        objs = [CollisionObject(id=i, operation=CollisionObject.REMOVE) for i in list(self.objs)]
        if co is not None:
            objs.append(co)
        o.planning_scene_diff.world.collision_objects = objs
        h = self.wait_cycle(self.mg.send_goal_async(g), 3.0)
        if h is None or not h.accepted:
            return None, self.why_none('request refused')
        res = self.wait_cycle(h.get_result_async(), planning_time + 3.0)
        if res is None:
            return None, self.why_none('no answer')
        if res.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return None, ERR.get(res.result.error_code.val, f'MoveIt error {res.result.error_code.val}')
        jt = res.result.planned_trajectory.joint_trajectory
        if not jt.points:
            return None, 'empty trajectory'
        idx = [list(jt.joint_names).index(n) for n in ARM]
        pos = np.array([[pt.positions[i] for i in idx] for pt in jt.points])
        tim = np.array([pt.time_from_start.sec + 1e-9 * pt.time_from_start.nanosec for pt in jt.points])
        if jdist(pos[0], q) > 0.005:
            return None, f'start state moved {jdist(pos[0], q):.3f} rad (FixStartState) - never executed'
        return (pos, tim, jt), None

    def activate(self, cur, i_s, cand, ver, info=None):
        """V3's activation (re-splice at dispatch, newest-box re-check, version-checked dispatch) + the handover log."""
        with self.lock:
            q_d, qd_d = self.q.copy(), self.qd.copy()
        t_call = time.monotonic()
        ok = super().activate(cur, i_s, cand, ver)
        if ok:
            self.hlog(ev='activated', kind=cand['kind'], full=bool(cand['full']), replaced_ver=ver, new_ver=ver + 1,
                      replaced_kind=cur['kind'], t_call=t_call, t_takeover=cur['t0'] + float(cur['tim'][i_s]),
                      takeover_speed_cmd=float(np.abs(cur['vel'][i_s]).max()), t_rest_replaced=cur['t0'] + float(cur['tim'][-1]),
                      q_dispatch=q_d, speed_dispatch=float(np.abs(qd_d).max()), cycle=info)
        return ok

    def _threat_cycle(self, cur, ver, i_s, q, now):
        """one decision on the safe cut (module docstring): at most one LOCAL and one GLOBAL request."""
        th = self.threat if self.threat is not None and self.threat['cut_ver'] == ver else None
        s = cur['pos'][i_s]
        end_dist = jdist(cur['pos'][-1], self.goal)
        t_s = now + max(cur['tim'][i_s] - (now - cur['t0']), 0.0)
        t_dead = t_s - self.reserve
        t_rest = cur['t0'] + float(cur['tim'][-1])
        intended = np.asarray(cur['path'][i_s:])
        info = dict(cut_ver=ver, t_cycle=now, t_takeover=t_s, t_rest=t_rest, t_dead=t_dead,
                    t_detect=None if th is None else th['t_detect'], plan_ms={}, val_ms={}, plan_why={})
        self.event(f'THREAT_CYCLE v{ver}: takeover in {t_s - now:.2f} s at {float(np.abs(cur["vel"][i_s]).max()):.3f} rad/s, '
                   f'cut rests in {t_rest - now:.2f} s' + ('' if th is None else f', {1000 * (now - th["t_detect"]):.0f} ms after the threat'))
        outcome = self._threat_decide(cur, ver, i_s, q, now, s, end_dist, t_s, t_dead, intended, info)
        info['outcome'] = outcome
        info['total_ms'] = round(1000 * (time.monotonic() - now))
        self.hlog(ev='cycle', **info)
        self.event(f'CYCLE_DONE v{ver} {outcome} in {info["total_ms"]} ms (plan {info["plan_ms"]}, validation {info["val_ms"]})')

    def _validate(self, cur, i_s, seq, rest, kind, end_dist, info, search_rest=True):
        t0 = time.monotonic()
        cand, why = self.build(cur, i_s, seq, rest, kind, end_dist, time.monotonic(), search_rest=search_rest)
        info['val_ms'][kind] = round(1000 * (time.monotonic() - t0))
        self.event(f'VALIDATED {kind} in {info["val_ms"][kind]} ms: '
                   + (why if cand is None else f'{"full" if cand["full"] else "resting point"}'))
        return cand, why

    def _threat_decide(self, cur, ver, i_s, q, now, s, end_dist, t_s, t_dead, intended, info):
        if len(intended) < 2:
            self.fallback(cur, q, True, now, ver)
            return 'no intended path'
        # 1. the threatened path from the takeover point, full check only (the box may have left it)
        cand, _ = self._validate(cur, i_s, intended, None, 'INTENDED', end_dist, info, search_rest=False)
        if cand is not None and cand['full']:
            if self.activate(cur, i_s, cand, ver, info):
                self.t_block = None
                return 'INTENDED clear again: continuing on it'
            return 'INTENDED not activated'
        if self.superseded(ver, 'after the threatened-path check'):
            return 'superseded'
        self.declare_conflict()
        if self.t_block is None:
            self.t_block = time.monotonic()
        cands = []
        ipos, itim, _ = self.retime(intended)
        T_direct = float(itim[-1])
        c = self.first_conflict(ipos, itim, t_s)
        # 2. LOCAL (V3): one request, several targets (states after the blocked segment, and the goal)
        if c is not None:
            last = c
            for j in self.samples(itim, c):
                if self.reach_clearance(ipos[j], t_s + itim[j] + self.t_slack) <= self.clear_req:
                    last = j
            e0 = min(last + 1, len(ipos) - 1)
            exits = sorted({min(int(np.searchsorted(itim, itim[e0] + off)), len(ipos) - 1)
                            for off in [0.0] + self.local_exit_offsets} - {len(ipos) - 1})
            H = float(itim[last] - itim[0]) + self.t_slack
            co, Hu, keep, ok = self.occupancy(H, s, [], [ipos[e] for e in exits] + [self.goal], t_s)
            budget = self.local_share * (t_dead - time.monotonic()) - self.plan_overhead
            if not ok:
                self.reject('LOCAL', 'start state inside the planning region at every horizon')
            elif not keep:
                self.reject('LOCAL', 'every target inside the planning region')
            elif budget < self.min_plan_time:
                self.event(f'LOCAL_SKIPPED budget {1000 * budget:.0f} ms')
            else:
                t0 = time.monotonic()
                lp, why = self.plan_to(s, keep, budget, co)
                info['plan_ms']['LOCAL'] = round(1000 * (time.monotonic() - t0))
                info['plan_why']['LOCAL'] = why
                self.event(f'LOCAL_PLAN {"found" if lp else "none (" + why + ")"} in {info["plan_ms"]["LOCAL"]} ms '
                           f'(budget {1000 * budget:.0f}, region {Hu:.2f} s, {len(keep)} targets)')
                if lp is None:
                    self.reject('LOCAL', why)
                else:
                    reached = int(np.argmin([jdist(lp[0][-1], x) for x in keep]))
                    e = next((x for x in exits if jdist(ipos[x], keep[reached]) < 1e-6), None)
                    rest = None if e is None else ipos[e + 1:]
                    cnd, why = self._validate(cur, i_s, lp[0], rest, 'LOCAL', end_dist, info)
                    if cnd is None:
                        self.reject('LOCAL', why)
                    else:
                        cands.append(cnd)
                        self.event(f'CANDIDATE LOCAL full={int(cnd["full"])} dur={cnd["dur"]:.2f} L={cnd["L"]:.2f} '
                                   f'clear={100 * cnd["clear"]:.1f} cm target={"goal" if e is None else f"rejoin {e}"}')
        if self.superseded(ver, 'after LOCAL'):
            return 'superseded'
        # 3. GLOBAL (V3): to the original goal, only if time is left and it could still arrive earlier
        fulls = [x for x in cands if x['full']]
        best = min((x['now'] + x['dur'] for x in fulls), default=None)
        left = t_dead - time.monotonic() - self.plan_overhead
        bound = t_s + T_direct
        if left < self.min_plan_time:
            self.event(f'GLOBAL_SKIPPED no time left ({1000 * left:.0f} ms)')
        elif best is not None and best <= bound + self.tie_time:
            self.event(f'GLOBAL_SKIPPED the local candidate arrives within {best - bound:.2f} s of any possible route')
        else:
            co, Hu, keep, ok = self.occupancy(T_direct, s, [self.goal], [], t_s)
            if not ok:
                self.reject('GLOBAL', 'start or goal inside the planning region at every horizon')
            else:
                t0 = time.monotonic()
                gp, why = self.plan_to(s, [self.goal], left, co)
                info['plan_ms']['GLOBAL'] = round(1000 * (time.monotonic() - t0))
                info['plan_why']['GLOBAL'] = why
                self.event(f'GLOBAL_PLAN {"found" if gp else "none (" + why + ")"} in {info["plan_ms"]["GLOBAL"]} ms '
                           f'(budget {1000 * left:.0f}, region {Hu:.2f} s)')
                if gp is None:
                    self.reject('GLOBAL', why)
                else:
                    cnd, why = self._validate(cur, i_s, gp[0], None, 'GLOBAL', end_dist, info)
                    if cnd is None:
                        self.reject('GLOBAL', why)
                    else:
                        cands.append(cnd)
                        self.event(f'CANDIDATE GLOBAL full={int(cnd["full"])} dur={cnd["dur"]:.2f} L={cnd["L"]:.2f} '
                                   f'clear={100 * cnd["clear"]:.1f} cm')
        if self.superseded(ver, 'after GLOBAL'):
            return 'superseded'
        # 4. choose and activate (V3)
        fulls = [x for x in cands if x['full']]
        pick = self.rank(fulls, True, t_s)
        if pick is not None:
            self.event(f'SELECT {pick["kind"]} of {len(fulls)} full: arrival in {pick["arrival"] - time.monotonic():.2f} s, '
                       f'L={pick["L"]:.2f} rad, clear={100 * pick["clear"]:.1f} cm; blockage->now {time.monotonic() - self.t_block:.2f} s')
            if self.activate(cur, i_s, pick, ver, info):
                self.event(f'ACTIVATED {pick["kind"]} blockage->activation {time.monotonic() - self.t_block:.2f} s')
                self.t_block = None
                return f'ACTIVATED {pick["kind"]}'
        parts = [x for x in cands if not x['full']]
        if parts:
            pick = min(parts, key=lambda x: jdist(x['end'], self.goal))
            if self.activate(cur, i_s, pick, ver, info):
                self.event(f'ACTIVATED {pick["kind"]}_PART (to a validated resting point)')
                return f'ACTIVATED {pick["kind"]}_PART'
        self.fallback(cur, q, True, now, ver)
        return 'NO_REPLACEMENT (the cut runs to its resting point)'


def main():
    rclpy.init()
    n = StrategyBEvent()
    n.moving_now = False
    n.start()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    ex.spin()


if __name__ == '__main__':
    main()
