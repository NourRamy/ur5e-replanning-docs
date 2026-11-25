"""STRATEGY B v4 + BLENDED HANDOVER (Stage 2, approved 2026-10-23 with conditions; SIMULATION only).
Builds on strategy_b_diag.py (V4 + junction diagnostics); V4, V3, Strategy A, their tests and the planner
configuration are unchanged. Only the THREAT CYCLE's LOCAL / GLOBAL candidates on a MOVING safe cut change.

Problem (diagnosis 2026-10-23): MoveIt plans from the takeover position as if the arm stood still there; the splice
then attaches the cut's velocity v_s to a path leaving in another direction -> a velocity jump inside the new path's
first, very short segment (MoveIt's from-rest sampling: 0.0025 rad) -> the junction check rejects it (correctly).

BLEND (takeover on the cut at index i_s with position q_s and velocity v_s, |v_s| = max-norm speed > blend_min_speed):
  stop component   q_d(t) = q_s + v_s t - v_s t^2 / (2 T_d),  t <= T_d = |v_s| / (a_stop amax);  then q_v
                   (straight line in joint space along v_s, constant deceleration a_stop*amax on the fastest joint)
  virtual rest     q_v = q_s + v_s T_d / 2: MoveIt plans LOCAL / GLOBAL FROM q_v (zero velocity there is then TRUE
                   for the component it plans); region / start checks as V4, with q_v as the start
  new component    q_n(t): MoveIt's path from q_v, timed from rest with acceleration <= a_new amax while
                   t < T_d and <= amax after (velocity <= vmax as always)
  blended motion   q(t) = q_d(t) + q_n(t) - q_v,  v(t) = v_d(t) + v_n(t)
  continuity       t = 0: q = q_s, v = v_s (exactly the cut's point i_s); end: q = the new path's end (goal or rejoin
                   state), v = 0. The kept part of the cut before i_s is unchanged (as V3's splice).
  bounds (a_new <= a_stop, per joint, for t < T_d):  |v_d| <= |v_s| (1 - t/T_d),  |v_n| <= a_new amax t
                   = |v_s| t/T_d a_new/a_stop <= |v_s| t/T_d  ->  |v| <= |v_s| <= vmax;
                   |a| <= (a_stop + a_new) amax (straight-segment kinematics; corners of the new path and the
                   controller's cubic interpolation are NOT covered by this bound -> checked numerically)
  a_stop = 1.0, a_new = 0.5: the stop component decelerates like the cut itself (V3's retime: amax), so q_v is the
  cut's own stopping distance ahead (|v_s|^2 / (2 amax): 0.099 rad at full speed); the sum 1.5 amax = the existing
  junction threshold. (First version a_stop = a_new = 0.5, sum amax: in the offline scenario E3 its q_v, 0.197 rad
  ahead at full speed, lay inside the planning region -> LOCAL / GLOBAL never requested; changed 2026-10-23 before
  any live blend trial. No check or threshold changed.)
VALIDATION of a blended candidate: everything V4 does to any candidate, unchanged (time-indexed reach check of the
  whole sent trajectory with the newest box data, MoveIt state validity on samples, joint limits, the junction
  check, ranking, re-check right before dispatch, version-checked dispatch), PLUS a TRANSITION CHECK on the
  trajectory as the controller executes it (joint_trajectory_controller 2.54: positions + velocities -> cubic
  Hermite per segment), from the last kept point before the takeover to the end of the overlap (T_d) and one
  segment beyond:  |q| within limits; |v| <= vmax at the points; the cubic's velocity peak inside each segment
  <= 1.05 vmax; the cubic's acceleration at both ends of each segment <= 1.5 amax (the existing junction threshold).
  Thresholds fixed before any blend result. After the overlap the trajectory is the new path timed as V3 times every
  route (inherited, unchanged; the corner accelerations of that timing are a separate, reported issue).
THREAT CYCLE (approved 2026-10-23 with the blend):
  takeover   threat_lead = 0.6 s ahead on the cut (V4: 1.0 s), or the cut's resting point if that comes first; the
             MoveIt deadline follows it (V4's reserves unchanged: dispatch 0.15 s, validation 0.15 s)
  1.         the threatened path from the takeover: clear again with the newest box data -> continue on it (V4)
  2. ONE     one MoveIt request from q_v (blend) or the takeover state (takeover at rest) to ANY of the rejoin states
             after the blocked segment (LOCAL ONLY, approved 10-23: the goal is a target only when no rejoin state is
             usable - none exists or every one is inside the planning region); V4's separate GLOBAL request and the
             ranking are dropped; waited for until the deadline; a late answer is discarded (V4). The original path
             after the rejoin state is kept unchanged as the goal-directed remainder.
  3.         its result through V4's full validation + the transition check; it is activated if it passes everything
             (LOCAL = rejoins the threatened path, GLOBAL = straight to the goal)
  REGION     the predicted-occupancy region is unchanged; its outcome (start / every target inside it) is logged per
             cycle (logs/SB_handover.jsonl ev=region) and counted separately
Nothing valid in time -> the cut runs to its validated resting point (V4's fallback, unchanged).
Logs: logs/SB_blend.jsonl (per blended candidate: takeover state, T_d, q_v, transition-check values and verdict).

    python3 experiments/candidates/strategy_b_blend.py        (same topics as V4 / V3: /strategy_b/*)
"""
import json
import os
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from strategy_a_global import jdist
from strategy_b_adaptive import Q_LIM
from strategy_b_diag import StrategyBDiag, hermite_acc


def hermite_vpeak(p0, p1, v0, v1, T):
    """max |velocity| inside each cubic Hermite segment (per segment, max over joints)."""
    d = p1 - p0
    c2 = (3 * d / T - 2 * v0 - v1) / T
    c3 = (-2 * d / T + v0 + v1) / T ** 2
    tv = np.clip(-c2 / (3 * np.where(np.abs(c3) < 1e-12, 1e-12, c3)), 0.0, T)
    vin = np.abs(v0 + 2 * c2 * tv + 3 * c3 * tv ** 2)
    return np.maximum(vin, np.maximum(np.abs(v0), np.abs(v1))).max(1)


class StrategyBBlend(StrategyBDiag):

    def __init__(self):
        super().__init__()
        p = self.declare_parameter
        self.a_stop = p('blend_stop_acc', 1.0).value               # x amax
        self.a_new = p('blend_new_acc', 0.5).value                 # x amax, while the stop component runs
        self.blend_min_speed = p('blend_min_speed', 0.02).value      # rad/s: below it the takeover is a rest point
        self.blend_dt = p('blend_sample_dt', 0.04).value             # s: samples of the overlap
        self.threat_lead = p('threat_lead', 0.6).value               # s: takeover ahead on the cut (approved 10-23)
        self._blend = None
        self._region = None
        d = os.environ.get('ROS_LOG_DIR')
        self._blog = open(os.path.join(os.path.dirname(d), 'logs', 'SB_blend.jsonl'), 'a') if d else None
        self.event(f'BLEND handover on: stop {self.a_stop} amax, new path {self.a_new} amax in the overlap, '
                   f'min takeover speed {self.blend_min_speed} rad/s')

    def blog(self, **kw):
        if self._blog:
            self._blog.write(json.dumps(kw, default=lambda x: np.round(x, 5).tolist() if hasattr(x, 'tolist') else str(x)) + '\n')
            self._blog.flush()

    # ------------------------------------------------------------------ threat cycle: plan from the virtual rest
    def _threat_cycle(self, cur, ver, i_s, q, now):
        # TAKEOVER HORIZON (approved 2026-10-23): threat_lead s ahead on the cut (V4: splice_lead 1.0 s), or its resting
        # point if that comes first; the cycle's MoveIt deadline follows it (V4's reserves unchanged)
        tau = now - cur['t0']
        i_s = min(int(np.searchsorted(cur['tim'], tau + self.threat_lead)), len(cur['pos']) - 1)
        t_s = now + max(cur['tim'][i_s] - tau, 0.0)
        if self._ctx is not None:
            self._ctx['wait_until'] = t_s - self.reserve - self.build_reserve
        v_s = np.asarray(cur['vel'][i_s], float)
        sp = float(np.abs(v_s).max())
        self._blend = None
        if sp > self.blend_min_speed:
            T_d = sp / (self.a_stop * self.amax)
            q_s = np.asarray(cur['pos'][i_s], float)
            q_v = q_s + v_s * T_d / 2
            if (np.abs(q_v) > Q_LIM).any():
                self.event('BLEND off: the virtual rest point is outside the joint limits')
            else:
                self._blend = dict(ver=ver, i_s=int(i_s), t0=cur['t0'], q_s=q_s, v_s=v_s, T_d=T_d, q_v=q_v, last=None)
                self.event(f'BLEND v{ver}: takeover speed {sp:.3f} rad/s, overlap T_d {T_d:.2f} s, virtual rest '
                           f'{np.abs(q_v - q_s).max():.3f} rad ahead along v_s')
        try:
            return super()._threat_cycle(cur, ver, i_s, q, now)
        finally:
            self._blend = None

    def _blend_on(self, cur, i_s):
        b = self._blend
        return b is not None and b['i_s'] == int(i_s) and b['t0'] == cur['t0']

    def _threat_decide(self, cur, ver, i_s, q, now, s, end_dist, t_s, t_dead, intended, info):
        """V4's decision with ONE planning request: 1. the threatened path from the takeover (unchanged: clear again
        -> continue on it); 2. one MoveIt request from the planning start (the virtual rest q_v when blending, else the
        takeover state) to ANY of the rejoin states after the blockage and the goal, waited for until the deadline;
        3. its result through V4's full validation (+ the transition check for a blend); the first candidate that
        passes everything is activated (V4's activation: newest-box re-check, version-checked dispatch); 4. nothing
        valid in time -> the cut runs to its resting point (V4's fallback)."""
        blend = self._blend_on(cur, i_s)
        if blend:
            info['blend'] = dict(T_d=round(self._blend['T_d'], 3), takeover_speed=round(float(np.abs(self._blend['v_s']).max()), 3))
        info['takeover_speed'] = round(float(np.abs(cur['vel'][i_s]).max()), 3)
        if len(intended) < 2:
            self.fallback(cur, q, True, now, ver)
            return 'no intended path'
        # 1. the threatened path from the takeover point, full check only (the box may have left it) - V4, unchanged
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
        p0 = self._blend['q_v'] if blend else s
        ipos, itim, _ = self.retime(intended)
        T_direct = float(itim[-1])
        c = self.first_conflict(ipos, itim, t_s)
        # 2. ONE request: rejoin states after the blocked segment + the goal (V4's LOCAL request, with the whole time)
        exits = []
        if c is not None:
            last = c
            for j in self.samples(itim, c):
                if self.reach_clearance(ipos[j], t_s + itim[j] + self.t_slack) <= self.clear_req:
                    last = j
            e0 = min(last + 1, len(ipos) - 1)
            exits = sorted({min(int(np.searchsorted(itim, itim[e0] + off)), len(ipos) - 1)
                            for off in [0.0] + self.local_exit_offsets} - {len(ipos) - 1})
            H = float(itim[last] - itim[0]) + self.t_slack
        # LOCAL ONLY (approved 2026-10-23): the rejoin states after the blocked segment; the goal only when none of
        # them is usable (none exists, or every one lies inside the planning region). The original path after the
        # rejoin state is kept unchanged as the goal-directed remainder.
        target_set = 'rejoin'
        if exits:
            co, Hu, keep, ok = self.occupancy(H, p0, [], [ipos[e] for e in exits], t_s)
            if ok and not keep:
                target_set = 'goal (every rejoin state inside the region)'
                co, Hu, keep, ok = self.occupancy(H, p0, [], [self.goal], t_s)
        elif c is not None:                                    # blocked up to the goal: no rejoin state exists
            target_set = 'goal (no rejoin state after the blockage)'
            co, Hu, keep, ok = self.occupancy(H, p0, [], [self.goal], t_s)
        else:                                                  # no conflict on the reach model: the goal only (V4's GLOBAL region)
            target_set = 'goal (no conflict on the reach model)'
            co, Hu, keep, ok = self.occupancy(T_direct, p0, [self.goal], [], t_s)
        n_t = len(exits) if target_set == 'rejoin' else 1
        info['region'] = dict(ok=bool(ok), target_set=target_set, targets=n_t, kept=len(keep) if ok else 0, horizon=round(float(Hu), 2),
                              result='start inside the region at every horizon' if not ok else
                              'every target inside the region' if not keep else 'ok')
        self.hlog(ev='region', cut_ver=ver, t=time.monotonic(), blend=bool(blend), **info['region'])
        if not ok or not keep:
            self.reject('ONE', info['region']['result'])
            self.fallback(cur, q, True, now, ver)
            return f'NO_REPLACEMENT ({info["region"]["result"]})'
        wait_until = self._ctx['wait_until'] if self._ctx is not None else t_dead
        budget = wait_until - time.monotonic() - self.plan_overhead
        if budget < self.min_plan_time:
            self.reject('ONE', f'no time to plan ({1000 * budget:.0f} ms)')
            self.fallback(cur, q, True, now, ver)
            return 'NO_REPLACEMENT (no time to plan)'
        t0 = time.monotonic()
        info['plan_t0'] = t0
        lp, why = self.plan_to(p0, keep, budget, co)
        info['plan_t1'] = time.monotonic()
        info['plan_ms']['ONE'] = round(1000 * (info['plan_t1'] - t0))
        info['plan_why']['ONE'] = why
        self.event(f'ONE_PLAN {"found" if lp else "none (" + why + ")"} in {info["plan_ms"]["ONE"]} ms '
                   f'(budget {1000 * budget:.0f}, region {Hu:.2f} s, {len(keep)} of {n_t} targets, from '
                   f'{"the virtual rest (blend)" if blend else "the takeover state"})')
        if lp is None and why.startswith('superseded'):
            self.superseded(ver, 'during the plan')
            return 'superseded'
        if lp is None:
            self.reject('ONE', why)
            self.fallback(cur, q, True, now, ver)
            return f'NO_REPLACEMENT (plan: {why})'
        if self.superseded(ver, 'after the plan'):
            return 'superseded'
        reached = int(np.argmin([jdist(lp[0][-1], x) for x in keep]))
        e = next((x for x in exits if jdist(ipos[x], keep[reached]) < 1e-6), None)
        rest = None if e is None else ipos[e + 1:]
        kind = 'GLOBAL' if e is None else 'LOCAL'              # to the goal / rejoining the threatened path
        # 3. full validation (V4) + the transition check; the first candidate that passes is activated
        cnd, why = self._validate(cur, i_s, lp[0], rest, kind, end_dist, info)
        if cnd is None:
            self.reject(kind, why)
            self.fallback(cur, q, True, now, ver)
            return f'NO_REPLACEMENT ({kind}: {why})'
        if self.activate(cur, i_s, cnd, ver, info):
            self.t_block = None
            k = kind + ('' if cnd['full'] else '_PART')
            self.event(f'ACTIVATED {k} ({"blend" if blend else "splice"}) at takeover speed {info["takeover_speed"]:.3f} rad/s')
            return f'ACTIVATED {k}'
        self.fallback(cur, q, True, now, ver)
        return f'NO_REPLACEMENT ({kind} not activated: re-check before dispatch or newer command)'

    # ------------------------------------------------------------------ timing of the new component
    def retime_two(self, pos, a1, T1):
        """retime from rest (same conventions as strategy_b_local.retime) with |accel| <= a1 while t < T1, amax after."""
        pos = np.asarray(pos, float)
        keep = np.r_[True, np.abs(np.diff(pos, axis=0)).max(1) > 1e-6]
        pos = pos[keep]
        if len(pos) < 2:
            return pos, np.zeros(len(pos)), np.zeros_like(pos)
        d = np.diff(pos, axis=0)
        ds = np.abs(d).max(1)
        u = d / ds[:, None]
        n = len(pos)
        v = np.full(n, self.vmax)
        v[0] = v[-1] = 0.0
        t = 0.0
        tk = np.zeros(n)
        for k in range(n - 1):                                 # forward: the limit depends on the time reached
            a = a1 if t < T1 else self.amax
            v[k + 1] = min(v[k + 1], np.sqrt(v[k] ** 2 + 2 * a * ds[k]))
            t += 2 * ds[k] / max(v[k] + v[k + 1], 1e-6)
            tk[k + 1] = t
        for k in range(n - 2, -1, -1):                         # backward (to rest at the end), same time-dependent limit
            a = a1 if tk[k] < T1 else self.amax
            v[k] = min(v[k], np.sqrt(v[k + 1] ** 2 + 2 * a * ds[k]))
        tim = np.r_[0.0, np.cumsum(2 * ds / np.maximum(v[:-1] + v[1:], 1e-6))]
        vel = np.zeros_like(pos)
        for k in range(1, n - 1):
            vel[k] = v[k] * (u[k - 1] + u[k]) / 2
        return pos, tim, vel, v, u

    def blend_component(self, seq):
        """the blended motion from the takeover (t = 0 at q_s): (pos, tim, vel), and the overlap info."""
        b = self._blend
        T_d, q_s, v_s, q_v = b['T_d'], b['q_s'], b['v_s'], b['q_v']
        out = self.retime_two(seq, self.a_new * self.amax, T_d)
        if len(out) == 3:                                      # degenerate path (one point): the stop component alone
            npos, ntim = out[0], out[1]
            nvel, sp, u = out[2], np.zeros(len(npos)), np.zeros((max(len(npos) - 1, 1), 6))
        else:
            npos, ntim, nvel, sp, u = out
        T_n = float(ntim[-1])
        T_end = max(T_d, T_n)
        grid = np.linspace(0.0, T_d, int(np.ceil(T_d / self.blend_dt)) + 1)
        # overlap samples only away from the new path's own points (a sample a fraction of a ms from a waypoint makes
        # a cubic segment that short: its acceleration = the small corner difference / that time)
        grid = grid[np.abs(grid[:, None] - ntim[None, :]).min(1) > self.blend_dt / 2]
        ts = np.unique(np.r_[ntim, grid, T_end])
        ts = ts[np.r_[True, np.diff(ts) > 1e-4]]

        def comp_n(t):
            if t >= T_n or len(npos) < 2:
                return npos[-1], np.zeros(6)
            k = int(np.clip(np.searchsorted(ntim, t, side='right') - 1, 0, len(npos) - 2))
            tau = t - ntim[k]
            if tau < 1e-9:
                return npos[k], nvel[k]
            dt = ntim[k + 1] - ntim[k]
            a = (sp[k + 1] - sp[k]) / max(dt, 1e-9)
            s = sp[k] * tau + 0.5 * a * tau ** 2
            if abs(ntim[k + 1] - t) < 1e-9:
                return npos[k + 1], nvel[k + 1]
            return npos[k] + s * u[k], (sp[k] + a * tau) * u[k]

        P, V = [], []
        for t in ts:
            if t < T_d:
                qd = q_s + v_s * t - v_s * t ** 2 / (2 * T_d)
                vd = v_s * (1 - t / T_d)
            else:
                qd, vd = q_v, np.zeros(6)
            qn, vn = comp_n(t)
            P.append(qd + qn - q_v)
            V.append(vd + vn)
        P, V = np.array(P), np.array(V)
        V[0] = v_s                                             # exact: v_n(0) = 0
        P[0] = q_s
        V[-1] = 0.0
        return P, ts, V, dict(T_d=T_d, T_n=T_n)

    # ------------------------------------------------------------------ splice: blended when planned from q_v
    def splice(self, cur, i_s, seq, now):
        if not (self._blend_on(cur, i_s) and len(seq) and np.abs(np.asarray(seq[0]) - self._blend['q_v']).max() < 1e-6):
            return super().splice(cur, i_s, seq, now)
        tau = now - cur['t0']
        t_s = cur['tim'][i_s] - tau
        if t_s < 0.08 and i_s < len(cur['pos']) - 1:
            return None
        keep = np.where((cur['tim'] - tau > 0.02) & (np.arange(len(cur['tim'])) < i_s))[0]
        bpos, btim, bvel, binfo = self.blend_component(seq)
        pos = np.vstack([cur['pos'][keep], bpos]) if len(keep) else bpos
        tim = np.r_[cur['tim'][keep] - tau, btim + max(t_s, 0.0)]
        vel = np.vstack([cur['vel'][keep], bvel]) if len(keep) else bvel
        self._blend['last'] = dict(pos=pos, n_keep=len(keep), t_splice=max(t_s, 0.0), **binfo)
        return pos, tim, vel

    # ------------------------------------------------------------------ added transition check
    def transition_check(self, pos, tim, vel, n_keep, t_splice, T_d):
        """the controller's cubic Hermite from the last kept point to one segment after the overlap: values, why."""
        i0 = max(n_keep - 1, 0)
        i1 = int(np.searchsorted(tim, t_splice + T_d)) + 1
        i1 = min(max(i1, i0 + 1), len(pos) - 1)
        P, Tm, V = pos[i0:i1 + 1], tim[i0:i1 + 1], vel[i0:i1 + 1]
        T = np.maximum(np.diff(Tm), 1e-6)[:, None]
        a0, a1 = hermite_acc(P[:-1], P[1:], V[:-1], V[1:], T)
        acc = np.maximum(np.abs(a0), np.abs(a1)).max(1)
        vpk = hermite_vpeak(P[:-1], P[1:], V[:-1], V[1:], T)
        val = dict(segments=int(len(T)), acc_max=float(acc.max()), acc_argmax=int(np.argmax(acc)),
                   vpeak_max=float(vpk.max()), v_points_max=float(np.abs(V).max()), dt_min=float(T.min()),
                   acc_lim=1.5 * self.amax, vpeak_lim=1.05 * self.vmax, vmax=self.vmax)
        if (np.abs(P) > Q_LIM).any():
            return val, 'transition: joint limit'
        if val['v_points_max'] > self.vmax * (1 + 1e-6):
            return val, f'transition: velocity {val["v_points_max"]:.3f} > vmax {self.vmax:.3f} rad/s'
        if val['vpeak_max'] > 1.05 * self.vmax:
            return val, f'transition: cubic velocity peak {val["vpeak_max"]:.3f} > {1.05 * self.vmax:.3f} rad/s'
        if val['acc_max'] > 1.5 * self.amax:
            return val, f'transition: cubic acceleration {val["acc_max"]:.2f} > {1.5 * self.amax:.2f} rad/s2'
        return val, None

    def kinematic_ok(self, pos, tim, vel, cur, i_s):
        why = super().kinematic_ok(pos, tim, vel, cur, i_s)     # V4's checks (+ diagnostics), unchanged
        b = self._blend
        if b is None or b['last'] is None or b['last']['pos'] is not pos:
            return why
        L = b['last']
        val, why2 = self.transition_check(np.asarray(pos), np.asarray(tim), np.asarray(vel), L['n_keep'], L['t_splice'], L['T_d'])
        self.blog(ev='blend_check', t=time.monotonic(), cycle_ver=b['ver'], kind=self._diag_kind,
                  takeover_pos=b['q_s'], takeover_vel=b['v_s'], takeover_speed=float(np.abs(b['v_s']).max()),
                  T_d=b['T_d'], T_n=L['T_n'], q_v=b['q_v'], parent_decision=why, transition=val, transition_decision=why2)
        return why if why is not None else why2


def main():
    rclpy.init()
    n = StrategyBBlend()
    n.moving_now = False
    n.start()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    ex.spin()


if __name__ == '__main__':
    main()
