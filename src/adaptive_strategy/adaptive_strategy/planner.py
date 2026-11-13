"""Beam-search planner over PlanState (no ROS). Phase 3: static opponent.

plan(model, state, guard) -> (actions, value). Value of a state = points if the
robot went home right now (boxes still carried count 0, park bonus only if it
can still get home in time), so every plan is judged as a complete, parked
match; ties go to the earlier finish. Intermediate states are ranked for the
beam by that value plus the points still reachable from them (_potential): the door
while it is still possible, free towers, and carried boxes that still have somewhere to go.
"""
import heapq
from dataclasses import replace

from .plan_state import NONE, apply, legal_actions, points

# Reachable-points ranking (Max: door and towers first - many points for few boxes).
POT_DOOR_W = 1.5          # share of the door's value credited while the door is still possible
POT_TOWER_W = 0.7         # share of a tower's value per free (not blocked) circle, as time allows
DOOR_MIN_TIME = 15.0      # s left needed for the door (pick + flip + posts + un-flip + lid + home)
TOWER_TIME = 4.0          # s per tower


def _potential(m, s, credit_h, credit_v):
    """Optimistic points still reachable from s (beam ranking only, never a plan's value)."""
    sc, ff = m.scoring, m.flip_face
    box = sc.get('box_in_zone', 0)
    t_left = m.duration - s.t
    door_zi = getattr(m, '_door_zi', None)
    if door_zi is None:
        door_zi = next((i for i, z in enumerate(m.zones) if z.shape == 'rect' and z.label.endswith('5')), -1)
        m._door_zi = door_zi
    door_open = (door_zi >= 0 and not s.door_used and (s.zbox[door_zi] == 0 or s.door_zone == door_zi))
    # room left for horizontal boxes: unbuilt walls (not the door zone while the door is open) + discards
    room = 0
    for i, z in enumerate(m.zones):
        if z.shape != 'rect' or (door_open and i == door_zi):
            continue
        if s.zkind[i] == NONE:
            room += max(0, z.capacity - s.zbox[i])
    for d in getattr(m, 'discard_node', {}):
        room += max(0, m.field.discard_max_boxes - max(s.zbox[d], m.zones[d].capacity))
    carried_h = sum(n for f, (n, v) in enumerate(zip(s.faces, s.vert)) if not v and f != ff)
    stranded = max(0, carried_h - room)
    pot = credit_h * (carried_h - stranded)
    blocked = set()
    for d in s.discarded:
        blocked |= m.discard_blocks.get(d, set())
    free = sum(1 for i, z in enumerate(m.zones) if z.shape == 'circle' and s.zbox[i] == 0 and i not in blocked)
    n_t = min(free, max(0, int((t_left - 6.0) / TOWER_TIME)))
    # standing boxes are worth more only as towers still to build: flipping early must not look good
    # by itself (Max: P8 flipped at 10 s, un-flipped at 20 s for a wall)
    # standing boxes earn the same as lying ones: the towers are already credited below (free
    # circles), so a flip must never look good by itself (Max: flips drove it into the castle early)
    pot += credit_h * sum(n for n, v in zip(s.faces, s.vert) if v)
    # door: still possible, time for it, nothing stranded on the other faces (Y5 is dropped with the flip
    # face only), and 3 boxes for it on the flip face or in a stack
    if (door_open and stranded == 0 and t_left > DOOR_MIN_TIME
            and (s.faces[ff] >= 3 or any(c >= 3 for c in s.stacks))):
        pot += POT_DOOR_W * (3 * box + sc.get('door_bonus', 0))
    pot += POT_TOWER_W * n_t * (box + sc.get('tower_bonus', 0))
    return pot


def _home_value(m, s, guard):
    h = apply(m, s, ('home',), guard)
    if h is None:
        return None, None
    return points(m, h), h


def plan(m, s0, guard=None, beam=150, depth=15, credit_h=2.0, credit_v=4.0, time_weight=0.02, rank_time_weight=0.0,
         deadline=None, collect=None, n_cand=12, per_first=3):
    """Return (list of actions, value). Empty list = go home / nothing better."""
    import time as _t
    best_val, best_plan = -1e9, []
    hv, h = _home_value(m, s0, guard)
    if hv is not None:
        best_val, best_plan = hv + h.bonus - h.risk_cost - time_weight * h.t, [('home',)]
    heap, n, firsts = [], 0, {}     # global top-n plus top-`per_first` for EACH first action
    frontier = [(0.0, s0, ())]
    seen = {s0.key(): s0.t}
    for _ in range(depth):
        cand = []
        for _, s, path in frontier:
            for a, ns in legal_actions(m, s, guard):
                if a[0] == 'home':
                    continue
                k = ns.key()
                t_prev = seen.get(k)
                if t_prev is not None and t_prev <= ns.t:
                    continue                      # dominated: same state, not faster
                seen[k] = ns.t
                hv, h = _home_value(m, ns, guard)
                if hv is None:
                    continue                      # cannot finish -> dead end
                p = path + (a,)
                val = hv + h.bonus - h.risk_cost - time_weight * h.t
                if val > best_val:
                    best_val, best_plan = val, list(p) + [('home',)]
                if collect is not None:
                    n += 1
                    item = (val, n, p)
                    if len(heap) < n_cand:
                        heapq.heappush(heap, item)
                    elif val > heap[0][0]:
                        heapq.heapreplace(heap, item)
                    fh = firsts.setdefault(p[0], [])
                    if len(fh) < per_first:
                        heapq.heappush(fh, item)
                    elif val > fh[0][0]:
                        heapq.heapreplace(fh, item)
                pot = _potential(m, ns, credit_h, credit_v)
                rank = val + pot - rank_time_weight * ns.t     # earlier = more time left to score
                cand.append((rank, ns, p))
        if not cand:
            break
        cand.sort(key=lambda c: -c[0])
        # stratified beam: at most `cap` states per first action, so one first move cannot crowd out the
        # others (its follow-ups may be better once the opponent scenarios are replayed)
        cap, per, frontier, rest = max(10, beam // 6), {}, [], []
        for c in cand:
            k = c[2][0]
            if per.get(k, 0) < cap and len(frontier) < beam:
                per[k] = per.get(k, 0) + 1
                frontier.append(c)
            else:
                rest.append(c)
        frontier += rest[:max(0, beam - len(frontier))]
        if deadline is not None and _t.time() > deadline:
            break
    if collect is not None:
        seen_p, items = set(), list(heap) + [it for fh in firsts.values() for it in fh]
        for v, _, p in sorted(items, key=lambda it: -it[0]):
            if p not in seen_p:
                seen_p.add(p)
                collect.append((v, list(p) + [('home',)]))
    return best_plan, best_val


def simulate_open_loop(m, s0, acts, scenario, guard=None):
    """Run a fixed plan against one opponent scenario. A pick whose stack the
    opponent emptied first fails (time lost, nothing loaded); actions that
    become illegal are skipped. Returns (points, finish_time, failed_picks)."""
    s, fails = s0, 0
    for a in acts:
        if a[0] == 'pick':
            si = a[1]
            node = m.stack_nodes[si]
            arrive = s.t + m.travel.time(s.node, node)
            if arrive + m.engage >= scenario.take[si] and s.stacks[si] > 0:
                s = replace(s, t=arrive + 0.5, node=node, last='pick')
                fails += 1
                continue
        ns = apply(m, s, a, guard, check_home=False, check_scen=False)
        if ns is None:
            continue
        if ns.t > m.duration:
            break
        s = ns
    return points(m, s), s.t, fails, s.bonus - s.risk_cost


def plan_robust(m, s0, guard, scen, threshold=50.0, beam=150, depth=15, n_cand=12,
                incumbent=None, hysteresis=2.0, p_enough=1.0, **kw):
    """Beam search on the probability-weighted opponent model, then rank the best
    complete plans lexicographically over the scenarios: P(points >= threshold),
    E[points], expected finish time (earlier better), expected failed picks.
    Returns (plan, info)."""
    cands = []
    tw = kw.get('time_weight', 0.02)      # the SAME time cost that generated the candidates
    best, val = plan(m, s0, guard, beam, depth, collect=cands, n_cand=n_cand, **kw)
    if incumbent:        # the previous plan's remainder competes too, with a small head start
        inc = [a for a in incumbent if a[0] != 'home'] + [('home',)]
        cands.append((val, inc))
    if not cands:
        return best, {'value': val}
    ranked = []
    for v, acts in cands:
        res = [simulate_open_loop(m, s0, acts, sc, guard) for sc in scen.scenarios]
        w = [sc.prob for sc in scen.scenarios]
        p_ok = sum(wi for wi, r in zip(w, res) if r[0] >= threshold)
        e_pts = sum(wi * (r[0] + r[3]) for wi, r in zip(w, res))   # own points + denial value
        is_inc = bool(incumbent) and acts is cands[-1][1]
        if is_inc:
            e_pts += hysteresis
        e_t = sum(wi * r[1] for wi, r in zip(w, res))
        risk = sum(wi * r[2] for wi, r in zip(w, res))
        # safe enough (p_ok >= p_enough) ties: then expected points decide
        ranked.append(((min(round(p_ok, 1), p_enough), e_pts - tw * e_t, -risk), acts,
                       (p_ok, e_pts, e_t, risk, is_inc)))
    ranked.sort(key=lambda r: r[0], reverse=True)
    top = ranked[0]
    return top[1], {'value': val, 'p_ok': top[2][0], 'e_pts': top[2][1], 'e_t': top[2][2],
                    'risk': top[2][3], 'n': len(ranked), 'kept_incumbent': top[2][4]}
