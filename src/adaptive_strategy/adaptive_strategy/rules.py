"""Drop rules as data -> a `guard(state, action) -> bool` for plan_state.apply.

strategy.yaml `drop_rules:` entries (labels written for yellow, Y/YC; mapped
to B/BC for blue):
  {zone: Y1, then_immediately: Y3}        # if Y1 is the FIRST drop, next must be Y3, no pick between
  {zone: YC1, requires_dropped: [Y1, Y3], requires_picked: [P1]}
  {stack: P3, requires_picked: [P5]}     # never pick P3 while P5 still has boxes (it crowds P3's approach)
  {pin_to_flip_face: [P1, P2]}           # these stacks feed towers: pick them onto the flip face
  {door_only_in: [Y5]}                   # door posts (vertical boxes in a rect zone) only here:
                                          # the lid needs an un-flip, which only works inside the castle
  {first_drop_in: [Y1, Y2]}              # the match's first drop must be one of these
  {zone: Y5, last: true, only_face_loaded: true}
                                          # last drop of the match; nothing but 'home' after it;
                                          # entered with only the dropping face loaded (36 cm corridor)
No ROS imports.
"""


def build_guard(model, rules):
    prefix = model.zone_label[0][0]

    def zi(label):
        lab = prefix + label[1:] if label[0] in 'YB' else label
        return model.zone_label.index(lab)

    labels = {s.label: i for i, s in enumerate(model.stacks)}
    seq, req, last, alone, first_ok, door_ok, pinned = [], [], [], [], set(), set(), set()
    pick_req = []           # (stack index, {stack indices that must be gone first})
    for r in rules or []:
        if 'stack' in r:
            pick_req.append((labels[r['stack']], {labels[x] for x in r.get('requires_picked', [])}))
            continue
        if 'pin_to_flip_face' in r:
            pinned |= {labels[x] for x in r['pin_to_flip_face']}
            continue
        if 'door_only_in' in r:
            door_ok |= {zi(x) for x in r['door_only_in']}
            continue
        if 'first_drop_in' in r:
            first_ok |= {zi(x) for x in r['first_drop_in']}
            continue
        z = zi(r['zone'])
        if 'then_immediately' in r:
            seq.append((z, zi(r['then_immediately'])))
        if 'requires_dropped' in r or 'requires_picked' in r:
            req.append((z, {zi(x) for x in r.get('requires_dropped', [])},
                        {labels[x] for x in r.get('requires_picked', [])}))
        if r.get('last'):
            last.append(z)
        if r.get('only_face_loaded'):
            alone.append(z)

    flip_face = model.flip_face

    def guard(s, a):
        kind = a[0]
        d = s.dropped
        # a door in progress (posts down, lid not yet) is atomic: flip, then lid
        if s.door_zone >= 0 and not s.door_used:
            if not (kind in ('flip', 'home') or (kind == 'drop' and a[1] == s.door_zone and a[2] == flip_face)):
                return False
        # a pending forced follow-up drop: only that drop (or a flip) is allowed
        for first, nxt in seq:
            if d and d[0] == first and nxt not in d:
                if not (kind in ('flip', 'home') or (kind == 'drop' and a[1] == nxt)):
                    return False
        # after a `last` zone only 'home' remains
        if any(z in d for z in last) and kind != 'home' and not (kind == 'pick' and a[1] in model.regrip):
            # ...except finishing a door built in that zone (posts down, lid to go)
            if not (s.door_zone in last and not s.door_used
                    and (kind == 'flip' or (kind == 'drop' and a[1] == s.door_zone))):
                return False
        if kind == 'pick' and a[1] in pinned and a[2] != flip_face:
            return False
        if kind == 'pick':
            for si, need in pick_req:
                # gone = picked by us or emptied by the opponent
                if a[1] == si and any(s.stacks[i] > 0 for i in need):
                    return False
        if kind == 'drop':
            z = a[1]
            if first_ok and not d and z not in first_ok:
                return False
            if z in alone and sum(s.faces) != s.faces[a[2]]:
                return False
            if door_ok and s.vert[a[2]] and model.zones[z].shape == 'rect' and z not in door_ok:
                return False
            for zone, need_d, need_p in req:
                # `requires_picked`: the stack is gone, whoever took it (us or the opponent)
                gone = {i for i in need_p if i in s.picked or s.stacks[i] <= 0}
                if z == zone and not (need_d <= set(d) and need_p <= gone):
                    return False
        return True

    return guard
