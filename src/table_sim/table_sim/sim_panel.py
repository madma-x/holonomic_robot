#!/usr/bin/env python3
"""Match panel for the table simulator (separate window, works in VNC).

Spectator panel: clock, score, our belief about each pick stack (vs. the
true count, for comparison - belief is what the strategy actually sees,
true is sim-only ground truth shown here for debugging), the tracked
opponent position estimate, and the event log. No manual pick/drop controls
or stack/zone grids - adaptive_strategy's strategy_node drives the robot
autonomously now, this panel just shows what it's doing and how well its
beliefs track reality.

Reads /sim/score (sim_node, ground truth) and /strategy/status
(adaptive_strategy's strategy_node, belief - absent if strategy_node isn't
running). Needs python3-tk (sudo apt install python3-tk).
"""

import json
import math
import threading

import rclpy
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

# Same robot design on both sides (CLAUDE.md: equilateral triangle, 340mm
# per face) - side/sqrt(3) is the vertex-to-centre circumradius, the same
# constant already used throughout nav2_params_sim.yaml's own inflation
# comments.
ROBOT_CIRCUMRADIUS_M = 0.34 / math.sqrt(3.0)
INFLATION_PAD_M = 0.08
# label -> assumed opponent physical radius (m). "Same as us" matches this
# robot's own circumradius - the only one actually validated so far this
# session; the other two are placeholders for a visibly smaller/larger
# opponent until real numbers are known for other teams' robots.
OPPONENT_SIZE_PRESETS = [
    ('Small', 0.15),
    ('Same as us', ROBOT_CIRCUMRADIUS_M),
    ('Large', 0.25),
]

try:
    import tkinter as tk
except ImportError:  # pragma: no cover
    tk = None

BG = '#1e1f26'
FG = '#e8e8e8'
DIM = '#8a8d99'
YEL = '#f2c230'
BLU = '#5b8de0'
GOOD = '#5bc99a'
BAD = '#d9684f'


class PanelNode(Node):
    def __init__(self):
        super().__init__('sim_panel')
        self.state = None
        self.strategy = None
        self.last_result = ''
        self.lock = threading.Lock()
        self.create_subscription(String, '/sim/score', self._on_state, 10)
        self.create_subscription(String, '/strategy/status', self._on_strategy, 10)
        self.start_cli = self.create_client(Trigger, '/sim/start')
        self.reset_cli = self.create_client(Trigger, '/sim/reset')
        self.infinite_time_cli = self.create_client(SetBool, '/sim/set_infinite_time')

        # Opponent size: one operator choice, pushed out to every mechanism
        # that currently hardcodes an assumed opponent size of its own -
        # motion_controller's mechanism 2 disk (opponent_radius_m) and
        # local_costmap's inflation_layer (inflation_radius). NOT
        # global_costmap's inflation_layer, even though it also inflates
        # the opponent: that layer is a single shared post-process step
        # over the WHOLE merged grid, also inflating static_layer's walls
        # and custom_objects_layer's stacks/zones - changing its radius to
        # resize the opponent's margin resizes wall/box margins right along
        # with it (confirmed live: wall/box inflation visibly changed when
        # picking an opponent size). local_costmap has no such problem -
        # opponent_layer is its only obstacle source, so its inflation_layer
        # is already opponent-only. Each node owns its own parameters, so
        # this is 2 separate SetParameters calls, not 1.
        self.opponent_size_m = ROBOT_CIRCUMRADIUS_M
        self.controller_param_cli = self.create_client(SetParameters, '/controller_server/set_parameters')
        self.local_costmap_param_cli = self.create_client(
            SetParameters, '/local_costmap/local_costmap/set_parameters')

    def _on_state(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.state = data

    def _on_strategy(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.strategy = data

    def call(self, cli, label):
        if not cli.service_is_ready():
            self.last_result = f'{label}: service not available'
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: setattr(self, 'last_result', f'{label}: {f.result().message}'))

    def set_infinite_time(self, value):
        if not self.infinite_time_cli.service_is_ready():
            self.last_result = 'infinite time: service not available'
            return
        fut = self.infinite_time_cli.call_async(SetBool.Request(data=value))
        fut.add_done_callback(lambda f: setattr(self, 'last_result', f'infinite time: {f.result().message}'))

    def _set_remote_param(self, cli, name, value, label, results):
        results[label] = 'service not available' if not cli.service_is_ready() else '...'
        self._render_opponent_size_result(results)
        if not cli.service_is_ready():
            return
        req = SetParameters.Request(parameters=[Parameter(
            name=name,
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))])

        def _done(fut, label=label):
            try:
                ok = all(r.successful for r in fut.result().results)
            except Exception as e:  # noqa: BLE001 - surfacing to the panel, not re-raising
                results[label] = f'error ({e})'
                self._render_opponent_size_result(results)
                return
            results[label] = 'ok' if ok else 'rejected'
            self._render_opponent_size_result(results)

        cli.call_async(req).add_done_callback(_done)

    def _render_opponent_size_result(self, results):
        self.last_result = 'opponent size %.3fm -> %s' % (
            self.opponent_size_m, ' | '.join(f'{k}: {v}' for k, v in results.items()))

    def set_opponent_size(self, size_m):
        """One operator-facing number (assumed opponent physical radius) ->
        every mechanism that has its own, independently-tuned notion of
        opponent size WITHOUT also touching something else's margin.
        Mechanism 1 (motion_controller's footprint clearance,
        avoidance_min_clearance_m/avoidance_caution_margin_m) is
        deliberately not included - it's robot-footprint-relative
        (edge-to-edge against whatever the costmap shows), not a function
        of the opponent's assumed size. global_costmap's inflation_layer is
        also deliberately not included (see __init__'s comment) - it's a
        single layer shared with walls/boxes, not opponent-specific."""
        self.opponent_size_m = size_m
        inflation_radius = size_m + ROBOT_CIRCUMRADIUS_M + INFLATION_PAD_M
        results = {}
        self._set_remote_param(
            self.controller_param_cli, 'FollowPath.opponent_radius_m', size_m,
            'opponent_radius_m', results)
        self._set_remote_param(
            self.local_costmap_param_cli, 'inflation_layer.inflation_radius', inflation_radius,
            'local inflation_radius', results)


class Panel:
    def __init__(self, node: PanelNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Eurobot 2027 sim - match panel')
        self.root.configure(bg=BG)
        self.root.geometry('480x760')
        big = ('DejaVu Sans', 30, 'bold')
        mid = ('DejaVu Sans', 13, 'bold')
        small = ('DejaVu Sans Mono', 11)
        self.small = small

        # ---- clock and score
        top = tk.Frame(self.root, bg=BG)
        top.pack(fill='x', padx=12, pady=(10, 4))
        self.lbl_time = tk.Label(top, text='--.-', font=big, fg=FG, bg=BG)
        self.lbl_time.pack(side='left')
        self.lbl_state = tk.Label(top, text='waiting', font=mid, fg=DIM, bg=BG)
        self.lbl_state.pack(side='left', padx=12)
        sc = tk.Frame(self.root, bg=BG)
        sc.pack(fill='x', padx=12)
        self.lbl_y = tk.Label(sc, text='YELLOW 0', font=big, fg=YEL, bg=BG)
        self.lbl_y.pack(side='left')
        self.lbl_b = tk.Label(sc, text='0 BLUE', font=big, fg=BLU, bg=BG)
        self.lbl_b.pack(side='right')
        self.lbl_detail = tk.Label(self.root, text='', font=small, fg=DIM, bg=BG, justify='left')
        self.lbl_detail.pack(fill='x', padx=12)

        # ---- buttons
        btns = tk.Frame(self.root, bg=BG)
        btns.pack(fill='x', padx=12, pady=6)
        from tkinter import ttk
        ttk.Button(btns, text='Start match', command=lambda: node.call(node.start_cli, 'start')).pack(side='left')
        ttk.Button(btns, text='Reset', command=lambda: node.call(node.reset_cli, 'reset')).pack(side='left', padx=6)
        self.infinite_time_wanted = False
        self.btn_infinite = ttk.Button(btns, text='∞ Time: OFF', command=self._toggle_infinite_time)
        self.btn_infinite.pack(side='left', padx=6)
        self.lbl_result = tk.Label(self.root, text='', font=small, fg=FG, bg=BG, anchor='w')
        self.lbl_result.pack(fill='x', padx=12)

        # ---- opponent size (choose before the match, drives mechanism 2's
        # disk + both costmaps' inflation at once - see PanelNode.set_opponent_size)
        osize = tk.Frame(self.root, bg=BG)
        osize.pack(fill='x', padx=12, pady=(0, 6))
        tk.Label(osize, text='Opponent size:', font=mid, fg=FG, bg=BG).pack(side='left')
        for label, size_m in OPPONENT_SIZE_PRESETS:
            ttk.Button(
                osize, text=f'{label} ({size_m:.2f}m)',
                command=lambda s=size_m: node.set_opponent_size(s)).pack(side='left', padx=4)

        # ---- strategy state + opponent belief
        self.lbl_strategy = tk.Label(self.root, text='', font=small, fg=FG, bg=BG, justify='left', anchor='w')
        self.lbl_strategy.pack(fill='x', padx=12, pady=6)

        # ---- stack beliefs (ours vs. true, for debugging)
        tk.Label(self.root, text='Pick-stack belief (ours / true)', font=mid, fg=FG, bg=BG,
                anchor='w').pack(fill='x', padx=12)
        self.lbl_belief = tk.Label(self.root, text='', font=small, fg=FG, bg=BG, justify='left', anchor='nw')
        self.lbl_belief.pack(fill='x', padx=12, pady=(0, 6))

        # ---- events
        tk.Label(self.root, text='Events', font=mid, fg=FG, bg=BG, anchor='w').pack(fill='x', padx=12, pady=(8, 0))
        self.lbl_events = tk.Label(self.root, text='', font=small, fg=DIM, bg=BG, justify='left', anchor='nw')
        self.lbl_events.pack(fill='both', expand=True, padx=12)

        self.root.after(100, self.refresh)

    def _toggle_infinite_time(self):
        with self.node.lock:
            current = bool(self.node.state['infinite_time']) if self.node.state else False
        self.node.set_infinite_time(not current)

    def refresh(self):
        with self.node.lock:
            s, strat = self.node.state, self.node.strategy
        if s is not None:
            self._render(s, strat)
        self.lbl_result.config(text=self.node.last_result)
        self.root.after(100, self.refresh)

    def _render(self, s, strat):
        infinite = bool(s.get('infinite_time'))
        self.lbl_time.config(text=f"{s['time_left']:4.1f} s" + (' (∞)' if infinite else ''))
        self.btn_infinite.config(text=f"∞ Time: {'ON' if infinite else 'OFF'}")
        state = 'MATCH OVER' if s['finished'] else ('running' if s['running'] else 'waiting for start')
        self.lbl_state.config(text=f"{state}\nt = {s['time']:.1f} s", fg=FG if s['running'] else DIM)
        y, b = s['score']['yellow'], s['score']['blue']
        self.lbl_y.config(text=f"YELLOW {y['total']}")
        self.lbl_b.config(text=f"{b['total']} BLUE")
        self.lbl_detail.config(text=(
            f"yellow: {y['boxes']} boxes, {y['walls']}W {y['towers']}T {y['doors']}D, end zone {y['end_zone']}\n"
            f"blue:   {b['boxes']} boxes, {b['walls']}W {b['towers']}T {b['doors']}D, end zone {b['end_zone']}"))

        if strat is None:
            self.lbl_strategy.config(text='strategy_node: not running (no /strategy/status)')
            self.lbl_belief.config(text='')
        else:
            opp = strat.get('opponent')
            if opp:
                opp_txt = (f"x {opp['x']:.2f} y {opp['y']:.2f} sigma {opp['sigma']:.2f} "
                          f"{'VISIBLE' if opp['visible'] else 'last seen'}")
            else:
                opp_txt = 'never seen'
            self.lbl_strategy.config(text=(
                f"strategy: {strat['state']} ({strat['mode']})  target {strat.get('target')}\n"
                f"faces {strat['faces']}\n"
                f"tracked opponent: {opp_txt}"))
            true_stacks = s.get('stacks', {})
            lines = []
            for label in sorted(strat['belief'], key=lambda k: int(''.join(c for c in k if c.isdigit()) or 0)):
                believed = strat['belief'][label]
                true = true_stacks.get(label)
                mismatch = true is not None and abs(believed - true) >= 1.0
                mark = ' !' if mismatch else ''
                lines.append(f"  {label:4s} {believed:4.1f} / {true if true is not None else '?'}{mark}")
            self.lbl_belief.config(text='\n'.join(lines))

        self.lbl_events.config(text='\n'.join(s['events'][-14:]))

    def run(self):
        self.root.mainloop()


def main(args=None):
    if tk is None:
        print('sim_panel needs tkinter: sudo apt install python3-tk')
        return
    rclpy.init(args=args)
    node = PanelNode()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        Panel(node).run()
    except tk.TclError as e:
        print(f'sim_panel: cannot open a window ({e}). Is DISPLAY set (VNC desktop)?')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
