#!/usr/bin/env python3
"""Match panel for the table simulator (separate window, works in VNC).

Shows the clock, the score, both robots, every pick stack (P1..P10) and drop
zone (Y1..Y5 / B1..B5), and the last game events. Buttons start / reset the
match and send pick / drop goals on each face of our robot.

Reads the JSON published by sim_node on /sim/score. Needs python3-tk
(sudo apt install python3-tk).
"""

import json
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from table_sim_interfaces.action import FaceTool

try:
    import tkinter as tk
    from tkinter import ttk
except ImportError:  # pragma: no cover
    tk = None

BG = '#1e1f26'
FG = '#e8e8e8'
DIM = '#8a8d99'
YEL = '#f2c230'
BLU = '#5b8de0'
BOX = '#c99a5b'
EMPTY = '#3a3c46'


class PanelNode(Node):
    def __init__(self):
        super().__init__('sim_panel')
        self.state = None
        self.last_result = ''
        self.lock = threading.Lock()
        self.create_subscription(String, '/sim/score', self._on_state, 10)
        self.start_cli = self.create_client(Trigger, '/sim/start')
        self.reset_cli = self.create_client(Trigger, '/sim/reset')
        self.tool = ActionClient(self, FaceTool, '/face_tool')

    def _on_state(self, msg):
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.state = data

    def call(self, cli, label):
        if not cli.service_is_ready():
            self.last_result = f'{label}: service not available'
            return
        fut = cli.call_async(Trigger.Request())
        fut.add_done_callback(lambda f: setattr(self, 'last_result', f'{label}: {f.result().message}'))

    def face_action(self, face, mode):
        if not self.tool.server_is_ready():
            self.last_result = '/face_tool not available'
            return
        goal = FaceTool.Goal()
        goal.face, goal.mode = face, mode
        verb = 'pick' if mode == FaceTool.Goal.PICK else 'drop'
        self.last_result = f'{verb} face {face}: sent'

        def on_goal(f):
            gh = f.result()
            if not gh.accepted:
                self.last_result = f'{verb} face {face}: rejected'
                return
            gh.get_result_async().add_done_callback(on_result)

        def on_result(f):
            r = f.result().result
            self.last_result = f'{verb} face {face}: {"OK" if r.success else "FAILED"} - {r.message}'

        self.tool.send_goal_async(goal).add_done_callback(on_goal)


class Panel:
    def __init__(self, node: PanelNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Eurobot 2027 sim - match panel')
        self.root.configure(bg=BG)
        self.root.geometry('520x860')
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
        ttk.Button(btns, text='Start match', command=lambda: node.call(node.start_cli, 'start')).pack(side='left')
        ttk.Button(btns, text='Reset', command=lambda: node.call(node.reset_cli, 'reset')).pack(side='left', padx=6)
        tools = tk.Frame(self.root, bg=BG)
        tools.pack(fill='x', padx=12, pady=(0, 6))
        for f in range(3):
            col = tk.Frame(tools, bg=BG)
            col.pack(side='left', padx=(0, 10))
            tk.Label(col, text=f'face {f}', font=mid, fg=FG, bg=BG).pack()
            ttk.Button(col, text='Pick', command=lambda f=f: node.face_action(f, FaceTool.Goal.PICK)).pack(fill='x')
            ttk.Button(col, text='Drop', command=lambda f=f: node.face_action(f, FaceTool.Goal.DROP)).pack(fill='x')
        self.lbl_result = tk.Label(self.root, text='', font=small, fg=FG, bg=BG, anchor='w')
        self.lbl_result.pack(fill='x', padx=12)

        # ---- robots
        self.lbl_robots = tk.Label(self.root, text='', font=small, fg=FG, bg=BG, justify='left', anchor='w')
        self.lbl_robots.pack(fill='x', padx=12, pady=6)

        # ---- stacks and zones
        tk.Label(self.root, text='Pick stacks', font=mid, fg=FG, bg=BG, anchor='w').pack(fill='x', padx=12)
        self.stack_frame = tk.Frame(self.root, bg=BG)
        self.stack_frame.pack(fill='x', padx=12)
        tk.Label(self.root, text='Drop zones', font=mid, fg=FG, bg=BG, anchor='w').pack(fill='x', padx=12, pady=(8, 0))
        self.zone_frame = tk.Frame(self.root, bg=BG)
        self.zone_frame.pack(fill='x', padx=12)
        self.cells = {}

        # ---- events
        tk.Label(self.root, text='Events', font=mid, fg=FG, bg=BG, anchor='w').pack(fill='x', padx=12, pady=(8, 0))
        self.lbl_events = tk.Label(self.root, text='', font=small, fg=DIM, bg=BG, justify='left', anchor='nw')
        self.lbl_events.pack(fill='both', expand=True, padx=12)

        self.root.after(100, self.refresh)

    def _cell(self, frame, key, row, col, color):
        if key not in self.cells:
            w = tk.Canvas(frame, width=88, height=46, bg=BG, highlightthickness=0)
            w.grid(row=row, column=col, padx=2, pady=2)
            self.cells[key] = (w, color)
        return self.cells[key][0]

    def _draw_cell(self, canvas, label, n, cap, color):
        canvas.delete('all')
        canvas.create_rectangle(1, 1, 87, 45, outline=color, width=2)
        canvas.create_text(8, 12, text=label, anchor='w', fill=FG, font=('DejaVu Sans', 11, 'bold'))
        for k in range(cap):
            x0 = 8 + k * 26
            canvas.create_rectangle(x0, 26, x0 + 22, 40, fill=BOX if k < n else EMPTY, outline='')
        canvas.create_text(80, 12, text=str(n), anchor='e', fill=FG, font=('DejaVu Sans', 11))

    def refresh(self):
        with self.node.lock:
            s = self.node.state
        if s is not None:
            self._render(s)
        self.lbl_result.config(text=self.node.last_result)
        self.root.after(100, self.refresh)

    def _render(self, s):
        self.lbl_time.config(text=f"{s['time_left']:4.1f} s")
        state = 'MATCH OVER' if s['finished'] else ('running' if s['running'] else 'waiting for start')
        self.lbl_state.config(text=f"{state}\nt = {s['time']:.1f} s", fg=FG if s['running'] else DIM)
        y, b = s['score']['yellow'], s['score']['blue']
        self.lbl_y.config(text=f"YELLOW {y['total']}")
        self.lbl_b.config(text=f"{b['total']} BLUE")
        self.lbl_detail.config(text=(
            f"yellow: {y['boxes']} boxes, {y['full_stacks']} full stacks, end zone {y['end_zone']}\n"
            f"blue:   {b['boxes']} boxes, {b['full_stacks']} full stacks, end zone {b['end_zone']}"))
        lines = []
        for name in ('us', 'opp'):
            r = s['robots'][name]
            who = f"US ({r['team']})" if name == 'us' else f"OPP ({r['team']}, {r.get('behavior', '')})"
            lines.append(f"{who}")
            lines.append(f"  pose  x {r['x']:.3f}  y {r['y']:.3f}  th {r['th_deg']:6.1f} deg")
            lines.append(f"  faces {r['faces']}   {r['action'] or r.get('status', '')}")
        self.lbl_robots.config(text='\n'.join(lines))
        cap_stack = max([3] + list(s['stacks'].values()))
        for i, k in enumerate(sorted(s['stacks'], key=lambda k: int(k[1:]))):
            c = self._cell(self.stack_frame, k, i // 5, i % 5, '#888888')
            self._draw_cell(c, k, s['stacks'][k], cap_stack, '#888888')
        zones = sorted(s['zones'], key=lambda k: (k[0] != 'Y', int(k[1:])))
        for i, k in enumerate(zones):
            color = YEL if k.startswith('Y') else BLU
            c = self._cell(self.zone_frame, k, i // 5, i % 5, color)
            self._draw_cell(c, k, s['zones'][k], s['zone_capacity'], color)
        self.lbl_events.config(text='\n'.join(s['events'][-12:]))

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
