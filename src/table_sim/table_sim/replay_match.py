"""Step through a recorded match, checking the angle asked for at every
pick and drop.

Stage 2 of the two-stage replay: reads the JSON written by bag2frames (or by
headless --frames), so it needs no ROS and runs anywhere matplotlib does.

    python3 -m table_sim.replay_match match.json
    python3 -m table_sim.replay_match match.json --video out.mp4
    python3 -m table_sim.replay_match match.json --list      # just the steps

Controls
    n / p        next / previous STEP - a step is one goal: the frames over
                 which (target, stage) stays the same. This is the one that
                 matters: it walks the match pick by pick, drop by drop.
    e            jump to the END of this step - the moment the pick/drop
                 fires, which is where the heading has to actually be right
    -> / <-      single frame            PgUp/PgDn   10 frames
    space        play / pause            home/end    first / last frame
    q            quit

The overlay draws the pose being driven to as a dashed robot ghost with an
arrow along the REQUESTED heading (cyan = the coarse Nav2 goal, green = the
final docking pose, which is never sent to Nav2 and so is invisible in RViz).
The panel prints the requested yaw and the live error against it, flagged
OK/OFF against the +-10 deg tolerance.
"""

import argparse
import json
import os
import sys

import yaml

from .field import Field, default_field_file
from .render import TableRenderer


def _default_sim_file():
    here = os.path.dirname(os.path.abspath(__file__))
    cands = [os.path.join(here, '..', 'config', 'sim.yaml')]
    try:
        from ament_index_python.packages import get_package_share_directory
        cands.insert(0, os.path.join(get_package_share_directory('table_sim'),
                                     'config', 'sim.yaml'))
    except Exception:  # noqa: BLE001 - no ament outside the container, fine
        pass
    for c in cands:
        if os.path.exists(c):
            return os.path.abspath(c)
    raise FileNotFoundError('sim.yaml not found')


def build_models(sim_cfg):
    """The two RobotModels, without a World. world.py imports no ROS, so this
    works on a bare python install."""
    from .world import RobotModel
    rob = dict(sim_cfg.get('robot', {}))
    opp = dict(rob)
    opp.update(sim_cfg.get('opponent_robot', {}) or {})
    return {'us': RobotModel(rob), 'opp': RobotModel(opp)}


def steps(frames):
    """Split the match into one entry per goal: a run of frames sharing the
    same (target, stage). Returns [(first_index, last_index, label), ...]."""
    out = []
    prev = None
    for i, fr in enumerate(frames):
        key = (fr.get('target'), fr.get('stage'), fr.get('us_state'))
        if key != prev:
            tgt, stage, state = key
            label = f"{state or '-'}"
            if tgt:
                label += f" {tgt}"
            if stage:
                label += f" [{stage}]"
            req = fr.get('req_yaw_deg')
            if req is not None:
                label += f"  want {req:+.0f} deg"
            if out:
                out[-1][1] = i - 1
            out.append([i, i, label])
            prev = key
    return [tuple(s) for s in out]


def print_steps(frames, tol=10.0):
    """One line per goal, flagged on the heading error AT ARRIVAL.

    The error at the start of a step is just "we haven't driven there yet"
    and is almost always large - the number that means anything is the one
    at the last frame of the step, i.e. the heading the robot actually had
    when the pick/drop fired.
    """
    bad = 0
    for i, j, label in steps(frames):
        fr, end = frames[i], frames[j]
        err = end.get('yaw_err_deg')
        if err is None or end.get('req_yaw_deg') is None:
            flag = ''
        elif abs(err) <= tol:
            flag = f'   arrive {err:+5.1f}  OK'
        else:
            flag = f'   arrive {err:+5.1f}  OFF'
            bad += 1
        print(f"  t={fr['t']:6.1f}s  frames {i:5d}-{j:<5d} {label}{flag}")
    print(f"\n  {bad} step(s) ended outside +-{tol:.0f} deg of the requested heading")


class Replay:
    def __init__(self, data, field, models, tol_deg=10.0):
        self.frames = data['frames']
        self.steps = steps(self.frames)
        self.tol = tol_deg
        self.i = 0
        self.playing = False
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig = plt.figure(figsize=(14.0, 7.6), dpi=100)
        ax = self.fig.add_axes([0.02, 0.08, 0.66, 0.86])
        side = self.fig.add_axes([0.70, 0.08, 0.29, 0.86])
        self.r = TableRenderer(field, models, team_us=data.get('team', 'yellow'),
                               lidar_range=data.get('lidar_range', 2.8),
                               opponent_label=data.get('opponent', '')).attach(ax, side)
        self.status = self.fig.text(0.02, 0.025, '', family='monospace', fontsize=9)
        self.fig.canvas.mpl_connect('key_press_event', self._on_key)
        self.timer = self.fig.canvas.new_timer(interval=50)
        self.timer.add_callback(self._on_tick)

    # ------------------------------------------------------------ control
    def _step_index(self):
        """Which step the current frame falls in."""
        k = 0
        for j, s in enumerate(self.steps):
            if s[0] <= self.i:
                k = j
            else:
                break
        return k

    def goto(self, i):
        self.i = max(0, min(int(i), len(self.frames) - 1))
        self.draw()

    def _on_key(self, ev):
        k = ev.key
        if k == 'q':
            self.plt.close(self.fig)
        elif k == ' ':
            self.playing = not self.playing
            (self.timer.start if self.playing else self.timer.stop)()
        elif k == 'right':
            self.goto(self.i + 1)
        elif k == 'left':
            self.goto(self.i - 1)
        elif k == 'pageup':
            self.goto(self.i + 10)
        elif k == 'pagedown':
            self.goto(self.i - 10)
        elif k == 'home':
            self.goto(0)
        elif k == 'end':
            self.goto(len(self.frames) - 1)
        elif k == 'n':
            j = self._step_index()
            if j + 1 < len(self.steps):
                self.goto(self.steps[j + 1][0])
        elif k == 'p':
            j = self._step_index()
            # first press rewinds to this step's start, second goes back one
            self.goto(self.steps[j - 1][0] if (self.i == self.steps[j][0] and j)
                      else self.steps[j][0])
        elif k == 'e':
            # jump to the END of this step: the moment the pick/drop fires,
            # which is where the heading actually has to be right
            self.goto(self.steps[self._step_index()][1])

    def _on_tick(self):
        if self.i >= len(self.frames) - 1:
            self.playing = False
            self.timer.stop()
            return
        self.goto(self.i + 1)

    # ------------------------------------------------------------- draw
    def draw(self):
        fr = self.frames[self.i]
        self.r.seed_trails(self.frames[:self.i])
        self.r.draw(fr)
        j = self._step_index()
        err = fr.get('yaw_err_deg')
        req = fr.get('req_yaw_deg')
        if req is None:
            angle = 'no goal'
        elif err is None:
            angle = f'want {req:+6.1f} deg'
        else:
            ok = 'OK ' if abs(err) <= self.tol else 'OFF'
            angle = f'want {req:+6.1f} deg   err {err:+6.1f}   {ok}'
        self.status.set_text(
            f'frame {self.i + 1}/{len(self.frames)}   t={fr["t"]:6.1f}s   '
            f'step {j + 1}/{len(self.steps)}: {self.steps[j][2] if self.steps else "-"}   '
            f'|   {angle}   |   n/p step  e end  <-/-> frame  space play  q quit')
        self.fig.canvas.draw_idle()

    def run(self):
        self.draw()
        self.plt.show()

    def save(self, path, fps=20):
        from matplotlib import animation

        def upd(i):
            self.i = i
            self.r.seed_trails(self.frames[:i])
            return self.r.draw(self.frames[i])
        anim = animation.FuncAnimation(self.fig, upd, frames=len(self.frames),
                                       interval=1000 / fps, blit=False)
        if path.endswith('.gif'):
            anim.save(path, writer=animation.PillowWriter(fps=fps))
        else:
            anim.save(path, writer=animation.FFMpegWriter(
                fps=fps, bitrate=2500, extra_args=['-pix_fmt', 'yuv420p']))
        self.plt.close(self.fig)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('frames_json', help='output of bag2frames')
    ap.add_argument('--field', default=None)
    ap.add_argument('--sim', default=None)
    ap.add_argument('--video', default=None, help='render to .mp4/.gif instead of opening a window')
    ap.add_argument('--fps', type=int, default=20)
    ap.add_argument('--tol-deg', type=float, default=10.0, help='heading tolerance to flag against')
    ap.add_argument('--list', action='store_true', help='print the steps and exit')
    args = ap.parse_args(argv)

    with open(args.frames_json) as f:
        data = json.load(f)
    if not data.get('frames'):
        print('no frames in that file', file=sys.stderr)
        return 1

    if args.list:
        print_steps(data['frames'], args.tol_deg)
        return 0

    import matplotlib
    if args.video:
        matplotlib.use('Agg')

    field = Field(args.field or default_field_file())
    with open(args.sim or _default_sim_file()) as f:
        sim_cfg = yaml.safe_load(f)
    data.setdefault('lidar_range', float(sim_cfg.get('lidar', {}).get('detect_range', 2.8)))

    rp = Replay(data, field, build_models(sim_cfg), tol_deg=args.tol_deg)
    if args.video:
        rp.save(args.video, fps=args.fps)
        print(f'-> {args.video}')
    else:
        rp.run()
    return 0


if __name__ == '__main__':
    sys.exit(main())
