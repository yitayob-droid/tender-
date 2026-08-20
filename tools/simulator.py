"""
Desktop simulator for mosaic_masters.py.

It fakes the SPIKE Prime Python API (motors, gyro, colour sensor, the
runloop) with a simple kinematic model of the robot on the mat, then
imports the real program and runs it.  Nothing here goes on the hub -
it exists so you can check a change to the mission before you spend
table time on it.

    python3 tools/simulator.py              # run MODE from the program
    python3 tools/simulator.py TEST_DRIVE   # override the mode
    python3 tools/simulator.py MISSION -v   # print a driving trace
"""

import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

# --- physical truth of the simulated robot ---------------------------
WHEEL_D = 5.6
TRACK = 12.0
MAX_DEG_S = 900.0
CM_PER_DEG = math.pi * WHEEL_D / 360.0

# small deliberate imperfections so the gyro control has something to do
WHEEL_SCALE_L = 1.00
WHEEL_SCALE_R = 0.99          # right wheel runs 1 % slow -> robot pulls
HEADING_NOISE_DEG_S = 0.0

VERBOSE = "-v" in sys.argv
LEGACY = "--legacy" in sys.argv          # exercise the SPIKE App 2 path
NO_BLOCKS = "--no-blocks" in sys.argv    # every grab misses


class World:
    def __init__(self):
        self.t_ms = 0.0
        self.x = 16.0            # start pose, overwritten by the program
        self.y = 22.0
        self.h = 90.0            # clockwise-positive compass heading
        self.yaw_ref = self.h
        self.motors = {}
        for letter in "ABCDEF":
            self.motors[letter] = {"pos": 0.0, "cmd": 0.0, "vel": 0.0,
                                   "lo": None, "hi": None}
        self.trace = []
        self.carrying = 0
        self.placed = []
        self._roles_done = False
        self._closed_flag = False
        self._prev_gpos = 0.0

        # ground truth mosaic used by the fake colour sensor
        # ground truth read off the mat photos: 4 across, 3 deep
        self.pattern = [["blue", "yellow", "green", "yellow"],
                        ["yellow", "blue", "white", "green"],
                        ["blue", "yellow", "green", "yellow"]]

    # -- wiring -------------------------------------------------------
    def prog(self):
        return sys.modules.get("mosaic_masters")

    def ports(self):
        """(left, right, lift, grabber) letters, taken from the program
        so re-wiring the robot does not need a simulator edit."""
        p = self.prog()
        if p is None or not hasattr(p, "PORT_LEFT"):
            return ("A", "E", "C", "B")
        return (p.PORT_LEFT, p.PORT_RIGHT, p.PORT_LIFT, p.PORT_GRAB)

    def _apply_roles(self):
        if self._roles_done or self.prog() is None:
            return
        _, _, lift, grab = self.ports()
        self.motors[lift]["lo"], self.motors[lift]["hi"] = 0.0, 170.0
        self.motors[grab]["lo"], self.motors[grab]["hi"] = 0.0, 100.0
        self._roles_done = True

    # -- integration --------------------------------------------------
    def step(self, ms):
        dt = ms / 1000.0
        if dt <= 0:
            dt = 0.001
        self.t_ms += ms

        self._apply_roles()
        self._update_blocks()
        for letter, m in self.motors.items():
            vel = m["cmd"]
            pos = m["pos"] + vel * dt
            if m["lo"] is not None:
                if pos < m["lo"]:
                    pos, vel = m["lo"], 0.0
                elif pos > m["hi"]:
                    pos, vel = m["hi"], 0.0
            m["pos"] = pos
            m["vel"] = vel

        # differential drive.  The program feeds the two sides opposite
        # signs (LEFT_SIGN / RIGHT_SIGN), undo that here.
        left, right, _, _ = self.ports()
        vl = self.motors[left]["vel"] * CM_PER_DEG * WHEEL_SCALE_L
        vr = -self.motors[right]["vel"] * CM_PER_DEG * WHEEL_SCALE_R
        v = (vl + vr) / 2.0
        omega = math.degrees((vl - vr) / TRACK)      # clockwise positive

        self.h = (self.h + omega * dt) % 360.0
        r = math.radians(self.h)
        self.x += math.sin(r) * v * dt
        self.y += math.cos(r) * v * dt

        if VERBOSE and int(self.t_ms) % 500 < ms:
            self.trace.append((round(self.t_ms / 1000.0, 1),
                               round(self.x, 1), round(self.y, 1),
                               round(self.h, 1)))

    def _update_blocks(self):
        """Model the magazine: blocks go in at a depot when the jaws
        stall on one, and come out at the plate when the jaws open past
        the release position with the lift down."""
        prog = self.prog()
        if prog is None or not hasattr(prog, "DEPOTS"):
            return
        g = prog.__dict__
        _, _, lift, grab = self.ports()
        gpos = self.motors[grab]["pos"]
        lpos = self.motors[lift]["pos"]
        size = g.get("MAGAZINE_SIZE", 1)

        # A depot is not a point: the blocks run in a line from just in
        # front of the stand position into the depot.  Measure the
        # distance to that line, not to the stand.
        near_depot = False
        depth = (g.get("DEPOT_APPROACH_CM", 9.0)
                 + g.get("DEPOT_BLOCK_PITCH_CM", 6.0) * (size - 1) + 4.0)
        for options in prog.DEPOTS.values():
            for opt in options:
                dx, dy = opt["stand"]
                fr = math.radians(opt["face"])
                fx, fy = math.sin(fr), math.cos(fr)
                # project the robot onto the block line and clamp to it
                t = (self.x - dx) * fx + (self.y - dy) * fy
                t = max(0.0, min(depth, t))
                if math.hypot(self.x - (dx + fx * t),
                              self.y - (dy + fy * t)) < 12.0:
                    near_depot = not NO_BLOCKS
        near_plate = math.hypot(self.x - g["PLATE_X"],
                                self.y - g["PLATE_Y"]) < 30.0
        room = self.carrying < size

        # The jaws stall early when there is a block to stall on, and
        # stay stalled while they are still shut around it - filling the
        # magazine does not make the block in the jaws disappear.
        self.motors[grab]["hi"] = (
            58.0 if ((near_depot and room) or self._closed_flag) else 100.0)

        if near_depot and room and gpos >= 57.0 and not self._closed_flag:
            self.carrying += 1
            self._closed_flag = True
        if gpos < 20.0:
            self._closed_flag = False

        # allow for the tool controller's own tolerance
        # A block leaves the magazine on the downward crossing of the
        # release position - edge triggered, so no hysteresis band has
        # to be guessed.
        release_at = (g.get("GRAB_RELEASE_ONE_DEG", 0)
                      + g.get("TOOL_TOL_DEG", 6) + 6.0)
        crossed = self._prev_gpos > release_at >= gpos
        if near_plate and self.carrying > 0 and lpos <= 20.0 and crossed:
            self.carrying -= 1
            # record where the grabber is, which is where the block lands
            bx, by = self.sensor_xy(g["GRAB_FWD_CM"], g["GRAB_SIDE_CM"])
            self.placed.append((round(bx, 1), round(by, 1)))
        self._prev_gpos = gpos

    # -- sensors ------------------------------------------------------
    def yaw_decideg(self):
        d = (self.h - self.yaw_ref + 180.0) % 360.0 - 180.0
        return d * 10.0

    def reset_yaw(self):
        self.yaw_ref = self.h

    def sensor_xy(self, fwd=8.0, side=0.0):
        r = math.radians(self.h)
        fx, fy = math.sin(r), math.cos(r)
        rx, ry = math.cos(r), -math.sin(r)
        return (self.x + fx * fwd + rx * side,
                self.y + fy * fwd + ry * side)

    def colour_under_sensor(self):
        prog = sys.modules.get("mosaic_masters")
        if prog is None or not hasattr(prog, "cell_xy"):
            return "mat"
        g = prog.__dict__
        sx, sy = self.sensor_xy(g["COLOUR_FWD_CM"], g["COLOUR_SIDE_CM"])
        pitch, rows, cols = g["CELL_PITCH_CM"], g["GRID_ROWS"], g["GRID_COLS"]
        half_w = pitch * cols / 2.0 + 1.5
        half_h = pitch * rows / 2.0 + 1.5
        if (abs(sx - g["PLATE_X"]) <= half_w
                and abs(sy - g["PLATE_Y"]) <= half_h):
            for row in range(rows):
                for col in range(cols):
                    cx, cy = prog.cell_xy(row, col)
                    if (abs(sx - cx) <= pitch / 2.0
                            and abs(sy - cy) <= pitch / 2.0):
                        return self.pattern[row][col]
            return "black"
        return "mat"


RGB = {
    "white":  (330, 330, 310, 640),
    "yellow": (470, 410, 140, 500),
    "green":  (150, 420, 190, 300),
    "blue":   (110, 200, 360, 260),
    "red":    (430, 160, 130, 250),
    "black":  (25, 26, 25, 40),
    "mat":    (330, 330, 310, 700),
}

WORLD = World()


# =====================================================================
#  fake modules
# =====================================================================

def _install():
    # ---- time --------------------------------------------------------
    time_mod = types.ModuleType("time")
    time_mod.ticks_ms = lambda: int(WORLD.t_ms)
    time_mod.ticks_diff = lambda a, b: a - b
    time_mod.sleep = lambda s: None
    time_mod.time = lambda: WORLD.t_ms / 1000.0
    sys.modules["time"] = time_mod

    # ---- runloop -----------------------------------------------------
    class _Sleep:
        def __init__(self, ms):
            self.ms = ms

        def __await__(self):
            yield self.ms

    runloop = types.ModuleType("runloop")
    runloop.sleep_ms = lambda ms: _Sleep(ms)

    def _run(coro, limit_s=600):
        while True:
            try:
                ms = coro.send(None)
            except StopIteration:
                return
            WORLD.step(ms if ms else 1)
            if WORLD.t_ms > limit_s * 1000:
                raise RuntimeError("simulated run exceeded %ds" % limit_s)

    runloop.run = _run
    sys.modules["runloop"] = runloop

    # ---- motor -------------------------------------------------------
    motor = types.ModuleType("motor")
    motor.HOLD, motor.BRAKE, motor.COAST = 2, 1, 0

    def _run_motor(p, velocity):
        WORLD.motors[p]["cmd"] = float(velocity)

    def _stop_motor(p, stop=None):
        WORLD.motors[p]["cmd"] = 0.0
        WORLD.motors[p]["vel"] = 0.0

    motor.run = _run_motor
    motor.stop = _stop_motor
    motor.relative_position = lambda p: int(WORLD.motors[p]["pos"])
    motor.velocity = lambda p: int(WORLD.motors[p]["vel"])

    def _reset(p, value=0):
        WORLD.motors[p]["pos"] = float(value)
        if WORLD.motors[p]["lo"] is not None:
            span = WORLD.motors[p]["hi"] - WORLD.motors[p]["lo"]
            WORLD.motors[p]["lo"] = float(value)
            WORLD.motors[p]["hi"] = float(value) + span
    motor.reset_relative_position = _reset
    sys.modules["motor"] = motor

    # ---- colour sensor ----------------------------------------------
    colour = types.ModuleType("color_sensor")

    def _rgbi(p):
        return RGB[WORLD.colour_under_sensor()]

    def _reflection(p):
        return 12 if WORLD.colour_under_sensor() == "black" else 90

    colour.rgbi = _rgbi
    colour.reflection = _reflection
    sys.modules["color_sensor"] = colour

    # ---- hub ---------------------------------------------------------
    hub = types.ModuleType("hub")
    port = types.SimpleNamespace(**{c: c for c in "ABCDEF"})
    motion = types.SimpleNamespace(
        tilt_angles=lambda: (WORLD.yaw_decideg(), 0, 0),
        reset_yaw=lambda a=0: WORLD.reset_yaw(),
    )
    matrix = types.SimpleNamespace(write=lambda s: None)
    hub.port = port
    hub.motion_sensor = motion
    hub.light_matrix = matrix
    sys.modules["hub"] = hub


def _install_legacy():
    """Fake the SPIKE App 2 API and hide the new one."""
    for name in ("runloop", "motor", "color_sensor", "hub"):
        sys.modules[name] = None          # makes `import name` raise

    def _step(seconds):
        left = seconds * 1000.0
        while left > 0:
            chunk = min(10.0, left)
            WORLD.step(chunk)
            left -= chunk

    class _Matrix:
        def write(self, text):
            pass

    class _Motion:
        def get_yaw_angle(self):
            return WORLD.yaw_decideg() / 10.0

        def reset_yaw_angle(self):
            WORLD.reset_yaw()

    class PrimeHub:
        def __init__(self):
            self.motion_sensor = _Motion()
            self.light_matrix = _Matrix()

    class Motor:
        def __init__(self, letter):
            self.letter = letter

        def start(self, speed):
            WORLD.motors[self.letter]["cmd"] = speed / 100.0 * MAX_DEG_S

        def stop(self):
            WORLD.motors[self.letter]["cmd"] = 0.0
            WORLD.motors[self.letter]["vel"] = 0.0

        def set_stop_action(self, action):
            pass

        def get_degrees_counted(self):
            return int(WORLD.motors[self.letter]["pos"])

        def set_degrees_counted(self, value):
            WORLD.motors[self.letter]["pos"] = float(value)
            m = WORLD.motors[self.letter]
            if m["lo"] is not None:
                span = m["hi"] - m["lo"]
                m["lo"] = float(value)
                m["hi"] = float(value) + span

        def get_speed(self):
            return int(WORLD.motors[self.letter]["vel"] / MAX_DEG_S * 100)

    class ColorSensor:
        def __init__(self, letter):
            self.letter = letter

        def get_rgb_intensity(self):
            return RGB[WORLD.colour_under_sensor()]

        def get_reflected_light(self):
            return 12 if WORLD.colour_under_sensor() == "black" else 90

    spike = types.ModuleType("spike")
    spike.PrimeHub = PrimeHub
    spike.Motor = Motor
    spike.ColorSensor = ColorSensor
    control = types.ModuleType("spike.control")
    control.wait_for_seconds = _step
    spike.control = control
    sys.modules["spike"] = spike
    sys.modules["spike.control"] = control

    time_mod = types.ModuleType("time")
    time_mod.ticks_ms = lambda: int(WORLD.t_ms)
    time_mod.ticks_diff = lambda a, b: a - b
    sys.modules["time"] = time_mod


def main():
    if LEGACY:
        _install_legacy()
    else:
        _install()

    mode = None
    for a in sys.argv[1:]:
        if not a.startswith("-"):
            mode = a

    # patch MODE before the program runs itself on import
    source = open(os.environ.get("PROG", os.path.join(ROOT, "mosaic_masters.py"))).read()
    if mode:
        source = source.replace('MODE = "MISSION"', 'MODE = "%s"' % mode, 1)

    module = types.ModuleType("mosaic_masters")
    module.__dict__["__name__"] = "mosaic_masters"
    sys.modules["mosaic_masters"] = module

    # run the program; it launches its own main() at the bottom
    exec(compile(source, "mosaic_masters.py", "exec"), module.__dict__)

    print("\n--- simulation finished ---")
    print("api           : %s" % ("SPIKE App 2 (legacy)" if LEGACY
                                   else "SPIKE App 3"))
    print("sim time      : %.1f s" % (WORLD.t_ms / 1000.0))
    print("final pose    : x=%.1f y=%.1f h=%.1f"
          % (WORLD.x, WORLD.y, WORLD.h))
    print("believed pose : x=%.1f y=%.1f h=%.1f"
          % (module.POSE["x"], module.POSE["y"], module.POSE["h"]))
    err = math.hypot(WORLD.x - module.POSE["x"], WORLD.y - module.POSE["y"])
    print("odometry error: %.2f cm" % err)
    print("blocks released: %d at %s" % (len(WORLD.placed), WORLD.placed))
    if VERBOSE:
        for row in WORLD.trace:
            print("   t=%5.1f  x=%6.1f  y=%6.1f  h=%6.1f" % row)


if __name__ == "__main__":
    main()
