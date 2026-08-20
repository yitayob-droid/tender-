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
        self.motors["A"]["lo"], self.motors["A"]["hi"] = 0.0, 170.0
        self.motors["C"]["lo"], self.motors["C"]["hi"] = 0.0, 100.0
        self.trace = []
        self.carrying = None
        self.placed = []

        # ground truth mosaic used by the fake colour sensor
        self.pattern = [["yellow", "green", "yellow"],
                        ["blue", "white", "green"],
                        ["green", "yellow", "blue"]]

    # -- integration --------------------------------------------------
    def step(self, ms):
        dt = ms / 1000.0
        if dt <= 0:
            dt = 0.001
        self.t_ms += ms

        self._update_grabber_load()
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

        # differential drive: B is left, F is right.  The program feeds
        # them opposite signs (LEFT_SIGN/RIGHT_SIGN), undo that here.
        vl = self.motors["B"]["vel"] * CM_PER_DEG * WHEEL_SCALE_L
        vr = -self.motors["F"]["vel"] * CM_PER_DEG * WHEEL_SCALE_R
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

    def _update_grabber_load(self):
        """A block in the jaws stops the grabber early.  A block is in
        reach whenever the robot is nosed in at one of the depots."""
        prog = sys.modules.get("mosaic_masters")
        if prog is None or not hasattr(prog, "DEPOTS"):
            return
        near = False
        for options in prog.DEPOTS.values():
            for opt in options:
                dx, dy = opt["stand"]
                if math.hypot(self.x - dx, self.y - dy) < 14.0:
                    near = not NO_BLOCKS
        c = self.motors["C"]
        if near and self.carrying is None:
            c["hi"] = 58.0            # jaws close onto a block
        elif self.carrying is not None:
            c["hi"] = 58.0            # still holding it
        else:
            c["hi"] = 100.0           # closing on air
        if near and c["pos"] >= 57.0:
            self.carrying = "block"
        if self.carrying is not None and c["pos"] < 20.0:
            self.placed.append((round(self.x, 1), round(self.y, 1)))
            self.carrying = None

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
