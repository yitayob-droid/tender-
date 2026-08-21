"""
Run mosaic.py against a model of the real WRO mat and report what
happens: where the robot actually goes, whether the positions in the
program point at the real depots and mosaic, and where every block
ends up. Renders the path onto the photo of the mat.

    python3 tools/sim.py
"""
import math, os, sys, types

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
PHOTO = "/root/.claude/uploads/fe246481-65e6-57fc-be77-d4e20d62b7ab/27b16b4b-IMG_2430.jpeg"

# --------------------------------------------------------------- the mat
# Corners of the printed mat in the photo, read off a grid overlay at
# 1000 px wide, and what they are in mat centimetres.
CORNERS_PX = {"bl": (33, 633), "br": (925, 620), "tr": (933, 180), "tl": (44, 197)}
MAT_L, MAT_W = 200.0, 100.0          # what the team measured

# Features read off the same photo, in that pixel space.
FEATURES_PX = {
    "mosaic":       (467, 340),
    "yellow left":  (68, 422),
    "blue left":    (80, 365),
    "green left":   (85, 315),
    "white left":   (112, 267),
    "yellow right": (870, 235),
    "green right":  (875, 350),
    "blue right":   (880, 420),
    "white right":  (888, 545),
    "start label":  (95, 530),
    # ends of the two long black lines nearest the start
    "lineA0": (150, 490), "lineA1": (700, 490),
    "lineB0": (150, 253), "lineB1": (150, 490),
}


def homography(src, dst):
    """8-parameter perspective transform from 4 point pairs."""
    A, b = [], []
    for (x, y), (u, v) in zip(src, dst):
        A.append([x, y, 1, 0, 0, 0, -u * x, -u * y]); b.append(u)
        A.append([0, 0, 0, x, y, 1, -v * x, -v * y]); b.append(v)
    n = 8
    for i in range(n):                                   # gaussian elimination
        p = max(range(i, n), key=lambda r: abs(A[r][i]))
        A[i], A[p] = A[p], A[i]; b[i], b[p] = b[p], b[i]
        for r in range(n):
            if r == i:
                continue
            f = A[r][i] / A[i][i]
            for c in range(i, n):
                A[r][c] -= f * A[i][c]
            b[r] -= f * b[i]
    h = [b[i] / A[i][i] for i in range(n)] + [1.0]
    return h


def apply_h(h, x, y):
    d = h[6] * x + h[7] * y + h[8]
    return ((h[0] * x + h[1] * y + h[2]) / d, (h[3] * x + h[4] * y + h[5]) / d)


PX2CM = homography([CORNERS_PX[k] for k in ("bl", "br", "tr", "tl")],
                   [(0, 0), (MAT_L, 0), (MAT_L, MAT_W), (0, MAT_W)])
CM2PX = homography([(0, 0), (MAT_L, 0), (MAT_L, MAT_W), (0, MAT_W)],
                   [CORNERS_PX[k] for k in ("bl", "br", "tr", "tl")])

REAL = {k: apply_h(PX2CM, *v) for k, v in FEATURES_PX.items()}

# --------------------------------------------------------------- physics
WHEEL_D, TRACK = 5.6, 12.0
CM_PER_DEG = math.pi * WHEEL_D / 360.0
MAXV = 1000.0

W = {"t": 0.0, "x": 0.0, "y": 0.0, "h": 0.0, "yaw_ref": 0.0,
     "pos": {c: 0.0 for c in "ABCEF"}, "cmd": {c: 0.0 for c in "ABCEF"},
     "path": [], "events": [], "offmat": 0, "placed": []}
LIMITS = {}                                   # filled once ports are known


def ports():
    p = sys.modules.get("mosaic")
    if p is None or not hasattr(p, "LEFT"):
        return ("A", "E", "C", "B")
    return (p.LEFT, p.RIGHT, p.CARRIAGE, p.GRAB)


def step(ms):
    dt = ms / 1000.0
    W["t"] += ms
    left, right, lift, grab = ports()
    if not LIMITS:
        LIMITS[lift] = (-40.0, 160.0)         # real end stops, offset on purpose
        LIMITS[grab] = (-25.0, 120.0)
    for c in W["pos"]:
        p = W["pos"][c] + W["cmd"][c] * dt
        if c in LIMITS:
            lo, hi = LIMITS[c]
            if p <= lo: p, W["cmd"][c] = lo, 0.0
            elif p >= hi: p, W["cmd"][c] = hi, 0.0
        W["pos"][c] = p
    vl = W["cmd"][left] * CM_PER_DEG
    vr = -W["cmd"][right] * CM_PER_DEG
    W["h"] = (W["h"] + math.degrees((vl - vr) / TRACK) * dt) % 360
    v = (vl + vr) / 2.0
    r = math.radians(W["h"])
    W["x"] += math.sin(r) * v * dt
    W["y"] += math.cos(r) * v * dt
    prog = sys.modules.get("mosaic")
    if prog is not None and hasattr(prog, "JAWS_OPEN"):
        gp, lp = W["pos"][grab], W["pos"][lift]
        low = lp < 60
        if gp >= prog.JAWS_SHUT - 8 and low:
            W["carrying"] = True
        elif gp <= prog.JAWS_OPEN + 8 and low and W.get("carrying"):
            W["carrying"] = False
            W["placed"].append((round(W["x"], 1), round(W["y"], 1),
                                round(W["t"] / 1000.0, 1)))
    if not (0 <= W["x"] <= MAT_L and 0 <= W["y"] <= MAT_W):
        W["offmat"] += 1
    if len(W["path"]) == 0 or W["t"] - W["path"][-1][2] > 60:
        W["path"].append((W["x"], W["y"], W["t"]))


# The two long black lines nearest the start, taken off the same photo.
# Only these are modelled - enough to answer "does find_line() find
# something", not to claim the whole network is here.
LINES = [(REAL["lineA0"], REAL["lineA1"]), (REAL["lineB0"], REAL["lineB1"])]


def on_line(x, y, tol=1.2):
    for (x0, y0), (x1, y1) in LINES:
        dx, dy = x1 - x0, y1 - y0
        L2 = dx * dx + dy * dy
        t = max(0.0, min(1.0, ((x - x0) * dx + (y - y0) * dy) / L2))
        if math.hypot(x - (x0 + dx * t), y - (y0 + dy * t)) < tol:
            return True
    return False


def sensor_xy():
    r = math.radians(W["h"])
    return (W["x"] + math.sin(r) * 8.0, W["y"] + math.cos(r) * 8.0)


# --------------------------------------------------------- fake spike api
class Sleep:
    def __init__(self, ms): self.ms = ms
    def __await__(self): yield self.ms


def install(start):
    W["x"], W["y"], W["h"] = start
    W["yaw_ref"] = start[2]

    runloop = types.ModuleType("runloop")
    runloop.sleep_ms = lambda ms: Sleep(ms)

    def run(coro, limit_s=900):
        while True:
            try:
                ms = coro.send(None)
            except StopIteration:
                return
            step(ms if ms else 1)
            if W["t"] > limit_s * 1000:
                raise RuntimeError("run exceeded %d s of robot time" % limit_s)
    runloop.run = run
    sys.modules["runloop"] = runloop

    mot = types.ModuleType("motor")
    mot.run = lambda p, v: W["cmd"].__setitem__(p, max(min(float(v), MAXV), -MAXV))
    mot.stop = lambda p, **k: W["cmd"].__setitem__(p, 0.0)
    mot.relative_position = lambda p: int(W["pos"][p])
    mot.velocity = lambda p: int(W["cmd"][p])

    def reset(p, v=0):
        if p in LIMITS:
            lo, hi = LIMITS[p]
            shift = W["pos"][p] - float(v)
            LIMITS[p] = (lo - shift, hi - shift)
        W["pos"][p] = float(v)
    mot.reset_relative_position = reset

    class ToPos:
        def __init__(self, p, t): self.p, self.t = p, t
        def __await__(self):
            t = float(self.t)
            if self.p in LIMITS:
                lo, hi = LIMITS[self.p]
                t = max(min(t, hi), lo)
            W["pos"][self.p] = t
            yield 60
    mot.run_to_relative_position = lambda p, pos, vel: ToPos(p, pos)
    sys.modules["motor"] = mot

    cs = types.ModuleType("color_sensor")
    def refl(p):
        hit = on_line(*sensor_xy())
        if hit and "junction" not in W:
            W["junction"] = (W["x"], W["y"], W["h"])
        return 15 if hit else 88
    cs.reflection = refl
    cs.rgbi = lambda p: mosaic_colour_at(*sensor_xy())
    sys.modules["color_sensor"] = cs

    hub = types.ModuleType("hub")
    hub.port = types.SimpleNamespace(**{c: c for c in "ABCDEF"})
    hub.motion_sensor = types.SimpleNamespace(
        tilt_angles=lambda: (((W["h"] - W["yaw_ref"] + 180) % 360 - 180) * 10, 0, 0),
        reset_yaw=lambda a=0: W.__setitem__("yaw_ref", W["h"]))
    def mark(tag):
        W["events"].append((str(tag), round(W["x"], 1), round(W["y"], 1),
                            round(W["h"], 1), round(W["t"] / 1000.0, 1)))
    hub.light_matrix = types.SimpleNamespace(write=mark)
    sys.modules["hub"] = hub


# the real pattern, as read off the mat by the team
PATTERN = [["yellow", "yellow", "yellow"],
           ["white", "blue", "white"],
           ["white", "blue", "white"],
           ["green", "white", "green"]]
RGB = {"blue": (110, 200, 360, 260), "yellow": (470, 410, 140, 500),
       "green": (150, 420, 190, 300), "white": (330, 330, 310, 640),
       "black": (25, 26, 25, 40)}


def mosaic_colour_at(sx, sy):
    """The real mosaic, placed where the photo says it is."""
    p = sys.modules.get("mosaic")
    if p is None or not hasattr(p, "CELL"):
        return RGB["black"]
    mx, my = REAL["mosaic"]
    for r in range(p.ROWS):
        for c in range(p.COLS):
            cx = mx + (c - (p.COLS - 1) / 2.0) * p.CELL
            cy = my + (r - (p.ROWS - 1) / 2.0) * p.CELL
            if abs(sx - cx) < p.CELL / 2 and abs(sy - cy) < p.CELL / 2:
                return RGB[PATTERN[r][c]]
    return RGB["black"]


# ------------------------------------------------------------- the report
def where_program_thinks(mod, spot, junction):
    """Where a (heading, distance) in the program actually lands you."""
    h, d = spot
    r = math.radians(junction[2] + h)
    return (junction[0] + math.sin(r) * d, junction[1] + math.cos(r) * d)


def main():
    print("=" * 68)
    print("THE REAL MAT, read off the photo (cm from the bottom-left corner)")
    print("=" * 68)
    for k in ("mosaic", "yellow left", "blue left", "green left", "white left",
              "yellow right", "green right", "blue right", "white right"):
        x, y = REAL[k]
        print("  %-14s %6.1f, %5.1f" % (k, x, y))
    dl = [REAL[c + " left"] for c in ("yellow", "blue", "green", "white")]
    gaps = [math.hypot(dl[i + 1][0] - dl[i][0], dl[i + 1][1] - dl[i][1])
            for i in range(3)]
    print("\n  left depots are %s cm apart (%.1f cm average)"
          % ([round(g, 1) for g in gaps], sum(gaps) / 3))

    start = (REAL["start label"][0], REAL["start label"][1], 0.0)
    install(start)
    print("\n  robot starts at %.1f, %.1f facing %.0f (the ROBOMISSION corner)"
          % start)

    src = open(os.path.join(ROOT, "mosaic.py")).read()
    mod = types.ModuleType("mosaic")
    sys.modules["mosaic"] = mod
    try:
        exec(compile(src, "mosaic.py", "exec"), mod.__dict__)
        crashed = None
    except Exception as e:
        crashed = e

    print("\n" + "=" * 68)
    print("WHAT THE PROGRAM ASSUMES vs WHAT IS THERE")
    print("=" * 68)
    junction = W.get("junction", start)
    print("  step 1 %s" % ("found a line at %.1f, %.1f" % junction[:2]
                           if "junction" in W else
                           "found NO line - using the start position"))
    mx, my = REAL["mosaic"]
    want_mosaic = (mx - (mod.COLS - 1) / 2.0 * mod.CELL,
                   my - (mod.ROWS - 1) / 2.0 * mod.CELL - mod.STANDOFF)
    for name, spot, real_key, target in (
            ("DEPOT", mod.DEPOT, "yellow left", REAL["yellow left"]),
            ("MOSAIC", mod.MOSAIC, "its approach spot", want_mosaic)):
        gx, gy = where_program_thinks(mod, spot, junction)
        rx, ry = target
        print("  %-7s sends the robot to %6.1f, %5.1f" % (name, gx, gy))
        print("          %-14s is at %6.1f, %5.1f   -> out by %.0f cm"
              % (real_key, rx, ry, math.hypot(gx - rx, gy - ry)))

    span = 3 * mod.GROUP_SPAN
    print("\n  depot model spans %.0f cm for 4 colours; on the mat they span "
          "%.0f cm" % (span, sum(gaps)))

    print("\n" + "=" * 68)
    print("THE RUN")
    print("=" * 68)
    if crashed:
        print("  CRASHED: %s" % crashed)
    print("  robot time      %.0f s" % (W["t"] / 1000.0))
    print("  path length     %.0f cm" % sum(
        math.hypot(W["path"][i + 1][0] - W["path"][i][0],
                   W["path"][i + 1][1] - W["path"][i][1])
        for i in range(len(W["path"]) - 1)))
    xs = [p[0] for p in W["path"]]; ys = [p[1] for p in W["path"]]
    print("  travelled x %.0f..%.0f, y %.0f..%.0f" %
          (min(xs), max(xs), min(ys), max(ys)))
    if W["offmat"]:
        print("  OFF THE MAT for %.1f s of the run" % (W["offmat"] * 0.01))
    else:
        print("  stayed on the mat")
    print("\n  where the robot is each time a step starts:")
    for tag, x, y, h, t in W["events"]:
        print("      step %-3s at %6.1f, %5.1f  heading %5.1f  (t=%5.1f s)"
              % (tag, x, y, h, t))
    print("\n  rows delivered  %d of %d" % (len(W["placed"]), mod.ROWS))
    mx, my = REAL["mosaic"]
    for i, (x, y, t) in enumerate(W["placed"]):
        want_y = my - (mod.ROWS - 1) / 2.0 * mod.CELL + mod.CELL * (mod.ROWS - 1 - i)
        print("      dropped at %6.1f, %5.1f (t=%5.1f s)  cell wants "
              "%6.1f, %5.1f  -> out by %.0f cm"
              % (x, y, t, mx - mod.CELL, want_y,
                 math.hypot(x - (mx - mod.CELL), y - want_y)))
    for x, y, t in W["placed"]:
        print("      at %6.1f, %5.1f  (t=%5.1f s)" % (x, y, t))

    draw(start)


def draw(start):
    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print("\n(no PIL, skipping the picture)")
        return
    im = Image.open(PHOTO).convert("RGB")
    im = im.resize((1000, int(im.size[1] * 1000.0 / im.size[0])))
    d = ImageDraw.Draw(im, "RGBA")

    def px(x, y):
        return apply_h(CM2PX, x, y)

    # the mat outline we assumed
    d.polygon([px(0, 0), px(MAT_L, 0), px(MAT_L, MAT_W), px(0, MAT_W)],
              outline=(255, 255, 255), width=2)

    # where the program thinks things are
    mod = sys.modules["mosaic"]
    for spot, tag in ((mod.DEPOT, "DEPOT"), (mod.MOSAIC, "MOSAIC")):
        gx, gy = where_program_thinks(mod, spot, start)
        p = px(gx, gy)
        d.ellipse([p[0] - 7, p[1] - 7, p[0] + 7, p[1] + 7],
                  outline=(255, 80, 0), width=3)
        d.text((p[0] + 9, p[1] - 6), tag, fill=(255, 80, 0))

    # the path
    pts = [px(p[0], p[1]) for p in W["path"]]
    if len(pts) > 1:
        d.line(pts, fill=(0, 255, 255, 220), width=3)
    s = px(start[0], start[1])
    d.ellipse([s[0] - 6, s[1] - 6, s[0] + 6, s[1] + 6], fill=(0, 255, 0))
    for x, y, _ in W["placed"]:
        p = px(x, y)
        d.rectangle([p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5],
                    outline=(255, 0, 255), width=3)

    out = os.path.join(ROOT, "dist", "sim.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    im.save(out)
    print("\n  picture written to dist/sim.png")


if __name__ == "__main__":
    main()
