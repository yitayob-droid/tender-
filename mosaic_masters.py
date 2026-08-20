# =====================================================================
#  WRO 2026 RoboMission Senior - "Mosaic Masters"
#  Colour-block collector for LEGO Education SPIKE Prime
# =====================================================================
#
#  WHAT THIS PROGRAM DOES
#  ----------------------
#   1. Starts in the corner of the mat that carries the ROBOMISSION
#      label (bottom-left, origin of the coordinate system).
#   2. Drives to the black mosaic plate in the middle of the mat and
#      scans every cell of the pattern with the colour sensor.
#   3. For every colour it finds, it drives to the matching depot on the
#      edge of the mat, grabs a block with the bottom grabber (port C),
#      lifts it (port A) and carries it back to the middle.
#   4. It puts the block down on the cell of the mosaic that has the
#      same colour, then goes back for the next one.
#   5. When every cell is served (or the time budget runs out) it parks
#      back in the start area.
#
#  All driving is closed loop on the hub's yaw (gyro) sensor, so the
#  robot keeps a true heading and re-squares itself on every move.
#
#  HARDWARE (as wired on your robot)
#  ---------------------------------
#      A : up / down mechanism (lift)
#      B : left drive wheel
#      C : grabber (bottom)
#      E : colour sensor (pointing down at the mat)
#      F : right drive wheel
#      hub : yaw / gyro used for straight driving and turning
#
#  IMPORTANT - READ README.md FIRST
#  --------------------------------
#  Every number in the CONFIG section below is a real, physical
#  measurement.  The defaults are sensible starting values taken from
#  the mat photo and from your robot dimensions, but you MUST measure
#  and tune them on your own table.  Run the built in test modes
#  (MODE = "TEST_...") to do that - it takes about 20 minutes and it is
#  the difference between a robot that scores and one that drifts.
#
#  Works with the new SPIKE App 3 / firmware 3.x Python API and falls
#  back automatically to the older SPIKE App 2 ("from spike import ...")
#  API, so you can paste it into whichever app you use.
# =====================================================================

import math

# =====================================================================
#  CONFIG  -  everything you tune lives here
# =====================================================================

# --- what to run -----------------------------------------------------
#   "MISSION"        full run
#   "CAL_COLOUR"     hold the sensor over a colour, read the numbers
#   "CAL_YAW"        checks the sign/scale of the gyro
#   "TEST_DRIVE"     drives 50 cm forward and 50 cm back
#   "TEST_TURN"      turns 90 / 180 / -90 and comes back to 0
#   "TEST_TOOLS"     opens/closes the grabber and cycles the lift
#   "TEST_SCAN"      only drives to the plate and scans the pattern
MODE = "MISSION"

# --- ports -----------------------------------------------------------
PORT_LIFT   = "A"      # up / down mechanism
PORT_LEFT   = "B"      # left wheel
PORT_GRAB   = "C"      # grabber
PORT_COLOUR = "E"      # colour sensor, looking down
PORT_RIGHT  = "F"      # right wheel

# Motor polarity.  If the robot drives backwards when you run
# TEST_DRIVE, flip BOTH of these.  If it spins on the spot instead of
# driving, flip only ONE of them.
LEFT_SIGN  = +1
RIGHT_SIGN = -1

# Sign of the tool motors: +1 means "positive power closes the grabber
# / raises the lift".  Flip if your build is mirrored.
GRAB_SIGN = +1
LIFT_SIGN = +1

# --- robot geometry (cm) --------------------------------------------
WHEEL_DIAMETER_CM = 5.6     # SPIKE Prime large wheel = 5.6 cm.  MEASURE.
AXLE_TRACK_CM     = 12.0    # distance between the two wheel contact points
ROBOT_LENGTH_CM   = 18.0
ROBOT_WIDTH_CM    = 25.0

# Where the tools sit relative to the point midway between the wheels.
# forward = towards the front of the robot, side = towards its right.
COLOUR_FWD_CM = 8.0
COLOUR_SIDE_CM = 0.0
GRAB_FWD_CM   = 10.0
GRAB_SIDE_CM  = 0.0

# Fudge factor applied to every straight move.  If a commanded 50 cm
# comes out as 48 cm, set this to 50/48 = 1.042.
DRIVE_SCALE = 1.000

DEG_PER_CM = 360.0 / (math.pi * WHEEL_DIAMETER_CM)

# --- speeds (percent of full motor speed, 0..100) --------------------
MAX_DEG_S    = 900.0   # what 100 % means on the new API (deg/s)
DRIVE_PCT    = 58      # normal travel
SLOW_PCT     = 25      # approaching blocks / scanning
TURN_PCT     = 40      # spin turns
TOOL_PCT     = 70      # grabber and lift

# --- gyro straight-drive controller ---------------------------------
KP_STRAIGHT = 2.4
KI_STRAIGHT = 0.02
KD_STRAIGHT = 6.0
RAMP_UP_CM   = 4.0     # accelerate over the first ... cm
RAMP_DOWN_CM = 8.0     # decelerate over the last ... cm
MIN_MOVE_PCT = 12      # never creep below this or the robot stalls

# --- gyro turn controller -------------------------------------------
KP_TURN   = 1.9
KD_TURN   = 5.5
TURN_TOL_DEG   = 1.5   # considered "on heading" inside this band
TURN_SETTLE_MS = 250   # must stay inside the band this long
TURN_MIN_PCT   = 14
TURN_TIMEOUT_S = 4.0
# After the main turn settles it is usually parked on the edge of the
# tolerance band.  These short pulses shave off the last degree without
# setting the robot oscillating (they also break stiction on carpet).
TURN_FINE_TOL_DEG     = 0.8
TURN_PULSE_PCT        = 12
TURN_PULSE_MS_PER_DEG = 20     # how long a pulse must be to move 1 deg
TURN_MAX_PULSES       = 8

# --- line handling ---------------------------------------------------
WHITE_REFLECT = 90     # reflection on the white part of the mat
BLACK_REFLECT = 12     # reflection on a black line
LINE_KP       = 0.55   # edge-following gain
EDGE_SIGN     = +1     # +1 follows the left edge, -1 the right edge

# --- gyro ------------------------------------------------------------
# +1 if the yaw value goes UP when the robot turns clockwise (to its
# right) seen from above.  Run MODE = "CAL_YAW" to check.
YAW_SIGN = +1

# --- colour recognition ----------------------------------------------
# Reference colours as normalised r,g,b (each channel divided by the sum
# of the three) plus the raw intensity.  Run MODE = "CAL_COLOUR", hold
# the sensor 5-10 mm over each block and paste the printed numbers here.
COLOUR_REFS = {
    "white":  (0.34, 0.34, 0.32),
    "yellow": (0.46, 0.40, 0.14),
    "green":  (0.20, 0.55, 0.25),
    "blue":   (0.16, 0.30, 0.54),
    "red":    (0.60, 0.22, 0.18),
    "black":  (0.33, 0.34, 0.33),
}
# Anything darker than this raw intensity is called "black", anything
# brighter with a flat spectrum is called "white".
BLACK_INTENSITY = 90       # 0..1024
WHITE_INTENSITY = 320      # 0..1024
COLOUR_MAX_DIST = 0.16     # farther than this from every reference -> unknown
COLOUR_SAMPLES  = 7        # readings averaged per measurement

# =====================================================================
#  FIELD MAP  -  coordinates in cm
# =====================================================================
#  Origin (0,0) is the bottom-left corner of the mat, the corner with
#  the WRO / ROBOMISSION label.
#  +X runs along the long (2 m) side to the right.
#  +Y runs along the short (1 m) side away from you.
#  Headings are compass style: 0 deg = facing +Y, 90 = +X, 180 = -Y,
#  270 = -X, and they grow CLOCKWISE seen from above.
#
#      Y
#      ^   white   |                            |  yellow
#      |   green   |         [ MOSAIC ]         |  green
#      |   blue    |         [ PLATE  ]         |  purple
#      |   yellow  |                            |  white
#      |  (START)                                        
#      +-------------------------------------------> X
#
#  >>> MEASURE THESE ON YOUR OWN MAT AND EDIT THEM. <<<

MAT_LENGTH_CM = 200.0
MAT_WIDTH_CM  = 100.0

# where the robot is placed before the run: x, y, heading
START_X = 16.0
START_Y = 22.0
START_H = 90.0          # facing along the mat, to the right

# The black mosaic plate in the middle.
PLATE_X = 95.0          # centre of the plate
PLATE_Y = 57.0
GRID_ROWS = 3
GRID_COLS = 3
CELL_PITCH_CM = 4.0     # centre-to-centre distance between two cells
# Safe line the robot travels along when it moves around the plate.
PLATE_APPROACH_Y = PLATE_Y - 22.0
PLATE_LEAVE_Y    = PLATE_Y - 22.0

# Depots.  "stand" is where the robot parks, "face" is the heading it
# takes there, and the block is straight ahead of the grabber.
# There is a set on the left edge and a set on the right edge; the robot
# automatically drives to whichever one is closer.
DEPOTS = {
    "yellow": [ {"stand": (30.0, 48.0), "face": 270.0},
                {"stand": (176.0, 88.0), "face":  90.0} ],
    "blue":   [ {"stand": (30.0, 60.0), "face": 270.0},
                {"stand": (176.0, 55.0), "face":  90.0} ],
    "green":  [ {"stand": (30.0, 72.0), "face": 270.0},
                {"stand": (176.0, 72.0), "face":  90.0} ],
    "white":  [ {"stand": (30.0, 86.0), "face": 270.0},
                {"stand": (176.0, 36.0), "face":  90.0} ],
}
# How far the robot noses in from the "stand" point to close the
# grabber around a block, and how far it backs off afterwards.
DEPOT_APPROACH_CM = 9.0

# Travel lanes: two horizontal corridors the robot uses so that it never
# cuts across the plate or the depots.
LANE_LOW_Y  = 30.0
LANE_HIGH_Y = 92.0

# --- mission behaviour -----------------------------------------------
# "SCAN"  : read the pattern off the plate at the start of the run
# "FIXED" : use FIXED_PATTERN below (type it in during inspection time)
PATTERN_SOURCE = "SCAN"
FIXED_PATTERN = [
    ["yellow", "green",  "yellow"],
    ["blue",   "white",  "green" ],
    ["green",  "yellow", "blue"  ],
]
# Cells whose colour is one of these are skipped (nothing to deliver).
SKIP_COLOURS = ("black", "unknown", "red")
# Time budget.  Set this to your rulebook's run time minus about 10 s
# so the robot always has time to park.  The mission also refuses to
# start a trip it cannot finish inside the budget.
MISSION_TIMEOUT_S = 110.0
TRIP_ESTIMATE_S   = 20.0     # first guess, then it measures itself

# --- tool positions (motor degrees from the homed position) ----------
GRAB_OPEN_DEG   = 0
GRAB_CLOSED_DEG = 95
# With a block in the jaws the grabber stalls before it reaches
# GRAB_CLOSED_DEG.  If it gets past GRAB_EMPTY_DEG the jaws are empty,
# which means the grab missed.  Read the real number off the console
# with MODE = "TEST_TOOLS" (closed on a block vs closed on air).
GRAB_EMPTY_DEG  = 85
PICK_RETRIES    = 1
LIFT_DOWN_DEG   = 0
LIFT_CARRY_DEG  = 95
LIFT_UP_DEG     = 160
TOOL_TOL_DEG    = 6
TOOL_TIMEOUT_S  = 3.0
STALL_MS        = 350        # motor considered stalled after this

# =====================================================================
#  HARDWARE LAYER  -  new SPIKE App 3 API with SPIKE App 2 fallback
# =====================================================================

NEW_API = True
try:
    import runloop
    import motor as _motor
    import color_sensor as _colour
    from hub import port as _hp, motion_sensor as _imu, light_matrix as _lm
except ImportError:                       # SPIKE App 2 / MINDSTORMS
    NEW_API = False
    from spike import PrimeHub, Motor, ColorSensor
    from spike.control import wait_for_seconds
    _legacy_hub = PrimeHub()
    _legacy_motors = {}
    _legacy_colour = None

try:
    from time import ticks_ms, ticks_diff
except ImportError:                       # very old runtimes
    import time as _time

    def ticks_ms():
        return int(_time.time() * 1000)

    def ticks_diff(a, b):
        return a - b


if NEW_API:
    _PORTS = {"A": _hp.A, "B": _hp.B, "C": _hp.C,
              "D": _hp.D, "E": _hp.E, "F": _hp.F}


def _pct_to_vel(pct):
    return int(pct * MAX_DEG_S / 100.0)


def m_run(letter, pct):
    """Start a motor at a percentage of full speed (-100..100)."""
    pct = clamp(pct, -100, 100)
    if NEW_API:
        _motor.run(_PORTS[letter], _pct_to_vel(pct))
    else:
        _legacy_motor(letter).start(int(pct))


def m_stop(letter, hold=True):
    if NEW_API:
        _motor.stop(_PORTS[letter],
                    stop=_motor.HOLD if hold else _motor.BRAKE)
    else:
        m = _legacy_motor(letter)
        m.set_stop_action("hold" if hold else "brake")
        m.stop()


def m_deg(letter):
    """Encoder position in degrees since the last reset."""
    if NEW_API:
        return _motor.relative_position(_PORTS[letter])
    return _legacy_motor(letter).get_degrees_counted()


def m_reset(letter):
    if NEW_API:
        _motor.reset_relative_position(_PORTS[letter], 0)
    else:
        _legacy_motor(letter).set_degrees_counted(0)


def m_speed(letter):
    """Current speed, used for stall detection."""
    if NEW_API:
        return _motor.velocity(_PORTS[letter])
    return _legacy_motor(letter).get_speed()


def _legacy_motor(letter):
    if letter not in _legacy_motors:
        _legacy_motors[letter] = Motor(letter)
    return _legacy_motors[letter]


def yaw_deg():
    """Signed yaw in degrees, positive clockwise seen from above."""
    if NEW_API:
        raw = _imu.tilt_angles()[0] / 10.0     # decidegrees -> degrees
    else:
        raw = _legacy_hub.motion_sensor.get_yaw_angle()
    return YAW_SIGN * raw


def yaw_reset():
    if NEW_API:
        _imu.reset_yaw(0)
    else:
        _legacy_hub.motion_sensor.reset_yaw_angle()


def rgbi():
    """Raw (r, g, b, intensity) from the colour sensor, 0..1024."""
    if NEW_API:
        return _colour.rgbi(_PORTS[PORT_COLOUR])
    return _legacy_sensor().get_rgb_intensity()


def reflection():
    """Reflected light, 0..100."""
    if NEW_API:
        return _colour.reflection(_PORTS[PORT_COLOUR])
    return _legacy_sensor().get_reflected_light()


def _legacy_sensor():
    global _legacy_colour
    if _legacy_colour is None:
        _legacy_colour = ColorSensor(PORT_COLOUR)
    return _legacy_colour


async def sleep_ms(ms):
    if NEW_API:
        await runloop.sleep_ms(int(ms))
    else:
        wait_for_seconds(ms / 1000.0)


def say(text):
    """Print to the console and, if it is short, show it on the hub."""
    print(text)
    try:
        if NEW_API:
            _lm.write(str(text)[:6])
        else:
            _legacy_hub.light_matrix.write(str(text)[:6])
    except Exception:
        pass


def start_program(coro):
    """Run the top level coroutine on either API."""
    if NEW_API:
        runloop.run(coro)
    else:
        try:
            while True:
                coro.send(None)
        except StopIteration:
            pass


# =====================================================================
#  MATHS HELPERS
# =====================================================================

def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap180(a):
    """Fold an angle into -180..180."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def norm360(a):
    return a % 360.0


def unit_forward(h):
    r = math.radians(h)
    return (math.sin(r), math.cos(r))


def unit_right(h):
    r = math.radians(h)
    return (math.cos(r), -math.sin(r))


def bearing_to(x0, y0, x1, y1):
    return norm360(math.degrees(math.atan2(x1 - x0, y1 - y0)))


def distance_to(x0, y0, x1, y1):
    return math.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def sign(v):
    return 1 if v >= 0 else -1


# =====================================================================
#  POSE  -  where the robot thinks it is
# =====================================================================

POSE = {"x": START_X, "y": START_Y, "h": START_H}
HEADING_REF = START_H          # field heading that matches yaw = 0


def heading():
    """Current field heading, 0..360, from the gyro."""
    return norm360(HEADING_REF + yaw_deg())


def set_pose(x, y, h=None):
    global HEADING_REF
    POSE["x"] = x
    POSE["y"] = y
    if h is not None:
        yaw_reset()
        HEADING_REF = norm360(h)
        POSE["h"] = norm360(h)


def advance_pose(distance_cm, h=None):
    if h is None:
        h = heading()
    fx, fy = unit_forward(h)
    POSE["x"] += fx * distance_cm
    POSE["y"] += fy * distance_cm
    POSE["h"] = h


def tool_pose(target_x, target_y, h, fwd_cm, side_cm):
    """Where the robot centre must be so that a tool mounted at
    (fwd_cm, side_cm) ends up exactly over (target_x, target_y)."""
    fx, fy = unit_forward(h)
    rx, ry = unit_right(h)
    return (target_x - fx * fwd_cm - rx * side_cm,
            target_y - fy * fwd_cm - ry * side_cm)


# =====================================================================
#  DRIVE BASE
# =====================================================================

def drive_power(left_pct, right_pct):
    m_run(PORT_LEFT, LEFT_SIGN * left_pct)
    m_run(PORT_RIGHT, RIGHT_SIGN * right_pct)


def drive_stop(hold=True):
    m_stop(PORT_LEFT, hold)
    m_stop(PORT_RIGHT, hold)


def drive_reset():
    m_reset(PORT_LEFT)
    m_reset(PORT_RIGHT)


def drive_travelled_cm():
    """Signed distance covered since the last drive_reset()."""
    left = LEFT_SIGN * m_deg(PORT_LEFT)
    right = RIGHT_SIGN * m_deg(PORT_RIGHT)
    return ((left + right) / 2.0) / DEG_PER_CM


async def drive_straight(distance_cm, pct=DRIVE_PCT, hold_heading=None,
                         stop_on_black=False, hold_at_end=True):
    """Drive in a perfectly straight line using the gyro.

    distance_cm may be negative to reverse.  Returns the distance the
    robot actually covered, which is useful when stop_on_black cuts the
    move short."""
    if abs(distance_cm) < 0.05:
        return 0.0

    target = abs(distance_cm) * DRIVE_SCALE
    direction = sign(distance_cm)
    if hold_heading is None:
        hold_heading = heading()

    drive_reset()
    integral = 0.0
    last_err = 0.0
    t0 = ticks_ms()
    timeout_ms = int(1000 * (target / 4.0 + 3.0))   # generous watchdog
    travelled = 0.0

    while True:
        travelled = abs(drive_travelled_cm())
        remaining = target - travelled
        if remaining <= 0.0:
            break
        if ticks_diff(ticks_ms(), t0) > timeout_ms:
            say("DRV TO")            # drive timed out - report it
            break
        if stop_on_black and reflection() <= BLACK_REFLECT + 6:
            break

        err = wrap180(hold_heading - heading())
        integral = clamp(integral + err, -60.0, 60.0)
        correction = (KP_STRAIGHT * err
                      + KI_STRAIGHT * integral
                      + KD_STRAIGHT * (err - last_err))
        last_err = err

        # trapezoidal speed profile
        speed = pct
        if travelled < RAMP_UP_CM:
            speed = pct * (0.35 + 0.65 * travelled / RAMP_UP_CM)
        if remaining < RAMP_DOWN_CM:
            speed = min(speed, pct * (0.25 + 0.75 * remaining / RAMP_DOWN_CM))
        speed = max(speed, MIN_MOVE_PCT)

        base = direction * speed
        drive_power(clamp(base + correction, -100, 100),
                    clamp(base - correction, -100, 100))
        await sleep_ms(10)

    drive_stop(hold_at_end)
    await sleep_ms(60)
    covered = abs(drive_travelled_cm()) / DRIVE_SCALE
    advance_pose(direction * covered, hold_heading)
    return direction * covered


async def turn_to(target_heading, pct=TURN_PCT, precise=True):
    """Spin on the spot until the gyro reads target_heading.

    precise=False skips the fine trim.  That is fine before a straight
    move, because drive_straight() steers out the last fraction of a
    degree anyway; use precise=True when the heading itself matters."""
    target_heading = norm360(target_heading)
    last_err = 0.0
    t0 = ticks_ms()
    settled_since = None

    while True:
        err = wrap180(target_heading - heading())

        # inside the tolerance band: cut the power and let it settle.
        # (Keeping the minimum power on here makes the robot hunt.)
        if abs(err) <= TURN_TOL_DEG:
            drive_stop(True)
            if settled_since is None:
                settled_since = ticks_ms()
            elif ticks_diff(ticks_ms(), settled_since) >= TURN_SETTLE_MS:
                break
            last_err = err
            await sleep_ms(10)
            continue

        settled_since = None
        if ticks_diff(ticks_ms(), t0) > TURN_TIMEOUT_S * 1000:
            say("TRN TO")            # turn timed out - report it
            break

        power = KP_TURN * err + KD_TURN * (err - last_err)
        last_err = err
        power = clamp(power, -pct, pct)
        if abs(power) < TURN_MIN_PCT:
            power = TURN_MIN_PCT * sign(err)

        # clockwise (heading increasing) = left wheel forward
        drive_power(power, -power)
        await sleep_ms(10)

    drive_stop(True)
    await sleep_ms(80)

    # fine trim
    for _ in range(TURN_MAX_PULSES if precise else 0):
        err = wrap180(target_heading - heading())
        if abs(err) <= TURN_FINE_TOL_DEG:
            break
        pulse = clamp(abs(err) * TURN_PULSE_MS_PER_DEG, 25, 150)
        power = TURN_PULSE_PCT * sign(err)
        drive_power(power, -power)
        await sleep_ms(pulse)
        drive_stop(True)
        await sleep_ms(90)

    POSE["h"] = heading()


async def turn_by(delta_deg, pct=TURN_PCT, precise=True):
    await turn_to(heading() + delta_deg, pct, precise)


async def goto(x, y, pct=DRIVE_PCT, final_heading=None, reverse=False):
    """Turn towards (x, y), drive there, optionally end on a heading."""
    dist = distance_to(POSE["x"], POSE["y"], x, y)
    if dist > 0.5:
        course = bearing_to(POSE["x"], POSE["y"], x, y)
        if reverse:
            # driving backwards: nose points the opposite way
            await turn_to(course + 180.0, TURN_PCT, precise=False)
            await drive_straight(-dist, pct, hold_heading=course + 180.0)
        else:
            await turn_to(course, TURN_PCT, precise=False)
            await drive_straight(dist, pct, hold_heading=course)
    if final_heading is not None:
        await turn_to(final_heading, TURN_PCT, precise=True)


async def route(points, pct=DRIVE_PCT, final_heading=None):
    """Drive through a list of (x, y) waypoints."""
    for i, (x, y) in enumerate(points):
        last = (i == len(points) - 1)
        await goto(x, y, pct, final_heading if last else None)


async def follow_line(distance_cm, pct=SLOW_PCT):
    """Edge-follow a black line for a set distance (single sensor)."""
    mid = (WHITE_REFLECT + BLACK_REFLECT) / 2.0
    drive_reset()
    t0 = ticks_ms()
    while abs(drive_travelled_cm()) < distance_cm * DRIVE_SCALE:
        if ticks_diff(ticks_ms(), t0) > 1000 * (distance_cm / 3.0 + 4.0):
            break
        error = mid - reflection()
        steer = clamp(EDGE_SIGN * LINE_KP * error, -pct, pct)
        drive_power(pct + steer, pct - steer)
        await sleep_ms(10)
    drive_stop(True)
    await sleep_ms(60)
    advance_pose(abs(drive_travelled_cm()) / DRIVE_SCALE)


async def find_black(max_cm=20.0, pct=SLOW_PCT):
    """Creep forward until the colour sensor is over a black line.
    Returns True if it found one.  Used to cancel odometry drift."""
    covered = await drive_straight(max_cm, pct, stop_on_black=True)
    return abs(covered) < max_cm - 0.5


def resync(axis, value):
    """Snap one coordinate of the pose to a known mat feature, e.g.
    after find_black() on a line whose position you know."""
    fwd_x, fwd_y = unit_forward(heading())
    if axis == "x":
        POSE["x"] = value - fwd_x * COLOUR_FWD_CM
    else:
        POSE["y"] = value - fwd_y * COLOUR_FWD_CM


# =====================================================================
#  COLOUR RECOGNITION
# =====================================================================

def classify(r, g, b, i):
    """Turn a raw sensor reading into a colour name."""
    if i <= BLACK_INTENSITY:
        return "black"
    total = float(r + g + b)
    if total <= 1.0:
        return "black"
    nr, ng, nb = r / total, g / total, b / total

    best, best_d = "unknown", 999.0
    for name, ref in COLOUR_REFS.items():
        d = math.sqrt((nr - ref[0]) ** 2
                      + (ng - ref[1]) ** 2
                      + (nb - ref[2]) ** 2)
        if d < best_d:
            best, best_d = name, d

    # white and black have the same flat spectrum, only brightness tells
    # them apart, so decide those two on intensity alone
    if best in ("white", "black"):
        best = "white" if i >= WHITE_INTENSITY else "black"
    elif best_d > COLOUR_MAX_DIST:
        best = "unknown"
    return best


async def read_colour(samples=COLOUR_SAMPLES):
    """Average several readings and return (name, r, g, b, intensity)."""
    sr = sg = sb = si = 0
    for _ in range(samples):
        r, g, b, i = rgbi()
        sr += r
        sg += g
        sb += b
        si += i
        await sleep_ms(15)
    n = float(samples)
    r, g, b, i = sr / n, sg / n, sb / n, si / n
    return (classify(r, g, b, i), r, g, b, i)


# =====================================================================
#  TOOLS  -  grabber (C) and lift (A)
# =====================================================================

async def run_tool_to(letter, target_deg, pct=TOOL_PCT, tol=TOOL_TOL_DEG,
                      timeout_s=TOOL_TIMEOUT_S, motor_sign=1):
    """Proportional move of a tool motor to an encoder position, with
    stall detection so nothing burns out against an end stop."""
    t0 = ticks_ms()
    slow_since = None
    while True:
        err = target_deg - motor_sign * m_deg(letter)
        if abs(err) <= tol:
            break
        if ticks_diff(ticks_ms(), t0) > timeout_s * 1000:
            break
        power = clamp(0.9 * err, -pct, pct)
        if abs(power) < 18:
            power = 18 * sign(power)
        m_run(letter, motor_sign * power)

        if abs(m_speed(letter)) < 20:
            if slow_since is None:
                slow_since = ticks_ms()
            elif ticks_diff(ticks_ms(), slow_since) > STALL_MS:
                break                      # hit an end stop or the block
        else:
            slow_since = None
        await sleep_ms(10)
    m_stop(letter, hold=True)
    await sleep_ms(50)


async def home_tool(letter, motor_sign, pct=25, timeout_s=2.5):
    """Drive a tool gently against its end stop and call that zero."""
    t0 = ticks_ms()
    slow_since = None
    m_run(letter, -motor_sign * pct)
    await sleep_ms(250)                    # let it get moving first
    while ticks_diff(ticks_ms(), t0) < timeout_s * 1000:
        if abs(m_speed(letter)) < 20:
            if slow_since is None:
                slow_since = ticks_ms()
            elif ticks_diff(ticks_ms(), slow_since) > STALL_MS:
                break
        else:
            slow_since = None
        await sleep_ms(10)
    m_stop(letter, hold=True)
    await sleep_ms(80)
    m_reset(letter)


async def grab_open():
    await run_tool_to(PORT_GRAB, GRAB_OPEN_DEG, motor_sign=GRAB_SIGN)


async def grab_close():
    await run_tool_to(PORT_GRAB, GRAB_CLOSED_DEG, motor_sign=GRAB_SIGN)


async def lift_to(position_deg):
    await run_tool_to(PORT_LIFT, position_deg, motor_sign=LIFT_SIGN,
                      timeout_s=TOOL_TIMEOUT_S + 1.0)


def has_block():
    """True if the grabber stalled on something instead of closing
    onto itself."""
    return GRAB_SIGN * m_deg(PORT_GRAB) < GRAB_EMPTY_DEG


async def pick_block():
    """Nose in on a block, close the grabber, lift and back off.
    Returns True if a block is actually in the jaws."""
    face = heading()
    await lift_to(LIFT_DOWN_DEG)

    for attempt in range(PICK_RETRIES + 1):
        await grab_open()
        reach = DEPOT_APPROACH_CM + (2.5 * attempt)   # dig deeper on a retry
        await drive_straight(reach, SLOW_PCT, hold_heading=face)
        await grab_close()
        if has_block():
            await lift_to(LIFT_CARRY_DEG)
            await drive_straight(-reach, DRIVE_PCT, hold_heading=face)
            return True
        print("grab missed (grabber at {} deg), attempt {}".format(
            GRAB_SIGN * m_deg(PORT_GRAB), attempt + 1))
        await drive_straight(-reach, SLOW_PCT, hold_heading=face)

    await grab_open()
    await lift_to(LIFT_CARRY_DEG)
    return False


async def place_block():
    """Put the carried block down where the grabber is now."""
    await lift_to(LIFT_DOWN_DEG)
    await grab_open()
    await drive_straight(-6.0, SLOW_PCT)
    await lift_to(LIFT_CARRY_DEG)


# =====================================================================
#  MOSAIC PLATE GEOMETRY
# =====================================================================

def cell_xy(row, col):
    """World position of one cell of the mosaic.
    Row 0 is the far side (larger Y), col 0 is the left side."""
    x = PLATE_X + (col - (GRID_COLS - 1) / 2.0) * CELL_PITCH_CM
    y = PLATE_Y + ((GRID_ROWS - 1) / 2.0 - row) * CELL_PITCH_CM
    return (x, y)


async def scan_pattern():
    """Sweep the colour sensor over every cell and read the mosaic.
    The robot always faces +Y (heading 0) while scanning, and works one
    column at a time from the near side to the far side."""
    say("SCAN")
    pattern = []
    for _ in range(GRID_ROWS):
        pattern.append(["unknown"] * GRID_COLS)

    for col in range(GRID_COLS):
        # line up below the column, sensor on the near-most cell
        near_row = GRID_ROWS - 1
        cx, cy = cell_xy(near_row, col)
        stand_x, stand_y = tool_pose(cx, cy, 0.0, COLOUR_FWD_CM, COLOUR_SIDE_CM)
        await goto(stand_x, stand_y - 6.0, DRIVE_PCT, final_heading=0.0)
        await drive_straight(6.0, SLOW_PCT)

        for row in range(GRID_ROWS - 1, -1, -1):
            if row != GRID_ROWS - 1:
                await drive_straight(CELL_PITCH_CM, SLOW_PCT)
            name, r, g, b, i = await read_colour()
            pattern[row][col] = name
            print("cell r{} c{} = {}  rgbi={} {} {} {}".format(
                row, col, name, int(r), int(g), int(b), int(i)))

        # reverse back out of the plate the way we came in
        await drive_straight(-(CELL_PITCH_CM * (GRID_ROWS - 1) + 8.0),
                             DRIVE_PCT)

    print("pattern:", pattern)
    return pattern


# =====================================================================
#  MISSION PLANNING
# =====================================================================

def plan_tasks(pattern):
    """Turn the pattern into an ordered list of (colour, row, col) jobs.
    Jobs are grouped by colour so the robot makes one trip per depot
    visit, and inside a colour they are ordered near-side first."""
    tasks = []
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            colour = pattern[row][col]
            if colour in SKIP_COLOURS:
                continue
            if colour not in DEPOTS:
                print("no depot for", colour, "- skipping cell", row, col)
                continue
            tasks.append((colour, row, col))

    ordered = []
    for colour in ("yellow", "blue", "green", "white"):
        group = [t for t in tasks if t[0] == colour]
        group.sort(key=lambda t: -t[1])        # nearest row first
        ordered.extend(group)
    # anything with a depot but not in the list above
    ordered.extend([t for t in tasks if t not in ordered])
    return ordered


def nearest_depot(colour):
    """Pick the depot of that colour that is closest to us right now."""
    options = DEPOTS[colour]
    best, best_d = options[0], 1e9
    for opt in options:
        d = distance_to(POSE["x"], POSE["y"], opt["stand"][0], opt["stand"][1])
        if d < best_d:
            best, best_d = opt, d
    return best


def lane_for(y):
    return LANE_LOW_Y if y < MAT_WIDTH_CM / 2.0 else LANE_HIGH_Y


def path_to(x, y):
    """Waypoints from the current pose to (x, y) that keep clear of the
    mosaic plate by going around it through a travel lane."""
    sx, sy = POSE["x"], POSE["y"]
    plate_zone_x = (PLATE_X - 25.0, PLATE_X + 25.0)
    crossing = ((sx < plate_zone_x[0] and x > plate_zone_x[1])
                or (sx > plate_zone_x[1] and x < plate_zone_x[0]))
    if not crossing:
        return [(x, y)]
    lane = lane_for((sy + y) / 2.0)
    return [(sx, lane), (x, lane), (x, y)]


async def fetch(colour):
    """Drive to a depot of that colour and pick up a block.
    Returns True if the robot is now carrying one."""
    say(colour[:5].upper())
    depot = nearest_depot(colour)
    dx, dy = depot["stand"]
    await route(path_to(dx, dy), DRIVE_PCT, final_heading=depot["face"])
    return await pick_block()


async def deliver(colour, row, col):
    """Carry the block to its cell and drop it there."""
    cx, cy = cell_xy(row, col)
    # always approach the plate from the near side, facing +Y
    stand_x, stand_y = tool_pose(cx, cy, 0.0, GRAB_FWD_CM, GRAB_SIDE_CM)
    await route(path_to(stand_x, PLATE_APPROACH_Y), DRIVE_PCT,
                final_heading=0.0)
    await drive_straight(stand_y - POSE["y"], SLOW_PCT, hold_heading=0.0)
    await place_block()
    # retreat to the travel lane before doing anything else
    await drive_straight(PLATE_LEAVE_Y - POSE["y"], DRIVE_PCT,
                         hold_heading=0.0)
    print("placed", colour, "at", row, col)


async def go_home():
    say("HOME")
    await lift_to(LIFT_CARRY_DEG)
    await route(path_to(START_X, START_Y), DRIVE_PCT, final_heading=START_H)


# =====================================================================
#  START UP
# =====================================================================

async def startup():
    say("INIT")
    print("robot {}x{}x{} cm, track {} cm, wheel {} cm, {} deg/cm".format(
        ROBOT_LENGTH_CM, ROBOT_WIDTH_CM, 25, AXLE_TRACK_CM,
        WHEEL_DIAMETER_CM, round(DEG_PER_CM, 2)))
    print("mat {}x{} cm, start ({}, {}) heading {}".format(
        MAT_LENGTH_CM, MAT_WIDTH_CM, START_X, START_Y, START_H))
    print("api:", "SPIKE App 3" if NEW_API else "SPIKE App 2 (legacy)")
    yaw_reset()
    await sleep_ms(400)                # let the gyro settle before moving
    set_pose(START_X, START_Y, START_H)
    drive_reset()
    await home_tool(PORT_GRAB, GRAB_SIGN)
    await home_tool(PORT_LIFT, LIFT_SIGN)
    await grab_open()
    await lift_to(LIFT_CARRY_DEG)
    say("GO")


# =====================================================================
#  MISSION
# =====================================================================

async def mission():
    t_start = ticks_ms()
    await startup()

    if PATTERN_SOURCE == "SCAN":
        pattern = await scan_pattern()
        print("scan took {:.1f}s".format(
            ticks_diff(ticks_ms(), t_start) / 1000.0))
    else:
        pattern = FIXED_PATTERN

    tasks = plan_tasks(pattern)
    print("tasks:", tasks)

    done = 0
    trip_estimate = TRIP_ESTIMATE_S
    for colour, row, col in tasks:
        elapsed = ticks_diff(ticks_ms(), t_start) / 1000.0
        # do not start a trip we cannot finish - park instead
        if elapsed + trip_estimate > MISSION_TIMEOUT_S:
            say("TIME")
            print("stopping after {:.0f}s, {} cells left".format(
                elapsed, len(tasks) - done))
            break
        t_task = ticks_ms()
        if not await fetch(colour):
            # nothing in the jaws - do not waste a trip to the plate
            print("skipping", colour, row, col, "- no block picked up")
            continue
        await deliver(colour, row, col)
        done += 1
        took = ticks_diff(ticks_ms(), t_task) / 1000.0
        trip_estimate = took * 1.1            # learn the real trip cost
        print("trip {} {} r{} c{} took {:.1f}s".format(
            done, colour, row, col, took))

    await go_home()
    drive_stop(False)
    say(str(done))
    print("finished, blocks placed:", done,
          "time:", ticks_diff(ticks_ms(), t_start) / 1000.0, "s")


# =====================================================================
#  CALIBRATION AND TEST MODES
# =====================================================================

async def cal_colour():
    """Hold the sensor over a block and read the normalised values.
    Paste the printed triple into COLOUR_REFS."""
    print("CAL_COLOUR - hold the sensor 5-10 mm over a surface")
    while True:
        name, r, g, b, i = await read_colour()
        total = float(r + g + b) or 1.0
        print("raw {} {} {} i={}   norm ({:.2f}, {:.2f}, {:.2f})   -> {}"
              .format(int(r), int(g), int(b), int(i),
                      r / total, g / total, b / total, name))
        say(name[:5].upper())
        await sleep_ms(700)


async def cal_yaw():
    """Turn the robot 90 degrees to its RIGHT by hand.  If the printed
    yaw goes positive, YAW_SIGN is correct."""
    yaw_reset()
    print("CAL_YAW - turn the robot by hand, watch the numbers")
    for _ in range(120):
        print("yaw =", yaw_deg(), " heading =", heading())
        await sleep_ms(500)


async def test_drive():
    await startup()
    print("50 cm forward")
    await drive_straight(50.0)
    await sleep_ms(800)
    print("pose after out:", POSE)
    print("50 cm back")
    await drive_straight(-50.0)
    print("pose after back:", POSE, "heading", heading())


async def test_turn():
    await startup()
    for target in (90.0, 180.0, 270.0, 0.0):
        await turn_to(START_H + target)
        print("asked", target, "got", wrap180(heading() - START_H))
        await sleep_ms(600)


async def test_tools():
    say("TOOL")
    await home_tool(PORT_GRAB, GRAB_SIGN)
    await home_tool(PORT_LIFT, LIFT_SIGN)
    for _ in range(2):
        await grab_close()
        await sleep_ms(400)
        await grab_open()
        await sleep_ms(400)
    for pos in (LIFT_CARRY_DEG, LIFT_UP_DEG, LIFT_DOWN_DEG):
        await lift_to(pos)
        await sleep_ms(400)


async def test_scan():
    await startup()
    pattern = await scan_pattern()
    print("tasks would be:", plan_tasks(pattern))
    await go_home()


async def main():
    if MODE == "MISSION":
        await mission()
    elif MODE == "CAL_COLOUR":
        await cal_colour()
    elif MODE == "CAL_YAW":
        await cal_yaw()
    elif MODE == "TEST_DRIVE":
        await test_drive()
    elif MODE == "TEST_TURN":
        await test_turn()
    elif MODE == "TEST_TOOLS":
        await test_tools()
    elif MODE == "TEST_SCAN":
        await test_scan()
    else:
        print("unknown MODE:", MODE)
    drive_stop(False)


start_program(main())
