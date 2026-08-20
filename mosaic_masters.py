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
#      A : left drive wheel
#      B : grabber (bottom)
#      C : up / down mechanism (lift)
#      E : right drive wheel
#      F : colour sensor (pointing down at the mat)
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
PORT_LEFT   = "A"      # left wheel
PORT_GRAB   = "B"      # grabber
PORT_LIFT   = "C"      # up / down mechanism
PORT_RIGHT  = "E"      # right wheel
PORT_COLOUR = "F"      # colour sensor, looking down

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
MAT_WIDTH_CM  = 100.0   # CONFIRM - see README section 5, note 1

# where the robot is placed before the run: x, y, heading
START_X = 16.0
START_Y = 22.0
START_H = 90.0          # facing along the mat, to the right

# --- the black mosaic plate ------------------------------------------
# From the AR tape measurements:
#   * the grey square the plate sits in is 31 x 26 cm
#   * from the middle of the plate's far edge (the "resolute." side of
#     the mat) to that edge of the mat is 37 cm
#   * the grid is 4 cells across by 3 deep, colours yellow / blue /
#     green / white, with one white cell in the middle
PLATE_ZONE_W_CM = 31.0        # grey square, across the mat (X)
PLATE_ZONE_D_CM = 26.0        # grey square, up the mat (Y)
PLATE_FAR_EDGE_TO_MAT_CM = 37.0

GRID_COLS = 4                 # cells across the mat
GRID_ROWS = 3                 # cells up the mat
CELL_PITCH_CM = 5.0           # MEASURE: centre to centre of two cells
PLATE_BORDER_CM = 1.5         # black rim outside the outer cells

# depth of the black plate in Y, worked out from the grid
PLATE_DEPTH_CM = (GRID_ROWS - 1) * CELL_PITCH_CM + 2 * PLATE_BORDER_CM

# X is not pinned down by the measurements yet - see README section 5.
PLATE_X = 95.0
# Y comes straight off the 37 cm measurement, so it stays right even if
# the mat turns out to be wider than MAT_WIDTH_CM says.
PLATE_Y = MAT_WIDTH_CM - PLATE_FAR_EDGE_TO_MAT_CM - PLATE_DEPTH_CM / 2.0

# Turning on the spot sweeps the CORNERS of the chassis, not its nose,
# so the radius that has to clear the plate is the half-diagonal of the
# robot - bigger than any tool sticking out the front.
CHASSIS_SWING_CM = math.sqrt((ROBOT_LENGTH_CM / 2.0) ** 2
                             + (ROBOT_WIDTH_CM / 2.0) ** 2)

# Safe line the robot travels along and turns on when working the plate.
PLATE_APPROACH_Y = (PLATE_Y - PLATE_DEPTH_CM / 2.0
                    - CHASSIS_SWING_CM - 2.0)
PLATE_LEAVE_Y    = PLATE_APPROACH_Y

# Depots.  "stand" is where the robot parks, "face" is the heading it
# takes there, and the block is straight ahead of the grabber.
# There is a set on the left edge and a set on the right edge; the robot
# automatically drives to whichever one is closer.
# "row" is the heading the robot takes to slide along to the next block
# in the same depot, and "pitch" is how far apart those blocks are.  That
# is what lets one visit collect a magazine full instead of just one.
DEPOTS = {
    "yellow": [ {"stand": (30.0, 48.0), "face": 270.0, "row": 0.0},
                {"stand": (176.0, 88.0), "face":  90.0, "row": 180.0} ],
    "blue":   [ {"stand": (30.0, 60.0), "face": 270.0, "row": 0.0},
                {"stand": (176.0, 55.0), "face":  90.0, "row": 180.0} ],
    "green":  [ {"stand": (30.0, 72.0), "face": 270.0, "row": 0.0},
                {"stand": (176.0, 72.0), "face":  90.0, "row": 180.0} ],
    "white":  [ {"stand": (30.0, 86.0), "face": 270.0, "row": 0.0},
                {"stand": (176.0, 36.0), "face":  90.0, "row": 180.0} ],
}
# How far the robot noses in from the "stand" point to close the
# grabber around a block, and how far it backs off afterwards.
DEPOT_APPROACH_CM = 9.0
DEPOT_BLOCK_PITCH_CM = 6.0     # MEASURE: spacing of the blocks in a depot
# How the blocks of one colour are laid out relative to the robot once
# it is facing them:
#   "IN_LINE"      one behind the other, so the robot just keeps driving
#                  forward and sweeps them up - much the faster option
#   "SIDE_BY_SIDE" spread across its path, so it has to step sideways
#                  between blocks ("row" gives the direction)
DEPOT_LAYOUT = "IN_LINE"

# Travel lanes: the two corridors the robot uses to get past the plate.
# They sit exactly on the clear line either side of it, so arriving at
# the plate from a lane costs no extra turn, and the high lane still
# leaves the robot's width inside the mat.
LANE_LOW_Y  = PLATE_APPROACH_Y
LANE_HIGH_Y = min(PLATE_Y + PLATE_DEPTH_CM / 2.0 + CHASSIS_SWING_CM + 2.0,
                  MAT_WIDTH_CM - ROBOT_WIDTH_CM / 2.0 - 2.0)

# --- mission behaviour -----------------------------------------------
# "SCAN"  : read the pattern off the plate at the start of the run
# "FIXED" : use FIXED_PATTERN below (type it in during inspection time)
PATTERN_SOURCE = "SCAN"
# rows far-to-near, columns left-to-right, as the robot sees them
FIXED_PATTERN = [
    ["blue",   "yellow", "green", "yellow"],
    ["yellow", "blue",   "white", "green" ],
    ["blue",   "yellow", "green", "yellow"],
]
# Cells whose colour is one of these are skipped (nothing to deliver).
SKIP_COLOURS = ("black", "unknown", "red")

# --- magazine --------------------------------------------------------
# How many blocks the bay can physically hold.
MAGAZINE_SIZE = 3
# How the mechanism lets go:
#   "ONE_AT_A_TIME" a gate/ratchet frees a single block and holds the
#                   rest, so one trip can serve several cells
#   "ALL_AT_ONCE"   opening the grabber drops everything it is holding
RELEASE_MODE = "ALL_AT_ONCE"

# An all-or-nothing release cannot put three blocks on three different
# cells - they would land in one pile - so a trip may only serve as many
# cells as the robot can let go of separately.  The program works this
# out rather than trusting MAGAZINE_SIZE, so a wrong setting can never
# dump the whole bay onto one cell.
CELLS_PER_TRIP = 1 if RELEASE_MODE == "ALL_AT_ONCE" else MAGAZINE_SIZE
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
LIFT_DOWN_DEG    = 0     # jaws on the mat, ready to take a block
LIFT_CARRY_DEG   = 95    # travelling height
LIFT_UP_DEG      = 160   # fully raised, block tipped back into the bay
LIFT_RELEASE_DEG = 10    # height the block is let go from
# The chassis does not fit over the plate at normal ride height - the
# up/down mechanism has to lift it clear before it drives on.  This is
# the position that gives that clearance, and the robot goes to it
# before every move that crosses the plate.
LIFT_CLEAR_DEG   = 160
PLATE_LIFT_CLEAR = True  # False if your robot can just drive over it
# The colour sensor is bolted to the chassis at the front, on the
# centreline, so lifting the robot clear of the plate lifts the sensor
# away from the mat as well.  This is the compromise height used while
# scanning: high enough to clear the tiles, low enough to still read a
# colour.  If the readings go unreliable at this height, stop scanning
# and set PATTERN_SOURCE = "FIXED" instead.
SCAN_LIFT_DEG    = 120
# Grabber position that frees exactly ONE block from the magazine while
# the rest stay held.  On a build with no separate gate this is just
# GRAB_OPEN_DEG.  Find it with MODE = "TEST_TOOLS".
GRAB_RELEASE_ONE_DEG = 35
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


# ---------------------------------------------------------------------
#  These two routines are the only place the program touches your
#  specific collecting mechanism.  Everything else - routing, ordering,
#  counting, timing - is independent of how the blocks are held, so if
#  the magazine works differently on your build, change these two and
#  nothing else.
# ---------------------------------------------------------------------

async def lift_clear(position=None):
    """Raise the robot clear of the plate before driving onto it."""
    if PLATE_LIFT_CLEAR:
        await lift_to(LIFT_CLEAR_DEG if position is None else position)


async def collect_one(step_cm, face):
    """Drive step_cm onto the next block, close on it and tip it back
    into the magazine.  The robot stays where it is afterwards - the
    caller drives out of the depot once, at the end.

    Returns (picked, distance_advanced)."""
    advanced = 0.0
    await lift_to(LIFT_DOWN_DEG)

    for attempt in range(PICK_RETRIES + 1):
        await grab_open()
        reach = step_cm if attempt == 0 else 2.5   # creep on a retry
        advanced += await drive_straight(reach, SLOW_PCT, hold_heading=face)
        await grab_close()
        if has_block():
            await lift_to(LIFT_UP_DEG)     # block goes back into the bay
            await lift_to(LIFT_DOWN_DEG)   # ready for the next one
            return (True, advanced)
        print("grab missed (grabber at {} deg), attempt {}".format(
            GRAB_SIGN * m_deg(PORT_GRAB), attempt + 1))

    await grab_open()
    return (False, advanced)


async def release_one():
    """Let go of a single block where the grabber is now, keeping hold
    of the rest of the magazine."""
    face = heading()
    await lift_to(LIFT_RELEASE_DEG)
    if RELEASE_MODE == "ALL_AT_ONCE":
        await grab_open()
    else:
        await run_tool_to(PORT_GRAB, GRAB_RELEASE_ONE_DEG,
                          motor_sign=GRAB_SIGN)
    await sleep_ms(150)                    # let it settle onto the cell
    # Raise BEFORE reversing.  Backing off while still low drags the
    # chassis over the tiles that were just placed.
    if PLATE_LIFT_CLEAR:
        await lift_to(LIFT_CLEAR_DEG)
    await drive_straight(-5.0, SLOW_PCT, hold_heading=face)
    await grab_close()                     # hold what is left
    if not PLATE_LIFT_CLEAR:
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

    # Stay at scanning height for the whole sweep.  The staging points
    # are close enough to the plate that even turning there sweeps a
    # chassis corner over it, so the robot must already be raised.
    await lift_clear(SCAN_LIFT_DEG)

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

        # reverse back out to the clear line - far enough that turning
        # away does not sweep a chassis corner over the plate
        await drive_straight(PLATE_APPROACH_Y - POSE["y"], DRIVE_PCT,
                             hold_heading=0.0)

    await lift_to(LIFT_CARRY_DEG)          # back down once safely off
    print("pattern:", pattern)
    return pattern


# =====================================================================
#  MISSION PLANNING
# =====================================================================

def plan_trips(pattern):
    """Turn the pattern into a list of trips: (colour, [cells]).

    Cells are grouped by colour so one depot visit fills the magazine,
    and each trip carries at most MAGAZINE_SIZE blocks.

    Inside a trip the cells are ordered FAR ROW FIRST.  That matters:
    the robot reaches over the near cells to get to the far ones, so
    filling the far row first means it never has to reach across a block
    it has already placed."""
    by_colour = {}
    for row in range(GRID_ROWS):
        for col in range(GRID_COLS):
            colour = pattern[row][col]
            if colour in SKIP_COLOURS:
                continue
            if colour not in DEPOTS:
                print("no depot for", colour, "- skipping cell", row, col)
                continue
            by_colour.setdefault(colour, []).append((row, col))

    trips = []
    for colour in ("yellow", "blue", "green", "white"):
        cells = by_colour.pop(colour, [])
        cells.sort()                       # row 0 (far side) first
        for i in range(0, len(cells), CELLS_PER_TRIP):
            trips.append((colour, cells[i:i + CELLS_PER_TRIP]))
    for colour, cells in by_colour.items():          # any other colour
        cells.sort()
        for i in range(0, len(cells), CELLS_PER_TRIP):
            trips.append((colour, cells[i:i + CELLS_PER_TRIP]))
    return trips


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


def in_plate_keepout(x, y):
    """True if a robot centred here would have part of itself over the
    plate.  The margin is the chassis swing, because the robot is 25 cm
    wide - driving past broadside puts a lot more of it near the plate
    than driving at it nose-first does."""
    half_w = (CELL_PITCH_CM * (GRID_COLS - 1) / 2.0 + PLATE_BORDER_CM
              + CHASSIS_SWING_CM)
    half_h = PLATE_DEPTH_CM / 2.0 + CHASSIS_SWING_CM
    return abs(x - PLATE_X) < half_w and abs(y - PLATE_Y) < half_h


def path_to(x, y):
    """Waypoints from the current pose to (x, y) that keep clear of the
    mosaic plate by going around it through a travel lane."""
    sx, sy = POSE["x"], POSE["y"]

    # walk the straight line and see whether any of it clips the plate
    steps = 12
    clips = False
    for i in range(steps + 1):
        t = i / float(steps)
        if in_plate_keepout(sx + (x - sx) * t, sy + (y - sy) * t):
            clips = True
            break
    if not clips:
        return [(x, y)]

    # Go round by the corners of the keep-out box rather than dropping
    # straight down to the lane, which costs one turn less.
    lane = lane_for((sy + y) / 2.0)
    half_w = (CELL_PITCH_CM * (GRID_COLS - 1) / 2.0 + PLATE_BORDER_CM
              + CHASSIS_SWING_CM)
    left_edge, right_edge = PLATE_X - half_w, PLATE_X + half_w

    points = [(left_edge if sx < PLATE_X else right_edge, lane)]
    if (x < PLATE_X) != (sx < PLATE_X):
        # target is on the far side, so run the length of the lane
        points.append((left_edge if x < PLATE_X else right_edge, lane))
    points.append((x, y))
    return points


async def fetch(colour, wanted):
    """Drive to a depot of that colour and fill the magazine.
    Returns how many blocks are actually on board."""
    say(colour[:5].upper())
    depot = nearest_depot(colour)
    dx, dy = depot["stand"]
    face = depot["face"]
    row_h = depot.get("row", 0.0)
    pitch = depot.get("pitch", DEPOT_BLOCK_PITCH_CM)

    await route(path_to(dx, dy), DRIVE_PCT, final_heading=face)

    got = 0
    into_depot = 0.0        # how far we have driven in, to reverse later
    for i in range(wanted):
        if i > 0 and DEPOT_LAYOUT == "SIDE_BY_SIDE":
            # blocks are spread across our path: back out, step along
            # the row, and line up on the next one
            await drive_straight(-into_depot, DRIVE_PCT, hold_heading=face)
            into_depot = 0.0
            await turn_to(row_h, TURN_PCT, precise=False)
            await drive_straight(pitch, DRIVE_PCT, hold_heading=row_h)
            await turn_to(face, TURN_PCT, precise=True)

        # blocks in line: the first one is DEPOT_APPROACH_CM ahead and
        # each of the rest is one pitch further on, so we never reverse
        step = DEPOT_APPROACH_CM if i == 0 else pitch
        picked, advanced = await collect_one(step, face)
        into_depot += advanced
        if picked:
            got += 1
        else:
            print("no block at position", i + 1, "of the", colour, "depot")
            break                          # ran out of blocks, go deliver

    await drive_straight(-into_depot, DRIVE_PCT, hold_heading=face)
    await lift_to(LIFT_CARRY_DEG)
    print("carrying", got, colour, "block(s)")
    return got


async def deliver(colour, cells):
    """Place one block on each of the cells, far row first."""
    approach_done = False
    for row, col in cells:
        cx, cy = cell_xy(row, col)
        # always approach the plate from the near side, facing +Y
        stand_x, stand_y = tool_pose(cx, cy, 0.0, GRAB_FWD_CM, GRAB_SIDE_CM)
        if not approach_done:
            await route(path_to(stand_x, PLATE_APPROACH_Y), DRIVE_PCT,
                        final_heading=0.0)
            approach_done = True
        else:
            # already lined up in front of the plate: back out to the
            # approach line, slide sideways, come in again
            await drive_straight(PLATE_APPROACH_Y - POSE["y"], DRIVE_PCT,
                                 hold_heading=0.0)
            await goto(stand_x, PLATE_APPROACH_Y, DRIVE_PCT,
                       final_heading=0.0)
        await lift_clear()                 # chassis has to clear the tiles
        # cover most of the run-in at speed, only creep the last bit
        gap = stand_y - POSE["y"]
        if gap > 7.0:
            await drive_straight(gap - 5.0, DRIVE_PCT, hold_heading=0.0)
            await drive_straight(5.0, SLOW_PCT, hold_heading=0.0)
        else:
            await drive_straight(gap, SLOW_PCT, hold_heading=0.0)
        await release_one()
        print("placed", colour, "at", row, col)

    # retreat to the travel lane before doing anything else
    await drive_straight(PLATE_LEAVE_Y - POSE["y"], DRIVE_PCT,
                         hold_heading=0.0)
    await lift_to(LIFT_CARRY_DEG)          # back down once safely off


async def go_home():
    say("HOME")
    await lift_to(LIFT_CARRY_DEG)
    await route(path_to(START_X, START_Y), DRIVE_PCT, final_heading=START_H)


# =====================================================================
#  START UP
# =====================================================================

def geometry_check():
    """Print anything about the measurements that does not add up, so a
    bad number shows up on the bench instead of on the table."""
    plate_near_y = PLATE_Y - PLATE_DEPTH_CM / 2.0
    far_row_y = cell_xy(0, 0)[1]

    # while the sensor is over the far row, where is the chassis?
    front_edge = far_row_y - COLOUR_FWD_CM + ROBOT_LENGTH_CM / 2.0
    if front_edge > plate_near_y:
        overlap = front_edge - plate_near_y
        if PLATE_LIFT_CLEAR:
            print("NOTE: the robot rides {:.1f} cm onto the plate to reach "
                  "the far row,".format(overlap))
            print("      so LIFT_CLEAR_DEG ({}) must give real clearance "
                  "over the tiles.".format(LIFT_CLEAR_DEG))
        else:
            print("WARNING: the robot drives {:.1f} cm onto the plate and "
                  "PLATE_LIFT_CLEAR is off.".format(overlap))
            print("         Either lift it clear, or the colour sensor "
                  "needs to reach")
            print("         {:.1f} cm ahead of the wheels (it is at {:.1f})."
                  .format(far_row_y - plate_near_y + ROBOT_LENGTH_CM / 2.0,
                          COLOUR_FWD_CM))

    # is there room to turn on the approach line without a corner of
    # the chassis sweeping through the plate?
    swing = max(CHASSIS_SWING_CM, GRAB_FWD_CM, COLOUR_FWD_CM) + 2.0
    if plate_near_y - PLATE_APPROACH_Y < swing:
        print("WARNING: PLATE_APPROACH_Y is only {:.1f} cm clear of the "
              "plate; turning".format(plate_near_y - PLATE_APPROACH_Y))
        print("         there sweeps the chassis corners {:.1f} cm."
              .format(swing))

    if PLATE_LIFT_CLEAR:
        print("NOTE: scanning happens at lift {} deg, so calibrate the "
              "colours".format(SCAN_LIFT_DEG))
        print("      at that height, not with the robot sitting down.")

    if RELEASE_MODE == "ALL_AT_ONCE" and MAGAZINE_SIZE > 1:
        print("NOTE: the grabber releases everything at once, so a trip "
              "serves 1 cell,")
        print("      not {}.  Fitting a gate that frees one block at a "
              "time is worth".format(MAGAZINE_SIZE))
        print("      roughly double the cells in the same run time.")


async def startup():
    say("INIT")
    print("robot {}x{}x{} cm, track {} cm, wheel {} cm, {} deg/cm".format(
        ROBOT_LENGTH_CM, ROBOT_WIDTH_CM, 25, AXLE_TRACK_CM,
        WHEEL_DIAMETER_CM, round(DEG_PER_CM, 2)))
    print("mat {}x{} cm, start ({}, {}) heading {}".format(
        MAT_LENGTH_CM, MAT_WIDTH_CM, START_X, START_Y, START_H))
    print("api:", "SPIKE App 3" if NEW_API else "SPIKE App 2 (legacy)")
    geometry_check()
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

    trips = plan_trips(pattern)
    print("{} cells to fill in {} trip(s) of up to {} cell(s)".format(
        sum(len(c) for _, c in trips), len(trips), CELLS_PER_TRIP))
    for colour, cells in trips:
        print("   ", colour, cells)

    done = 0
    cells_left = sum(len(c) for _, c in trips)
    trip_estimate = TRIP_ESTIMATE_S
    for trip_no, (colour, cells) in enumerate(trips):
        elapsed = ticks_diff(ticks_ms(), t_start) / 1000.0
        # do not start a trip we cannot finish - park instead
        if elapsed + trip_estimate > MISSION_TIMEOUT_S:
            say("TIME")
            print("stopping after {:.0f}s, {} cells left".format(
                elapsed, cells_left))
            break

        t_trip = ticks_ms()
        got = await fetch(colour, len(cells))
        if got == 0:
            # empty jaws - do not waste a trip to the plate
            print("skipping", colour, cells, "- depot gave nothing")
            cells_left -= len(cells)
            continue
        await deliver(colour, cells[:got])
        done += got
        cells_left -= got

        took = ticks_diff(ticks_ms(), t_trip) / 1000.0
        # scale the estimate to a full magazine so a short trip does not
        # make the next one look cheaper than it is
        trip_estimate = took * 1.1 * (CELLS_PER_TRIP / float(max(got, 1)))
        print("trip {} - {} x {} took {:.1f}s".format(
            trip_no + 1, got, colour, took))

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
    for colour, cells in plan_trips(pattern):
        print("trip:", colour, cells)
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
