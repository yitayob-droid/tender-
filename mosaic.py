# WRO 2026 "Mosaic Masters" - robot 13
#
#   1  find the black line
#   2  scan the mosaic, one column at a time, and store the pattern
#   3  push blocks into colour order, pick up the 3, place them in a row
#   4  repeat 3 until the mosaic is solved
#
# Ports:  A left wheel   B grabber   C carriage/pusher   E right wheel
#         F colour sensor (front, pointing down)

import runloop
import motor
import color_sensor
from hub import port, motion_sensor, light_matrix

# =====================================================================
#  THE MOSAIC PATTERN - this is the only thing you should need to change
#  when the pattern is different on the day.
#
#      Y yellow    B blue    G green    W white    . leave empty
#
#  Write it out exactly as it looks on the mat, with the row NEAREST the
#  green part of the map at the top. Spaces are ignored, so line it up
#  however is easiest to read. The size of the mosaic is taken from what
#  you type here, so a 3x4 or a 4x4 both just work.
# =====================================================================
PATTERN = """
    Y Y Y
    W B W
    W B W
    G W G
"""

LETTERS = {"Y": "yellow", "B": "blue", "G": "green", "W": "white",
           ".": None, "-": None}


def parse_pattern(text):
    """Turn the letter grid above into pattern[row][col], complaining
    about anything it does not recognise instead of failing later."""
    grid = []
    for lineno, line in enumerate(text.strip().splitlines()):
        row = []
        for ch in line.replace(" ", "").replace("\t", "").upper():
            if ch in LETTERS:
                row.append(LETTERS[ch])
            else:
                print("PATTERN line", lineno + 1, "- unknown letter", ch,
                      "(use Y B G W or .)")
        if row:
            grid.append(row)
    if not grid:
        print("PATTERN is empty")
        return grid
    width = len(grid[0])
    for i, row in enumerate(grid):
        if len(row) != width:
            print("PATTERN line", i + 1, "has", len(row), "tiles but the"
                  " first line has", width)
    return grid


def show_pattern(grid, title):
    print(title)
    back = {v: k for k, v in LETTERS.items() if v}
    for row in grid:
        print("   ", " ".join(back.get(c, ".") for c in row))


TILES = parse_pattern(PATTERN)


# ------------------------------------------------------------------ tune
LEFT, RIGHT, GRAB, CARRIAGE, EYE = port.A, port.E, port.B, port.C, port.F
LEFT_SIGN, RIGHT_SIGN = 1, -1          # flip if it drives backwards / spins

WHEEL_CM = 17.6                        # wheel circumference, MEASURE THIS
FAST, SLOW, SPIN = 400, 180, 250       # deg/s

BLACK, WHITE = 20, 85                  # reflection on line / on mat
LINE_KP = 6.0                          # line-follow gain

# The grabber is three fixed pockets underneath, not a pair of jaws, so
# "open" and "shut" release and capture all three blocks together.
CARRIAGE_DOWN, CARRIAGE_UP = 0, 150    # pusher down / carried clear
JAWS_OPEN, JAWS_SHUT = 0, 95

# Homing: the direction that runs each tool to the end stop that counts
# as zero - carriage all the way down, grabber all the way open. Flip a
# sign if a tool homes the wrong way. HOME_SPEED is deliberately gentle
# because it ends by stalling against a hard stop.
CARRIAGE_HOME_DIR, GRAB_HOME_DIR = -1, -1
HOME_SPEED = 200
TOOL_SPEED = 500                       # deg/s for carriage and grabber moves
TURN_TIMEOUT_MS = 5000                 # a turn that takes longer has a fault

# ---- mosaic. The size is whatever you typed into PATTERN above.
ROWS = len(TILES)
COLS = len(TILES[0]) if TILES else 0
CELL = 5.0                             # cm between cell centres, MEASURE
COL_STEP = CELL                        # columns are one cell apart
STANDOFF = 12.0                        # cm from where MOSAIC parks you to
                                       # the centre of the nearest row

# How far ahead of the wheels each tool sits. The sensor is out on the
# front; the pockets are underneath, near the middle.
SENSOR_FWD = 8.0
POCKET_FWD = 0.0

# ---- depot geometry -------------------------------------------------
# One LEGO block is 31.8 mm. Blocks of the same colour sit one block
# apart, so centre to centre is two blocks. The colour groups sit two
# blocks apart, so group to group is three blocks.
BLOCK = 3.18                           # cm, one block
# "One block away" is one step on the block grid, so same-colour blocks
# sit 3.18 cm centre to centre and the groups two steps beyond that.
# Checked against the mat photo: this puts the four groups 12.7 cm apart
# and they measure 12.4 cm, which the old reading missed by nearly two to
# one.
PITCH = BLOCK                          # 3.18 cm between same-colour blocks
GROUP_GAP = BLOCK * 2                  # 6.36 cm between colour groups

# Where the 6 blocks of one colour sit, as (column, row) in units of
# PITCH. Row 0 is the row nearest the robot. This is the staggered
# layout in the photo - EDIT IT to match your depot.
LAYOUT = [(0, 0), (1, 0), (2, 0),
          (0, 1), (1, 1), (2, 1)]
GROUP_WIDTH = 2 * PITCH                # widest column offset in a group
GROUP_SPAN = GROUP_WIDTH + GROUP_GAP   # start of one group to the next

# Order of the colour groups along the depot, and the heading you travel
# to go from the first to the last. On this mat the groups run UP the
# left edge, not across it, which is what DEPOT_AXIS captures.
GROUPS = ["yellow", "blue", "green", "white"]
DEPOT_AXIS = 14                        # +x of the depot frame
DEPOT_OUT = DEPOT_AXIS + 90            # the way blocks get pushed onto the lane

# The staging lane: a clear line in front of the depot where the three
# blocks get lined up before they are scooped.
LANE_Y = -8.0                          # cm in front of depot row 0

# The three pockets are one brick's short side apart. That is the GAP -
# a block is 3.18 cm wide, so the pockets cannot be 1.59 cm apart or the
# blocks would overlap. Centre to centre is block + gap.
STUD = 0.8
BRICK_SHORT = 2 * STUD                 # 1.59 cm
SLOT_PITCH = BLOCK + BRICK_SHORT       # 4.77 cm between staging slots

# Each place on the mat is (heading, distance) from the junction the
# robot ends up on after finding the line. MEASURE THESE.
HOME = 0                               # heading of the line it works from
# Bearings and distances measured off the overhead photo of the mat,
# from the ROBOMISSION corner where the robot starts. Re-measure once
# find_line() settles on its junction, because these move with it.
DEPOT = (345, 25.7)                    # to the near corner of the depot
MOSAIC = (74, 81.1)                    # to a spot STANDOFF cm in front of
                                       # the NEAREST cell of the LEFTMOST
                                       # column
PICK_CM = 9.0                          # nose-in to close on the lined-up 3
PUSH_STANDOFF = 3.0                    # the pusher parks this far behind a
                                       # block before shoving it

# Colour references: normalised r, g, b. Set MODE = "COLOURS", hold the
# sensor over each block, paste the printed triples in.
COLOURS = {
    "white":  (0.34, 0.34, 0.32),
    "yellow": (0.46, 0.40, 0.14),
    "green":  (0.20, 0.55, 0.25),
    "blue":   (0.16, 0.30, 0.54),
}
DARK = 90                              # below this intensity it is black

# "TYPED" trusts the letter grid at the top of the file and skips the
# scan, which saves about 30 seconds. "SCAN" reads the mat and prints any
# row that disagrees with what you typed.
PATTERN_SOURCE = "SCAN"                # SCAN | TYPED

# RUN      the whole mission
# MOVE     turn each motor on in turn - use this first if nothing moves
# TEST     drive, turn, home, and cycle both tools
# COLOURS  print what the colour sensor sees
MODE = "RUN"


# ------------------------------------------------------------------ basics
def yaw():
    return motion_sensor.tilt_angles()[0] / 10.0


def wheels(l, r):
    motor.run(LEFT, int(LEFT_SIGN * max(min(l, 1000), -1000)))
    motor.run(RIGHT, int(RIGHT_SIGN * max(min(r, 1000), -1000)))


def stop():
    motor.stop(LEFT)
    motor.stop(RIGHT)


def error_to(heading):
    return (heading - yaw() + 180) % 360 - 180


async def turn_to(heading, tol=0.5):
    """Spin on the spot to a gyro heading.

    The tolerance is tight on purpose. Every leg of this program starts
    with a turn, and at 1.5 degrees the leftover error piles up into
    something like 20 cm of drift across a run. It has to settle inside
    the band rather than just touch it, or the robot hunts."""
    settled = 0
    t = 0
    while True:
        if t > TURN_TIMEOUT_MS:
            print("turn to %.0f gave up %.1f deg out - check the gyro and"
                  " the wheel signs" % (heading, error_to(heading)))
            break
        t += 10
        e = error_to(heading)
        if abs(e) < tol:
            stop()
            settled += 1
            if settled >= 5:                       # 50 ms inside the band
                break
            await runloop.sleep_ms(10)
            continue
        settled = 0
        p = max(min(e * 6, SPIN), -SPIN)
        floor = 60 if abs(e) < 5 else 90           # creep when close
        if abs(p) < floor:
            p = floor if p > 0 else -floor
        wheels(p, -p)
        await runloop.sleep_ms(10)
    stop()
    await runloop.sleep_ms(60)


async def drive(cm, speed=FAST, heading=None):
    """Straight line, gyro corrected. Negative cm reverses."""
    if abs(cm) < 0.2:
        return
    if heading is None:
        heading = yaw()
    motor.reset_relative_position(LEFT, 0)
    goal = abs(cm) / WHEEL_CM * 360.0
    way = 1 if cm > 0 else -1
    t, limit = 0, int(abs(cm) / 3.0 * 1000) + 3000     # 3 cm/s worst case
    while abs(motor.relative_position(LEFT)) < goal:
        if t > limit:
            done = abs(motor.relative_position(LEFT)) / 360.0 * WHEEL_CM
            print("drive stalled: %.1f cm of %.1f - blocked, or a wheel is"
                  " on the wrong port" % (done, abs(cm)))
            break
        t += 10
        c = error_to(heading) * 5.0
        wheels(way * speed + c, way * speed - c)
        await runloop.sleep_ms(10)
    stop()
    await runloop.sleep_ms(60)


async def follow_line(cm, speed=SLOW):
    """Step 1's payoff: track the edge of the black line for a distance."""
    mid = (BLACK + WHITE) / 2.0
    motor.reset_relative_position(LEFT, 0)
    goal = cm / WHEEL_CM * 360.0
    t, limit = 0, int(cm / 3.0 * 1000) + 3000
    while abs(motor.relative_position(LEFT)) < goal:
        if t > limit:
            print("line following stalled")
            break
        t += 10
        c = (mid - color_sensor.reflection(EYE)) * LINE_KP
        wheels(speed + c, speed - c)
        await runloop.sleep_ms(10)
    stop()
    await runloop.sleep_ms(60)


async def find_line(max_cm=60.0, speed=SLOW):
    """STEP 1. Creep forward until the sensor is over the black line.

    If there is no line within reach, back up to where the search
    started. Every position in this program is measured from that spot,
    so leaving the robot 60 cm up the mat would throw off everything
    after it."""
    light_matrix.write("1")
    motor.reset_relative_position(LEFT, 0)
    goal = max_cm / WHEEL_CM * 360.0
    while color_sensor.reflection(EYE) > BLACK + 10:
        if abs(motor.relative_position(LEFT)) > goal:
            stop()
            gone = abs(motor.relative_position(LEFT)) / 360.0 * WHEEL_CM
            print("no line within %.0f cm - backing up %.0f cm to the start"
                  % (max_cm, gone))
            await drive(-gone, SLOW)
            return False
        wheels(speed, speed)
        await runloop.sleep_ms(10)
    stop()
    await runloop.sleep_ms(60)
    return True


def read_colour():
    r, g, b, i = color_sensor.rgbi(EYE)
    if i < DARK:
        return "black"
    t = float(r + g + b) or 1.0
    best, best_d = "?", 9.0
    for name, ref in COLOURS.items():
        d = (abs(r / t - ref[0]) + abs(g / t - ref[1]) + abs(b / t - ref[2]))
        if d < best_d:
            best, best_d = name, d
    return best


async def home(m, direction, timeout_ms=2500):
    """Run a tool gently against its end stop and call that zero, so it
    does not matter what position the robot is handed over in."""
    motor.run(m, direction * HOME_SPEED)
    await runloop.sleep_ms(300)                    # let it get moving
    t = 0
    while t < timeout_ms and abs(motor.velocity(m)) > 20:
        await runloop.sleep_ms(20)
        t += 20
    motor.stop(m)
    await runloop.sleep_ms(150)
    motor.reset_relative_position(m, 0)


async def run_tool(m, target, name, tol=6, timeout_ms=3000):
    """Drive a tool motor to a position under our own control.

    run_to_relative_position() blocks until it arrives, so if the
    mechanism cannot reach the number it is given the whole program stops
    dead with the robot sitting still. This gives up instead, and says
    so."""
    t, slow = 0, 0
    while t < timeout_ms:
        err = target - motor.relative_position(m)
        if abs(err) <= tol:
            break
        p = max(min(err * 4, TOOL_SPEED), -TOOL_SPEED)
        if abs(p) < 120:
            p = 120 if p > 0 else -120
        motor.run(m, int(p))
        if abs(motor.velocity(m)) < 20:
            slow += 20
            if slow > 400:                          # jammed on something
                print(name, "stalled at", motor.relative_position(m),
                      "on the way to", target)
                break
        else:
            slow = 0
        await runloop.sleep_ms(20)
        t += 20
    else:
        print(name, "did not reach", target, "- stopped at",
              motor.relative_position(m))
    motor.stop(m)
    await runloop.sleep_ms(50)


async def carriage(pos):
    await run_tool(CARRIAGE, pos, "carriage")


async def jaws(pos):
    await run_tool(GRAB, pos, "grabber")


def check_ports():
    """Name the device that is not answering, instead of the program
    dying on its first move with nothing on the display."""
    ok = True
    for name, p in (("A left wheel", LEFT), ("E right wheel", RIGHT),
                    ("B grabber", GRAB), ("C carriage", CARRIAGE)):
        try:
            motor.relative_position(p)
        except Exception as e:
            print("PORT", name, "is not answering -", e)
            ok = False
    try:
        color_sensor.reflection(EYE)
    except Exception as e:
        print("PORT F colour sensor is not answering -", e)
        ok = False
    if ok:
        print("all five ports answered")
    return ok


async def move_test():
    """The simplest question: does anything turn at all? Each motor runs
    on its own for a second, and prints how far it actually got."""
    check_ports()
    for name, m in (("A left wheel", LEFT), ("E right wheel", RIGHT),
                    ("B grabber", GRAB), ("C carriage", CARRIAGE)):
        print("running", name, "...")
        motor.reset_relative_position(m, 0)
        motor.run(m, 300)
        await runloop.sleep_ms(1000)
        motor.stop(m)
        await runloop.sleep_ms(300)
        moved = motor.relative_position(m)
        print("   ", name, "moved", moved, "degrees",
              "" if abs(moved) > 50 else "  <-- BARELY MOVED, check this one")
    print("now both wheels together - the robot should drive forwards")
    wheels(300, 300)
    await runloop.sleep_ms(1500)
    stop()


# ------------------------------------------------------------------ step 2
async def scan_mosaic():
    """STEP 2. Sensor on the leftmost column, drive forward reading each
    cell, back out, shift right, repeat for middle and rightmost.
    Returns pattern[row][col], row 0 = nearest the robot."""
    light_matrix.write("2")
    pattern = [["?"] * COLS for _ in range(ROWS)]
    await go(MOSAIC)
    await turn_to(HOME)                            # square up to the grid
    lane = HOME                                    # heading down the columns
    first = STANDOFF - SENSOR_FWD                  # sensor onto the near row

    for col in range(COLS):
        if col:
            await turn_to(lane + 90)               # shift one column right
            await drive(COL_STEP, SLOW)
            await turn_to(lane)
        await drive(first, SLOW, lane)
        for row in range(ROWS):
            if row:
                await drive(CELL, SLOW, lane)
            pattern[row][col] = read_colour()
        await drive(-(first + CELL * (ROWS - 1)), FAST, lane)

    await turn_to(lane + 270)                      # back to the first column
    await drive(COL_STEP * (COLS - 1), SLOW)
    await go_back(MOSAIC)
    print("pattern", pattern)
    return pattern


# ------------------------------------------------------------------ step 3
async def go(spot):
    """Turn and drive out to a place on the mat."""
    heading, distance = spot
    await turn_to(heading)
    await drive(distance, FAST)


async def go_back(spot):
    """Reverse the trip and face down the line again."""
    heading, distance = spot
    await turn_to(heading + 180)
    await drive(distance, FAST)
    await turn_to(HOME)


# --------------------------------------------------- the depot as a map
# Every block has an (x, y) in depot coordinates, in cm:
#   x runs along the depot, left to right, 0 at the first yellow block
#   y runs away from the robot, 0 at the row nearest it
# So the whole depot is known relative to itself, and the robot only has
# to find ONE corner of it on the mat.

def block_xy(colour, n):
    """Where block n (0-5) of this colour sits, in depot coordinates."""
    col, row = LAYOUT[n]
    return (GROUPS.index(colour) * GROUP_SPAN + col * PITCH, row * PITCH)


STOCK = {c: [True] * len(LAYOUT) for c in GROUPS}     # what is left


def nearest_block(colour, to_x, taken):
    """Pick the block of this colour that needs the least sideways
    shoving to reach to_x, taking from the front row first because the
    robot can reach those without driving through the depot. `taken`
    stops the same block being chosen twice inside one row."""
    best, best_cost = None, 1e9
    for n in range(len(LAYOUT)):
        if not STOCK[colour][n] or (colour, n) in taken:
            continue
        bx, by = block_xy(colour, n)
        cost = abs(bx - to_x) + by * 2.0      # back rows cost double
        if cost < best_cost:
            best, best_cost = n, cost
    return best


def median(values):
    v = sorted(values)
    return v[len(v) // 2]


def plan_row(colours):
    """Work out which three blocks to use and where to line them up.

    The three have to end up touching, in order, somewhere on the lane -
    but nothing says WHERE on the lane. So we put the lineup wherever it
    makes the sideways pushing shortest, which is the median of the
    three blocks' own positions (median, not average, because what we
    are minimising is total distance, and for that the median is exact).

    Done in two passes: guess a spot from the colour groups, pick the
    real blocks nearest their slots, then re-centre on the blocks we
    actually chose."""
    # a cell the sensor could not read is skipped rather than crashing
    colours = [c for c in colours if c in GROUPS]
    if not colours:
        print("no readable colours in this row")
        return None
    mid = (len(colours) - 1) / 2.0

    def group_centre(colour):
        return GROUPS.index(colour) * GROUP_SPAN + GROUP_WIDTH / 2.0

    # pass 1 - a rough spot from where the colour groups sit
    spot = median([group_centre(c) - (j - mid) * SLOT_PITCH
                   for j, c in enumerate(colours)])

    # pass 2 - choose the real blocks, never the same one twice
    chosen, taken = [], []
    for j, colour in enumerate(colours):
        n = nearest_block(colour, spot + (j - mid) * SLOT_PITCH, taken)
        if n is None:
            print("out of", colour)
            return None
        taken.append((colour, n))
        chosen.append((colour, n))

    # pass 3 - re-centre on the blocks we actually picked
    spot = median([block_xy(c, n)[0] - (j - mid) * SLOT_PITCH
                   for j, (c, n) in enumerate(chosen)])
    return [(c, n, spot + (j - mid) * SLOT_PITCH)
            for j, (c, n) in enumerate(chosen)]


def push_cost(plan):
    """Total driving for one ordering of the pushes, so we can compare."""
    total, at = 0.0, (0.0, 0.0)
    for colour, n, slot_x in plan:
        bx, by = block_xy(colour, n)
        total += abs(bx - at[0]) + abs(by - at[1])     # drive to the block
        total += abs(bx - slot_x) + abs(by - LANE_Y)   # the push itself
        at = (slot_x, LANE_Y)
    return total


def order_pushes(plan):
    """Innermost slot first, then outwards. The middle block needs no
    sideways travel at all, and doing it first gives the outer ones
    something to stop against instead of being shoved through an empty
    slot. Sorting by distance from the centre also copes with a row that
    has fewer than three readable colours."""
    centre = sum(p[2] for p in plan) / float(len(plan))
    return sorted(plan, key=lambda p: abs(p[2] - centre))


# ------------------------------------------------- driving in the depot
# Inside the depot the robot moves on the block grid: along x, then
# along y. Never diagonally, because a diagonal cuts through the gaps
# between blocks and knocks them over.

AT = [0.0, 0.0]                        # where the robot is in the depot


async def depot_goto(x, y):
    """Move on the block grid: along the depot, then across it. Never
    diagonally - a diagonal cuts through the gaps between blocks."""
    await turn_to(DEPOT_AXIS)
    await drive(x - AT[0], FAST, DEPOT_AXIS)
    await turn_to(DEPOT_OUT - 180)
    await drive(y - AT[1], FAST, DEPOT_OUT - 180)
    AT[0], AT[1] = x, y


async def push_block(colour, n, slot_x):
    """Shove one block out onto the lane and along to its slot.

    The three pockets ARE the pusher, so the carriage only comes down
    for the push stroke itself. Driving across the depot with it down
    would plough through every other block on the way."""
    bx, by = block_xy(colour, n)

    # line up behind the block with the pusher held clear
    await carriage(CARRIAGE_UP)
    await depot_goto(bx, by + PUSH_STANDOFF)       # pusher just behind it
    await turn_to(DEPOT_OUT)

    # push stroke: down, drive, back up. The robot travels exactly as far
    # as the block does, so it ends one standoff short of the lane.
    await carriage(CARRIAGE_DOWN)
    await drive(by - LANE_Y, SLOW, DEPOT_OUT)
    AT[1] = LANE_Y + PUSH_STANDOFF
    await carriage(CARRIAGE_UP)

    # then along the lane into its slot, if it is not already there
    if abs(slot_x - bx) > 0.3:
        way = 1 if slot_x > bx else -1
        side = DEPOT_AXIS if way > 0 else DEPOT_AXIS + 180
        # Line up one standoff behind the block. The robot then travels
        # the same distance the block does, so it finishes one standoff
        # short of the slot - NOT at the slot.
        await depot_goto(bx - way * PUSH_STANDOFF, LANE_Y + PUSH_STANDOFF)
        await turn_to(side)
        await carriage(CARRIAGE_DOWN)
        await drive(abs(slot_x - bx), SLOW, side)
        AT[0] = slot_x - way * PUSH_STANDOFF
        await carriage(CARRIAGE_UP)

    STOCK[colour][n] = False


# ------------------------------------------------------------------ step 3
async def collect_and_place(row, colours):
    """STEP 3. Order three blocks on the lane, scoop all three, put them
    in the mosaic row."""
    light_matrix.write("3")
    plan = plan_row(colours)
    if plan is None:
        return
    plan = order_pushes(plan)
    print("row", row, colours, "push %.0f cm" % push_cost(plan))

    await go(DEPOT)
    AT[0], AT[1] = 0.0, 0.0
    for colour, n, slot_x in plan:
        await push_block(colour, n, slot_x)

    # Scoop all three at once. order_pushes() puts the middle slot first,
    # so plan[0] is where the centre pocket has to end up.
    # stand PICK_CM back on the depot side of the lane, then drive onto
    # it. Standing on the far side and driving out again put the robot
    # two pick-lengths past the lane and lost the depot origin.
    await depot_goto(plan[0][2], LANE_Y + PICK_CM)
    await turn_to(DEPOT_OUT)
    await carriage(CARRIAGE_DOWN)
    await jaws(JAWS_OPEN)
    await drive(PICK_CM, SLOW, DEPOT_OUT)
    await jaws(JAWS_SHUT)
    AT[1] = LANE_Y
    await carriage(CARRIAGE_UP)
    await depot_goto(0.0, 0.0)                     # back to the depot corner
    await go_back(DEPOT)

    await go(MOSAIC)                               # and into the row
    await turn_to(HOME)                            # square up to the grid
    # the pockets, not the sensor, have to land on the cells
    reach = STANDOFF - POCKET_FWD + CELL * row
    await drive(reach, SLOW, HOME)
    await carriage(CARRIAGE_DOWN)
    await jaws(JAWS_OPEN)
    await carriage(CARRIAGE_UP)
    await drive(-reach, FAST, HOME)
    await go_back(MOSAIC)
    print("placed row", row, colours)


# ------------------------------------------------------------------ main
async def run():
    show_pattern(TILES, "mosaic to build (%d wide, %d deep):" % (COLS, ROWS))
    if not check_ports():
        print("fix the ports above before running the mission")
        return
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(300)
    await home(CARRIAGE, CARRIAGE_HOME_DIR)        # carriage down = 0
    await home(GRAB, GRAB_HOME_DIR)                # grabber open = 0
    await carriage(CARRIAGE_UP)

    if not await find_line():                      # 1
        print("running off the start position instead of a line junction")
    if PATTERN_SOURCE == "SCAN":                   # 2
        pattern = await scan_mosaic()
        show_pattern(pattern, "scanned:")
        for r in range(min(ROWS, len(pattern))):
            if pattern[r] != TILES[r]:
                print("row", r, "scanned", pattern[r],
                      "but you typed", TILES[r])
    else:
        pattern = TILES
    for row in range(ROWS - 1, -1, -1):            # 3, repeated -> 4
        await collect_and_place(row, pattern[row])

    stop()
    light_matrix.write("OK")


async def show_colours():
    """Hold the sensor over each block and paste the numbers into COLOURS."""
    while True:
        r, g, b, i = color_sensor.rgbi(EYE)
        t = float(r + g + b) or 1.0
        print("({:.2f}, {:.2f}, {:.2f})  i={}  -> {}".format(
            r / t, g / t, b / t, i, read_colour()))
        await runloop.sleep_ms(600)


async def test():
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(300)
    await drive(50)
    await drive(-50)
    await turn_to(90)
    await turn_to(0)
    await home(CARRIAGE, CARRIAGE_HOME_DIR)
    await home(GRAB, GRAB_HOME_DIR)
    print("homed - carriage and grabber are both at 0")
    await carriage(CARRIAGE_UP)
    await carriage(CARRIAGE_DOWN)
    await jaws(JAWS_SHUT)
    await jaws(JAWS_OPEN)


async def main():
    try:
        if MODE == "COLOURS":
            await show_colours()
        elif MODE == "MOVE":
            await move_test()
        elif MODE == "TEST":
            await test()
        else:
            await run()
    except Exception as e:
        # without this the hub just stops and the robot sits there with
        # no clue as to why
        print("STOPPED BY AN ERROR:", e)
        light_matrix.write("ERR")
    stop()
    motor.stop(GRAB)
    motor.stop(CARRIAGE)


runloop.run(main())
