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

# ---- mosaic: 3 wide, 4 deep. A row is 3 blocks = exactly one grabber load
COLS, ROWS = 3, 4
CELL = 5.0                             # cm between cell centres, MEASURE
FIRST_CELL = 8.0                       # cm from the line to the first cell
COL_STEP = 5.0                         # cm sideways between columns

# ---- depot geometry -------------------------------------------------
# One LEGO block is 31.8 mm. Blocks of the same colour sit one block
# apart, so centre to centre is two blocks. The colour groups sit two
# blocks apart, so group to group is three blocks.
BLOCK = 3.18                           # cm, one block
PITCH = BLOCK * 2                      # 6.36 cm between same-colour blocks
GROUP_GAP = BLOCK * 3                  # 9.54 cm between colour groups

# Where the 6 blocks of one colour sit, as (column, row) in units of
# PITCH. Row 0 is the row nearest the robot. This is the staggered
# layout in the photo - EDIT IT to match your depot.
LAYOUT = [(0, 0), (1, 0), (2, 0),
          (0, 1), (1, 1), (2, 1)]
GROUP_WIDTH = 2 * PITCH                # widest column offset in a group
GROUP_SPAN = GROUP_WIDTH + GROUP_GAP   # start of one group to the next

# Order of the colour groups along the depot, left to right
GROUPS = ["yellow", "blue", "green", "white"]

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
DEPOT = (270, 20.0)                    # to the near corner of the depot
MOSAIC = (0, 30.0)                     # to the near edge of the mosaic
PICK_CM = 9.0                          # nose-in to close on the lined-up 3

# Colour references: normalised r, g, b. Set MODE = "COLOURS", hold the
# sensor over each block, paste the printed triples in.
COLOURS = {
    "white":  (0.34, 0.34, 0.32),
    "yellow": (0.46, 0.40, 0.14),
    "green":  (0.20, 0.55, 0.25),
    "blue":   (0.16, 0.30, 0.54),
}
DARK = 90                              # below this intensity it is black

MODE = "RUN"                           # RUN | COLOURS | TEST


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


async def turn_to(heading):
    """Spin on the spot to a gyro heading."""
    while True:
        e = error_to(heading)
        if abs(e) < 1.5:
            break
        p = max(min(e * 6, SPIN), -SPIN)
        if abs(p) < 90:
            p = 90 if p > 0 else -90
        wheels(p, -p)
        await runloop.sleep_ms(10)
    stop()
    await runloop.sleep_ms(80)


async def drive(cm, speed=FAST, heading=None):
    """Straight line, gyro corrected. Negative cm reverses."""
    if abs(cm) < 0.2:
        return
    if heading is None:
        heading = yaw()
    motor.reset_relative_position(LEFT, 0)
    goal = abs(cm) / WHEEL_CM * 360.0
    way = 1 if cm > 0 else -1
    while abs(motor.relative_position(LEFT)) < goal:
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
    while abs(motor.relative_position(LEFT)) < goal:
        c = (mid - color_sensor.reflection(EYE)) * LINE_KP
        wheels(speed + c, speed - c)
        await runloop.sleep_ms(10)
    stop()
    await runloop.sleep_ms(60)


async def find_line(max_cm=60.0, speed=SLOW):
    """STEP 1. Creep forward until the sensor is over the black line."""
    light_matrix.write("1")
    motor.reset_relative_position(LEFT, 0)
    goal = max_cm / WHEEL_CM * 360.0
    while color_sensor.reflection(EYE) > BLACK + 10:
        if abs(motor.relative_position(LEFT)) > goal:
            stop()
            print("no line found")
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


async def carriage(pos):
    await motor.run_to_relative_position(CARRIAGE, pos, 500)


async def jaws(pos):
    await motor.run_to_relative_position(GRAB, pos, 500)


# ------------------------------------------------------------------ step 2
async def scan_mosaic():
    """STEP 2. Sensor on the leftmost column, drive forward reading each
    cell, back out, shift right, repeat for middle and rightmost.
    Returns pattern[row][col], row 0 = nearest the robot."""
    light_matrix.write("2")
    pattern = [["?"] * COLS for _ in range(ROWS)]
    await go(MOSAIC)
    lane = MOSAIC[0]                               # heading down the columns

    for col in range(COLS):
        if col:
            await turn_to(lane + 90)               # shift one column right
            await drive(COL_STEP, SLOW)
            await turn_to(lane)
        await drive(FIRST_CELL, SLOW, lane)
        for row in range(ROWS):
            if row:
                await drive(CELL, SLOW, lane)
            pattern[row][col] = read_colour()
        await drive(-(FIRST_CELL + CELL * (ROWS - 1)), FAST, lane)

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
    def group_centre(colour):
        return GROUPS.index(colour) * GROUP_SPAN + GROUP_WIDTH / 2.0

    # pass 1 - a rough spot from where the three colour groups sit
    spot = median([group_centre(c) - (j - 1) * SLOT_PITCH
                   for j, c in enumerate(colours)])

    # pass 2 - choose the real blocks, never the same one twice
    chosen, taken = [], []
    for j, colour in enumerate(colours):
        n = nearest_block(colour, spot + (j - 1) * SLOT_PITCH, taken)
        if n is None:
            print("out of", colour)
            return None
        taken.append((colour, n))
        chosen.append((colour, n))

    # pass 3 - re-centre on the blocks we actually picked
    spot = median([block_xy(c, n)[0] - (j - 1) * SLOT_PITCH
                   for j, (c, n) in enumerate(chosen)])
    return [(c, n, spot + (j - 1) * SLOT_PITCH)
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
    """Middle slot first, then the outer two, each pushed inward from
    its own side. Doing the middle first means the outer blocks always
    stop against something already in place instead of being shoved
    through an empty slot, and coming in from the outside means one
    block never has to travel through another."""
    mid = [p for p in plan if p[2] == plan[1][2]]
    left = [p for p in plan if p[2] < plan[1][2]]
    right = [p for p in plan if p[2] > plan[1][2]]
    return mid + left + right


# ------------------------------------------------- driving in the depot
# Inside the depot the robot moves on the block grid: along x, then
# along y. Never diagonally, because a diagonal cuts through the gaps
# between blocks and knocks them over.

AT = [0.0, 0.0]                        # where the robot is in the depot


async def depot_goto(x, y):
    await turn_to(HOME + 90)
    await drive(x - AT[0], FAST, HOME + 90)
    await turn_to(HOME)
    await drive(y - AT[1], FAST, HOME)
    AT[0], AT[1] = x, y


async def push_block(colour, n, slot_x):
    """Shove one block out onto the lane and along to its slot, carriage
    DOWN the whole time so the pusher is at block height."""
    bx, by = block_xy(colour, n)
    await carriage(CARRIAGE_DOWN)

    # get behind the block, then push it forward onto the lane
    await depot_goto(bx, by + PITCH)
    await turn_to(HOME + 180)
    await drive(by + PITCH - LANE_Y, SLOW, HOME + 180)
    AT[1] = LANE_Y

    # then along the lane into its slot, if it is not already there
    if abs(slot_x - bx) > 0.3:
        side = HOME + 90 if slot_x > bx else HOME + 270
        await depot_goto(bx - (slot_x - bx), LANE_Y)   # line up behind it
        await turn_to(side)
        await drive(abs(slot_x - bx), SLOW, side)
        AT[0] = slot_x

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
    await depot_goto(plan[0][2], LANE_Y - PICK_CM)
    await turn_to(HOME + 180)
    await carriage(CARRIAGE_DOWN)
    await jaws(JAWS_OPEN)
    await drive(PICK_CM, SLOW, HOME + 180)
    await jaws(JAWS_SHUT)
    await carriage(CARRIAGE_UP)
    await go_back(DEPOT)

    await go(MOSAIC)                               # and into the row
    await drive(FIRST_CELL + CELL * row, SLOW, MOSAIC[0])
    await carriage(CARRIAGE_DOWN)
    await jaws(JAWS_OPEN)
    await carriage(CARRIAGE_UP)
    await drive(-(FIRST_CELL + CELL * row), FAST, MOSAIC[0])
    await go_back(MOSAIC)
    print("placed row", row, colours)


# ------------------------------------------------------------------ main
async def run():
    motion_sensor.reset_yaw(0)
    await runloop.sleep_ms(300)
    motor.reset_relative_position(CARRIAGE, 0)
    motor.reset_relative_position(GRAB, 0)
    await carriage(CARRIAGE_UP)
    await jaws(JAWS_OPEN)

    await find_line()                              # 1
    pattern = await scan_mosaic()                  # 2
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
    motor.reset_relative_position(CARRIAGE, 0)
    motor.reset_relative_position(GRAB, 0)
    await carriage(CARRIAGE_UP)
    await carriage(CARRIAGE_DOWN)
    await jaws(JAWS_SHUT)
    await jaws(JAWS_OPEN)


async def main():
    if MODE == "COLOURS":
        await show_colours()
    elif MODE == "TEST":
        await test()
    else:
        await run()
    stop()


runloop.run(main())
