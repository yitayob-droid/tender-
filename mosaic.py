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

CARRIAGE_DOWN, CARRIAGE_UP = 0, 150    # pusher down / carried clear
JAWS_OPEN, JAWS_SHUT = 0, 95

ROWS, COLS = 3, 3                      # mosaic grid
CELL = 5.0                             # cm between cell centres
FIRST_CELL = 8.0                       # cm from the line to the first cell
COL_STEP = 5.0                         # cm sideways between columns

SLOT = 6.0                             # cm between the 3 staging slots
PUSH_CM = 12.0                         # how far the pusher shoves a block
PICK_CM = 9.0                          # nose-in to close on the lined-up 3

# colour references: normalised r,g,b. Run MODE = "COLOURS" and paste yours.
COLOURS = {
    "white":  (0.34, 0.34, 0.32),
    "yellow": (0.46, 0.40, 0.14),
    "green":  (0.20, 0.55, 0.25),
    "blue":   (0.16, 0.30, 0.54),
}
DARK = 90                              # below this intensity it is black

# Everywhere the robot goes is (heading, distance) from the junction it
# sits on after following the line. MEASURE THESE.
HOME = 0                               # heading of the line it works from
MOSAIC  = (0,   30.0)                  # to the near edge of the mosaic
STAGING = (180, 15.0)                  # to the line the 3 blocks end up on
STORE = {                              # to each colour's pile
    "yellow": (270, 20.0),
    "blue":   (270, 32.0),
    "green":  (270, 44.0),
    "white":  (270, 56.0),
}

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


async def push_into_slot(colour, slot):
    """Take one block of this colour and shove it into staging slot
    0 / 1 / 2, carriage DOWN so the pusher is at block height."""
    await go(STORE[colour])
    await carriage(CARRIAGE_DOWN)                  # pusher at block height
    await turn_to(STORE[colour][0] + 90)
    await drive(slot * SLOT, SLOW)                 # line up with the slot
    await turn_to(STORE[colour][0])
    await drive(PUSH_CM, SLOW)                     # shove it across
    await drive(-PUSH_CM, SLOW)
    await turn_to(STORE[colour][0] + 270)
    await drive(slot * SLOT, FAST)
    await go_back(STORE[colour])


async def collect_and_place(row, colours):
    """STEP 3. Order the three blocks, pick all three up, put them in the
    mosaic row."""
    light_matrix.write("3")
    for slot in range(len(colours)):
        await push_into_slot(colours[slot], slot)

    await go(STAGING)                              # pick up the ordered 3
    await carriage(CARRIAGE_DOWN)
    await jaws(JAWS_OPEN)
    await drive(PICK_CM, SLOW)
    await jaws(JAWS_SHUT)
    await carriage(CARRIAGE_UP)
    await drive(-PICK_CM, FAST)
    await go_back(STAGING)

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
