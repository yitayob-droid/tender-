# WRO 2026 RoboMission Senior — "Mosaic Masters" collector

LEGO Education SPIKE Prime program that reads the mosaic pattern in the middle of
the mat and ferries matching colour blocks to it, driving on the hub's gyro so it
holds a true heading the whole run.

```
mosaic_masters.py     the program - this is the only file that goes on the hub
tools/simulator.py    desktop simulator, runs the program without the robot
```

---

## 1. What the robot does

1. Starts in the ROBOMISSION corner of the mat (bottom-left = coordinate origin).
2. Homes the grabber and the lift against their end stops so both know where zero is.
3. Drives to the black plate in the middle and reads all 9 cells with the colour
   sensor, one column at a time.
4. For each cell it needs to serve: drives to the depot holding that colour, closes
   the grabber on a block, **checks the block is actually in the jaws**, carries it
   back and puts it down on the matching cell.
5. Stops collecting when the time budget will not fit another round trip, and parks
   back in the start area.

Everything is closed loop: every straight line is steered by the gyro, every turn
ends on a gyro heading, and the robot keeps a running estimate of where it is on the
mat (x, y, heading) so moves can be written as "go to this point".

---

## 2. Wiring

| Port | Device |
|------|--------|
| A | up / down mechanism (lift) |
| B | left drive wheel |
| C | grabber (bottom) |
| E | colour sensor, pointing down at the mat |
| F | right drive wheel |

Runs on **SPIKE App 3** (current firmware). If you are still on SPIKE App 2 /
MINDSTORMS the program detects it and switches to the old API automatically — both
paths are tested. Paste the whole file into one project slot; it is deliberately a
single file because the SPIKE app cannot upload more than one.

---

## 3. Do this before your first run (about 20 minutes)

Change `MODE` at the top of the file, run, then set `MODE = "MISSION"` when done.

**Step 1 — `MODE = "TEST_DRIVE"`.** The robot should drive 50 cm out and 50 cm back.
* Drives backwards → flip **both** `LEFT_SIGN` and `RIGHT_SIGN`.
* Spins on the spot → flip **one** of them.
* Ends up short or long → measure the real distance and set
  `DRIVE_SCALE = 50 / measured`.
* Curves instead of driving straight → raise `KP_STRAIGHT`; if it snakes, lower it
  and raise `KD_STRAIGHT`.

**Step 2 — `MODE = "CAL_YAW"`.** Turn the robot 90° to its **right** by hand. The
printed yaw must go **positive**. If it goes negative, set `YAW_SIGN = -1`.

**Step 3 — `MODE = "TEST_TURN"`.** It turns 90/180/270/0 and prints the error. Under
1° is good. If it overshoots and hunts, lower `KP_TURN`; if it stops short and never
arrives, raise `TURN_PULSE_MS_PER_DEG`.

**Step 4 — `MODE = "TEST_TOOLS"`.** The grabber opens and closes, the lift cycles.
* Moving the wrong way → flip `GRAB_SIGN` / `LIFT_SIGN`.
* Note the console number for the grabber closed **on a block** versus closed **on
  air**, then set `GRAB_EMPTY_DEG` between the two. That is how the robot knows a
  grab missed.
* Set `LIFT_DOWN_DEG` / `LIFT_CARRY_DEG` / `GRAB_OPEN_DEG` / `GRAB_CLOSED_DEG` to the
  positions your build actually uses.

**Step 5 — `MODE = "CAL_COLOUR"`.** Hold the sensor 5–10 mm over each block and over
the black plate and the white mat. The console prints a normalised triple like
`(0.46, 0.40, 0.14)`. Paste each one into `COLOUR_REFS`. Also set `BLACK_INTENSITY`
just above what the black plate reads and `WHITE_INTENSITY` just below what the white
mat reads. **Do this under the lighting you will compete in** — this is the single
biggest cause of a robot that works at home and fails at the venue.

**Step 6 — `MODE = "TEST_SCAN"`.** The robot should drive to the plate and print the
9 cell colours correctly. If it reads the wrong cells, your `PLATE_X` / `PLATE_Y` /
`CELL_PITCH_CM` / `COLOUR_FWD_CM` are off.

**Step 7 — `MODE = "MISSION"`.**

---

## 4. The numbers you must measure

The field map at the top of the file is my best reading of your photo. **It is an
estimate — put a tape measure on your mat and correct it.** Coordinates are in cm
from the bottom-left corner (the ROBOMISSION label), X along the 2 m side, Y along
the 1 m side. Headings are compass style: 0 = facing +Y, 90 = +X, growing clockwise.

| Constant | What it is |
|---|---|
| `START_X`, `START_Y`, `START_H` | where you place the robot before the run |
| `PLATE_X`, `PLATE_Y` | centre of the black mosaic plate |
| `GRID_ROWS`, `GRID_COLS`, `CELL_PITCH_CM` | the mosaic grid |
| `DEPOTS` | where the robot parks to grab each colour, and which way it faces |
| `DEPOT_APPROACH_CM` | how far it noses in from there to reach a block |
| `COLOUR_FWD_CM`, `GRAB_FWD_CM` | how far ahead of the wheel axis each tool sits |
| `WHEEL_DIAMETER_CM` | measure it, do not trust the box |

`COLOUR_FWD_CM` and `GRAB_FWD_CM` are the ones people get wrong. The program works
out where to park so that the *tool* — not the robot — lands on the target, so if
these are wrong every placement is off by the same amount, which is at least easy to
spot and fix.

---

## 5. Assumptions I had to make

Flagging these because they are guesses about the rules, not about your robot:

1. **What "matching" means.** I assumed: read the colour of each cell on the black
   plate, then deliver a block of that same colour to that cell. If the real rule is
   different — say the pattern is a *template* and the mosaic must be built in a
   separate area — the fix is small and local: change `cell_xy()` to return the build
   area coordinates and leave everything else alone.
2. **The pattern can be read by driving up to the plate.** With `COLOUR_FWD_CM = 8`
   the sensor reaches the far row while the wheels stay at the near edge, so the
   robot never has to climb onto the plate. Check this on your build; if the sensor
   sits closer to the wheels, either extend the mount or set
   `PATTERN_SOURCE = "FIXED"` and type the pattern into `FIXED_PATTERN` during
   inspection time (that also saves the ~25 s the scan costs).
3. **Grid is 3×3 at 4 cm pitch** — from the photo. Change `GRID_ROWS`, `GRID_COLS`
   and `CELL_PITCH_CM` if not.
4. **One block per trip**, because your grabber holds one. A round trip is about 18 s,
   so in a 2-minute run expect 4–6 cells, not 9. The single biggest scoring
   improvement available to you is a magazine that carries 2–3 blocks per trip — the
   mission loop is already written so that only `fetch()` and `deliver()` would need
   to change.
5. **One colour sensor**, so the robot cannot square itself on a line. It re-squares
   on the gyro instead, which is why gyro calibration matters. A second colour sensor
   at the front is the standard fix and would let you cancel drift on every black
   line; the helpers `find_black()` and `resync()` are already in the file for that.

---

## 6. Tuning cheat sheet

| Symptom | Fix |
|---|---|
| Drifts left/right on long drives | raise `KP_STRAIGHT`, check the gyro is calibrated on a level surface |
| Snakes down the mat | lower `KP_STRAIGHT`, raise `KD_STRAIGHT` |
| Overshoots turns | lower `KP_TURN` or `TURN_PCT` |
| Never finishes a turn (`TRN TO` on the display) | raise `TURN_MIN_PCT`, or widen `TURN_TOL_DEG` |
| Stalls / stops short (`DRV TO`) | wheels slipping — lower `DRIVE_PCT`, or raise `MIN_MOVE_PCT` |
| Wrong colours | redo step 5 under venue lighting, raise `COLOUR_SAMPLES` |
| Drops blocks next to the cell | check `GRAB_FWD_CM` and `PLATE_X`/`PLATE_Y` |
| Runs out of time | `PATTERN_SOURCE = "FIXED"`, raise `DRIVE_PCT`, lower `MISSION_TIMEOUT_S` to your rulebook time minus 10 s |

Positions and timings print to the console during a run, so if something goes wrong
on the table, the console tells you which trip and which phase it was in.

---

## 7. Simulator

Runs the real program against a kinematic model of the robot on the mat, so you can
check a change before you take the table.

```bash
python3 tools/simulator.py                # full mission
python3 tools/simulator.py TEST_TURN      # any MODE
python3 tools/simulator.py MISSION -v     # with a driving trace
python3 tools/simulator.py MISSION --no-blocks   # every grab misses
python3 tools/simulator.py MISSION --legacy      # via the SPIKE App 2 path
```

It reports the true pose, the pose the robot *believed* it had (odometry error) and
where every block was released. Current result: the scan recovers the pattern exactly
and every block lands within 0.4 cm of its cell centre. The model is deliberately
imperfect — the right wheel runs 1 % slow — so the gyro controller has something to
correct. It cannot tell you anything about grip, weight or friction; that is what the
table is for.
