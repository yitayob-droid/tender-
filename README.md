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
3. Drives to the black plate in the middle and reads all 12 cells (4 across, 3 deep)
   with the colour sensor, one column at a time.
4. Groups the cells by colour into trips. Each trip: drive to that colour's depot,
   sweep up blocks in one pass, **check each block is actually in the jaws**, carry
   them to the plate and place them. The robot raises itself on the lift before it
   drives onto the plate, and stays raised until it is clear again.
5. Stops collecting when the time budget will not fit another trip, and parks back in
   the start area.

Cells inside a trip are filled **far row first**, because the robot reaches over the
near cells to get to the far ones — filling the far row first means it never reaches
across a block it has already placed.

Everything is closed loop: every straight line is steered by the gyro, every turn
ends on a gyro heading, and the robot keeps a running estimate of where it is on the
mat (x, y, heading) so moves can be written as "go to this point".

---

## 2. Wiring

| Port | Device |
|------|--------|
| A | left drive wheel |
| B | grabber (bottom) |
| C | up / down mechanism (lift) |
| E | right drive wheel |
| F | colour sensor, pointing down at the mat |

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
| `PLATE_X` | distance from the short end of the mat to the middle of the plate |
| `CELL_PITCH_CM` | centre to centre between two neighbouring cells |
| `DEPOTS` | where the robot parks to grab each colour, and which way it faces |
| `DEPOT_APPROACH_CM` | how far it noses in from there to reach the first block |
| `DEPOT_BLOCK_PITCH_CM` | spacing between blocks of one colour in a depot |
| `MAGAZINE_SIZE` | blocks carried per trip (3) |
| `GRAB_RELEASE_ONE_DEG` | grabber position that frees one block and holds the rest |
| `COLOUR_FWD_CM`, `GRAB_FWD_CM` | how far ahead of the wheel axis each tool sits |
| `WHEEL_DIAMETER_CM` | measure it, do not trust the box |

`COLOUR_FWD_CM` and `GRAB_FWD_CM` are the ones people get wrong. The program works
out where to park so that the *tool* — not the robot — lands on the target, so if
these are wrong every placement is off by the same amount, which is at least easy to
spot and fix.

---

## 5. What is settled, and what is still open

Your AR tape measurements went in as:

| Measured | Used as |
|---|---|
| 37 cm, plate's far edge → mat edge on the "resolute." side | `PLATE_FAR_EDGE_TO_MAT_CM`, which `PLATE_Y` is derived from |
| grey square around the plate = 31 × 26 cm | `PLATE_ZONE_W_CM` / `PLATE_ZONE_D_CM` |
| grid is 4 across × 3 deep, colours yellow/blue/green/white | `GRID_COLS` / `GRID_ROWS`, and it confirms the depot colours |
| colour sensor at the front, on the centreline | `COLOUR_SIDE_CM = 0` |

`PLATE_Y` is worked out **from the far edge inwards**, so it stays right even if the
mat turns out to be a different width than `MAT_WIDTH_CM` says.

### The release is the thing costing you the most

You said the grabber lets go of everything at once. That means a trip can only serve
**one** cell — three blocks released together land in a pile, not on three cells. The
program works this out itself (`CELLS_PER_TRIP`) rather than trusting `MAGAZINE_SIZE`,
so a wrong setting can never dump the bay onto one cell, and it says so at startup.

A round trip is about 26 s. Twelve cells is therefore out of reach in a 2-minute run
however the rest is tuned. **Fitting a gate, ratchet, or pusher that frees one block
while the bay holds the rest is worth roughly double the cells in the same run time**,
and the three-block path is already written and tested — set
`RELEASE_MODE = "ONE_AT_A_TIME"`, find the grabber angle with `TEST_TOOLS`, put it in
`GRAB_RELEASE_ONE_DEG`, and the planner switches over on its own.

Second-biggest: `PATTERN_SOURCE = "FIXED"` skips the 33 s scan if the rules let you
read the pattern before the run.

### Lifting clear of the plate

Because the chassis has to be raised to cross the plate, the robot goes to
`LIFT_CLEAR_DEG` before every move onto it and only comes down to place a block —
and it raises again *before* reversing off, so it never drags over what it just
placed. The simulator checks this and reports any tick where the chassis was over the
plate un-raised while moving.

The catch: the colour sensor is bolted to the chassis, so raising the robot raises the
sensor away from the mat. Scanning therefore happens at `SCAN_LIFT_DEG` (a compromise
height), and **you must calibrate the colours at that height**, not with the robot
sitting down. If the readings are unreliable up there, use `PATTERN_SOURCE = "FIXED"`.

Turning is what really sets the safe distance: the robot's **corners** sweep 15.4 cm
(the half-diagonal of an 18 × 25 cm chassis), which is further than any tool on the
front. `PLATE_APPROACH_Y` and both travel lanes are derived from that, and `path_to()`
routes around a keep-out box so the robot never passes the plate broadside either.

### Still to measure

1. **Cell pitch.** `CELL_PITCH_CM = 5.0` is a guess from the photos. Measure centre to
   centre between two neighbouring cells; it sets every placement position.
2. **`PLATE_X`.** Not pinned down by any measurement. Measure from one short end of the
   mat to the middle of the plate.
3. **Mat size.** Your 1.15 m diagonal is longer than a 200 × 100 cm mat and the plate
   position allow, which suggests the printed mat is the official 2362 × 1143 mm.
   Confirm it and set `MAT_LENGTH_CM` / `MAT_WIDTH_CM`.
4. **Depot layout.** `DEPOT_BLOCK_PITCH_CM = 6.0` and `DEPOT_LAYOUT = "IN_LINE"` assume
   the blocks of one colour sit one behind the other. If they are spread across the
   robot's path, set `DEPOT_LAYOUT = "SIDE_BY_SIDE"`.
5. **What "matching" means.** I assumed: read each cell's colour, deliver a block of
   that colour to that cell. If the plate is a *template* and the mosaic gets built
   elsewhere, change `cell_xy()` and nothing else moves.

**One colour sensor**## 6. Tuning cheat sheet

| Symptom | Fix |
|---|---|
| Drifts left/right on long drives | raise `KP_STRAIGHT`, check the gyro is calibrated on a level surface |
| Snakes down the mat | lower `KP_STRAIGHT`, raise `KD_STRAIGHT` |
| Overshoots turns | lower `KP_TURN` or `TURN_PCT` |
| Never finishes a turn (`TRN TO` on the display) | raise `TURN_MIN_PCT`, or widen `TURN_TOL_DEG` |
| Stalls / stops short (`DRV TO`) | wheels slipping — lower `DRIVE_PCT`, or raise `MIN_MOVE_PCT` |
| Wrong colours | redo step 5 under venue lighting, raise `COLOUR_SAMPLES` |
| Drops blocks next to the cell | check `GRAB_FWD_CM` and `PLATE_X`/`PLATE_Y` |
| Runs out of time | `PATTERN_SOURCE = "FIXED"` saves ~30 s, raise `DRIVE_PCT`, set `MISSION_TIMEOUT_S` to your rulebook time minus 10 s |
| Only picks 1–2 blocks per trip | `DEPOT_BLOCK_PITCH_CM` is wrong, or the depot is `SIDE_BY_SIDE` not `IN_LINE` |
| Whole magazine falls out at once | `GRAB_RELEASE_ONE_DEG` is too far open — find it with `TEST_TOOLS` |
| Chassis scrapes the tiles | raise `LIFT_CLEAR_DEG`; the simulator reports every drag |
| Colours misread while scanning | recalibrate at `SCAN_LIFT_DEG` height, or lower it |

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
where every block was released. It also counts every tick the chassis was over the plate without being raised, which
is how the drag-on-reverse bug and the corner-clipping approach line were both found.

Current result: the scan recovers all 12 cells exactly, a one-block trip takes about
26 s, blocks land within 0.5 cm of their cell centres, and there are no plate drags —
on both the App 3 and App 2 code paths. The model is deliberately
imperfect — the right wheel runs 1 % slow — so the gyro controller has something to
correct. It cannot tell you anything about grip, weight or friction; that is what the
table is for.
