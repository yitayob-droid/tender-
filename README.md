# WRO 2026 "Mosaic Masters" — robot 13

One file: **`mosaic.py`**. Paste it into a Python project in the SPIKE app.

## Ports

| A | B | C | E | F |
|---|---|---|---|---|
| left wheel | grabber | carriage / pusher | right wheel | colour sensor |

## What it does, in order

1. **`find_line()`** — creeps forward until the colour sensor is over the black line.
2. **`scan_mosaic()`** — drives out to the mosaic, reads the leftmost column cell by
   cell, backs out, shifts one column right, repeats for the middle and the rightmost.
   Stores `pattern[row][col]`.
3. **`collect_and_place(row, colours)`** — for each of the three colours that row needs,
   drives to that colour's pile, drops the carriage so the pusher is at block height,
   and shoves one block into staging slot 0, 1 or 2. Once all three are lined up in
   order it picks up all three at once and puts them in the mosaic row.
4. The `for` loop in `run()` repeats step 3 for each row until the mosaic is done.

## Numbers to measure before it will work

All at the top of the file.

- `WHEEL_CM` — wheel circumference. Everything scales off this.
- `BLACK` / `WHITE` — reflection on the line and on the mat.
- `COLOURS` — set `MODE = "COLOURS"`, hold the sensor over each block, paste the
  printed triples in.
- `CARRIAGE_DOWN` / `CARRIAGE_UP`, `JAWS_OPEN` / `JAWS_SHUT` — motor positions.
- `DEPOT`, `MOSAIC` — each is `(heading, distance)` from the junction the robot sits
  on after finding the line. These are the only two places it has to find on the mat.
- `LAYOUT` — where the 6 blocks of one colour sit, as (column, row). Edit to match.
- `GROUPS` — the order of the colour groups along the depot, left to right.
- `LANE_Y`, `SLOT_PITCH`, `PICK_CM` — the staging lane.
- `COLS`, `ROWS`, `CELL`, `FIRST_CELL`, `COL_STEP` — the mosaic grid (3 wide, 4 deep).

## The depot map

`block_xy(colour, n)` gives the position of every one of the 24 blocks in depot
coordinates, built from three numbers: a block is 3.18 cm, same-colour blocks are one
block apart (6.36 cm centre to centre), colour groups are two blocks apart (9.54 cm).
So the robot only has to find one corner of the depot; everything else follows from
the grid.

## Modes

`MODE = "RUN"` the mission · `"COLOURS"` colour calibration · `"TEST"` drive 50 cm out
and back, turn 90 and back, cycle the carriage and jaws.

If it drives backwards, flip both `LEFT_SIGN` and `RIGHT_SIGN`. If it spins instead of
driving, flip one.
