# Mosaic Masters — the word-block version

The Python program does closed-loop navigation: it tracks where it is on the mat and
works out every turn as it goes. **Word blocks cannot do that** — not in any sane
number of blocks. So this is the same mission built the way blocks are good at: a
measured sequence of *face this heading, drive this far*, with the gyro keeping every
heading honest.

It is genuinely competitive. Plenty of teams score well with exactly this. It is just
less forgiving: if one distance is wrong, everything after it is wrong too, whereas the
Python version re-derives its position at each step.

---

## 1. Ports

| Port | Device |
|---|---|
| A | left wheel |
| B | grabber |
| C | up / down lift |
| E | right wheel |
| F | colour sensor |

## 2. Headings

Every turn in this program is an **absolute compass heading**, never "turn left a bit".
That is what stops errors piling up.

```
              0 deg  = away from you, across the short side of the mat
   270 deg  <---+--->  90 deg = to the right, along the long side
             180 deg  = back towards you
```

Place the robot in the ROBOMISSION corner facing **90 degrees** (along the mat, to the
right) and the program resets the gyro to that at the start.

---

## 3. Setup — `when program starts`

Drag these in order:

1. **Movement → set movement motors to** `A` `E`
2. **Movement → set movement speed to** `40` %
3. **Motors → B set speed to** `60` %
4. **Motors → C set speed to** `60` %
5. **Sensors → reset yaw angle** (this makes the start direction "0" for the gyro, so
   every heading below is measured from where you placed the robot)
6. **Motors → C go to position** `0`  ← lift down
7. **Motors → B go to position** `0`  ← grabber open

> The gyro resets to 0 wherever you place the robot, so the headings in the tables are
> **relative to the start direction**. Face the robot along the mat, to the right, and
> heading 90 in the tables means "quarter turn right from how you set it down".

---

## 4. Build these three My Blocks first

Everything else is made of them. **My Blocks → Make a Block.**

### `FACE (heading)` — turn to a gyro heading

This is the block that makes the whole thing work. Take a number input called
`heading`.

```
repeat until  <  abs( heading - (yaw angle) )  <  2  >
    if  < (heading - yaw angle) > 0 >  then
        Movement → start moving with steering  100   (turn right)
    else
        Movement → start moving with steering -100   (turn left)
Movement → stop moving
wait 0.2 seconds
```

Set the movement speed to about `20 %` inside this block and back to `40 %` after —
a slow turn stops far more accurately.

### `GRAB` — take one block

```
Motors → C go to position  0        (lift down)
Motors → B go to position  0        (grabber open)
Movement → move  9  cm  forward
Motors → B go to position  95       (close on the block)
Motors → C go to position  160      (lift it, and lift the robot clear)
Movement → move  9  cm  backward
```

### `PLACE` — put it down

```
Motors → C go to position  10       (lower onto the cell)
Motors → B go to position  0        (open — everything comes out)
Motors → C go to position  160      (raise BEFORE reversing, or you drag over it)
Movement → move  5  cm  backward
```

**The order in `PLACE` matters.** Raising before you reverse is not fussiness — with
the lift down, backing off scrapes the chassis over the block you just placed. This was
a real bug in the Python version before the simulator caught it.

---

## 5. One trip

The pattern is always the same six steps. Repeat it once per block:

```
FACE  (heading to the depot)
move  (distance)  cm
FACE  (the depot's facing)
GRAB
FACE  (heading to the lane corner)
move  (distance)  cm
FACE  90
move  (along the lane)  cm
FACE  0
move  (run in)  cm
PLACE
```

### Step 1–2 — start to a depot

| From start to | face | move | then face |
|---|---|---|---|
| yellow depot (left) | 28 deg | 29.5 cm | 270 deg |
| blue depot (left) | 20 deg | 40.5 cm | 270 deg |
| green depot (left) | 16 deg | 51.9 cm | 270 deg |
| white depot (left) | 12 deg | 65.5 cm | 270 deg |

### Step 3–4 — depot round to the lane corner

The lane corner is the spot just clear of the plate where it is safe to turn. **Do not
cut the corner** — the robot is 25 cm wide and its corners sweep 15.4 cm when it spins,
so turning any closer sweeps a corner over the tiles.

| From depot | face | move | (arrives at the lane corner) |
|---|---|---|---|
| yellow | 111 deg | 43.4 cm | 70.6, 32.6 |
| blue | 124 deg | 49.0 cm | 70.6, 32.6 |
| green | 134 deg | 56.6 cm | 70.6, 32.6 |
| white | 143 deg | 67.1 cm | 70.6, 32.6 |

### Step 5–6 — along the lane, then in to the cell

After the corner: `FACE 90`, drive along the lane, `FACE 0`, run in, `PLACE`.

| Cell | face 90, move along lane | face 0, run in | colour there |
|---|---|---|---|
| row 0 col 0 | 16.9 cm | 18.9 cm | blue |
| row 0 col 1 | 21.9 cm | 18.9 cm | yellow |
| row 0 col 2 | 26.9 cm | 18.9 cm | green |
| row 0 col 3 | 31.9 cm | 18.9 cm | yellow |
| row 1 col 0 | 16.9 cm | 13.9 cm | yellow |
| row 1 col 1 | 21.9 cm | 13.9 cm | blue |
| row 1 col 2 | 26.9 cm | 13.9 cm | white |
| row 1 col 3 | 31.9 cm | 13.9 cm | green |
| row 2 col 0 | 16.9 cm | 8.9 cm | blue |
| row 2 col 1 | 21.9 cm | 8.9 cm | yellow |
| row 2 col 2 | 26.9 cm | 8.9 cm | green |
| row 2 col 3 | 31.9 cm | 8.9 cm | yellow |

**Fill the far row (row 0) first.** The robot reaches over the near cells to get to the
far ones, so doing row 0 first means it never reaches across a block already placed.

---

## 6. Reading the pattern

Blocks *can* read the mosaic: drive the sensor over a cell and use
**Sensors → is F colour \_\_\_ ?**. But the sensor is bolted to the chassis, so when the
lift raises the robot to clear the plate the sensor rises too and the readings get
worse.

The practical answer for a block program: **look at the plate yourself before the run
and pick the matching sequence.** The colour of every cell is in the last column of the
table above — fill in your own after you have seen the real pattern.

---

## 7. Order to test it

Do not build the whole thing and run it. Build and test in this order:

1. **Setup + `FACE`.** Run `FACE 90`, `FACE 180`, `FACE 0`. If the robot spins the wrong
   way, swap the two steering values inside `FACE`. If it hunts back and forth, slow it
   down or widen the `< 2` to `< 3`.
2. **`move 50 cm`.** Measure what you actually get. If it comes out at 48, every
   distance in the tables needs multiplying by 50/48.
3. **`GRAB`** on a block sitting on the table. Fix the `9 cm` and the position numbers
   until it takes the block cleanly.
4. **`PLACE`.**
5. **One full trip** to one cell.
6. **The rest of the trips.**

---

## 8. What you give up versus the Python version

| | Blocks | Python |
|---|---|---|
| Keeps a true heading | yes, via `FACE` | yes |
| Knows where it is on the mat | no | yes |
| Recovers if a distance is off | no | partly — it re-derives each move |
| Checks the block is really in the jaws | no | yes, from the grabber's stall position |
| Reads the mosaic automatically | possible, unreliable at lift height | yes |
| Refuses a trip it cannot finish in time | no | yes |
| Blocks to drag | about 120 | — |

If the blocks version keeps drifting off after the third or fourth trip, that is the
thing Python fixes, and it is the reason to switch.
