# AI Usage Log

<!-- DRAFT written by the AI tool (Claude Code). Edit into your own words, and
     delete or fix anything marked TODO that doesn't match what you actually did. -->

## 1. Choosing a track and setting up the environment

**What I asked:** Which of the two take-homes was realistic in about an hour for Part 1, and later why `python sim/launch.py --no-node` crashed with `TypeError: unsupported operand type(s) for |`.

**What I kept vs. rewrote, and why:** Kept the diagnosis: my `python` was Anaconda 3.9.13, and `float | None` needs 3.10+. Kept the fix of building the venv from Homebrew's Python 3.12.

**What the AI got wrong that I had to catch:** Its first fix did not work. It told me to run `python3.12 -m venv .venv` on top of a `.venv` that already existed, which left `python` pointing at 3.9 while `pip` installed for 3.12. I pasted the output showing `python --version` still said 3.9.13, and only then did it tell me to delete the venv and rebuild it.

**How I verified it ran correctly (not just that it compiled):** `python --version` prints 3.12.14 inside the venv, and `python sim/launch.py --no-node` prints the launcher banner instead of a traceback.

## 2. Step 2: `/wheel_ticks` subscription and QoS

**What I asked:** To implement the `/wheel_ticks` subscription and print what arrives.

**What I kept vs. rewrote, and why:** Kept the generated subscription. It uses `BEST_EFFORT` / `VOLATILE`, depth 10, because that is what `sim/encoder_publisher.py` publishes with, and a default `RELIABLE` subscriber would receive nothing. TODO: say whether you changed anything.

**What the AI got wrong that I had to catch:** Nothing in the code. TODO: confirm, or note anything you spotted.

**How I verified it ran correctly (not just that it compiled):** Ran `python sim/launch.py --no-faults`: no "NOT connected" warning from the launcher, and the measured receive rate was 50.0 Hz. About 382 ticks per second times 0.00131 m per tick is 0.5 m/s, which matches the speed in `sim/ground_truth.py`. TODO: confirm you read the QoS in `sim/encoder_publisher.py` yourself.

## 3. Step 3: wheel odometry and the `/odom` message

**What I asked:** To turn tick changes into distance and velocity and publish `/odom` from the wheels alone, driving straight east.

**What I kept vs. rewrote, and why:** Kept the generated code: distance is `delta_ticks * DIST_PER_TICK` added along `self.heading` (fixed at 0 for now), velocity is distance over the difference of two encoder timestamps, and the quaternion is `z = sin(h/2)`, `w = cos(h/2)`. TODO: say whether you changed anything.

**What the AI got wrong that I had to catch:** It reported the step as working without opening the `--visualize` window, which is the README's check for this step. When I ran it, the blue line started at (0.56, 0) instead of (0, 0). The AI's explanation was that the visualizer subscribes to `/odom` about a second after the node starts (it imports matplotlib first), so it misses the first ~0.56 m; that came from reading `sim/launch.py`, not from a test. Also, the printed velocity jitters between 0.49 and 0.55 m/s because each message carries 7 or 8 whole ticks; the AI left it unsmoothed.

**How I verified it ran correctly (not just that it compiled):** The terminal shows x growing by 0.50 m per second with y at 0, starting from `pos=(0.49, 0.00)` after the first 50 messages, so the node itself counts up from zero. TODO: confirm the blue error in the bottom plot started near 0 m, which is what separates "plot started late" from "node has a 0.56 m offset".

## 4. Step 4: duplicates, dropped ticks, safe velocity

**What I asked:** To handle the injected encoder noise so nothing crashes or produces NaN/infinity.

**What I kept vs. rewrote, and why:** Kept the generated guards: a message is a duplicate only if both `tick_count` and `timestamp` equal the previous one (a stopped rover has the same ticks but a new timestamp, and is still processed), and anything going backwards in ticks or time is skipped and counted. Dropped ticks are not detected because they cannot be; the comment in the callback says why. TODO: say whether you changed anything.

**What the AI got wrong that I had to catch:** Nothing found in the code. It did not measure the dropped-tick drift, so the README's "~6 cm per minute" was unverified at this step. TODO: confirm, or note anything you spotted.

**How I verified it ran correctly (not just that it compiled):** 60 s with faults on (`--seed 7`): no tracebacks, 63 duplicates ignored out of about 3,150 messages (2.0%, matching the simulator's 2% rate), `bad=0`, and printed velocity stayed between 0.39 and 0.57 m/s against a true 0.50.

## 5. Step 5: GPS fusion and heading

**What I asked:** To subscribe to `/gps_estimate`, fuse it with the wheel position, and decide where heading comes from, without hard-coding the circle.

**What I kept vs. rewrote, and why:** Kept the generated design. Position is a variance-weighted average: the wheel position's variance grows per metre driven and the GPS weight is `pos_var / (pos_var + covariance)`, so noisy fixes after an outage count for less. Heading is inferred from the sideways part of each GPS correction divided by the distance driven since the last fix, applied as a small fraction; the same error also adjusts a turn-per-metre estimate that keeps the heading turning between fixes and through outages. TODO: rewrite this in your own words; this is the part you will be asked to explain.

**What the AI got wrong that I had to catch:** Its first choice of the three gains (0.005, 0.5, 0.1) was a guess, and it was worse than using raw GPS: mean error 0.90 m against 0.54 m for the GPS fixes, with heading off by about 23 degrees. That fails the README's step 5 check. It found this itself with an offline replay script, not by reasoning, then searched the gains over seven path variants. The first search's best values were on the edge of the range searched, so it widened the search before choosing 0.04, 0.3, 0.02. TODO: note anything you spotted in the plot.

**How I verified it ran correctly (not just that it compiled):** The AI's offline replay (its own copy of the encoder and GPS noise models, not the real simulator) on 20 seeds not used for tuning, 300 s each: mean error 0.27 m on the README circle against 0.54 m for raw GPS, and 0.24 to 0.42 m on the other paths (clockwise, radius 4 m, radius 30 m at 1.5 m/s, 0.2 m/s, straight, S-curve). Worst single spike was 2.7 m on the S-curve, in the noisy seconds after an outage. Live run of `python sim/launch.py --seed 7` for 100 s: no tracebacks, heading went from 0 round through 180 degrees. TODO: run `python sim/launch.py --visualize` yourself and confirm the blue error in the bottom plot sits below the GPS dots; the AI did not open the window.

## 6. Step 6: monitoring output

**What I asked:** To implement the once-per-second status line: time since each source, `/wheel_ticks` rate against 50 Hz, and the fused position.

**What I kept vs. rewrote, and why:** Kept the generated callback and its thresholds: encoder stale after 0.2 s (10 missed messages), GPS stale after 2.5 s (two missed fixes), and a rate warning outside 45 to 55 Hz. Ages use `time.monotonic()` at arrival, not `msg.timestamp`, because the encoder clock drifts. The rate leaves duplicates out, so a re-sent message cannot hide a lost one. TODO: say whether you agree with the thresholds or changed them; the write-up asks why you chose them.

**What the AI got wrong that I had to catch:** Nothing found in the code. The simulator never stops the encoder, so the live run cannot show the encoder warnings; the AI tested those by calling the callback directly instead. TODO: confirm, or note anything you spotted.

**How I verified it ran correctly (not just that it compiled):** 60 s of `python sim/launch.py --seed 7`: 61 status lines, no tracebacks, rate between 49.8 and 50.2 Hz, and `WARN: GPS STALE` on exactly the three lines where the GPS age read 3, 4 and 5 s during the outage, clearing when fixes resumed. Calling the callback on a fresh node printed `enc=never` and `gps=never` with both "NO DATA" warnings, and 0.3 s after the last encoder message it printed `ENCODER STALE`.
