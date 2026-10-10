# AI Usage Log


## 1. Choosing a track and setting up the environment

**What I asked:** I asked for more document related to the track to read more about it, and later why `python sim/launch.py --no-node` crashed with `TypeError: unsupported operand type(s) for |`.

**What I kept vs. rewrote, and why:** Kept the diagnosis: my `python` was Anaconda 3.9.13, and `float | None` needs 3.10+. Kept the fix of building the venv from Homebrew's Python 3.12.

**What the AI got wrong that I had to catch:** Its first fix did not work. It told me to run `python3.12 -m venv .venv` on top of a `.venv` that already existed, which left `python` pointing at 3.9 while `pip` installed for 3.12. I pasted the output showing `python --version` still said 3.9.13, and only then did it tell me to delete the venv and rebuild it.

**How I verified it ran correctly (not just that it compiled):** `python --version` prints 3.12.14 inside the venv, and `python sim/launch.py --no-node` prints the launcher banner instead of a traceback.

## 2. Step 2: `/wheel_ticks` subscription and QoS

**What I asked:** To implement the `/wheel_ticks` subscription and print what arrives.

**What I kept vs. rewrote, and why:** Kept the generated subscription. It uses `BEST_EFFORT` / `VOLATILE`, depth 10, because that is what `sim/encoder_publisher.py` publishes with, and a default `RELIABLE` subscriber would receive nothing. I did not change it.

**What the AI got wrong that I had to catch:** Nothing in the code.

**How I verified it ran correctly (not just that it compiled):** Ran `python sim/launch.py --no-faults`: no "NOT connected" warning from the launcher, and the measured receive rate was 50.0 Hz. About 382 ticks per second times 0.00131 m per tick is 0.5 m/s, which matches the speed in `sim/ground_truth.py`. The publisher's QoS is set in `sim/encoder_publisher.py` (`BEST_EFFORT`, `VOLATILE`, depth 10).

## 3. Step 3: wheel odometry and the `/odom` message

**What I asked:** To turn tick changes into distance and velocity and publish `/odom` from the wheels alone, driving straight east.

**What I kept vs. rewrote, and why:** Kept the generated code: distance is `delta_ticks * DIST_PER_TICK` added along `self.heading` (fixed at 0 for now), velocity is distance over the difference of two encoder timestamps, and the quaternion is `z = sin(h/2)`, `w = cos(h/2)`. I did not change it.

**What the AI got wrong that I had to catch:** It reported the step as working without opening the `--visualize` window, which is the README's check for this step. When I ran it, the blue line started at (0.56, 0) instead of (0, 0). The AI's explanation was that the visualizer subscribes to `/odom` about a second after the node starts (it imports matplotlib first), so it misses the first ~0.56 m; that came from reading `sim/launch.py`, not from a test. Also, the printed velocity jitters between 0.49 and 0.55 m/s because each message carries 7 or 8 whole ticks; the AI left it unsmoothed.

**How I verified it ran correctly (not just that it compiled):** The terminal shows x growing by 0.50 m per second with y at 0, starting from `pos=(0.49, 0.00)` after the first 50 messages, so the node itself counts up from zero. A plot saved later (entry 5) confirms it: the blue error starts at 0 m at t = 0, so the node has no 0.56 m offset.

## 4. Step 4: duplicates, dropped ticks, safe velocity

**What I asked:** To handle the injected encoder noise so nothing crashes or produces NaN/infinity.

**What I kept vs. rewrote, and why:** Kept the generated guards: a message is a duplicate only if both `tick_count` and `timestamp` equal the previous one (a stopped rover has the same ticks but a new timestamp, and is still processed), and anything going backwards in ticks or time is skipped and counted. Dropped ticks are not detected because they cannot be; the comment in the callback says why. I did not change them.

**What the AI got wrong that I had to catch:** Nothing found in the code. It did not measure the dropped-tick drift, so the README's "~6 cm per minute" was unverified at this step. It was measured later, for the write-up: 5.9 cm per minute.

**How I verified it ran correctly (not just that it compiled):** 60 s with faults on (`--seed 7`): no tracebacks, 63 duplicates ignored out of about 3,150 messages (2.0%, matching the simulator's 2% rate), `bad=0`, and printed velocity stayed between 0.39 and 0.57 m/s against a true 0.50.

## 5. Step 5: GPS fusion and heading

**What I asked:** To subscribe to `/gps_estimate`, fuse it with the wheel position, and decide where heading comes from, without hard-coding the circle.

**What I kept vs. rewrote, and why:** Kept the generated design. Position is a variance-weighted average: the wheel position's variance grows per metre driven and the GPS weight is `pos_var / (pos_var + covariance)`, so noisy fixes after an outage count for less. Heading is inferred from the sideways part of each GPS correction divided by the distance driven since the last fix, applied as a small fraction; the same error also adjusts a turn-per-metre estimate that keeps the heading turning between fixes and through outages.

**What the AI got wrong that I had to catch:** Its first choice of the three gains (0.005, 0.5, 0.1) was a guess, and it was worse than using raw GPS: mean error 0.90 m against 0.54 m for the GPS fixes, with heading off by about 23 degrees. That fails the README's step 5 check. It found this itself with an offline replay script, not by reasoning, then searched the gains over seven path variants. The first search's best values were on the edge of the range searched, so it widened the search before choosing 0.04, 0.3, 0.02. The plot shows what the tuning did not fix: in the first 10 s, while GPS is at its noisiest and the turn rate is not yet learned, the blue error climbs to about 1.0 m before settling.

**How I verified it ran correctly (not just that it compiled):** The AI's offline replay (its own copy of the encoder and GPS noise models, not the real simulator) on 20 seeds not used for tuning, 300 s each: mean error 0.27 m on the README circle against 0.54 m for raw GPS, and 0.24 to 0.42 m on the other paths (clockwise, radius 4 m, radius 30 m at 1.5 m/s, 0.2 m/s, straight, S-curve). Worst single spike was 2.7 m on the S-curve, in the noisy seconds after an outage. Live run of `python sim/launch.py --seed 7` for 100 s: no tracebacks, heading went from 0 round through 180 degrees. The AI could not look at a window, so it ran the real `python sim/launch.py --visualize --seed 7` with the plot rendered to an image file instead of the screen and read that. Over the first 58 s the blue line follows the circle and its error averaged 0.28 m against 0.60 m for the GPS readings. It is below the GPS line for most of the run but not all of it: it rises to 1.0 m in the first 10 s and to about 0.7 m just after the outage, and it steps at each GPS fix.

## 6. Step 6: monitoring output

**What I asked:** To implement the once-per-second status line: time since each source, `/wheel_ticks` rate against 50 Hz, and the fused position.

**What I kept vs. rewrote, and why:** Kept the generated callback and its thresholds: encoder stale after 0.2 s (10 missed messages), GPS stale after 2.5 s (two missed fixes), and a rate warning outside 45 to 55 Hz. Ages use `time.monotonic()` at arrival, not `msg.timestamp`, because the encoder clock drifts. The rate leaves duplicates out, so a re-sent message cannot hide a lost one. I kept the thresholds as generated.

**What the AI got wrong that I had to catch:** Nothing found in the code. The simulator never stops the encoder, so the live run cannot show the encoder warnings; the AI tested those by calling the callback directly instead.

**How I verified it ran correctly (not just that it compiled):** 60 s of `python sim/launch.py --seed 7`: 61 status lines, no tracebacks, rate between 49.8 and 50.2 Hz, and `WARN: GPS STALE` on exactly the three lines where the GPS age read 3, 4 and 5 s during the outage, clearing when fixes resumed. Calling the callback on a fresh node printed `enc=never` and `gps=never` with both "NO DATA" warnings, and 0.3 s after the last encoder message it printed `ENCODER STALE`.

## 7. Step 7: long run with faults on

**What I asked:** To run the node for several minutes with faults on and fix whatever turned up.

**What I kept vs. rewrote, and why:** Kept one fix: when the encoder has been silent for more than 0.2 s, the monitor sets the velocity to 0 and publishes once, instead of repeating the last measured speed for as long as the feed is down. Nothing else was changed, because the long run showed no other problem. I kept reporting 0.

**What the AI got wrong that I had to catch:** The run itself found no fault. The stale velocity was found by reading the code, not by the run, because the simulator never stops the encoder. The AI's scoring script also wires up the simulator itself instead of going through `sim/launch.py`, so it is a close copy of the real launch, not the launch itself.

**How I verified it ran correctly (not just that it compiled):** 6 minutes of the real simulator classes with faults on (seed 7), scored against ground truth: 18,318 `/odom` messages with none non-finite, mean position error 0.26 m against 0.54 m for raw GPS, worst 1.17 m, mean heading error 5.3 degrees, per-minute mean between 0.21 and 0.32 m with no upward trend, 360 status lines, no tracebacks, and `GPS STALE` on 24 lines (three per outage, eight outages). The velocity fix was tested by calling the callbacks directly: 0.52 m/s while moving, 0.0 published once after 0.3 s of silence, 0.51 m/s again on the next message. A 60 s rerun after the fix gave 0.30 m against 0.63 m for GPS with no encoder warnings. The same image capture of `--visualize` after 4 minutes: over the last 60 s shown, the blue error averaged 0.27 m (worst 0.84 m) against 0.57 m for GPS, so it had not drifted.

## 8. Write-up documents

**What I asked:** To draft `METHODOLOGY.md`, the "Your write-up" section of the README, and bring this log up to date. It also measured the dropped-tick drift quoted in the write-up (5.9 cm per minute) by replaying the encoder's drop model for 200 seeds.

**What I kept vs. rewrote, and why:** The AI wrote the first drafts of `METHODOLOGY.md`, the README write-up and this log from the session history. I kept the run steps, the numbers and the list of limitations, because they come from runs in this repo that I can repeat. I rewrote the first entry of this log to say what I actually asked, and the what I kept vs rewrote so that I can get a clear view of the overall project.

**What the AI got wrong that I had to catch:**

- It wrote the documents in first person, as if I had done every check. Where a sentence depended on something only I could confirm (what I saw in `--visualize`, whether I read the simulator source myself), it left a `TODO` instead of making it up. When I asked it to fill those in, it filled them only with what the session record and a saved plot could support.
- It did not look at the `--visualize` plot until the very end of the session, after steps 5 to 7 had already been reported as working. It then rendered the real visualizer to an image and read that, which is where the plot observations in this log and the write-up come from.
- The offline replay and scoring scripts behind the quoted numbers were written to a temporary folder, not to this repo. A reviewer cannot rerun them from the repo, so those numbers rest on the AI's scripts and my description of them.
- `METHODOLOGY.md` still said `git clone <repo-url>`, a placeholder I have to replace with the real link.

**How I verified it ran correctly (not just that it compiled):** The run steps in `METHODOLOGY.md` were followed in a fresh clone of the repo at the step 7 commit: new venv from Python 3.12.14, `pip install -r requirements.txt`, then `python sim/launch.py --seed 7` for 12 s, which printed 11 status lines and no tracebacks. That checks the steps work as written; `--visualize` was not run in the fresh clone. The dropped-tick figure was checked against the README's own "~6 cm per minute" (5.9 cm measured).
