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
