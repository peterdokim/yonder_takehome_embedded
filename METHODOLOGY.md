# Methodology


## 1. How to run it

Tested on macOS 26.7 (Apple Silicon) with Python 3.12.14. Python 3.10 or newer is required: the simulator uses `float | None` type hints, which crash on 3.9.

```bash
git clone https://github.com/peterdokim/yonder_takehome_embedded.git
cd embedded-take-home
python3.12 -m venv .venv
source .venv/bin/activate
python --version                 # must print 3.10 or newer
pip install -r requirements.txt  # numpy, matplotlib
python sim/launch.py --visualize
```

- If `python --version` shows 3.9 inside the venv (this happened to me with Anaconda active), run `conda deactivate`, delete `.venv`, and create it again.
- `python sim/launch.py` runs without the plot and prints one status line per second.
- `python sim/launch.py --seed 7` repeats the exact noise of the runs quoted below.
- `python sim/launch.py --no-faults` turns the injected noise off.
- Do not run `python odometry_node.py` directly; the simulator loads it.

What you should see:

- **Terminal:** a line every second such as
  `[odom] pos=(9.19, 14.63)m hdg=110deg v=0.50m/s  enc=0.01s ago @50.0Hz (expect 50)  gps=2.01s ago  dup=37 bad=0`.
  About 40 s into each 45 s cycle the GPS goes quiet and the line ends with `WARN: GPS STALE` for three seconds.
- **Plot:** the blue line (my `/odom`) follows the true circle, and its error in the bottom plot sits below the GPS dots.

All my code is in `odometry_node.py`. Nothing under `sim/` is changed.

## 2. Thought process

- **Order of work.** I followed the README's steps and committed after each: subscription, wheel odometry, noise handling, GPS fusion, monitoring, long run.
- **QoS first.** Both simulator publishers are `BEST_EFFORT`, so the subscriptions are `BEST_EFFORT` too. A default `RELIABLE` subscriber connects to nothing and fails silently.
- **Wheels give distance only.** Each message's new ticks times 1.31 mm is the distance rolled. Speed is that distance over the difference of two encoder timestamps, which stays valid even though the encoder clock drifts.
- **Duplicates.** A message is a duplicate only if both `tick_count` and `timestamp` match the previous one. A stopped rover has the same ticks but a new timestamp, so it is still processed as "moved 0 m". Duplicates are dropped before any division, so the velocity is never NaN or infinite.
- **Dropped ticks cannot be detected.** A message with 7 new ticks looks the same whether 7 happened or 8 happened and one was lost. The wheel distance is therefore slightly short and falls behind over time. Only an absolute reference can correct that, which is the job of GPS.
- **Position fusion is a variance-weighted average.** I keep one number for how uncertain the wheel position is. It grows with every metre driven and shrinks at each GPS fix. The share taken from GPS is `pos_var / (pos_var + gps_covariance)`, so GPS counts for more after a long gap and for less when it reports high noise (the 10 s after each outage).
- **Heading comes from GPS, not from a sensor.** Nothing reports heading. If GPS keeps putting the rover to the left of my estimate, my heading must be too far right. The sideways error divided by the distance driven since the last fix is the heading error that fix implies, and I apply 30% of it scaled by the GPS weight, because a single fix is far too noisy to trust.
- **Turn rate is learned as well.** If the heading keeps needing the same correction, the rover is turning more than I assume. So the same error also adjusts a turn-per-metre estimate, which keeps the heading rotating between fixes and through the 5 s outages.
- **No path is hard-coded.** The only assumptions are the start (origin, facing east) and that the turn rate changes slowly. Radius, speed and turn direction are all learned.
- **The gains were tuned, not guessed.** My first three gain values gave a mean error of 0.90 m, worse than raw GPS at 0.54 m. I then searched the gains with an offline replay of the simulator's noise models over seven paths and checked the result on 20 seeds that were not used for tuning.
- **Monitoring thresholds.** Encoder stale after 0.2 s (10 missed messages), GPS stale after 2.5 s (two missed fixes, so one late fix is quiet but an outage warns), and a rate warning outside 45 to 55 Hz. Ages are measured with `time.monotonic()` at arrival, never from message timestamps.
- **Results.** In a 6-minute live run with faults on (`--seed 7`), mean position error was 0.26 m against 0.54 m for raw GPS, mean heading error was 5.3 degrees, the error did not grow from minute to minute, and there were no exceptions or non-finite values. In the offline replay, the other paths (clockwise, radius 4 m, radius 30 m at 1.5 m/s, 0.2 m/s, straight line) averaged 0.24 to 0.34 m.

## 3. Known limitations

- **Sudden changes of turn.** The turn rate is learned slowly, so a path that reverses its turn is tracked worst: 0.42 m mean error on an S-curve in the offline replay, with a worst spike of 2.7 m.
- **Spikes after an outage and at start-up.** The largest errors (1.2 m in the live run, up to about 2 m offline) fall in the noisy seconds after GPS returns or in the first 10 s, when the turn rate is not yet learned.
- **Heading needs movement.** A rover moving less than 0.1 m between fixes gets no heading correction, and a rover that spins on the spot or drives backwards would not be tracked at all.
- **No GPS outlier rejection.** A wildly wrong fix that reports a small covariance would be believed. The simulator never sends one, so this is untested.
- **Gains fit the simulator.** They were tuned against a copy of the simulator's noise, which is independent from fix to fix. Real GPS errors wander slowly, and the gains would need retuning on real data.
- **Simplified uncertainty.** `pos_var` is a single tuning number, not a measured covariance, and the covariance fields of the `/odom` message are left empty.
- **Velocity is unsmoothed.** It jitters between about 0.39 and 0.57 m/s around a true 0.50, because each 20 ms message holds 7 or 8 whole ticks.
- **Encoder failure.** If the encoder stops, velocity is reported as 0 and the position only moves with GPS fixes. The simulator never stops the encoder, so this was tested only by calling the code directly.
- **Stretch goals** were not attempted.

What I would do next: reject GPS fixes that disagree with the wheel distance by more than their stated noise allows, smooth the velocity over a few messages, and replace the hand-tuned gains with a small Kalman filter over position, heading and turn rate.
