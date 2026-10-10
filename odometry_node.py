"""
Odometry Node — your implementation goes here.

Run via:
    python sim/launch.py
    python sim/launch.py --visualize      # with live plot
    python sim/launch.py --no-faults      # clean feeds while getting started

This file uses rclpy_lite, a lightweight simulator shim that mirrors the real
rclpy (ROS 2 Python) API. The class names, method signatures and message field
names match real ROS 2, and the import paths differ:

    Real ROS 2:     import rclpy / from rclpy.node import Node
    This shim:      import rclpy_lite as rclpy / from rclpy_lite.node import Node

(The README lists the few small differences, e.g. header stamps are plain floats.)

Two things that make life easier here:
  - Callbacks never run at the same time as each other (real rclpy does the same
    with its default executor), so you do not need locks around your state.
  - If a callback raises an exception you will see a traceback in the terminal.
    The run keeps going, so read the terminal, don't just look for a crash.
"""

import sys

if __name__ == "__main__":
    # This file is loaded BY the simulator; running it directly can't work.
    # (Delete this guard if you ever port the node to a real ROS 2 install.)
    sys.exit("Don't run this file directly. Start everything with:\n\n    python sim/launch.py\n")

import math
import time

import rclpy_lite as rclpy
from rclpy_lite.node import Node
from rclpy_lite.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

# Message types provided by the simulator
from sim.messages import WheelTicks, GPSEstimate

# Standard ROS message types (mirrored by the shim)
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

# --------------------------------------------------------------------------
# Constants  (do not change — these match the real rover's encoder setup)
# --------------------------------------------------------------------------
WHEEL_RADIUS_M = 0.075          # metres
TICKS_PER_REVOLUTION = 360
DIST_PER_TICK = (2.0 * math.pi * WHEEL_RADIUS_M) / TICKS_PER_REVOLUTION  # ~0.00131 m

# --------------------------------------------------------------------------
# Fusion tuning  (my choices — see "Your write-up" in the README)
# --------------------------------------------------------------------------
# How fast the wheel-derived position gets less trustworthy: variance (m²)
# added per metre driven. Not a measured property of the encoder; it is the
# knob that sets how hard GPS pulls. Bigger = follow GPS more closely.
WHEEL_DRIFT_VAR_PER_M = 0.04

# Fraction of the heading error implied by one GPS fix that is applied to the
# heading, and to the turn-per-metre estimate. Both are also scaled by the
# position gain, so a noisy fix moves them less.
HEADING_GAIN = 0.3
CURVATURE_GAIN = 0.02

# Below this distance since the last fix, sideways GPS error says nothing
# about heading (a stopped rover would just be steering on GPS noise).
MIN_HEADING_BASELINE_M = 0.1

# Never believe a turn tighter than this radius; stops a bad run of fixes
# from winding the turn estimate up without limit.
MAX_CURVATURE = 1.0 / 0.5       # rad per metre

# Floor for the GPS variance so a bogus covariance of 0 can't take the gain to 1.
MIN_GPS_VARIANCE = 1e-4         # m²


def wrap_angle(angle: float) -> float:
    """Wrap an angle to (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class OdometryNode(Node):
    """
    Fuses wheel encoder ticks with GPS estimates to produce odometry.

    Subscribes:
        /wheel_ticks   (sim.messages.WheelTicks)   ~50 Hz, BEST_EFFORT
        /gps_estimate  (sim.messages.GPSEstimate)   ~1 Hz,  BEST_EFFORT

    Publishes:
        /odom          (nav_msgs/Odometry)          matches input rate
    """

    def __init__(self):
        super().__init__("odometry_node")

        # ------------------------------------------------------------------
        # Subscribers
        # ------------------------------------------------------------------
        # Both sim publishers are BEST_EFFORT / VOLATILE (see sim/encoder_publisher.py).
        # A RELIABLE subscriber can't be served by a BEST_EFFORT publisher, so the
        # default QoS would connect to nothing. Match the publisher instead.
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.tick_sub = self.create_subscription(
            WheelTicks, "/wheel_ticks", self.wheel_tick_callback, sensor_qos
        )

        self.gps_sub = self.create_subscription(
            GPSEstimate, "/gps_estimate", self.gps_callback, sensor_qos
        )

        # ------------------------------------------------------------------
        # Publisher
        # ------------------------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # ------------------------------------------------------------------
        # Timer — 1 Hz monitoring output
        # ------------------------------------------------------------------
        # TODO: Create a timer that calls self.monitoring_callback once per second.
        #
        # self.monitor_timer = self.create_timer(1.0, self.monitoring_callback)

        # ------------------------------------------------------------------
        # State  — add whatever you need
        # ------------------------------------------------------------------
        self.x: float = 0.0                  # fused position, metres (east)
        self.y: float = 0.0                  # fused position, metres (north)
        self.heading: float = 0.0            # radians. The rover starts facing EAST (0 rad);
                                             # x is east, y is north, counter-clockwise is positive

        self.velocity: float = 0.0           # forward speed from the wheels, m/s

        # Turn per metre driven (rad/m, positive = turning left). There is no
        # heading sensor, so this is learned from GPS in gps_callback and used
        # to keep the heading turning between fixes. Starts at 0 = straight.
        self.curvature: float = 0.0

        # Uncertainty (variance, m²) of the fused position. 0 at the start
        # because the start position is given exactly; grows as we drive on
        # wheels alone and shrinks at every GPS fix.
        self.pos_var: float = 0.0

        self.dist_since_gps: float = 0.0     # metres driven since the last GPS fix
        self.gps_msg_count: int = 0
        self.gps_rejected_count: int = 0     # GPS messages ignored as not finite

        self.last_tick_count: int | None = None
        self.last_tick_time: float | None = None   # encoder timestamp of the last message

        self.last_gps_time: float | None = None
        self.last_wheel_time: float | None = None

        self.wheel_msg_count: int = 0        # every message received, duplicates included
        self.duplicate_count: int = 0        # messages ignored as duplicates
        self.bad_tick_count: int = 0         # messages ignored as backwards in ticks or time
        self.start_time: float = time.monotonic()   # use time.monotonic() to measure durations

    # -----------------------------------------------------------------------
    # Wheel encoder callback
    # -----------------------------------------------------------------------

    def wheel_tick_callback(self, msg: WheelTicks) -> None:
        """
        Called each time a wheel tick message arrives (~50 Hz).

        Convert the change in tick count to a distance, then update your
        position estimate and publish odometry.

        Args:
            msg.tick_count   Cumulative encoder ticks (monotonically increases).
            msg.timestamp    Wall-clock seconds when the reading was taken.
                             Subject to slow clock drift — see README.

        Noise to handle:
            DUPLICATE messages   The same (tick_count, timestamp) pair is
                                 occasionally re-sent within a few ms.
                                 If you compute velocity as delta_ticks / delta_time
                                 and delta_time is 0, you get a NaN, an infinite
                                 velocity, or a ZeroDivisionError. Check for this.

            DROPPED ticks        Now and then the encoder misses one tick (~1.3mm)
                                 and the cumulative counter is simply 1 lower than
                                 it should be. The data never shows a gap, so you
                                 cannot spot an individual drop. The error just
                                 adds up (~6cm per minute), and GPS fusion is your
                                 correction. (Think about why you can't see it.)

            CLOCK DRIFT          msg.timestamp wanders slowly away from the real
                                 time. Use differences between timestamps as
                                 durations; never compare them with time.time().

        Useful constants:
            DIST_PER_TICK   metres per encoder tick (~0.00131 m)
        """
        # TODO: implement
        #
        # Suggested approach:
        #   1. Handle first message (initialise last_tick_count / last_tick_time).
        #   2. Compute delta_ticks = msg.tick_count - self.last_tick_count
        #   3. Guard against duplicate (delta_ticks == 0 and delta_time ≈ 0).
        #   4. Convert ticks to distance: distance = delta_ticks * DIST_PER_TICK
        #   5. Update self.x and self.y: the rover moves `distance` in the direction
        #      it is FACING. Ticks tell you how far, not which way. Where does your
        #      heading come from and how does it change? See "Heading is not
        #      measured" in the README before you write this line.
        #   6. Call self.publish_odometry().
        now = time.monotonic()
        self.last_wheel_time = now
        self.wheel_msg_count += 1

        # First message: nothing to difference against yet, just remember it.
        if self.last_tick_count is None:
            self.last_tick_count = msg.tick_count
            self.last_tick_time = msg.timestamp
            return

        # DUPLICATE: a re-sent message carries exactly the same tick_count AND
        # timestamp as the one before it. Both must match: a stopped rover also
        # reports delta_ticks == 0, but with a new timestamp, and that is a real
        # "moved 0 m" reading, not a duplicate.
        if (
            msg.tick_count == self.last_tick_count
            and msg.timestamp == self.last_tick_time
        ):
            self.duplicate_count += 1
            return

        delta_ticks = msg.tick_count - self.last_tick_count
        # Both timestamps are on the encoder's clock, so their difference is a
        # valid duration even though that clock drifts from ours.
        delta_time = msg.timestamp - self.last_tick_time

        # The counter is cumulative and the encoder's clock only moves forward,
        # so a message that goes backwards in either is not usable (a late
        # arrival, or the counter restarting). Skip it without moving the
        # baseline, so the next good message still differences correctly.
        # This also guarantees delta_time > 0 for the division below.
        if delta_ticks < 0 or delta_time <= 0.0:
            self.bad_tick_count += 1
            return

        self.last_tick_count = msg.tick_count
        self.last_tick_time = msg.timestamp

        # DROPPED ticks are not handled here because they cannot be: a dropped
        # tick just makes the cumulative count 1 lower than the truth, and a
        # message with 7 new ticks looks the same whether 7 happened or 8 did
        # and one was lost. So `distance` is biased slightly short and the
        # position slowly falls behind; only an absolute reference (GPS, in
        # gps_callback) can pull it back.
        distance = delta_ticks * DIST_PER_TICK
        self.velocity = distance / delta_time

        # Dead reckoning: move `distance` along the heading, and turn the
        # heading by (turn per metre) x distance. Using the heading halfway
        # through the step makes a constant turn come out as an arc rather
        # than a string of straight segments.
        turn = self.curvature * distance
        mid_heading = self.heading + 0.5 * turn
        self.x += distance * math.cos(mid_heading)
        self.y += distance * math.sin(mid_heading)
        self.heading = wrap_angle(self.heading + turn)

        # Every metre on wheels alone makes the position a little less certain.
        self.pos_var += WHEEL_DRIFT_VAR_PER_M * distance
        self.dist_since_gps += distance

        self.publish_odometry()

        # Temporary progress print (~1 Hz) until the step 6 monitoring output.
        if self.wheel_msg_count % 50 == 0:
            print(
                f"[odom] pos=({self.x:.2f}, {self.y:.2f})m "
                f"hdg={math.degrees(self.heading):.0f}deg v={self.velocity:.2f}m/s "
                f"dup={self.duplicate_count} bad={self.bad_tick_count}"
            )

    # -----------------------------------------------------------------------
    # GPS callback
    # -----------------------------------------------------------------------

    def gps_callback(self, msg: GPSEstimate) -> None:
        """
        Called each time a GPS estimate arrives (~1 Hz, with occasional outages).

        Fuse the GPS absolute position with your wheel-derived position.
        A weighted average is sufficient — no EKF required.

        Args:
            msg.x           GPS east position (metres from start).
            msg.y           GPS north position (metres from start).
            msg.timestamp   Seconds since epoch when the reading was taken.
            msg.covariance  Uncertainty (m²). Higher = less reliable.
                            Use this to adjust how much you trust the GPS
                            reading relative to your wheel estimate.

        Design question:
            GPS is noisy but absolute (no accumulated drift).
            Wheel odometry drifts but is precise over short distances.
            How do you weight them? Should the weighting change over time?
            (That's Stretch Goal B — but even a fixed weighting is fine here.)
        """
        # Arrival time on OUR monotonic clock, for the staleness monitor.
        self.last_gps_time = time.monotonic()
        self.gps_msg_count += 1

        if not (
            math.isfinite(msg.x)
            and math.isfinite(msg.y)
            and math.isfinite(msg.covariance)
        ):
            self.gps_rejected_count += 1
            return

        # --- Position: variance-weighted average (the README's worked example).
        # gain = share of the correction taken from GPS. It is large when we
        # have driven far since the last fix (pos_var grew) and small when the
        # fix is noisy (big covariance).
        gps_var = max(msg.covariance, MIN_GPS_VARIANCE)
        gain = self.pos_var / (self.pos_var + gps_var)

        err_x = msg.x - self.x
        err_y = msg.y - self.y
        # Sideways part of the error: positive = GPS puts us to the LEFT of
        # where we think we are, relative to the way we think we are facing.
        cross = -math.sin(self.heading) * err_x + math.cos(self.heading) * err_y

        self.x += gain * err_x
        self.y += gain * err_y
        self.pos_var *= (1.0 - gain)

        # --- Heading: no sensor gives it, so infer it from the sideways error.
        # Driving `baseline` metres with a heading that is off by a small angle
        # puts us sideways by about baseline x angle, so cross / baseline is the
        # heading error this fix implies. One fix is far too noisy to believe
        # outright (0.3 m of GPS noise over 0.5 m of travel), so take a small
        # fraction of it each time and let the fixes average out.
        #
        # If we keep having to nudge the heading the same way, the rover is
        # turning more than we think, so the same error also adjusts the turn
        # per metre. That is what lets the heading keep up on a curve between
        # fixes and through a GPS outage.
        baseline = self.dist_since_gps
        if baseline >= MIN_HEADING_BASELINE_M:
            implied_heading_err = cross / baseline
            self.heading = wrap_angle(
                self.heading + HEADING_GAIN * gain * implied_heading_err
            )
            self.curvature += CURVATURE_GAIN * gain * implied_heading_err / baseline
            self.curvature = max(-MAX_CURVATURE, min(MAX_CURVATURE, self.curvature))
        self.dist_since_gps = 0.0

        self.publish_odometry()

    # -----------------------------------------------------------------------
    # Odometry publisher
    # -----------------------------------------------------------------------

    def publish_odometry(self) -> None:
        """
        Build a nav_msgs/Odometry message from the current state and publish it.

        The Odometry message layout (real ROS 2 spec, mirrored by the shim):
            msg.header.stamp         float, seconds since epoch
            msg.header.frame_id      str,   "odom"
            msg.child_frame_id       str,   "base_link"
            msg.pose.pose.position.x float, metres east
            msg.pose.pose.position.y float, metres north
            msg.pose.pose.orientation  Quaternion(x, y, z, w)
                For a 2D robot rotating around z: yaw → quaternion:
                    z = sin(heading / 2)
                    w = cos(heading / 2)
                    x = 0, y = 0
            msg.twist.twist.linear.x float, forward speed (m/s)
            msg.twist.twist.angular.z float, yaw rate (rad/s)

        Look up the real nav_msgs/Odometry definition for the full field list.
        Not every field is required — position is the minimum.
        """
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(self.heading / 2.0),
            w=math.cos(self.heading / 2.0),
        )

        msg.twist.twist.linear.x = self.velocity
        msg.twist.twist.angular.z = self.curvature * self.velocity   # rad/m x m/s

        self.odom_pub.publish(msg)

    # -----------------------------------------------------------------------
    # Monitoring callback  (1 Hz)
    # -----------------------------------------------------------------------

    def monitoring_callback(self) -> None:
        """
        Print a one-line status update to the terminal once per second.

        Required output (see README rubric):
            - Seconds since last /wheel_ticks message
            - Seconds since last /gps_estimate message
            - Measured /wheel_ticks receive rate vs expected ~50 Hz
            - Current fused position (x, y)

        Tip: record time.monotonic() each time a message ARRIVES and subtract.
        Don't use msg.timestamp for this: the encoder's clock drifts.

        Example format (feel free to change the layout):
            [odom] pos=(1.23, 0.45)m  enc=0.02s ago @49.8Hz  gps=0.91s ago
        """
        # TODO: implement
        pass


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    rclpy.init()
    node = OdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
