#!/usr/bin/env python3
"""
redirection_target_playlist_sender.py

Publishes a time-programmable playlist of PoseStamped targets to
/redirection_commander/target. Useful for sequential spoofing targets.

Parameters (ros2 param set or CLI -p):
  frame_id               str   default 'vicon/world'
  radius                 float default 2.0        # circle radius [m]
  n_points               int   default 10         # number of random targets
  dwell_time_s           float default 10.0       # seconds per target
  attack_start_delay_s   float default 20.0       # delay before starting playlist
  random_seed            int   default -1         # seed <0 => random
  loop                   bool  default False      # repeat playlist forever
  reannounce_rate_hz     float default 0.0        # if >0, periodically republish current target at this rate
  publish_before_attack  bool  default False      # if True, publish origin (0,0) before attack_start_delay_s
"""

import math
import random
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped


def uniform_point_in_circle(radius: float) -> Tuple[float, float]:
    # Uniform distribution in circle: r = R * sqrt(u), theta = 2*pi*v
    u = random.random()
    v = random.random()
    r = radius * math.sqrt(u)
    theta = 2.0 * math.pi * v
    return r * math.cos(theta), r * math.sin(theta)


class RedirectionTargetPlaylistSender(Node):
    def __init__(self):
        super().__init__('redirection_target_playlist_sender')

        # --- parameters ---
        self.declare_parameter('frame_id', 'vicon/world')
        self.declare_parameter('radius', 1.5)
        self.declare_parameter('n_points', 10)
        self.declare_parameter('dwell_time_s', 10.0)
        self.declare_parameter('attack_start_delay_s', 20.0)
        self.declare_parameter('random_seed', -1)
        self.declare_parameter('loop', False)
        self.declare_parameter('reannounce_rate_hz', 0.0)
        self.declare_parameter('publish_before_attack', False)

        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.radius = float(self.get_parameter('radius').value)
        self.n_points = int(self.get_parameter('n_points').value)
        self.dwell_time_s = float(self.get_parameter('dwell_time_s').value)
        self.attack_start_delay_s = float(self.get_parameter('attack_start_delay_s').value)
        self.random_seed = int(self.get_parameter('random_seed').value)
        self.loop = bool(self.get_parameter('loop').value)
        self.reannounce_rate_hz = float(self.get_parameter('reannounce_rate_hz').value)
        self.publish_before_attack = bool(self.get_parameter('publish_before_attack').value)

        # QoS (match redirection_commander): RELIABLE + TRANSIENT_LOCAL, depth=1
        qos_target = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub = self.create_publisher(PoseStamped, '/redirection_commander/target', qos_target)

        # Timing / playlist state
        self.node_start_time = self.get_clock().now()
        self.attack_start_time = (self.node_start_time.nanoseconds * 1e-9) + self.attack_start_delay_s

        # Build playlist
        if self.random_seed >= 0:
            random.seed(self.random_seed)
        self.playlist: List[Tuple[float, float]] = [
            uniform_point_in_circle(self.radius) for _ in range(self.n_points)
        ]
        # current index and timestamps
        self.current_index = 0
        self.play_started = False
        self.current_target_start_time = None  # wallclock time (seconds)
        self.reannounce_timer = None

        # periodic timer to check time and advance when needed (small-ish period)
        # use a modest timer period (0.2s) so timing is reasonably accurate
        self.timer = self.create_timer(0.2, self._periodic_tick)

        self.get_logger().info(
            f"Started playlist sender: radius={self.radius}, n_points={self.n_points}, "
            f"dwell_time_s={self.dwell_time_s}, attack_start_delay_s={self.attack_start_delay_s}, "
            f"loop={self.loop}, seed={self.random_seed}"
        )

        # Optional: support runtime param updates (simplified behavior: changes won't retroactively rebuild playlist)
        self.add_on_set_parameters_callback(self._on_param_update)

    def _on_param_update(self, params):
        updated = False
        for p in params:
            if p.name == 'frame_id':
                self.frame_id = p.value; updated = True
            elif p.name == 'radius':
                self.radius = float(p.value); updated = True
            elif p.name == 'n_points':
                self.n_points = int(p.value); updated = True
            elif p.name == 'dwell_time_s':
                self.dwell_time_s = float(p.value); updated = True
            elif p.name == 'attack_start_delay_s':
                self.attack_start_delay_s = float(p.value); updated = True
            elif p.name == 'random_seed':
                self.random_seed = int(p.value); updated = True
            elif p.name == 'loop':
                self.loop = bool(p.value); updated = True
            elif p.name == 'reannounce_rate_hz':
                self.reannounce_rate_hz = float(p.value); updated = True
            elif p.name == 'publish_before_attack':
                self.publish_before_attack = bool(p.value); updated = True

        # If playlist params changed, rebuild playlist (simple behavior)
        if updated:
            if self.random_seed >= 0:
                random.seed(self.random_seed)
            self.playlist = [uniform_point_in_circle(self.radius) for _ in range(self.n_points)]
            self.current_index = 0
            self.play_started = False
            # recalc attack start time relative to now
            now = self.get_clock().now()
            self.node_start_time = now
            self.attack_start_time = (now.nanoseconds * 1e-9) + self.attack_start_delay_s
            self.get_logger().info("Playlist rebuilt after parameter update; will start on next attack_start_time")

        from rclpy.parameter import SetParametersResult
        return SetParametersResult(successful=True)

    def _periodic_tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # Before attack start
        if not self.play_started and now < self.attack_start_time:
            if self.publish_before_attack:
                # optionally publish origin as pre-attack 'safe' target
                self._publish_pose(0.0, 0.0)
            return

        # If attack not started yet, initialize
        if not self.play_started:
            self.play_started = True
            self.current_index = 0
            self.current_target_start_time = now
            x, y = self.playlist[self.current_index]
            self._publish_pose(x, y)
            self._maybe_start_reannounce_timer()
            self.get_logger().info(f"Attack started: publishing playlist[0] -> ({x:.3f}, {y:.3f})")
            return

        # If playlist finished and not looping, stop
        elapsed = now - (self.current_target_start_time or now)
        if elapsed >= self.dwell_time_s:
            # advance index
            self.current_index += 1
            if self.current_index >= len(self.playlist):
                if self.loop:
                    self.current_index = 0
                    self.current_target_start_time = now
                    x, y = self.playlist[self.current_index]
                    self._publish_pose(x, y)
                    self.get_logger().info(f"Looping playlist: publishing index 0 -> ({x:.3f}, {y:.3f})")
                else:
                    # finished: optionally stop reannouncing and log finalize
                    self._stop_reannounce_timer()
                    self.get_logger().info("Playlist finished (no loop). No further targets will be published.")
                    # do not destroy node; just stop updating
                    # keep last target latched on TransientLocal for subscribers
                    self.timer.cancel()
                    return
            else:
                self.current_target_start_time = now
                x, y = self.playlist[self.current_index]
                self._publish_pose(x, y)
                self.get_logger().info(f"Advancing playlist: publishing index {self.current_index} -> ({x:.3f}, {y:.3f})")

    def _publish_pose(self, x: float, y: float, z: float = 0.0):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # leave orientation default (0,0,0,0) — commander ignores orientation
        self.pub.publish(msg)

    def _maybe_start_reannounce_timer(self):
        self._stop_reannounce_timer()
        if self.reannounce_rate_hz > 0.0:
            period = max(1.0 / self.reannounce_rate_hz, 0.01)
            self.reannounce_timer = self.create_timer(period, self._reannounce_current_target)

    def _stop_reannounce_timer(self):
        if self.reannounce_timer is not None:
            try:
                self.reannounce_timer.cancel()
            except Exception:
                pass
            self.reannounce_timer = None

    def _reannounce_current_target(self):
        # republish current target (useful so late-joiners get it repeatedly while attack is active)
        if not self.play_started:
            return
        idx = max(0, min(self.current_index, len(self.playlist) - 1))
        x, y = self.playlist[idx]
        self._publish_pose(x, y)
        # keep this at DEBUG-level to avoid log spam
        self.get_logger().debug(f"Re-announced current playlist target idx={idx} -> ({x:.3f}, {y:.3f})")


def main(args=None):
    rclpy.init(args=args)
    node = RedirectionTargetPlaylistSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
