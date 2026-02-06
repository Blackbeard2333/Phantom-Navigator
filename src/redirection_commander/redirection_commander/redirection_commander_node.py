#!/usr/bin/env python3
"""
redirection_commander (updated for TransformStamped VICON ground-truth)

- Accepts VICON ground truth as a TransformStamped (default topic '/vicon/x500_4/x500_4')
  and converts it to an internal PoseStamped for use by the controller.
- Keeps the original tracked PoseStamped input.
- control_mode: 'cascade' (pos->vel PID) or 'pos_only' (position-only PID).
"""

import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy 
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, TransformStamped

def vec2_norm(x, y):
    return math.sqrt(x * x + y * y)


class RedirectionCommander(Node):
    def __init__(self):
        super().__init__('redirection_commander')

        # ---------------- Parameters ----------------
        self.declare_parameter('lambda_a', 0.5)
        self.declare_parameter('v_ref_max', 0.5)
        self.declare_parameter('v_spoof_max', 0.5)  # [m/s]
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('deadband_radius', 0.8)
        self.declare_parameter('frame_id', 'vicon/world')
        self.declare_parameter('attack_start_delay_s', 0.0)
        self.declare_parameter('hold_spoof_when_disarmed', False)

        # Control mode: 'cascade' (pos->vel) or 'pos_only' (no measured velocity)
        self.declare_parameter('control_mode', 'pos_only')

        # Tracked pose (PoseStamped) topic
        self.declare_parameter('pose_topic_tracked', '/tracked_uav_pose_vicon')

        # Ground-truth VICON TransformStamped topic (default as you provided)
        self.declare_parameter('ground_truth_pose_topic', '/vicon/x500_4/x500_4')
        # Use the VICON ground truth transform (TransformStamped) instead of tracked pose
        self.declare_parameter('use_vicon_ground_truth', False)

        # Outer loop (pos->v_ref) PID gains
        self.declare_parameter('base_Kp_pos', 0.5)
        self.declare_parameter('Ki_pos', 0.005)
        self.declare_parameter('Kd_pos', 0.1)
        self.declare_parameter('pos_gain_decay', 0.1)

        # Inner loop (vel->accel) PID gains (for cascade mode)
        self.declare_parameter('Kp_vel', 0.05)
        self.declare_parameter('Ki_vel', 0.002)
        self.declare_parameter('Kd_vel', 0.001)

        # Max accel used when smoothing spoof velocity in pos_only mode
        self.declare_parameter('vspoof_accel_limit', float(self.get_parameter('lambda_a').value))

        # Read params
        self.lambda_a = float(self.get_parameter('lambda_a').value)
        self.v_ref_max = float(self.get_parameter('v_ref_max').value)
        self.v_spoof_max = float(self.get_parameter('v_spoof_max').value)
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.deadband_radius = float(self.get_parameter('deadband_radius').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.attack_start_delay_s = float(self.get_parameter('attack_start_delay_s').value)
        self.hold_spoof_when_disarmed = bool(self.get_parameter('hold_spoof_when_disarmed').value)

        self.control_mode = str(self.get_parameter('control_mode').value).lower()
        if self.control_mode not in ('cascade', 'pos_only'):
            self.get_logger().warn(f"Unknown control_mode '{self.control_mode}', defaulting to 'cascade'")
            self.control_mode = 'cascade'

        self.pose_topic_tracked = str(self.get_parameter('pose_topic_tracked').value)
        self.ground_truth_pose_topic = str(self.get_parameter('ground_truth_pose_topic').value)
        self.use_vicon_ground_truth = bool(self.get_parameter('use_vicon_ground_truth').value)

        self.base_Kp_pos = float(self.get_parameter('base_Kp_pos').value)
        self.Ki_pos = float(self.get_parameter('Ki_pos').value)
        self.Kd_pos = float(self.get_parameter('Kd_pos').value)
        self.pos_gain_decay = float(self.get_parameter('pos_gain_decay').value)

        self.Kp_vel = float(self.get_parameter('Kp_vel').value)
        self.Ki_vel = float(self.get_parameter('Ki_vel').value)
        self.Kd_vel = float(self.get_parameter('Kd_vel').value)

        self.vspoof_accel_limit = float(self.get_parameter('vspoof_accel_limit').value)

        # ---------------- QoS ----------------
        qos_pose = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        qos_vel = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        qos_target = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST
        )

        # ---------------- Subs ----------------
        # Tracked pose (PoseStamped) subscription (existing tracked input)
        self.pose_sub_tracked = self.create_subscription(
            PoseStamped, self.pose_topic_tracked, self._pose_tracked_cb, qos_pose
        )

        # VICON ground-truth subscription using TransformStamped (user provided example)
        # Example the user gave:
        #   self.victim_vicon_sub = self.create_subscription(
        #       TransformStamped, '/vicon/x500_4/x500_4', self.vicon_callback, 10
        #   )
        # We use the param ground_truth_pose_topic by default and convert TransformStamped -> PoseStamped.
        self.victim_vicon_sub = self.create_subscription(
            TransformStamped, self.ground_truth_pose_topic, self._vicon_transform_cb, qos_pose
        )

        # Velocity (may not be available) - same topic as before
        self.vel_sub = self.create_subscription(TwistStamped, '/tracked_uav_velocity_vicon', self._vel_cb, qos_vel)

        # Target subscription
        self.target_sub = self.create_subscription(PoseStamped, '/redirection_commander/target', self._target_cb, qos_target)

        # ---------------- Pubs ----------------
        self.accel_pub = self.create_publisher(Vector3Stamped, '/spoofing_accel_cmd_vicon', 10)
        self.spoof_vel_pub = self.create_publisher(Vector3Stamped, '/spoofing_velocity_cmd_vicon', 10)
        self.spoof_pos_pub = self.create_publisher(Vector3Stamped, '/spoofing_position_cmd_vicon', 10)
        self.vref_pub = self.create_publisher(TwistStamped, '/redirection_commander/v_ref', 10)
        self.delta_pub = self.create_publisher(Float32, '/redirection_commander/delta_s_pid', 10)
        self.state_pub = self.create_publisher(String, '/redirection_commander/state', 5)

        # ---------------- State ----------------
        self.integral_pos_x = self.integral_pos_y = 0.0
        self.prev_error_pos_x = self.prev_error_pos_y = 0.0

        self.integral_vel_x = self.integral_vel_y = 0.0
        self.prev_error_vel_x = self.prev_error_vel_y = 0.0

        self.s_vx_prev = self.s_vy_prev = 0.0
        self.s_px_prev = self.s_py_prev = 0.0

        self.last_time = self.get_clock().now()
        self.latest_pose_tracked: Optional[PoseStamped] = None
        self.latest_pose_gt: Optional[PoseStamped] = None  # we store converted PoseStamped from TransformStamped
        self.latest_vel: Optional[TwistStamped] = None

        # Target handling
        self.target = PoseStamped()
        self.target.header.frame_id = self.frame_id
        self.target.pose.position.x = -0.36
        self.target.pose.position.y = -1.67
        self.target.pose.position.z = 0.8
        self.target_received = False

        # Arming timing
        now = self.get_clock().now()
        self.node_start_time = now
        self.attack_start_time = now.nanoseconds * 1e-9 + self.attack_start_delay_s

        # Timer
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self._control_step)

        self.get_logger().info(
            f"redirection_commander started: mode={self.control_mode}, use_vicon_ground_truth={self.use_vicon_ground_truth}"
        )
        self.get_logger().info(f"pose_tracked_topic={self.pose_topic_tracked}, ground_truth_topic={self.ground_truth_pose_topic}")

    # ------------ Callbacks ------------
    def _pose_tracked_cb(self, msg: PoseStamped):
        """Stores the tracked PoseStamped message (legacy tracked input)."""
        self.latest_pose_tracked = msg

    def _vicon_transform_cb(self, tf_msg: TransformStamped):
        """
        Converts a TransformStamped (typical VICON message) into a PoseStamped and stores it
        as the latest ground-truth pose. This is the callback shape you provided:
          TransformStamped, '/vicon/x500_4/x500_4', self.vicon_callback, 10
        """
        p = PoseStamped()
        p.header = tf_msg.header
        p.header.frame_id = tf_msg.header.frame_id or self.frame_id
        # TransformStamped has .transform.translation.{x,y,z}
        p.pose.position.x = tf_msg.transform.translation.x
        p.pose.position.y = tf_msg.transform.translation.y
        p.pose.position.z = tf_msg.transform.translation.z
        # Copy rotation quaternion directly into orientation
        p.pose.orientation = tf_msg.transform.rotation
        self.latest_pose_gt = p

    def _vel_cb(self, msg: TwistStamped):
        """Stores incoming measured velocity if available."""
        self.latest_vel = msg

    def _target_cb(self, msg: PoseStamped):
        self.target = msg
        self.target_received = True
        self.get_logger().info(
            f"Target set: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f}) in frame '{msg.header.frame_id or self.frame_id}'"
        )

    # ------------ Control core ------------
    def _control_step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1.0 / self.control_rate_hz
        self.last_time = now
        current_time = now.nanoseconds * 1e-9

        # Choose pose source
        pose_msg = self.latest_pose_gt if self.use_vicon_ground_truth else self.latest_pose_tracked

        if pose_msg is None:
            self._publish_state("IDLE")
            return

        px = pose_msg.pose.position.x
        py = pose_msg.pose.position.y

        vx = self.latest_vel.twist.linear.x if self.latest_vel else 0.0
        vy = self.latest_vel.twist.linear.y if self.latest_vel else 0.0

        tx = self.target.pose.position.x
        ty = self.target.pose.position.y

        # Pre-arm behavior
        if current_time < self.attack_start_time:
            self.integral_pos_x = self.integral_pos_y = 0.0
            self.prev_error_pos_x = self.prev_error_pos_y = 0.0
            self.integral_vel_x = self.integral_vel_y = 0.0
            self.prev_error_vel_x = self.prev_error_vel_y = 0.0

            if not self.hold_spoof_when_disarmed:
                self.s_vx_prev = self.s_vy_prev = 0.0
                self.s_px_prev = self.s_py_prev = 0.0

            self._publish_all(0.0, 0.0, self.s_vx_prev, self.s_vy_prev, self.s_px_prev, self.s_py_prev, 0.0, 0.0, "IDLE")
            ex, ey = (tx - px), (ty - py)
            self._print_iter(ex, ey, self.s_px_prev, self.s_py_prev, self.s_vx_prev, self.s_vy_prev)
            return

        # Position error
        ex = tx - px
        ey = ty - py
        r = vec2_norm(ex, ey)

        # Deadband handling
        if r <= self.deadband_radius:
            self.integral_pos_x = self.integral_pos_y = 0.0
            self.integral_vel_x = self.integral_vel_y = 0.0
            self.prev_error_pos_x = ex
            self.prev_error_pos_y = ey
            self.prev_error_vel_x = 0.0
            self.prev_error_vel_y = 0.0
            self._publish_all(0.0, 0.0, self.s_vx_prev, self.s_vy_prev, self.s_px_prev, self.s_py_prev, 0.0, 0.0, "HOLD")
            self._print_iter(ex, ey, self.s_px_prev, self.s_py_prev, self.s_vx_prev, self.s_vy_prev)
            return

        # Outer PID (pos -> v_ref)
        Kp_pos = self.base_Kp_pos * (1.0 - math.exp(-self.pos_gain_decay * r))
        Ki_pos = self.Ki_pos
        Kd_pos = self.Kd_pos

        if math.copysign(1.0, ex) != math.copysign(1.0, self.prev_error_pos_x) and self.prev_error_pos_x != 0.0:
            self.integral_pos_x = 0.0
        if math.copysign(1.0, ey) != math.copysign(1.0, self.prev_error_pos_y) and self.prev_error_pos_y != 0.0:
            self.integral_pos_y = 0.0

        self.integral_pos_x += ex * dt
        self.integral_pos_y += ey * dt
        dex = (ex - self.prev_error_pos_x) / dt
        dey = (ey - self.prev_error_pos_y) / dt

        vdx = Kp_pos * ex + Ki_pos * self.integral_pos_x + Kd_pos * dex
        vdy = Kp_pos * ey + Ki_pos * self.integral_pos_y + Kd_pos * dey

        vnorm = vec2_norm(vdx, vdy)
        if vnorm > self.v_ref_max and vnorm > 1e-9:
            s = self.v_ref_max / vnorm
            vdx *= s
            vdy *= s
        v_ref_x, v_ref_y = vdx, vdy

        self.prev_error_pos_x = ex
        self.prev_error_pos_y = ey

        # Mode-specific inner behavior
        if self.control_mode == 'cascade':
            # vel error: v_ref - measured velocity
            evx = vdx - vx
            evy = vdy - vy

            if math.copysign(1.0, evx) != math.copysign(1.0, self.prev_error_vel_x) and self.prev_error_vel_x != 0.0:
                self.integral_vel_x = 0.0
            if math.copysign(1.0, evy) != math.copysign(1.0, self.prev_error_vel_y) and self.prev_error_vel_y != 0.0:
                self.integral_vel_y = 0.0

            self.integral_vel_x += evx * dt
            self.integral_vel_y += evy * dt
            devx = (evx - self.prev_error_vel_x) / dt
            devy = (evy - self.prev_error_vel_y) / dt

            a_sx = self.Kp_vel * evx + self.Ki_vel * self.integral_vel_x + self.Kd_vel * devx
            a_sy = self.Kp_vel * evy + self.Ki_vel * self.integral_vel_y + self.Kd_vel * devy

            an = vec2_norm(a_sx, a_sy)
            if an > self.lambda_a and an > 1e-9:
                s = self.lambda_a / an
                a_sx *= s
                a_sy *= s
                an = self.lambda_a

            self.prev_error_vel_x = evx
            self.prev_error_vel_y = evy

            # Integrate spoof velocity
            self.s_vx_prev += a_sx * dt
            self.s_vy_prev += a_sy * dt

        else:
            # pos_only: set spoof velocity towards v_ref but limit dv/dt by vspoof_accel_limit
            desired_s_vx = vdx
            desired_s_vy = vdy

            dvx = desired_s_vx - self.s_vx_prev
            dvy = desired_s_vy - self.s_vy_prev

            req_ax = dvx / dt
            req_ay = dvy / dt
            req_a_norm = vec2_norm(req_ax, req_ay)

            if req_a_norm > self.vspoof_accel_limit and req_a_norm > 1e-9:
                scale = self.vspoof_accel_limit / req_a_norm
                req_ax *= scale
                req_ay *= scale

            a_sx = req_ax
            a_sy = req_ay

            self.s_vx_prev += a_sx * dt
            self.s_vy_prev += a_sy * dt

        # clamp spoof velocity magnitude
        vspoof = math.hypot(self.s_vx_prev, self.s_vy_prev)
        if vspoof > self.v_spoof_max and vspoof > 1e-9:
            s = self.v_spoof_max / vspoof
            self.s_vx_prev *= s
            self.s_vy_prev *= s

        # integrate spoof position
        self.s_px_prev += self.s_vx_prev * dt
        self.s_py_prev += self.s_vy_prev * dt

        self._publish_all(a_sx, a_sy, self.s_vx_prev, self.s_vy_prev, self.s_px_prev, self.s_py_prev, v_ref_x, v_ref_y, "TRACK")
        self._print_iter(ex, ey, self.s_px_prev, self.s_py_prev, self.s_vx_prev, self.s_vy_prev)

    # ------------ Publish helpers ------------
    def _publish_all(self, a_sx, a_sy, s_vx, s_vy, s_px, s_py, v_ref_x, v_ref_y, state):
        stamp = self.get_clock().now().to_msg()

        accel = Vector3Stamped()
        accel.header.stamp = stamp
        accel.header.frame_id = self.frame_id
        accel.vector.x = a_sx
        accel.vector.y = a_sy
        accel.vector.z = 0.0
        self.accel_pub.publish(accel)

        sv = Vector3Stamped()
        sv.header.stamp = stamp
        sv.header.frame_id = self.frame_id
        sv.vector.x = s_vx
        sv.vector.y = s_vy
        sv.vector.z = 0.0
        self.spoof_vel_pub.publish(sv)

        sp = Vector3Stamped()
        sp.header.stamp = stamp
        sp.header.frame_id = self.frame_id
        sp.vector.x = s_px
        sp.vector.y = s_py
        sp.vector.z = 0.0
        self.spoof_pos_pub.publish(sp)

        vref = TwistStamped()
        vref.header.stamp = stamp
        vref.header.frame_id = self.frame_id
        vref.twist.linear.x = v_ref_x
        vref.twist.linear.y = v_ref_y
        vref.twist.linear.z = 0.0
        self.vref_pub.publish(vref)

        d = Float32()
        d.data = float(math.sqrt(a_sx * a_sx + a_sy * a_sy))
        self.delta_pub.publish(d)

        self._publish_state(state)

    def _publish_state(self, state: str):
        s = String()
        s.data = state
        self.state_pub.publish(s)

    # ------------ Logging helper ------------
    def _print_iter(self, ex, ey, s_px, s_py, s_vx, s_vy):
        self.get_logger().info(
            f"err_pos=({ex:+.3f}, {ey:+.3f}) | spoof_pos=({s_px:+.3f}, {s_py:+.3f}) | spoof_vel=({s_vx:+.3f}, {s_vy:+.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = RedirectionCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
