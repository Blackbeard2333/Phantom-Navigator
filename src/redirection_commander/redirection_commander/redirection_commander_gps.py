#!/usr/bin/env python3
"""
redirection_commander_gps.py
Outdoor FMU-only version with:
 - VehicleLocalPosition subscription (attacker local NED)
 - Victim estimated local position computed in tracker's NED
 - Target defined in tracker-local NED (takeoff + target_bias_ned), with optional linear ramp
 - PID on victim_local -> target_local (N,E,D)
 - Publishes victim estimated local pos + SensorGps estimate + spoof topics (NED, NED_error, latlon)
"""

import math
from typing import Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Vector3Stamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header

from px4_msgs.msg import VehicleAttitude, SensorGps, VehicleLocalPosition
try:
    from px4_msgs.msg import VehicleGpsPosition
except Exception:
    VehicleGpsPosition = None

# WGS-84 constants
_A = 6378137.0
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)
_B = _A * (1.0 - _F)
_E2P = (_A*_A - _B*_B) / (_B*_B)


# --- math helpers ---
def quat_to_rotmat(qx, qy, qz, qw):
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n == 0.0:
        return [[1,0,0],[0,1,0],[0,0,1]]
    qx, qy, qz, qw = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    xy, xz, yz = qx*qy, qx*qz, qy*qz
    wx, wy, wz = qw*qx, qw*qy, qw*qz
    return [
        [1 - 2*(yy + zz),     2*(xy - wz),       2*(xz + wy)],
        [2*(xy + wz),         1 - 2*(xx + zz),   2*(yz - wx)],
        [2*(xz - wy),         2*(yz + wx),       1 - 2*(xx + yy)],
    ]


def matvec(R, v):
    return [
        R[0][0]*v[0] + R[0][1]*v[1] + R[0][2]*v[2],
        R[1][0]*v[0] + R[1][1]*v[1] + R[1][2]*v[2],
        R[2][0]*v[0] + R[2][1]*v[1] + R[2][2]*v[2],
    ]


def matmul(A, B):
    return [
        [A[r][0]*B[0][c] + A[r][1]*B[1][c] + A[r][2]*B[2][c] for c in range(3)]
        for r in range(3)
    ]


def lla_to_ecef(lat_deg, lon_deg, alt_m):
    lat = math.radians(lat_deg); lon = math.radians(lon_deg)
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    sin_lon, cos_lon = math.sin(lon), math.cos(lon)
    N = _A / math.sqrt(1.0 - _E2 * sin_lat*sin_lat)
    x = (N + alt_m) * cos_lat * cos_lon
    y = (N + alt_m) * cos_lat * sin_lon
    z = (N*(1.0 - _E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_lla(x, y, z):
    p = math.sqrt(x*x + y*y)
    lon = 0.0 if p < 1e-12 else math.atan2(y, x)
    theta = math.atan2(z * _A, p * _B)
    sin_t, cos_t = math.sin(theta), math.cos(theta)
    lat = math.atan2(z + _E2P * _B * (sin_t ** 3),
                     p - _E2  * _A * (cos_t ** 3))
    sin_lat = math.sin(lat)
    N = _A / math.sqrt(1.0 - _E2 * sin_lat * sin_lat)
    alt = p / math.cos(lat) - N
    return math.degrees(lat), math.degrees(lon), alt


def enu_to_ecef_delta(dE, dN, dU, lat0_deg, lon0_deg):
    lat0 = math.radians(lat0_deg); lon0 = math.radians(lon0_deg)
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)
    e_ecef = [-sin_lon,             cos_lon,              0.0]
    n_ecef = [-sin_lat*cos_lon,    -sin_lat*sin_lon,     cos_lat]
    u_ecef = [ cos_lat*cos_lon,     cos_lat*sin_lon,     sin_lat]
    dx = dE*e_ecef[0] + dN*n_ecef[0] + dU*u_ecef[0]
    dy = dE*e_ecef[1] + dN*n_ecef[1] + dU*u_ecef[1]
    dz = dE*e_ecef[2] + dN*n_ecef[2] + dU*u_ecef[2]
    return dx, dy, dz


def enu_from_ecef_delta(dx, dy, dz, lat0_deg, lon0_deg):
    lat0 = math.radians(lat0_deg); lon0 = math.radians(lon0_deg)
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)
    R = [
        [-sin_lon,              cos_lon,              0.0],
        [-sin_lat*cos_lon,     -sin_lat*sin_lon,     cos_lat],
        [ cos_lat*cos_lon,      cos_lat*sin_lon,     sin_lat],
    ]
    dE = R[0][0]*dx + R[0][1]*dy + R[0][2]*dz
    dN = R[1][0]*dx + R[1][1]*dy + R[1][2]*dz
    dU = R[2][0]*dx + R[2][1]*dy + R[2][2]*dz
    return dE, dN, dU


def ned_from_lla_to_lla(ref: Tuple[float,float,float], tgt: Tuple[float,float,float]) -> Tuple[float,float,float]:
    lat0, lon0, alt0 = ref
    lat1, lon1, alt1 = tgt
    x0, y0, z0 = lla_to_ecef(lat0, lon0, alt0)
    x1, y1, z1 = lla_to_ecef(lat1, lon1, alt1)
    dx, dy, dz = (x1 - x0, y1 - y0, z1 - z0)
    dE, dN, dU = enu_from_ecef_delta(dx, dy, dz, lat0, lon0)
    return dN, dE, -dU


class RedirectionCommanderGPS(Node):
    def __init__(self):
        super().__init__('redirection_commander_gps')

        # QoS
        q_ctrl = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                            history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        q_sensor = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)

        # Parameters
        self.declare_parameter('tracked_pose_body_topic', '/tracked_uav_pose')
        self.declare_parameter('attacker_gps_topic_vehicle', '/fmu/out/vehicle_gps_position')
        self.declare_parameter('attacker_gps_topic_sensor', '/fmu/out/sensor_gps')
        self.declare_parameter('attacker_attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter('attacker_local_topic', '/fmu/out/vehicle_local_position')
        self.declare_parameter('target_lla_topic', '/redirection_commander_gps/target_lla')  # accepted but local-first

        # (kept for compatibility with older configs; unused when FRD confirmed)
        self.declare_parameter('tracked_pose_is_ros_base_link_flu', True)

        # Target bias (goal) expressed in N,E,D (m) relative to takeoff local position
        self.declare_parameter('target_bias_ned', [-3.0, -1.0, 0.0]) #(-3,2) for hover 01 and 02 and move 01, (-2, 1) for move 02 to 08, (-3, -1) for 09

        # NEW: ramp options (linear interpolation of target from start -> goal)
        self.declare_parameter('use_bias_ramp', False)
        self.declare_parameter('bias_ramp_duration_s', 10.0)
        self.declare_parameter('target_bias_start_ned', [0.0, 0.0, 0.0])

        # defaults (fallbacks)
        self.declare_parameter('default_target_lat_deg', 36.0054164)
        self.declare_parameter('default_target_lon_deg', -78.9392033)
        self.declare_parameter('default_target_alt_m', float('nan'))

        self.declare_parameter('log_status_rate_hz', 2.0)

        # PID params (mild)
        self.declare_parameter('pid_kp', [0.5, 0.5, 0.3])
        self.declare_parameter('pid_ki', [0.005, 0.005, 0.002])
        self.declare_parameter('pid_kd', [0.1, 0.1, 0.05])
        self.declare_parameter('pid_u_lim', [1.0, 1.0, 0.5])
        self.declare_parameter('pid_i_lim', [5.0, 5.0, 2.0])
        self.declare_parameter('pid_deriv_tau', 0.1)
        self.declare_parameter('pid_reset_on_target_change', True)

        # State
        self.latest_tracked_body: Optional[PoseStamped] = None
        self.latest_attacker_nav: Optional[NavSatFix] = None
        self.latest_attacker_pose_R_enu_wb = None
        self.latest_att_R_ned_wb = None

        # Attacker local NED (VehicleLocalPosition)
        self.latest_attacker_local: Optional[VehicleLocalPosition] = None
        self.first_attacker_local: Optional[Tuple[float,float,float]] = None  # numeric takeoff (N,E,D)

        # Ramp bookkeeping
        self._bias_ramp_t0_ns: Optional[int] = None
        self._bias_start_ned: Tuple[float,float,float] = (0.0, 0.0, 0.0)
        self._bias_goal_ned:  Tuple[float,float,float] = (-1.0, 1.0, 0.0)
        self._bias_curr_ned:  Tuple[float,float,float] = (0.0, 0.0, 0.0)

        # Last computed values for status/logging
        self._last_victim_lla: Optional[Tuple[float,float,float]] = None
        self._last_victim_local: Optional[Tuple[float,float,float]] = None
        self._last_target_local: Optional[Tuple[float,float,float]] = None
        self._last_bias_latlon: Optional[Tuple[float,float]] = None
        self._last_bias_ned: Optional[Tuple[float,float,float]] = None
        self._last_bias_ned_error: Optional[Tuple[float,float,float]] = None

        # PID state
        self._pid_integrator = [0.0, 0.0, 0.0]
        self._pid_prev_err   = [0.0, 0.0, 0.0]
        self._pid_prev_time_ns = None
        self._pid_deriv_state = [0.0, 0.0, 0.0]
        self._last_target_for_reset = None

        # Flags
        self._got_tracked = False
        self._got_att = False
        self._got_gps = False
        self._got_local = False
        self._warned_tracked_frame = False

        # Subscribers
        self.sub_tracked = self.create_subscription(PoseStamped,
                                                    self.get_parameter('tracked_pose_body_topic').value,
                                                    self._cb_tracked, q_ctrl)
        self.sub_fmu_gps_vehicle = self.create_subscription(SensorGps,
                                                            self.get_parameter('attacker_gps_topic_vehicle').value,
                                                            self._cb_fmu_gps_generic, q_sensor)
        self.sub_fmu_gps_sensor = self.create_subscription(SensorGps,
                                                           self.get_parameter('attacker_gps_topic_sensor').value,
                                                           self._cb_fmu_gps_generic, q_sensor)
        self.sub_fmu_att = self.create_subscription(VehicleAttitude,
                                                    self.get_parameter('attacker_attitude_topic').value,
                                                    self._cb_fmu_att, q_sensor)
        self.sub_fmu_local = self.create_subscription(VehicleLocalPosition,
                                                      self.get_parameter('attacker_local_topic').value,
                                                      self._cb_fmu_local, q_sensor)

        # legacy LLA target (fallback only)
        self.sub_target = self.create_subscription(Vector3Stamped,
                                                  self.get_parameter('target_lla_topic').value,
                                                  self._cb_target, q_ctrl)

        # Publishers
        self.pub_victim_sensorgps = self.create_publisher(SensorGps, '/victim/estimated_sensor_gps', 10)
        self.pub_victim_local = self.create_publisher(VehicleLocalPosition, '/victim/estimated_local_position', 10)
        self.pub_bias_latlon = self.create_publisher(Vector3Stamped, '/spoofing_gps_bias/latlon', 10)
        self.pub_bias_ned = self.create_publisher(Vector3Stamped, '/spoofing_gps_bias/ned', 10)
        self.pub_bias_ned_error = self.create_publisher(Vector3Stamped, '/spoofing_gps_bias/ned_error', 10)

        self.get_logger().info('redirection_commander_gps started (FMU-only, local-target mode)')

        # Fixed NED->ENU transform
        self._R_enu_from_ned = [
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0,-1.0],
        ]

        # Timers
        self.timer = self.create_timer(0.05, self._step)
        rate = float(self.get_parameter('log_status_rate_hz').value)
        rate = 2.0 if rate <= 0.0 else rate
        self.status_timer = self.create_timer(1.0 / rate, self._status_log)


    # ----------------- PID helper -----------------
    def _pid_step(self, err_vec, now_ns):
        kp = [float(x) for x in self.get_parameter('pid_kp').value]
        ki = [float(x) for x in self.get_parameter('pid_ki').value]
        kd = [float(x) for x in self.get_parameter('pid_kd').value]
        u_lim = [float(x) for x in self.get_parameter('pid_u_lim').value]
        i_lim = [float(x) for x in self.get_parameter('pid_i_lim').value]
        tau  = float(self.get_parameter('pid_deriv_tau').value)

        if self._pid_prev_time_ns is None:
            dt = 0.0
        else:
            dt = max((now_ns - self._pid_prev_time_ns) * 1e-9, 1e-6)

        u = [0.0, 0.0, 0.0]
        for i in range(3):
            e = float(err_vec[i])
            self._pid_integrator[i] += ki[i] * e * dt
            if i_lim[i] > 0.0:
                self._pid_integrator[i] = max(-i_lim[i], min(i_lim[i], self._pid_integrator[i]))
            if dt > 0.0:
                de = (e - self._pid_prev_err[i]) / dt
                alpha = tau / (tau + dt) if tau > 0.0 else 0.0
                self._pid_deriv_state[i] = alpha * self._pid_deriv_state[i] + (1 - alpha) * de
                d_term = kd[i] * self._pid_deriv_state[i]
            else:
                d_term = 0.0

            u[i] = kp[i] * e + self._pid_integrator[i] + d_term
            if u_lim[i] > 0.0:
                if u[i] >  u_lim[i]: u[i] =  u_lim[i]
                if u[i] < -u_lim[i]: u[i] = -u_lim[i]

            self._pid_prev_err[i] = e

        self._pid_prev_time_ns = now_ns
        return u


    # ----------------- Callbacks -----------------
    def _cb_tracked(self, msg: PoseStamped):
        self.latest_tracked_body = msg
        if not self._got_tracked:
            self._got_tracked = True
            self.get_logger().info(f"✅ First tracked pose on {self.get_parameter('tracked_pose_body_topic').value}")
            fid = getattr(msg.header, 'frame_id', '')
            if fid and fid != 'cpsl_uav_1/base_link' and not self._warned_tracked_frame:
                self._warned_tracked_frame = True
                self.get_logger().warn(f"tracked_uav_pose frame_id is '{fid}', expected 'cpsl_uav_1/base_link' (FRD).")

    def _cb_target(self, msg: Vector3Stamped):
        # legacy/fallback; local ramp target takes precedence
        self.latest_target_lla = msg
        self.get_logger().info("Received explicit target_lla (will be used as fallback LLA target).")

    def _cb_fmu_gps_generic(self, msg):
        def get_any(obj, names):
            for n in names:
                if hasattr(obj, n):
                    return getattr(obj, n)
            return None

        lat_raw = get_any(msg, ['latitude', 'lat', 'lat_deg', 'latitude_deg'])
        lon_raw = get_any(msg, ['longitude', 'lon', 'lon_deg', 'longitude_deg'])
        alt_raw = get_any(msg, ['altitude', 'alt', 'height', 'height_m', 'alt_m',
                                'altitude_msl_m', 'altitude_ellipsoid_m'])
        def to_deg(v):
            if v is None: return None
            try: f = float(v)
            except Exception: return None
            return f * 1e-7 if abs(f) > 1000.0 else f
        def to_m(v):
            if v is None: return None
            try: f = float(v)
            except Exception: return None
            return f * 1e-3 if abs(f) > 1.0e4 else f

        lat = to_deg(lat_raw); lon = to_deg(lon_raw); alt = to_m(alt_raw)
        if lat is None or lon is None:
            return

        nav = NavSatFix()
        nav.latitude = float(lat)
        nav.longitude = float(lon)
        nav.altitude = 0.0 if alt is None else float(alt)
        self.latest_attacker_nav = nav

        if not self._got_gps:
            self._got_gps = True
            self.get_logger().info("✅ First GPS received; attacker LLA available.")

    def _cb_fmu_att(self, msg: VehicleAttitude):
        try:
            if hasattr(msg, 'q') and len(msg.q) >= 4:
                qw, qx, qy, qz = float(msg.q[0]), float(msg.q[1]), float(msg.q[2]), float(msg.q[3])
            else:
                qw = float(getattr(msg, 'q_w', getattr(msg, 'w', 1.0)))
                qx = float(getattr(msg, 'q_x', getattr(msg, 'x', 0.0)))
                qy = float(getattr(msg, 'q_y', getattr(msg, 'y', 0.0)))
                qz = float(getattr(msg, 'q_z', getattr(msg, 'z', 0.0)))
        except Exception:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0

        # PX4: quaternion is body->NED
        R_ned_wb = quat_to_rotmat(qx, qy, qz, qw)
        self.latest_att_R_ned_wb = R_ned_wb
        self.latest_attacker_pose_R_enu_wb = matmul(self._R_enu_from_ned, R_ned_wb)

        if not self._got_att:
            self._got_att = True
            self.get_logger().info("✅ First attitude received (stored body->NED and body->ENU).")

    def _cb_fmu_local(self, msg: VehicleLocalPosition):
        # read x,y,z (assume local N,E,D)
        def get_any(o, names):
            for n in names:
                if hasattr(o, n):
                    return getattr(o, n)
            return None
        x = get_any(msg, ['x','x_local','local_x'])
        y = get_any(msg, ['y','y_local','local_y'])
        z = get_any(msg, ['z','z_local','local_z'])
        try:
            x = float(x); y = float(y); z = float(z)
        except Exception:
            return

        self.latest_attacker_local = msg
        self._attacker_local_tuple = (x, y, z)

        if not self._got_local:
            self._got_local = True
            # takeoff numeric NED
            self.first_attacker_local = (x, y, z)

            # initialize ramp bookkeeping
            bs = self.get_parameter('target_bias_start_ned').value or [0.0, 0.0, 0.0]
            bg = self.get_parameter('target_bias_ned').value or [0.0, 0.0, 0.0]
            try:
                self._bias_start_ned = (float(bs[0]), float(bs[1]), float(bs[2]))
            except Exception:
                self._bias_start_ned = (0.0, 0.0, 0.0)
            try:
                self._bias_goal_ned = (float(bg[0]), float(bg[1]), float(bg[2]))
            except Exception:
                self._bias_goal_ned = (0.0, 0.0, 0.0)
            self._bias_curr_ned = self._bias_start_ned
            self._bias_ramp_t0_ns = self.get_clock().now().nanoseconds

            self.get_logger().info(
                f"✅ Takeoff saved. Ramp from start bias {self._bias_start_ned} -> goal bias {self._bias_goal_ned} "
                f"in {self.get_parameter('bias_ramp_duration_s').value:.1f}s "
                f"(use_bias_ramp={self.get_parameter('use_bias_ramp').value})."
            )


    # ----------------- Main loop -----------------
    def _step(self):
        if (self.latest_tracked_body is None or
            self.latest_attacker_nav is None or
            self.latest_attacker_pose_R_enu_wb is None or
            self.latest_attacker_local is None or
            self.latest_att_R_ned_wb is None or
            self.first_attacker_local is None):
            return

        # Attacker LLA + local NED
        lat0 = float(self.latest_attacker_nav.latitude)
        lon0 = float(self.latest_attacker_nav.longitude)
        alt0 = float(self.latest_attacker_nav.altitude)

        att_local_x, att_local_y, att_local_z = self._attacker_local_tuple  # N,E,D

        # --- body(FRD) -> NED(world) ---
        bx = float(self.latest_tracked_body.pose.position.x)
        by = float(self.latest_tracked_body.pose.position.y)
        bz = float(self.latest_tracked_body.pose.position.z)
        v_body_frd = [bx, by, bz]  # tracked pose is already FRD

        dN, dE, dD = matvec(self.latest_att_R_ned_wb, v_body_frd)

        # Victim estimated LOCAL NED (tracker frame)
        victim_local_N = att_local_x + dN
        victim_local_E = att_local_y + dE
        victim_local_D = att_local_z + dD
        self._last_victim_local = (victim_local_N, victim_local_E, victim_local_D)

        # For LLA work (SensorGps + Δlat/Δlon), NED -> ENU -> LLA
        dE_enu = dE; dN_enu = dN; dU_enu = -dD
        dx_ecef, dy_ecef, dz_ecef = enu_to_ecef_delta(dE_enu, dN_enu, dU_enu, lat0, lon0)
        x0, y0, z0 = lla_to_ecef(lat0, lon0, alt0)
        xv, yv, zv = x0 + dx_ecef, y0 + dy_ecef, z0 + dz_ecef
        lat_v, lon_v, alt_v = ecef_to_lla(xv, yv, zv)
        self._last_victim_lla = (lat_v, lon_v, alt_v)

        # (1) Publish victim estimate as SensorGps
        now_ns = self.get_clock().now().nanoseconds
        victim_gps = SensorGps()
        def set_one(obj, names: Sequence[str], value):
            for n in names:
                if hasattr(obj, n):
                    try:
                        setattr(obj, n, value); return True
                    except Exception:
                        pass
            return False
        set_one(victim_gps, ['latitude_deg'],  float(lat_v))
        set_one(victim_gps, ['longitude_deg'], float(lon_v))
        if not set_one(victim_gps, ['altitude_msl_m'], float(alt_v)):
            set_one(victim_gps, ['altitude_ellipsoid_m'], float(alt_v))
        set_one(victim_gps, ['latitude', 'lat'], int(round(lat_v * 1e7)))
        set_one(victim_gps, ['longitude', 'lon'], int(round(lon_v * 1e7)))
        set_one(victim_gps, ['altitude', 'alt', 'height', 'height_m', 'alt_m'], int(round(alt_v * 1000.0)))
        if hasattr(victim_gps, 'timestamp'):
            try: victim_gps.timestamp = now_ns
            except Exception:
                try: victim_gps.timestamp = int(now_ns // 1000)
                except Exception: pass
        if hasattr(victim_gps, 'time_utc_usec'):
            try: victim_gps.time_utc_usec = int(now_ns // 1000)
            except Exception: pass
        if hasattr(victim_gps, 'fix_type') and getattr(victim_gps, 'fix_type', 0) == 0:
            try: victim_gps.fix_type = 3
            except Exception: pass
        self.pub_victim_sensorgps.publish(victim_gps)

        # (2) Publish victim estimated local position
        victim_local_msg = VehicleLocalPosition()
        try:
            setattr(victim_local_msg, 'x', float(victim_local_N))
            setattr(victim_local_msg, 'y', float(victim_local_E))
            setattr(victim_local_msg, 'z', float(victim_local_D))
        except Exception:
            pass
        if hasattr(victim_local_msg, 'timestamp'):
            try: victim_local_msg.timestamp = now_ns
            except Exception: pass
        self.pub_victim_local.publish(victim_local_msg)

        # ----------------- Target selection in tracker-local NED (with ramp) -----------------
        takeoff_N, takeoff_E, takeoff_D = self.first_attacker_local
        use_ramp = bool(self.get_parameter('use_bias_ramp').value)

        # compute current bias (linear ramp from start->goal)
        if use_ramp and self._bias_ramp_t0_ns is not None:
            dur_s = max(0.0, float(self.get_parameter('bias_ramp_duration_s').value))
            if dur_s <= 1e-6:
                alpha = 1.0
            else:
                t_now = self.get_clock().now().nanoseconds
                alpha = (t_now - self._bias_ramp_t0_ns) * 1e-9 / dur_s
                if alpha < 0.0: alpha = 0.0
                if alpha > 1.0: alpha = 1.0
            bsN, bsE, bsD = self._bias_start_ned
            bgN, bgE, bgD = self._bias_goal_ned
            bN = bsN + alpha * (bgN - bsN)
            bE = bsE + alpha * (bgE - bsE)
            bD = bsD + alpha * (bgD - bsD)
            self._bias_curr_ned = (bN, bE, bD)
        else:
            # immediate jump to goal bias
            self._bias_curr_ned = self._bias_goal_ned

        # current target in local NED
        bN, bE, bD = self._bias_curr_ned
        target_N = takeoff_N + bN
        target_E = takeoff_E + bE
        target_D = takeoff_D + bD
        self._last_target_local = (target_N, target_E, target_D)

        # (3) Raw local NED error (victim -> target)
        eN = target_N - victim_local_N
        eE = target_E - victim_local_E
        eD = target_D - victim_local_D
        self._last_bias_ned_error = (eN, eE, eD)

        # publish raw error
        h = Header(); h.stamp = self.get_clock().now().to_msg()
        err_msg = Vector3Stamped(); err_msg.header = h
        err_msg.vector.x = eN; err_msg.vector.y = eE; err_msg.vector.z = eD
        self.pub_bias_ned_error.publish(err_msg)

        # Reset PID if target changed meaningfully (position key)
        reset_on_change = bool(self.get_parameter('pid_reset_on_target_change').value)
        curr_target_key = (round(target_N,6), round(target_E,6), round(target_D,6))
        if reset_on_change and self._last_target_for_reset is not None and curr_target_key != self._last_target_for_reset:
            self._pid_integrator = [0.0, 0.0, 0.0]
            self._pid_prev_err   = [0.0, 0.0, 0.0]
            self._pid_deriv_state = [0.0, 0.0, 0.0]
        self._last_target_for_reset = curr_target_key

        # (4) PID on local NED error -> command in N,E,D
        now_ns2 = self.get_clock().now().nanoseconds
        uN, uE, uD = self._pid_step([eN, eE, eD], now_ns2)
        self._last_bias_ned = (uN, uE, uD)

        # publish PID NED command
        cmd_ned = Vector3Stamped(); cmd_ned.header = h
        cmd_ned.vector.x = uN; cmd_ned.vector.y = uE; cmd_ned.vector.z = uD
        self.pub_bias_ned.publish(cmd_ned)

        # (5) Convert PID NED command -> Δlat/Δlon for logging (apply at victim LLA)
        ddx, ddy, ddz = enu_to_ecef_delta(dE=uE, dN=uN, dU=-uD, lat0_deg=lat_v, lon0_deg=lon_v)
        xv2, yv2, zv2 = lla_to_ecef(lat_v, lon_v, alt_v)
        xcmd, ycmd, zcmd = (xv2 + ddx, yv2 + ddy, zv2 + ddz)
        lat_cmd, lon_cmd, _ = ecef_to_lla(xcmd, ycmd, zcmd)
        dlat_cmd = lat_cmd - lat_v
        dlon_cmd = lon_cmd - lon_v
        bias_latlon = Vector3Stamped(); bias_latlon.header = h
        bias_latlon.vector.x = dlat_cmd; bias_latlon.vector.y = dlon_cmd; bias_latlon.vector.z = 0.0
        self.pub_bias_latlon.publish(bias_latlon)
        self._last_bias_latlon = (dlat_cmd, dlon_cmd)


    # ----------------- Status logger (NED-focused) -----------------
    def _status_log(self):
        # 1) Attacker pos in NED
        if hasattr(self, "_attacker_local_tuple"):
            aN, aE, aD = self._attacker_local_tuple
            line1 = f"ATTACKER NED: N={aN:+.2f} m, E={aE:+.2f} m, D={aD:+.2f} m"
        else:
            line1 = "ATTACKER NED: (waiting...)"

        # 2) Victim pos (estimated) in NED
        if self._last_victim_local is not None:
            vN, vE, vD = self._last_victim_local
            line2 = f"VICTIM NED:   N={vN:+.2f} m, E={vE:+.2f} m, D={vD:+.2f} m"
        else:
            line2 = "VICTIM NED:   (waiting...)"

        # 3) Current target in NED (ramped)
        if self._last_target_local is not None:
            tN, tE, tD = self._last_target_local
            # also expose current bias wrt takeoff for clarity
            if self.first_attacker_local is not None:
                t0N, t0E, t0D = self.first_attacker_local
                bN, bE, bD = (tN - t0N, tE - t0E, tD - t0D)
                bias_str = f" (bias NED: N={bN:+.2f}, E={bE:+.2f}, D={bD:+.2f})"
            else:
                bias_str = ""
            line3 = f"TARGET NED:   N={tN:+.2f} m, E={tE:+.2f} m, D={tD:+.2f} m{bias_str}"
        else:
            line3 = "TARGET NED:   (waiting...)"

        # 4) Spoof cmd in NED
        if self._last_bias_ned is not None:
            uN, uE, uD = self._last_bias_ned
            line4 = f"SPOOF CMD NED: N={uN:+.2f} m, E={uE:+.2f} m, D={uD:+.2f} m"
        else:
            line4 = "SPOOF CMD NED: (waiting...)"

        self.get_logger().info("\n".join([line1, line2, line3, line4]))


def main(args=None):
    rclpy.init(args=args)
    node = RedirectionCommanderGPS()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
