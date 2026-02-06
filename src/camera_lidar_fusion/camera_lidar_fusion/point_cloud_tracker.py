#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d
from filterpy.kalman import KalmanFilter
from sklearn.cluster import DBSCAN


# ============================
# Kalman-based 3D point cloud tracker
# ============================
class PointCloudTracker3D:
    def __init__(self, dt=0.1):
        # --- CHANGED ---
        # The filter will be explicitly initialized with shaped arrays below.
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        self._init_filter(dt)
        self.initialized = False
        # Use numeric nanoseconds timestamp internally (int)
        self.last_update_time_ns = None  # --- CHANGED ---
        self.estimated_cloud = None

    def _init_filter(self, dt):
        # State: [x y z vx vy vz]^T
        # --- CHANGED ---
        # Explicit initialization (avoid relying on filterpy defaults or *= scalings)
        self.kf = KalmanFilter(dim_x=6, dim_z=3)
        # Transition (will be updated per-dt using _update_transition)
        self.kf.F = np.eye(6)
        self._update_transition(dt)

        # Measurement matrix: observe x,y,z
        self.kf.H = np.zeros((3, 6))
        self.kf.H[0, 0] = 1
        self.kf.H[1, 1] = 1
        self.kf.H[2, 2] = 1

        # state vector (6x1)
        self.kf.x = np.zeros((6, 1))

        # Explicit covariance matrices (tunable)
        self.kf.P = np.eye(6) * 5.0       # initial state covariance (tune as needed)
        self.kf.R = np.eye(3) * 0.05      # measurement noise covariance (tune)
        self.kf.Q = np.eye(6) * 0.01      # process noise covariance (tune)

    def initialize(self, cloud, time_stamp_ns):
        centroid = np.mean(cloud, axis=0).reshape(3, 1)
        # --- CHANGED ---
        # set full state vector explicitly as (6,1)
        self.kf.x[:3, 0] = centroid.flatten()
        self.kf.x[3:, 0] = 0.0
        self.last_update_time_ns = int(time_stamp_ns)  # store numeric ns
        self.initialized = True
        self.estimated_cloud = cloud

    def _update_transition(self, dt):
        # dt: seconds (float)
        self.kf.F = np.eye(6)
        self.kf.F[0, 3] = dt
        self.kf.F[1, 4] = dt
        self.kf.F[2, 5] = dt

    def predict(self, now_ns):
        """Predict using numeric nanoseconds time (int). Returns (predicted_centroid (3,), predicted_cloud (Nx3) or None)."""
        # --- CHANGED ---
        # now_ns expected to be integer nanoseconds
        if not self.initialized or self.last_update_time_ns is None:
            return None, None
        dt = (int(now_ns) - int(self.last_update_time_ns)) * 1e-9
        dt = max(dt, 0.0)
        # update transition with dt and predict
        self._update_transition(dt)
        self.kf.predict()
        predicted_centroid = self.kf.x[:3].flatten()
        # last_centroid is approximate previous centroid (used to offset cloud)
        last_centroid = (self.kf.x[:3].flatten() - self.kf.x[3:].flatten() * dt)
        offset = predicted_centroid - last_centroid
        predicted_cloud = None
        if self.estimated_cloud is not None:
            predicted_cloud = self.estimated_cloud + offset
        return predicted_centroid, predicted_cloud

    def try_kf_update(self, centroid):
        """
        Apply Mahalanobis gating before updating KF.
        centroid: shape (3,) or (3,1)
        Returns True if update applied, False if rejected.
        """
        # ensure column vector
        z = np.atleast_2d(centroid).reshape(3, 1)
        y = z - (self.kf.H @ self.kf.x)  # innovation
        S = self.kf.H @ self.kf.P @ self.kf.H.T + self.kf.R
        # compute Mahalanobis squared
        try:
            invS = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            # numerically bad; accept update but warn
            invS = np.linalg.pinv(S)
            # We will still update below
        maha2 = float(y.T @ invS @ y)
        # chi-square threshold for 3 DOF (e.g., 95% ~7.815, 99%~11.345). using 99% conservative
        threshold = 11.345
        if maha2 > threshold:
            # reject measurement
            return False
        # apply update
        self.kf.update(z)
        return True

    def update(self, cloud, time_stamp_ns):
        centroid = np.mean(cloud, axis=0).reshape(3, 1)
        if not self.initialized:
            # Use numeric ns
            self.initialize(cloud, time_stamp_ns)
        else:
            dt = (int(time_stamp_ns) - int(self.last_update_time_ns)) * 1e-9
            dt = max(dt, 0.0)
            self._update_transition(dt)
            # Always predict first
            self.kf.predict()
            # Try update with gating — if rejected, skip update and keep predicted state
            updated = self.try_kf_update(centroid)
            if not updated:
                # measurement rejected - keep predicted state, but still update timestamp and estimated cloud to avoid repeated large dt
                # (or optionally skip updating estimated_cloud to preserve previous geometry)
                self.last_update_time_ns = int(time_stamp_ns)
                # do not set estimated_cloud to the new noisy cloud; keep previous to avoid contamination
                return
            # update successful
            self.estimated_cloud = cloud
            self.last_update_time_ns = int(time_stamp_ns)

    def get_state(self):
        return self.kf.x.flatten(), self.estimated_cloud


# ============================
# ROS2 Node
# ============================
class PointCloudTrackerNode(Node):
    def __init__(self):
        super().__init__('point_cloud_tracker')

        self.tracker = PointCloudTracker3D(dt=0.1)

        # ---- Runtime param to toggle KF ----
        self.declare_parameter('use_kf', True)
        self.use_kf = bool(self.get_parameter('use_kf').value)
        self.add_on_set_parameters_callback(self._on_param_update)

        # Publishers (relative in base_link)
        self.pose_pub = self.create_publisher(PoseStamped, '/tracked_uav_pose', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/tracked_uav_velocity', 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/tracked_uav_cluster', 10)

        # NEW: absolute (Vicon world) outputs
        self.pose_pub_vicon = self.create_publisher(PoseStamped, '/tracked_uav_pose_vicon', 10)
        self.velocity_pub_vicon = self.create_publisher(TwistStamped, '/tracked_uav_velocity_vicon', 10)

        # TF (LiDAR -> base_link)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Frames
        self.target_frame = 'cpsl_uav_1/base_link'   # FRD (relative outputs here)
        self.lidar_frame = 'cpsl_uav_1/livox_frame'

        # Subscriptions
        self.subscription = self.create_subscription(
            PointCloud2, '/camera_lidar_fusion/uav_cluster', self.cluster_callback, 10
        )
        self.pc_sub = self.create_subscription(
            PointCloud2, '/cpsl_uav_1/livox/lidar', self.lidar_callback, 10
        )
        # Vicon hunter pose (TransformStamped: vicon/world -> vicon/x500_3/x500_3)
        self.vicon_sub = self.create_subscription(
            TransformStamped, '/vicon/x500_3/x500_3', self.vicon_callback, 10
        )

        self.latest_lidar_msg = None
        self.last_valid_cluster = None
        # --- CHANGED ---
        # Track the frame and header of the camera cluster we store.
        self.last_valid_cluster_frame = None
        self.last_valid_cluster_header = None

        # Anchor ROI on last confirmed LiDAR match centroid
        self.last_lidar_match_centroid = None

        # When KF is disabled, velocity via finite differences (measurement only)
        self.prev_centroid = None
        self.prev_stamp_ns = None

        # --- Storage for Vicon pose/vel (hunter) ---
        self.world_frame_id = 'vicon/world'  # overwritten by incoming Vicon header if different
        self.t_world_base = None            # hunter position in world (ENU), shape (3,)
        self.R_base_to_world = None         # rotation base -> world (3x3)
        self.prev_t_world_base = None
        self.prev_vicon_stamp_ns = None
        self.v_hunter_world = np.zeros(3, dtype=float)  # finite-diff hunter velocity in world

        # --- CHANGED ---
        # Explicit FRD -> Vicon (ENU-like) conversion matrix.
        # Default maps FRD (forward,right,down) -> ENU-like (x-east,y-north,z-up) by swapping and flipping axes.
        # IMPORTANT: Validate and adjust this in your lab. See comment below for how to test/adjust.
        self.FRD_TO_VICON = np.array([
            [1.0,  0.0,  0.0],   # x_base (forward) -> x_world (east)  (example)
            [0.0, -1.0,  0.0],   # y_base (right)   -> y_world (north) (flip sign if needed)
            [0.0,  0.0, -1.0],   # z_base (down)    -> z_world (up)     (flip)
        ], dtype=float)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- CHANGED ---
        # Immediate startup print so you know the node started and the log destination
        # self.get_logger().info("point_cloud_tracker node started")
        print("[point_cloud_tracker] node started")  # visible on stdout

    # ---------- Param callback ----------
    def _on_param_update(self, params):
        for p in params:
            if p.name == 'use_kf':
                self.use_kf = bool(p.value)
                # self.get_logger().info(f"Parameter update: use_kf={self.use_kf}")
                # print(f"[point_cloud_tracker] PARAM update: use_kf={self.use_kf}")  # --- CHANGED ---
        from rclpy.parameter import SetParametersResult
        return SetParametersResult(successful=True)

    # ---------- Callbacks ----------
    def lidar_callback(self, msg):
        self.latest_lidar_msg = msg

    def cluster_callback(self, msg: PointCloud2):
        cloud = np.array(
            [[p[0], p[1], p[2]]
             for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)],
            dtype=np.float32
        )
        if cloud.shape[0] == 0:
            # self.get_logger().warn("Empty cluster received from camera node")
            print("[point_cloud_tracker] CASE: empty camera cluster")  # --- CHANGED ---
            return

        # --- CHANGED ---
        # Store cluster and its original frame/header so we can transform it explicitly when needed.
        self.last_valid_cluster = cloud
        self.last_valid_cluster_frame = msg.header.frame_id if msg.header.frame_id else self.lidar_frame
        self.last_valid_cluster_header = msg.header

        # Inform runtime what happened
        # self.get_logger().info(f"camera cluster received (frame={self.last_valid_cluster_frame}, size={cloud.shape[0]})")
        print(f"[point_cloud_tracker] CASE: camera cluster received (frame={self.last_valid_cluster_frame}, size={cloud.shape[0]})")  # --- CHANGED ---

        # Initialize tracker at first sighting; also seed the LiDAR anchor
        if not self.tracker.initialized:
            # --- CHANGED ---
            # Use numeric ns timestamp for tracker initialization (msg header stamp)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            self.tracker.initialize(cloud, stamp_ns)
            self.last_lidar_match_centroid = np.mean(cloud, axis=0)
            # self.get_logger().info("KF initialized from camera cluster")
            print("[point_cloud_tracker] CASE: KF initialized from camera cluster")  # --- CHANGED ---

    def vicon_callback(self, msg: TransformStamped):
        # msg is transform from vicon/world (parent) -> vicon/x500_3/x500_3 (child)
        if msg.header.frame_id:
            self.world_frame_id = msg.header.frame_id

        t = msg.transform.translation
        q = msg.transform.rotation

        # Hunter (base) position in world:
        t_world_base = np.array([t.x, t.y, t.z], dtype=float)

        # Rotation: maps base-frame vectors to world (parent->child orientation)
        # For TransformStamped (parent->child), quaternion encodes R^W_B: base -> world.
        R_wb = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        # Finite-diff hunter linear velocity in world frame
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if self.prev_t_world_base is not None and self.prev_vicon_stamp_ns is not None:
            dt = max((stamp_ns - self.prev_vicon_stamp_ns) * 1e-9, 1e-6)
            self.v_hunter_world = (t_world_base - self.prev_t_world_base) / dt

        self.prev_t_world_base = t_world_base
        self.prev_vicon_stamp_ns = stamp_ns

        self.t_world_base = t_world_base
        self.R_base_to_world = R_wb

    # ---------- Utilities ----------
    def extract_roi_points(self, cloud_np, center, radius):
        if cloud_np.size == 0:
            return cloud_np
        distances = np.linalg.norm(cloud_np - center, axis=1)
        return cloud_np[distances < radius]

    def cluster_with_dbscan(self, points, tolerance=0.2, min_size=20, max_size=1000):
        if points.shape[0] == 0:
            # self.get_logger().warn("No points provided for clustering.")
            print("[point_cloud_tracker] CASE: no points for DBSCAN")  # --- CHANGED ---
            return []  # --- CHANGED: return empty list not None
        # --- CHANGED ---
        # Ensure min_samples is reasonable relative to min_size.
        min_samples = max(2, int(max(2, min_size * 0.1)))
        db = DBSCAN(eps=tolerance, min_samples=min_samples).fit(points)
        labels = db.labels_
        unique = set(labels) - {-1}
        clusters = []
        for u in unique:
            indices = np.where(labels == u)[0]
            if min_size <= len(indices) <= max_size:
                clusters.append(points[indices])
        # If no clusters passed the strict min_size filter, allow smaller clusters near ROI (fallback)
        if len(clusters) == 0:
            # gather small clusters that are not noise and not too small
            small_clusters = []
            for u in (set(labels) - {-1}):
                indices = np.where(labels == u)[0]
                if 3 <= len(indices) < min_size:  # accept down to 3 points in fallback
                    small_clusters.append(points[indices])
            if small_clusters:
                # self.get_logger().debug(f"DBSCAN: no full-size clusters; using {len(small_clusters)} small clusters as fallback")
                print(f"[point_cloud_tracker] CASE: DBSCAN fallback with {len(small_clusters)} small clusters")  # --- CHANGED ---
                return small_clusters
        if len(clusters) == 0:
            # truly no clusters found
            # self.get_logger().info("DBSCAN found no clusters")
            print("[point_cloud_tracker] CASE: DBSCAN found no clusters")  # --- CHANGED ---
        return clusters

    def icp_match(self, reference_cloud, candidate_clouds):
        ref_o3d = o3d.geometry.PointCloud()
        ref_o3d.points = o3d.utility.Vector3dVector(reference_cloud)
        best_fitness = 0.0
        # --- CHANGED ---
        # Lowered fitness threshold to be more tolerant; added rmse check
        fitness_threshold = 0.45
        inlier_rmse_thresh = 0.25
        best_cluster = None

        if not candidate_clouds:
            # self.get_logger().debug("icp_match: no candidate_clouds provided")
            print("[point_cloud_tracker] CASE: icp_match no candidates")  # --- CHANGED ---
            return None

        for cluster in candidate_clouds:
            if cluster is None:
                continue
            c = np.asarray(cluster)
            if c.ndim != 2 or c.shape[1] != 3 or c.shape[0] == 0:
                continue
            tgt = o3d.geometry.PointCloud()
            tgt.points = o3d.utility.Vector3dVector(c)
            result = o3d.pipelines.registration.registration_icp(
                tgt, ref_o3d,
                max_correspondence_distance=0.5,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            # log ICP metrics for debugging
            self.get_logger().debug(f"ICP candidate size={c.shape[0]}, fitness={result.fitness:.3f}, rmse={result.inlier_rmse:.3f}")
            if result.fitness > best_fitness and result.fitness > fitness_threshold and result.inlier_rmse < inlier_rmse_thresh:
                best_fitness = result.fitness
                best_cluster = c

        # --- CHANGED ---
        # If ICP did not find any cluster, fallback to nearest centroid match (if reasonable)
        if best_cluster is None:
            try:
                ref_centroid = np.mean(reference_cloud, axis=0)
                min_dist = float('inf')
                nearest = None
                for c in candidate_clouds:
                    if c is None or c.size == 0:
                        continue
                    d = np.linalg.norm(np.mean(c, axis=0) - ref_centroid)
                    if d < min_dist:
                        min_dist = d
                        nearest = c
                # Choose nearest if within reasonable threshold (0.6 m)
                if nearest is not None and min_dist < 0.6:
                    # self.get_logger().debug(f"ICP fallback: selected nearest cluster (dist={min_dist:.3f} m)")
                    print(f"[point_cloud_tracker] CASE: ICP fallback selected nearest (dist={min_dist:.3f} m)")  # --- CHANGED ---
                    best_cluster = nearest
            except Exception as e:
                self.get_logger().debug(f"ICP fallback exception: {e}")
        else:
            # ICP found a fit
            # self.get_logger().info(f"ICP matched cluster with fitness={best_fitness:.3f}")
            print(f"[point_cloud_tracker] CASE: ICP matched cluster (fitness={best_fitness:.3f})")  # --- CHANGED ---

        return best_cluster

    # --- CHANGED ---
    # Utility: transform Nx3 points from from_frame to to_frame using TF at message stamp
    def transform_points(self, points, from_frame, to_frame, stamp_header):
        """
        points: (N,3) numpy
        from_frame, to_frame: strings
        stamp_header: ROS2 Header (with stamp.sec/nanosec)
        Returns transformed points (N,3). On TF failure returns original points and logs a warning.
        """
        if points is None or points.size == 0:
            return points
        try:
            # Use the message stamp for lookup to get correct transform at the time of measurement
            tf_time = rclpy.time.Time.from_msg(stamp_header.stamp)
            # prefer can_transform first (safer)
            can = False
            try:
                can = self.tf_buffer.can_transform(to_frame, from_frame, tf_time, timeout=rclpy.duration.Duration(seconds=0.05))
            except Exception:
                # fallback: still attempt lookup; can_transform may not be available in some runtimes
                can = True
            if not can:
                # self.get_logger().warn(f"TF cannot transform ({from_frame} -> {to_frame}) at stamp {stamp_header.stamp}; skipping transform")
                # print(f"[point_cloud_tracker] CASE: TF cannot_transform ({from_frame} -> {to_frame})")  # --- CHANGED ---
                # return original points; caller should handle fallback behavior
                return points
            tf = self.tf_buffer.lookup_transform(
                to_frame,  # target
                from_frame,  # source
                tf_time
            )
            R_mat = R.from_quat([
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z,
                tf.transform.rotation.w
            ]).as_matrix()
            t_vec = np.array([
                tf.transform.translation.x,
                tf.transform.translation.y,
                tf.transform.translation.z
            ], dtype=float)
            # apply rotation then translation
            transformed = (R_mat @ points.T).T + t_vec
            return transformed
        except Exception as e:
            # self.get_logger().warn(f"TF transform_points failed ({from_frame} -> {to_frame}): {e}")
            print(f"[point_cloud_tracker] CASE: TF transform_points failed ({from_frame} -> {to_frame}): {e}")  # --- CHANGED ---
            # return original points to avoid crash; caller should consider fallback behavior
            return points

    # ---------- Publishing helpers ----------
    def publish_pose_and_twist(self, now, pos_lidar, vel_lidar):
        """Publish RELATIVE pose/vel in base_link (FRD) and ABSOLUTE in Vicon world (if Vicon present)."""
        # Transform LiDAR -> base_link if needed
        if self.target_frame == self.lidar_frame:
            pos_base = pos_lidar
            vel_base = vel_lidar
        else:
            try:
                # --- CHANGED ---
                # Query TF at the time of the latest LiDAR message (if available) for correct transform
                stamp = self.latest_lidar_msg.header.stamp if self.latest_lidar_msg is not None else None
                tf_time = rclpy.time.Time.from_msg(stamp) if stamp is not None else rclpy.time.Time()
                # Use can_transform first to avoid silent identity fallback
                try:
                    can = self.tf_buffer.can_transform(self.target_frame, self.lidar_frame, tf_time, timeout=rclpy.duration.Duration(seconds=0.05))
                except Exception:
                    can = True
                if not can:
                    raise RuntimeError("TF can_transform returned False")
                tf = self.tf_buffer.lookup_transform(
                    self.target_frame,  # target
                    self.lidar_frame,   # source
                    tf_time
                )
                R_bl_lidar = R.from_quat([
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z,
                    tf.transform.rotation.w
                ]).as_matrix()
                t_bl_lidar = np.array([
                    tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z
                ])
                pos_base = R_bl_lidar @ pos_lidar + t_bl_lidar
                vel_base = R_bl_lidar @ vel_lidar
            except Exception as e:
                # --- CHANGED ---
                # If TF is unavailable, skip publishing to avoid inconsistent frames.
                # self.get_logger().warn(f"TF lookup failed ({self.lidar_frame} -> {self.target_frame}): {e}. Skipping publish to avoid inconsistent frames.")
                print(f"[point_cloud_tracker] CASE: TF lookup failed ({self.lidar_frame} -> {self.target_frame}): {e}")  # --- CHANGED ---
                return

        # ---- Publish relative pose/vel in base_link ----
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now.to_msg()
        pose_msg.header.frame_id = self.target_frame  # 'cpsl_uav_1/base_link' (FRD)
        pose_msg.pose.position.x = float(pos_base[0])
        pose_msg.pose.position.y = float(pos_base[1])
        pose_msg.pose.position.z = float(pos_base[2])
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

        vel_msg = TwistStamped()
        vel_msg.header = pose_msg.header
        vel_msg.twist.linear.x = float(vel_base[0])
        vel_msg.twist.linear.y = float(vel_base[1])
        vel_msg.twist.linear.z = float(vel_base[2])
        self.velocity_pub.publish(vel_msg)

        # ---- Publish absolute pose/vel in Vicon world (ENU) ----
        if (self.t_world_base is not None) and (self.R_base_to_world is not None):
            # --- CHANGED ---
            # We assume pos_base is expressed in base_link (FRD). Convert FRD -> Vicon-frame convention (ENU-like)
            pos_base_for_vicon = self.FRD_TO_VICON @ pos_base
            vel_base_for_vicon = self.FRD_TO_VICON @ vel_base

            # Position: p_world = t_world_base + R_wb * p_base_converted
            p_world = self.t_world_base + self.R_base_to_world @ pos_base_for_vicon
            # Velocity: v_world ≈ v_hunter_world + R_wb * v_base_converted
            v_world = self.v_hunter_world + self.R_base_to_world @ vel_base_for_vicon

            pose_msg_w = PoseStamped()
            pose_msg_w.header.stamp = now.to_msg()
            pose_msg_w.header.frame_id = self.world_frame_id  # e.g., 'vicon/world'
            pose_msg_w.pose.position.x = float(p_world[0])
            pose_msg_w.pose.position.y = float(p_world[1])
            pose_msg_w.pose.position.z = float(p_world[2])
            pose_msg_w.pose.orientation.w = 1.0
            self.pose_pub_vicon.publish(pose_msg_w)

            vel_msg_w = TwistStamped()
            vel_msg_w.header = pose_msg_w.header
            vel_msg_w.twist.linear.x = float(v_world[0])
            vel_msg_w.twist.linear.y = float(v_world[1])
            vel_msg_w.twist.linear.z = float(v_world[2])
            self.velocity_pub_vicon.publish(vel_msg_w)

    def publish_cluster(self, cloud_np, header):
        if cloud_np is None or cloud_np.size == 0:
            return
        # --- CHANGED ---
        # Ensure header stamp is set (use latest lidar stamp if header missing)
        if header is None:
            header = self.latest_lidar_msg.header if self.latest_lidar_msg is not None else None
        cloud_msg = pc2.create_cloud_xyz32(header, cloud_np.tolist())
        self.cloud_pub.publish(cloud_msg)

    # ---------- Main timer ----------
    def timer_callback(self):
        now = self.get_clock().now()

        # KF branch
        if self.use_kf:
            # --- CHANGED ---
            # Use latest LiDAR message header stamp as now_ns for prediction (consistent message-driven timebase)
            if self.latest_lidar_msg is None:
                # self.get_logger().debug("KF branch: waiting for latest_lidar_msg")
                print("[point_cloud_tracker] CASE: KF waiting for latest_lidar_msg")  # --- CHANGED ---
                return
            now_ns = self.latest_lidar_msg.header.stamp.sec * 1_000_000_000 + self.latest_lidar_msg.header.stamp.nanosec
            prediction, predicted_cloud = self.tracker.predict(now_ns)

            if prediction is not None and self.latest_lidar_msg is not None and self.last_valid_cluster is not None:
                # Read full LiDAR cloud
                lidar_cloud = np.array([
                    [p[0], p[1], p[2]]
                    for p in pc2.read_points(
                        self.latest_lidar_msg, field_names=("x", "y", "z"), skip_nans=True
                    )
                ], dtype=np.float32)

                # ROI center: prefer last LiDAR match; fallback to last camera centroid; then prediction
                if self.last_lidar_match_centroid is not None:
                    roi_center = self.last_lidar_match_centroid
                else:
                    # --- CHANGED ---
                    # If the last_valid_cluster was recorded in a different frame than the current LiDAR, transform it.
                    if (self.last_valid_cluster_frame is not None) and (self.last_valid_cluster_frame != self.latest_lidar_msg.header.frame_id):
                        ref_cloud_for_roi = self.transform_points(
                            self.last_valid_cluster,
                            self.last_valid_cluster_frame,
                            self.latest_lidar_msg.header.frame_id,
                            self.last_valid_cluster_header
                        )
                    else:
                        ref_cloud_for_roi = self.last_valid_cluster

                    roi_center = np.mean(ref_cloud_for_roi, axis=0) if ref_cloud_for_roi is not None else None
                    if roi_center is None and prediction is not None:
                        roi_center = prediction

                if roi_center is None:
                    if predicted_cloud is not None:
                        self.publish_cluster(predicted_cloud, self.latest_lidar_msg.header)
                    # self.get_logger().info("Prediction only: no ROI center")
                    print("[point_cloud_tracker] CASE: KF prediction-only (no ROI center)")  # --- CHANGED ---
                else:
                    roi_points = self.extract_roi_points(lidar_cloud, roi_center, radius=1.5)
                    candidate_clusters = self.cluster_with_dbscan(
                        roi_points, tolerance=0.2, min_size=20, max_size=1000
                    )

                    if candidate_clusters:
                        # --- CHANGED ---
                        # Ensure the reference passed to ICP is in the same frame as candidate clusters (LiDAR frame)
                        if (self.last_valid_cluster_frame is not None) and (self.last_valid_cluster_frame != self.latest_lidar_msg.header.frame_id):
                            icp_ref = self.transform_points(
                                self.last_valid_cluster,
                                self.last_valid_cluster_frame,
                                self.latest_lidar_msg.header.frame_id,
                                self.last_valid_cluster_header
                            )
                        else:
                            icp_ref = self.last_valid_cluster

                        best = self.icp_match(icp_ref, candidate_clusters)
                        if best is not None and best.shape[0] > 0:
                            # Update KF using numeric ns timestamp of latest LiDAR message
                            stamp_ns = self.latest_lidar_msg.header.stamp.sec * 1_000_000_000 + self.latest_lidar_msg.header.stamp.nanosec
                            # Add a log for Mahalanobis gating outcome inside tracker.update() or try_kf_update()
                            pre_state = self.tracker.kf.x.copy()
                            self.tracker.update(best, stamp_ns)
                            # check if update actually changed the state (rough check)
                            post_state = self.tracker.kf.x
                            if not np.allclose(pre_state, post_state):
                                # self.get_logger().info("KF updated with ICP measurement")
                                print("[point_cloud_tracker] CASE: KF updated with ICP measurement")  # --- CHANGED ---
                            else:
                                # self.get_logger().info("KF prediction kept (ICP measurement rejected by gating)")
                                print("[point_cloud_tracker] CASE: KF measurement rejected by Mahalanobis gating")  # --- CHANGED ---
                            self.last_lidar_match_centroid = np.mean(best, axis=0)
                            self.publish_cluster(best, self.latest_lidar_msg.header)
                        else:
                            self.publish_cluster(predicted_cloud, self.latest_lidar_msg.header)
                            # self.get_logger().info("Prediction only: ICP no fit")
                            print("[point_cloud_tracker] CASE: KF prediction-only (ICP no fit)")  # --- CHANGED ---
                    else:
                        self.publish_cluster(predicted_cloud, self.latest_lidar_msg.header)
                        # self.get_logger().info("Prediction only: no candidates")
                        print("[point_cloud_tracker] CASE: KF prediction-only (no candidates)")  # --- CHANGED ---

                # publish both base_link and Vicon world (absolute)
                state, _ = self.tracker.get_state()
                pos_lidar = np.array([float(state[0]), float(state[1]), float(state[2])])
                vel_lidar = np.array([float(state[3]), float(state[4]), float(state[5])])
                self.publish_pose_and_twist(now, pos_lidar, vel_lidar)
            return

        # --------- No-KF branch (pure measurement) ----------
        if self.latest_lidar_msg is None or self.last_valid_cluster is None:
            # self.get_logger().debug("No-KF branch: waiting for data")
            print("[point_cloud_tracker] CASE: No-KF waiting for data")  # --- CHANGED ---
            return  # nothing we can publish yet

        # Read full LiDAR cloud
        lidar_cloud = np.array([
            [p[0], p[1], p[2]]
            for p in pc2.read_points(
                self.latest_lidar_msg, field_names=("x", "y", "z"), skip_nans=True
            )
        ], dtype=np.float32)

        # ROI center: last LiDAR match if known, else last camera/frustum centroid
        if self.last_lidar_match_centroid is not None:
            roi_center = self.last_lidar_match_centroid
        else:
            # --- CHANGED ---
            # Transform stored camera cluster into the LiDAR frame if necessary, then use its centroid
            if (self.last_valid_cluster_frame is not None) and (self.last_valid_cluster_frame != self.latest_lidar_msg.header.frame_id):
                ref_cloud_for_roi = self.transform_points(
                    self.last_valid_cluster,
                    self.last_valid_cluster_frame,
                    self.latest_lidar_msg.header.frame_id,
                    self.last_valid_cluster_header
                )
            else:
                ref_cloud_for_roi = self.last_valid_cluster

            roi_center = np.mean(ref_cloud_for_roi, axis=0)

        roi_points = self.extract_roi_points(lidar_cloud, roi_center, radius=2.5)
        candidate_clusters = self.cluster_with_dbscan(
            roi_points, tolerance=0.2, min_size=20, max_size=1000
        )

        chosen_cloud = None
        if candidate_clusters:
            # --- CHANGED ---
            # Use transformed reference for ICP (ensure same frame)
            if (self.last_valid_cluster_frame is not None) and (self.last_valid_cluster_frame != self.latest_lidar_msg.header.frame_id):
                icp_ref = self.transform_points(
                    self.last_valid_cluster,
                    self.last_valid_cluster_frame,
                    self.latest_lidar_msg.header.frame_id,
                    self.last_valid_cluster_header
                )
            else:
                icp_ref = self.last_valid_cluster

            best = self.icp_match(icp_ref, candidate_clusters)
            if best is not None and best.shape[0] > 0:
                chosen_cloud = best
                self.last_lidar_match_centroid = np.mean(best, axis=0)
                self.get_logger().info("Measurement-only: selected cluster")
                print("[point_cloud_tracker] CASE: Measurement-only selected cluster")  # --- CHANGED ---
            else:
                self.get_logger().info("Measurement only: ICP no fit (KF off)")
                print("[point_cloud_tracker] CASE: Measurement-only ICP no fit (KF off)")  # --- CHANGED ---
        else:
            self.get_logger().info("Measurement only: no candidates (KF off)")
            print("[point_cloud_tracker] CASE: Measurement-only no candidates (KF off)")  # --- CHANGED ---

        if chosen_cloud is None:
            chosen_cloud = self.last_valid_cluster
            print("[point_cloud_tracker] CASE: Measurement-only fallback to last_valid_cluster")  # --- CHANGED ---

        self.publish_cluster(chosen_cloud, self.latest_lidar_msg.header)

        # Pose/velocity from centroid (finite differences) in LiDAR
        centroid = np.mean(chosen_cloud, axis=0)
        t_ns = self.latest_lidar_msg.header.stamp.sec * 1_000_000_000 + self.latest_lidar_msg.header.stamp.nanosec
        if self.prev_centroid is not None and self.prev_stamp_ns is not None:
            dt = max((t_ns - self.prev_stamp_ns) * 1e-9, 1e-6)
            vel = (centroid - self.prev_centroid) / dt
        else:
            vel = np.zeros(3, dtype=float)

        self.prev_centroid = centroid.copy()
        self.prev_stamp_ns = t_ns

        pos_lidar = centroid.astype(float)
        vel_lidar = vel.astype(float)
        self.publish_pose_and_twist(now, pos_lidar, vel_lidar)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudTrackerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
