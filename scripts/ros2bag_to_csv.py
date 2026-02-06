#!/usr/bin/env python3
"""
ROS 2 bag → CSV pipeline

Tested with Ubuntu 24.04, ROS 2 Jazzy.
- Opens both MCAP and SQLite3 rosbags (auto-detect).
- Streams messages in time order (no big RAM spike), one CSV per topic.
- Works with any message type (dynamic introspection), flattens fields.
- Adds bag timestamp (t) and relative time (t_rel). If Header.stamp exists,
  includes header.stamp as seconds.
- Optional quaternion→Euler (roll/pitch/yaw, rad) for common fields.
- Topic filter by exact name or regex; time-window trimming; rate summary.

Usage examples:
  python3 ros2bag_to_csv.py /path/to/bag_dir --out ./csv_out
  python3 ros2bag_to_csv.py /path/to/bag_dir --topics /odom '/vicon/.*' --euler
  python3 ros2bag_to_csv.py /path/to/bag_dir --start 5.0 --end 45.0
  python3 ros2bag_to_csv.py /path/to/bag_dir --rate-only

  python3 ros2bag_to_csv.py "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbags/rosbags_110625/rosbag_110403_outdoor_track" \
  --out "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbag_csvs/csv_out_1106_outdoor_track" \
  --topics /tracked_uav_pose /tracked_uav_velocity /fmu/out/vehicle_gps_position /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry


  python3 ros2bag_to_csv.py "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbags/rosbags_110625/outdoor_uav_4_1106" \
  --out "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbag_csvs/csv_out_1106_outdoor_track_uav_4" \
  --topics /cpsl_uav_4/fmu/out/vehicle_gps_position /cpsl_uav_4/fmu/out/vehicle_local_position /cpsl_uav_4/fmu/out/vehicle_odometry

    python3 ros2bag_to_csv.py "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbags/rosbags_111025_attacker/rosbag_111001_track_and_spoof_04" \
  --out "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbag_csvs/csv_out_1110_attacker_04" \
  --topics /tracked_uav_pose /fmu/out/vehicle_gps_position /fmu/out/vehicle_local_position /fmu/out/vehicle_odometry /victim/estimated_sensor_gps /victim/estimated_local_position /spoofing_gps_bias/latlon /spoofing_gps_bias/ned


    python3 ros2bag_to_csv.py "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbags/rosbags_111025_victim/spoof_1110_03_upload" \
  --out "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbag_csvs/csv_out_1110_victim_03" \
  --topics /diagnostics/actual_ned /diagnostics/believed_ned /diagnostics/cmd_ned /remote/spoofing_gps_bias/ned


    python3 ros2bag_to_csv.py "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbags/rosbags_111125_victim/hover/spoof_1111_02_hover_upload" \
  --out "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbag_csvs/csv_out_1111_hover_02" \
  --topics /cpsl_uav_4/fmu/out/vehicle_odometry /cpsl_uav_4/fmu/out/vehicle_gps_position /cpsl_uav_4/fmu/out/vehicle_local_position diagnostics/actual_ned diagnostics/believed_ned diagnostics/cmd_ned diagnostics/target_ned /remote/spoofing_gps_bias/ned /remote/spoofing_gps_bias/latlon /remote/fmu/out/vehicle_local_position /remote/fmu/out/vehicle_gps_position /remote/tracked_uav_pose

  
      python3 ros2bag_to_csv.py "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbags/rosbags_111125_victim/move/spoof_1111_move_noatt" \
  --out "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbag_csvs/csv_out_1111_move_no_att" \
  --topics /cpsl_uav_4/fmu/out/vehicle_odometry /cpsl_uav_4/fmu/out/vehicle_gps_position /cpsl_uav_4/fmu/out/vehicle_local_position diagnostics/actual_ned diagnostics/believed_ned diagnostics/cmd_ned diagnostics/target_ned /remote/spoofing_gps_bias/ned /remote/spoofing_gps_bias/latlon /remote/fmu/out/vehicle_local_position /remote/fmu/out/vehicle_gps_position /remote/tracked_uav_pose

  




NOTE: Run inside a ROS 2 environment, e.g. `source /opt/ros/jazzy/setup.bash`.
"""
from __future__ import annotations
import argparse
import csv
import json
import math
import os
import re
import sys
from dataclasses import is_dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except Exception as e:
    print("[ERROR] Could not import ROS 2 Python libs. Did you `source /opt/ros/jazzy/setup.bash`?\n"
          f"Details: {e}", file=sys.stderr)
    sys.exit(1)

# ------------------------------ helpers ------------------------------

def sanitize_topic(topic: str) -> str:
    s = topic.strip('/')
    if not s:
        s = 'root'
    s = re.sub(r"[^A-Za-z0-9._-]+", "__", s)
    return s


def quat_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """Return (roll, pitch, yaw) in radians, ZYX convention (ROS standard)."""
    # roll (x-axis rotation)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    # pitch (y-axis rotation)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else (-1.0 if t2 < -1.0 else t2)
    pitch = math.asin(t2)
    # yaw (z-axis rotation)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


def is_ros_message(obj: Any) -> bool:
    return hasattr(obj, 'get_fields_and_field_types')


def to_basic(obj: Any) -> Any:
    """Recursively convert a ROS message to Python basic types (for JSON strings)."""
    if is_ros_message(obj):
        out = {}
        for fname in obj.get_fields_and_field_types().keys():
            out[fname] = to_basic(getattr(obj, fname))
        return out
    elif isinstance(obj, (list, tuple)):
        return [to_basic(v) for v in obj]
    else:
        return obj


def flatten_fields(obj: Any, prefix: str, out: Dict[str, Any]) -> None:
    """Flatten ROS messages into a dict of dotted keys → values.
    Sequences are JSON-encoded strings to keep a stable schema.
    """
    if is_ros_message(obj):
        for fname in obj.get_fields_and_field_types().keys():
            val = getattr(obj, fname)
            new_prefix = f"{prefix}.{fname}" if prefix else fname
            flatten_fields(val, new_prefix, out)
    elif isinstance(obj, (list, tuple)):
        out[prefix] = json.dumps(to_basic(obj))
    else:
        out[prefix] = obj


def add_header_stamp_seconds(flat: Dict[str, Any]) -> None:
    # Common pattern: header.stamp.sec + header.stamp.nanosec
    sec_key = None
    nsec_key = None
    for k in list(flat.keys()):
        if k.endswith('header.stamp.sec'):
            sec_key = k
        elif k.endswith('header.stamp.nanosec'):
            nsec_key = k
    if sec_key and nsec_key:
        try:
            flat['header.stamp'] = float(flat[sec_key]) + float(flat[nsec_key]) * 1e-9
        except Exception:
            pass


def add_euler_if_orientation_present(flat: Dict[str, Any]) -> None:
    # Check common bases where quaternions live
    bases = [
        'orientation',
        'pose.orientation',
        'transform.rotation',
        'rotation',
        'pose.pose.orientation',  # nav_msgs/Odometry
    ]
    for base in bases:
        xk, yk, zk, wk = f'{base}.x', f'{base}.y', f'{base}.z', f'{base}.w'
        if xk in flat and yk in flat and zk in flat and wk in flat:
            try:
                r, p, y = quat_to_euler(float(flat[xk]), float(flat[yk]), float(flat[zk]), float(flat[wk]))
                flat[f'{base}.roll'] = r
                flat[f'{base}.pitch'] = p
                flat[f'{base}.yaw'] = y
            except Exception:
                pass


# ------------------------------ reading ------------------------------

def open_reader(uri: str) -> Tuple[SequentialReader, str]:
    last_err = None
    for storage_id in ('mcap', 'sqlite3'):
        try:
            reader = SequentialReader()
            reader.open(
                StorageOptions(uri=uri, storage_id=storage_id),
                ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
            )
            return reader, storage_id
        except Exception as e:
            last_err = e
    raise RuntimeError(f"Failed to open bag at {uri!r} as MCAP or SQLite3. Last error: {last_err}")


def build_topic_type_map(reader: SequentialReader) -> Dict[str, str]:
    d: Dict[str, str] = {}
    for t in reader.get_all_topics_and_types():
        d[t.name] = t.type
    return d


def compile_topic_filters(topics: Optional[List[str]]) -> Optional[List[re.Pattern]]:
    if not topics:
        return None
    pats: List[re.Pattern] = []
    for s in topics:
        if s.startswith('^') or any(ch in s for ch in '.*+?$[]()|'):
            pats.append(re.compile(s))
        else:
            pats.append(re.compile('^' + re.escape(s) + '$'))
    return pats


def is_selected(topic: str, include: Optional[List[re.Pattern]], exclude: Optional[List[re.Pattern]]) -> bool:
    ok = True
    if include:
        ok = any(p.search(topic) for p in include)
    if ok and exclude:
        if any(p.search(topic) for p in exclude):
            ok = False
    return ok


# ------------------------------ writers ------------------------------
class CsvWriter:
    def __init__(self, path: str):
        self.path = path
        self.f = open(path, 'w', newline='')
        self.writer: Optional[csv.DictWriter] = None
        self.header_keys: List[str] = []

    def ensure_header(self, keys: Iterable[str]) -> None:
        if self.writer is None:
            # Deterministic preferred ordering
            keys = list(keys)
            preferred = ['t', 't_rel', 'header.stamp', 'header.frame_id']
            ordered: List[str] = []
            for p in preferred:
                if p in keys:
                    ordered.append(p)
            for k in keys:
                if k not in ordered:
                    ordered.append(k)
            self.header_keys = ordered
            self.writer = csv.DictWriter(self.f, fieldnames=self.header_keys)
            self.writer.writeheader()

    def write(self, row: Dict[str, Any]) -> None:
        assert self.writer is not None
        self.writer.writerow({k: row.get(k, '') for k in self.header_keys})

    def close(self):
        try:
            self.f.close()
        except Exception:
            pass


# ------------------------------ main conversion ------------------------------

def convert_bag_to_csv(
    bag_uri: str,
    out_dir: str,
    topics_include: Optional[List[str]] = None,
    topics_exclude: Optional[List[str]] = None,
    start_s: Optional[float] = None,
    end_s: Optional[float] = None,
    add_euler: bool = False,
    rate_only: bool = False,
) -> None:
    os.makedirs(out_dir, exist_ok=True)

    reader, storage_id = open_reader(bag_uri)
    topic_type = build_topic_type_map(reader)

    include_pats = compile_topic_filters(topics_include)
    exclude_pats = compile_topic_filters(topics_exclude)

    # Dynamic type cache
    type_cache: Dict[str, Any] = {}

    # Per-topic CSV writers
    writers: Dict[str, CsvWriter] = {}

    # Stats for rates
    stats: Dict[str, Dict[str, Any]] = {
        t: {"type": typ, "count": 0, "t_first": None, "t_last": None}
        for t, typ in topic_type.items() if is_selected(t, include_pats, exclude_pats)
    }

    bag_t0_ns: Optional[int] = None

    def within_window(t_ns: int) -> bool:
        nonlocal bag_t0_ns
        if start_s is None and end_s is None:
            return True
        if bag_t0_ns is None:
            # We define first seen timestamp as t0
            bag_t0_ns = t_ns
        t_rel_s = (t_ns - bag_t0_ns) * 1e-9
        if start_s is not None and t_rel_s < start_s:
            return False
        if end_s is not None and t_rel_s > end_s:
            return False
        return True

    # Iterate
    while reader.has_next():
        topic, serialized, t_ns = reader.read_next()
        if not is_selected(topic, include_pats, exclude_pats):
            continue
        if not within_window(t_ns):
            # We still want stats to reflect full window only
            # (skip counts outside window)
            pass
        else:
            # Load type class
            typ = topic_type[topic]
            if typ not in type_cache:
                try:
                    type_cache[typ] = get_message(typ)
                except Exception as e:
                    print(f"[WARN] Cannot load message type {typ} for {topic}: {e}")
                    # Skip if we cannot load
                    continue
            cls = type_cache[typ]
            # Deserialize
            try:
                msg = deserialize_message(serialized, cls)
            except Exception as e:
                print(f"[WARN] Deserialization failed for {topic} ({typ}): {e}")
                continue

            # Flatten
            flat: Dict[str, Any] = {}
            flatten_fields(msg, '', flat)
            add_header_stamp_seconds(flat)
            if add_euler:
                add_euler_if_orientation_present(flat)

            # Add times
            if bag_t0_ns is None:
                bag_t0_ns = t_ns
            flat['t'] = t_ns * 1e-9
            flat['t_rel'] = (t_ns - bag_t0_ns) * 1e-9

            # Writer
            if topic not in writers:
                fname = sanitize_topic(topic) + '.csv'
                writers[topic] = CsvWriter(os.path.join(out_dir, fname))
                # Establish header on first write
                writers[topic].ensure_header(flat.keys())
            writers[topic].write(flat)

            # Stats (only counting messages we wrote)
            st = stats.get(topic)
            if st is None:
                stats[topic] = st = {"type": topic_type[topic], "count": 0, "t_first": None, "t_last": None}
            st['count'] += 1
            st['t_first'] = t_ns if st['t_first'] is None else st['t_first']
            st['t_last'] = t_ns

    # Close files
    for w in writers.values():
        w.close()

    # Write a summary CSV with rates (for messages inside the selected window)
    summary_path = os.path.join(out_dir, 'bag_summary.csv')
    with open(summary_path, 'w', newline='') as f:
        wr = csv.writer(f)
        wr.writerow(['topic', 'type', 'count', 'start_time_s', 'end_time_s', 'duration_s', 'avg_rate_hz'])
        for topic, st in sorted(stats.items()):
            if st['count'] == 0 or st['t_first'] is None or st['t_last'] is None:
                wr.writerow([topic, st['type'], 0, '', '', '', ''])
                continue
            dt_s = (st['t_last'] - st['t_first']) * 1e-9 if st['t_last'] and st['t_first'] else 0.0
            rate = (st['count'] / dt_s) if dt_s > 0 else ''
            wr.writerow([
                topic,
                st['type'],
                st['count'],
                f"{st['t_first'] * 1e-9:.9f}",
                f"{st['t_last'] * 1e-9:.9f}",
                f"{dt_s:.6f}",
                f"{rate:.3f}" if rate != '' else ''
            ])

    if rate_only:
        # If user only wanted rates, remove the per-topic CSVs we wrote (if any)
        for topic, w in writers.items():
            try:
                os.remove(w.path)
            except Exception:
                pass
        print(f"[OK] Rate summary written to {summary_path}")
    else:
        print(f"[OK] Wrote CSVs to: {out_dir}\n[OK] Summary: {summary_path}")


# ------------------------------ CLI ------------------------------

def main():
    ap = argparse.ArgumentParser(description='ROS 2 bag → CSV pipeline (streams per-topic CSVs).')
    ap.add_argument('bag', help='Path to rosbag2 directory (containing metadata.yaml and data files).')
    ap.add_argument('--out', default='csv_out', help='Output directory for CSVs (default: csv_out).')
    ap.add_argument('--topics', nargs='+', help='Topic names or regex (quote your regex). Default: all topics.')
    ap.add_argument('--exclude', nargs='+', help='Exclude topics by name/regex.')
    ap.add_argument('--start', type=float, help='Start time offset [s] from first message in bag (inclusive).')
    ap.add_argument('--end', type=float, help='End time offset [s] from first message in bag (inclusive).')
    ap.add_argument('--euler', action='store_true', help='Add roll/pitch/yaw (rad) if quaternion fields are present.')
    ap.add_argument('--rate-only', action='store_true', help='Only produce bag_summary.csv with rates; skip per-topic CSVs.')

    args = ap.parse_args()

    convert_bag_to_csv(
        bag_uri=args.bag,
        out_dir=args.out,
        topics_include=args.topics,
        topics_exclude=args.exclude,
        start_s=args.start,
        end_s=args.end,
        add_euler=args.euler,
        rate_only=args.rate_only,
    )


if __name__ == '__main__':
    main()
