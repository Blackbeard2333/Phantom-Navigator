# ROS 2 bag → CSV converter (`ros2bag_to_csv.py`)

A lightweight, streaming converter for **ROS 2 Jazzy (Ubuntu 24.04)** that turns a rosbag2 directory (MCAP or SQLite3) into **CSV files**—one CSV per topic—plus a compact summary of per‑topic rates.

---

## ✨ Features
- **MCAP / SQLite3 auto‑detect** (no config)
- **Streaming** read (memory‑safe on large bags)
- **Works with any message type** via dynamic introspection
- **Field flattening** into dotted column names (stable, grep‑able)
- Optional **quaternion → Euler** (`roll`, `pitch`, `yaw`, radians)
- **Topic include/exclude** (exact or regex), **time window** cropping
- **`bag_summary.csv`** with message counts & average rates

---

## ✅ Requirements
- **ROS 2 Jazzy** runtime available to Python
  ```bash
  source /opt/ros/jazzy/setup.bash
  ```
- Python 3.10+ (tested on **Python 3.12**, Ubuntu 24.04)

The script relies on:
- `rosbag2_py`, `rclpy`, `rosidl_runtime_py` (provided by your ROS 2 install)

---

## 📦 Get the script
Copy **`ros2bag_to_csv.py`** from the canvas into your repo (e.g., `tools/ros2bag_to_csv.py`) and make it executable if you like:
```bash
chmod +x ros2bag_to_csv.py
```

---

## 🚀 Quick start
Convert **all topics** from a bag to CSV files in `./csv_out`:
```bash
source /opt/ros/jazzy/setup.bash
python3 ros2bag_to_csv.py /path/to/bag_dir --out ./csv_out
```

**Your specific example (Vicon only + Euler):**
```bash
python3 ros2bag_to_csv.py \
  "/home/cpsl/Documents/CPSL_UAV_Tracking/rosbags/rosbag_0615_dataset_2" \
  --out "/home/cpsl/Documents/CPSL_UAV_Tracking/csv_out_0615" \
  --topics /vicon/x500_3/x500_3 \
  --euler
```
This writes one file:
```
/home/cpsl/Documents/CPSL_UAV_Tracking/csv_out_0615/vicon__x500_3__x500_3.csv
```
…and a `bag_summary.csv` in the same folder.

---

## 🔧 CLI reference
```
usage: ros2bag_to_csv.py BAG [--out OUT] [--topics TOPIC ...] [--exclude TOPIC ...]
                             [--start SECONDS] [--end SECONDS] [--euler] [--rate-only]
```
- `BAG` — path to the rosbag2 directory (must contain `metadata.yaml`).
- `--out OUT` — output directory for CSVs (default: `csv_out`).
- `--topics ...` — include only these topics; each entry can be an exact name **or a regex** (quote regex!).
- `--exclude ...` — exclude topics by name/regex (applied after `--topics`).
- `--start SECONDS` — start offset **relative to the first message** in the bag.
- `--end SECONDS` — end offset **relative to the first message** in the bag.
- `--euler` — add Euler angles when quaternion fields are present.
- `--rate-only` — skip per‑topic CSVs; only write `bag_summary.csv`.

### Examples
Only Vicon:
```bash
python3 ros2bag_to_csv.py BAG --out out --topics '/vicon/.*'
```
Exclude TF & diagnostics:
```bash
python3 ros2bag_to_csv.py BAG --out out --exclude '/tf.*' '/diagnostics.*'
```
Crop to 5–45 s window (from first seen msg):
```bash
python3 ros2bag_to_csv.py BAG --start 5 --end 45
```
Rates summary only:
```bash
python3 ros2bag_to_csv.py BAG --rate-only
```

---

## 🗂️ Output layout & schema
- **One CSV per topic** (filename = sanitized topic, e.g., `/vicon/x500_3/x500_3` → `vicon__x500_3__x500_3.csv`).
- A **`bag_summary.csv`** with per‑topic stats.

### CSV columns (per topic)
- `t` — bag storage timestamp (float seconds, from epoch in bag)
- `t_rel` — seconds relative to the **first message written** for that bag
- `header.stamp` — message header stamp in seconds (if present)
- `header.frame_id` — if present
- **Flattened message fields** with **dotted names**, e.g.:
  - `transform.translation.x`
  - `transform.rotation.w`
  - `pose.pose.position.z`
  - `twist.twist.linear.x`
- **Sequences/arrays** are stored as **JSON strings** in a single cell (stable schema; easy to parse later).
- With `--euler`, extra fields are added beside the quaternion source, e.g.:
  - `orientation.roll`, `orientation.pitch`, `orientation.yaw` (radians)
  - or `transform.rotation.roll`/`pitch`/`yaw` for TF‑like messages

> **Schema note:** the header (column set) is established from the **first written message** for that topic. If later messages introduce new fields (rare for consistent types), those extra fields won’t create new columns.

### `bag_summary.csv`
Columns: `topic`, `type`, `count`, `start_time_s`, `end_time_s`, `duration_s`, `avg_rate_hz`.

---

## ⏱️ Time semantics
- `t` and `t_rel` come from the **bag storage timestamp** (`rosbag2_py` returns `t_ns`).
- `header.stamp` is from the message header and may differ (e.g., logging delays, replay offsets).
- Use `t_rel` for within‑bag alignment, or `header.stamp` if all publishers timestamp consistently at acquisition.

---

## 🧠 How it works (brief)
- Opens the bag with `rosbag2_py.SequentialReader` (tries **mcap** then **sqlite3**).
- Builds a topic→type map using `get_all_topics_and_types()`.
- For each message:
  1. **Deserializes** via `rclpy.serialization.deserialize_message` + `rosidl_runtime_py.get_message(type)`
  2. **Flattens** into dotted fields; sequences → JSON
  3. Adds time fields; optionally adds Euler angles
  4. **Streams** row to the topic CSV (no large buffers)

---

## ⚙️ Performance tips
- Use `--topics` or `--exclude` to reduce I/O for giant bags.
- Crop with `--start/--end` to the interval you care about.
- Write to a fast local disk (SSD) for best throughput.

---

## 🧪 Verifying with pandas
```python
import pandas as pd

df = pd.read_csv('csv_out/vicon__x500_3__x500_3.csv')
# Example: parse a JSON array column if present
import json
if 'wrench.forces' in df.columns:
    df['forces_list'] = df['wrench.forces'].map(lambda s: json.loads(s) if isinstance(s, str) else s)
print(df.head())
```

---

## 🛠️ Troubleshooting
**"Could not import ROS 2 Python libs"**  
You didn’t source ROS: `source /opt/ros/jazzy/setup.bash`.

**"Cannot load message type …" or deserialization warnings**  
Ensure the **message packages** are available in the same environment as the script (source your ROS workspace overlay if needed). Messages of that type will be skipped with a warning if unavailable.

**Regex not matching**  
Quote your regex in the shell: `' /vicon/.* '`. For exact match, pass the topic name verbatim.

**Bag won’t open**  
Make sure you point to the **directory** (with `metadata.yaml`). The reader auto‑tries MCAP and SQLite3 backends.

**Permission or path errors**  
Check paths; create output directory or let the script create it.

---

## ❓FAQ
**Can I get a single combined CSV?**  
Not by default. The tool writes **one CSV per topic** (clean schemas). If you need a unified table ("long" or "wide"), say the word and we’ll extend the script.

**Can it convert ENU↔NED or remap frames?**  
Not in this script—by design it’s a faithful exporter. We can add helpers (e.g., extra columns with remapped axes) if you need them.

**Which quaternions are converted with `--euler`?**  
Common locations such as `orientation.*`, `pose.pose.orientation.*`, and `transform.rotation.*`.

---

## 🙌 Credits
Built on `rosbag2_py`, `rclpy`, and `rosidl_runtime_py` from ROS 2 Jazzy.

