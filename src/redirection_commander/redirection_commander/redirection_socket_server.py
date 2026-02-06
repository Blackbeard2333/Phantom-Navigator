#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import json
import queue
import socket
import struct
import threading
import time
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.serialization import serialize_message, deserialize_message
# Works across distros (Foxy→Jazzy)
from rosidl_runtime_py import message_to_ordereddict as message_to_ordered_dict

from rosidl_runtime_py.utilities import get_message

# ------------------------------------------------------------
# Wire protocol
#   Each frame: [4-byte big-endian length][UTF-8 JSON payload]
#   JSON envelope (outbound):
#     {
#       "topic": "/uav1/spoof_cmd",
#       "type": "geometry_msgs/msg/Vector3",
#       "stamp": 1731160000.123,   # UNIX seconds (float)
#       "msg": { ... }             # message as dict
#     }
#   JSON envelope (inbound -> optional local publish): same schema
# ------------------------------------------------------------

def _now_sec() -> float:
    return time.time()


class SocketServer(Node):
    def __init__(self):
        super().__init__('redirection_socket_server')

        # -------------------- Parameters --------------------
        self.declare_parameter('bind_host', '0.0.0.0')   # listen on all NICs
        self.declare_parameter('port', 5005)
        self.declare_parameter('backlog', 1)
        self.declare_parameter('send_qos_reliability', 'best_effort')  # reliable|best_effort
        self.declare_parameter('topics_to_bridge', ['/spoofing_gps_bias/ned', '/spoofing_gps_bias/latlon', '/fmu/out/vehicle_gps_position', '/fmu/out/vehicle_local_position', '/tracked_uav_pose'])   # list[str]
        # If you want to accept inbound frames and publish locally, set e.g.:
        # inbound_topics: ['/remote_waypoints']
        # inbound_types:  ['geometry_msgs/msg/PoseArray']
        self.declare_parameter('inbound_topics', [])
        self.declare_parameter('inbound_types', [])
        self.declare_parameter('heartbeat_hz', 1.0)  # 0 to disable
        self.declare_parameter('connect_timeout_sec', 3.0)
        self.declare_parameter('socket_recv_timeout_sec', 0.5)
        self.declare_parameter('max_frame_bytes', 10 * 1024 * 1024)  # 10 MB safety

        bind_host = self.get_parameter('bind_host').get_parameter_value().string_value
        port = int(self.get_parameter('port').value)
        backlog = int(self.get_parameter('backlog').value)
        hb_hz = float(self.get_parameter('heartbeat_hz').value)
        self._hb_period = (1.0 / hb_hz) if hb_hz > 0 else None
        self._connect_timeout = float(self.get_parameter('connect_timeout_sec').value)
        self._recv_timeout = float(self.get_parameter('socket_recv_timeout_sec').value)
        self._max_frame = int(self.get_parameter('max_frame_bytes').value)

        # QoS for subscribers
        rel = self.get_parameter('send_qos_reliability').get_parameter_value().string_value
        reliability = ReliabilityPolicy.RELIABLE if rel.lower().startswith('rel') else ReliabilityPolicy.BEST_EFFORT
        self._qos = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Topics to bridge (outbound)
        self._topics_to_bridge: List[str] = [str(t) for t in self.get_parameter('topics_to_bridge').value]

        # Optional inbound mapping
        inbound_topics = [str(t) for t in self.get_parameter('inbound_topics').value]
        inbound_types = [str(t) for t in self.get_parameter('inbound_types').value]
        if len(inbound_topics) != len(inbound_types):
            self.get_logger().warn('inbound_topics and inbound_types length mismatch; inbound publish disabled.')
            self._inbound_publishers: Dict[str, Tuple[object, str]] = {}
        else:
            self._inbound_publishers = self._make_inbound_publishers(inbound_topics, inbound_types)

        # Networking primitives
        self._bind_host = bind_host
        self._port = port
        self._backlog = backlog
        self._srv_sock: Optional[socket.socket] = None
        self._conn_sock: Optional[socket.socket] = None
        self._conn_addr: Optional[Tuple[str, int]] = None

        # Queues and threads
        self._send_q: "queue.Queue[bytes]" = queue.Queue(maxsize=1000)
        self._shutdown = threading.Event()

        # Subscriptions (created lazily after we know types)
        self._subs = {}  # topic -> Subscription

        # Start server thread
        self._server_thread = threading.Thread(target=self._accept_loop, name='socket-accept', daemon=True)
        self._server_thread.start()

        # Timer to attach subscriptions once graph knows types
        self._attach_timer = self.create_timer(0.5, self._ensure_subscriptions)

        # Heartbeat timer (optional)
        if self._hb_period is not None:
            self._hb_timer = self.create_timer(self._hb_period, self._send_heartbeat)
        else:
            self._hb_timer = None

        self.get_logger().info(
            f"Socket server listening on {self._bind_host}:{self._port} | "
            f"topics: {self._topics_to_bridge} | inbound publishers: {list(self._inbound_publishers.keys())}"
        )

    # -------------------- Inbound publishers --------------------

    def _make_inbound_publishers(self, topics: List[str], types: List[str]) -> Dict[str, Tuple[object, str]]:
        pubs = {}
        for topic, type_str in zip(topics, types):
            try:
                msg_cls = get_message(type_str)
            except Exception as e:
                self.get_logger().error(f'Cannot import message type "{type_str}" for inbound topic {topic}: {e}')
                continue
            pub = self.create_publisher(msg_cls, topic, self._qos)
            pubs[topic] = (pub, type_str)
            self.get_logger().info(f'Inbound publisher ready: {topic} [{type_str}]')
        return pubs

    # -------------------- Subscription setup --------------------

    def _ensure_subscriptions(self):
        # Discover types then attach subscribers for each configured topic
        graph = dict(self.get_topic_names_and_types())
        for topic in self._topics_to_bridge:
            if topic in self._subs:
                continue
            type_list = graph.get(topic, [])
            if not type_list:
                # not yet available in graph
                continue
            # If multiple types are present, choose the first
            type_str = type_list[0]
            try:
                msg_cls = get_message(type_str)
            except Exception as e:
                self.get_logger().error(f'Failed to import type "{type_str}" for topic "{topic}": {e}')
                continue

            def _mk_cb(tname: str, ttype: str):
                def _cb(msg):
                    # Convert to JSON-able dict
                    try:
                        msg_dict = message_to_ordered_dict(msg)
                    except Exception as e:
                        self.get_logger().error(f'Failed to convert message on {tname} to dict: {e}')
                        return
                    payload = {
                        "topic": tname,
                        "type": ttype,
                        "stamp": _now_sec(),
                        "msg": msg_dict,
                    }
                    try:
                        blk = json.dumps(payload, separators=(',', ':')).encode('utf-8')
                        frame = struct.pack('>I', len(blk)) + blk
                        # Non-blocking; drop if queue full
                        try:
                            self._send_q.put_nowait(frame)
                        except queue.Full:
                            self.get_logger().warn('Send queue full; dropping frame')
                    except Exception as e:
                        self.get_logger().error(f'Encoding error for {tname}: {e}')
                return _cb

            sub = self.create_subscription(msg_cls, topic, _mk_cb(topic, type_str), self._qos)
            self._subs[topic] = sub
            self.get_logger().info(f'Bridging topic: {topic} [{type_str}]')

        # If all requested subs are attached, we can stop this timer
        if len(self._subs) == len(self._topics_to_bridge):
            self.get_logger().info('All requested subscriptions attached.')
            self._attach_timer.cancel()

    # -------------------- Networking --------------------

    def _accept_loop(self):
        """Accepts a single client at a time; runs a sender+receiver thread for that client."""
        while not self._shutdown.is_set():
            try:
                # Prepare listening socket
                if self._srv_sock is None:
                    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                    srv.bind((self._bind_host, self._port))
                    srv.listen(self._backlog)
                    self._srv_sock = srv
                    self.get_logger().info(f'Listening on {self._bind_host}:{self._port}')

                # Accept (blocking)
                self.get_logger().info('Waiting for client...')
                self._srv_sock.settimeout(1.0)
                try:
                    conn, addr = self._srv_sock.accept()
                except socket.timeout:
                    continue

                self._conn_sock = conn
                self._conn_addr = addr
                conn.settimeout(self._recv_timeout)
                self.get_logger().info(f'Client connected: {addr[0]}:{addr[1]}')

                # Start sender/receiver loops
                stop_evt = threading.Event()
                sender = threading.Thread(target=self._sender_loop, args=(conn, stop_evt), name='socket-sender', daemon=True)
                receiver = threading.Thread(target=self._receiver_loop, args=(conn, stop_evt), name='socket-recv', daemon=True)
                sender.start()
                receiver.start()

                # Join receiver (when it returns, drop the connection)
                receiver.join()
                stop_evt.set()
                sender.join(timeout=1.0)

                # Cleanup
                try:
                    conn.shutdown(socket.SHUT_RDWR)
                except Exception:
                    pass
                try:
                    conn.close()
                except Exception:
                    pass
                self._conn_sock = None
                self._conn_addr = None
                self.get_logger().warn('Client disconnected; returning to accept.')

            except Exception as e:
                self.get_logger().error(f'Accept loop error: {e}')
                time.sleep(1.0)

        # Final cleanup
        if self._srv_sock:
            try:
                self._srv_sock.close()
            except Exception:
                pass
            self._srv_sock = None

    def _sender_loop(self, conn: socket.socket, stop_evt: threading.Event):
        """Drains _send_q and writes frames to the socket."""
        while not (stop_evt.is_set() or self._shutdown.is_set()):
            try:
                # Wait up to 0.5s for data
                frame = self._send_q.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                conn.sendall(frame)
            except Exception as e:
                self.get_logger().warn(f'Send failed: {e}')
                break

    def _receiver_loop(self, conn: socket.socket, stop_evt: threading.Event):
        """Receives frames and (optionally) publishes inbound topics."""
        buf = b''
        expected = None

        while not (stop_evt.is_set() or self._shutdown.is_set()):
            try:
                chunk = conn.recv(4096)
                if not chunk:
                    # peer closed
                    break
                buf += chunk

                while True:
                    if expected is None:
                        if len(buf) < 4:
                            break
                        (expected,) = struct.unpack('>I', buf[:4])
                        buf = buf[4:]
                        if expected <= 0 or expected > self._max_frame:
                            self.get_logger().warn(f'Bad frame size: {expected}, dropping connection.')
                            return
                    if len(buf) < expected:
                        break
                    payload = buf[:expected]
                    buf = buf[expected:]
                    expected = None

                    # Decode JSON and optionally publish locally
                    try:
                        obj = json.loads(payload.decode('utf-8'))
                        tname = obj.get('topic')
                        ttype = obj.get('type')
                        msg_dict = obj.get('msg')
                        if tname in self._inbound_publishers:
                            pub, type_str = self._inbound_publishers[tname]
                            if type_str != ttype:
                                self.get_logger().warn(f'Inbound type mismatch for {tname}: {ttype} vs {type_str}')
                                continue
                            msg_cls = get_message(type_str)
                            msg = msg_cls()
                            _dict_to_msg(msg, msg_dict)
                            pub.publish(msg)
                        # else: ignore inbound if no publisher configured
                    except Exception as e:
                        self.get_logger().warn(f'Inbound frame parse error: {e}')
                        continue

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().warn(f'Receive error: {e}')
                break

    # -------------------- Heartbeat --------------------

    def _send_heartbeat(self):
        if self._conn_sock is None:
            return
        hb = {
            "topic": "__heartbeat__",
            "type": "std_msgs/msg/String",
            "stamp": _now_sec(),
            "msg": {"data": "ping"}
        }
        try:
            blk = json.dumps(hb, separators=(',', ':')).encode('utf-8')
            frame = struct.pack('>I', len(blk)) + blk
            # Try non-blocking put
            try:
                self._send_q.put_nowait(frame)
            except queue.Full:
                # drop heartbeat if congested
                pass
        except Exception as e:
            self.get_logger().warn(f'Heartbeat encode error: {e}')

    # -------------------- Shutdown --------------------

    def destroy_node(self):
        self._shutdown.set()
        try:
            if self._srv_sock:
                self._srv_sock.close()
        except Exception:
            pass
        try:
            if self._conn_sock:
                self._conn_sock.close()
        except Exception:
            pass
        super().destroy_node()


def _dict_to_msg(msg, d):
    """Populate a ROS message instance from a (possibly nested) dict."""
    # Works for basic ROS messages; for arrays and nested, recurse
    for field_name in d:
        val = d[field_name]
        current = getattr(msg, field_name)
        # sequences
        if isinstance(current, (list, tuple)):
            seq = []
            # Detect sequence of primitives vs sequence of nested
            if len(val) > 0 and isinstance(val[0], dict) and hasattr(current, 'append'):
                # sequence of nested messages (not typical in Python repr)
                raise NotImplementedError('Nested sequences not implemented in this quick helper.')
            else:
                seq = val
            setattr(msg, field_name, type(current)(seq))
        elif hasattr(current, '__slots__'):
            # Nested message
            _dict_to_msg(current, val)
        else:
            setattr(msg, field_name, val)


def main():
    rclpy.init()
    node = SocketServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
