#!/usr/bin/env python3
import os
import sys
import time
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rajant_api import Breadcrumb
from concurrent.futures import ThreadPoolExecutor

# Keep your original configuration globals
RADIOS = ["10.195.219.1", "10.195.220.1", "10.195.221.1"]
# Rajant BreadCrumb ADMIN password. Read from the environment, never hardcoded
# (it used to be a literal here). Set RAJANT_ADMIN_PASSWORD, e.g. via
# ~/.fair_chain.env on the host and `docker run -e RAJANT_ADMIN_PASSWORD`.
PW = os.environ.get("RAJANT_ADMIN_PASSWORD")
if not PW:
    raise SystemExit("RAJANT_ADMIN_PASSWORD is not set")
macstr = lambda b: ":".join(f"{x:02x}" for x in b) if b else "-"
TIMER_INTERVAL = 0.2      # execution rate in seconds
RECONNECT_INTERVAL = 10.0 # how often a background thread retries dead radios
LOG_THROTTLE = 0.0        # seconds between console dumps; 0 = every cycle.
                          # Override at runtime: --ros-args -p log_throttle:=2.0


def connect_radio(host):
    """Open + authenticate one persistent session. Returns Breadcrumb or None.

    Only called from the background reconnect thread, never from the publish
    cycle, because reachable() shells out to ping and costs ~3s on a dead host.
    """
    try:
        bc = Breadcrumb(host=host, port=2300, role="ADMIN", password=PW)
        if not bc.reachable() or not bc.authenticate():
            return None
        # The library re-pings inside every get_state(). We already hold an
        # authenticated socket, so short-circuit that subprocess out of the hot
        # path; a genuinely dead radio now surfaces as a socket error instead.
        bc.reachable = lambda: True
        return bc
    except Exception:
        return None


class RajantMeshMonitor(Node):
    def __init__(self):
        super().__init__('rajant_mesh_monitor')

        # Initialize the ROS 2 Publisher
        self.publisher_ = self.create_publisher(String, 'weakest_link', 10)

        self.sessions = {}                 # host -> live authenticated Breadcrumb
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.pool = ThreadPoolExecutor(max_workers=len(RADIOS))
        self.last_log = 0.0
        self.declare_parameter('log_throttle', LOG_THROTTLE)
        # live publish-rate measurement, so the console shows the real Hz
        self.rate_hz = 0.0
        self.rate_n = 0
        self.rate_t0 = time.monotonic()

        # Dead radios are retried off the critical path so they never gate a cycle.
        self.reconnector = threading.Thread(target=self.reconnect_loop, daemon=True)
        self.reconnector.start()

        # Create a timer that executes the monitoring logic at the set interval
        self.timer = self.create_timer(TIMER_INTERVAL, self.check_mesh_links)
        self.get_logger().info(f"Rajant Mesh Monitor Node started. Checking every {TIMER_INTERVAL}s (persistent sessions)...")

    def reconnect_loop(self):
        while not self.stop_event.is_set():
            for host in RADIOS:
                if self.stop_event.is_set():
                    break
                with self.lock:
                    already = host in self.sessions
                if already:
                    continue
                bc = connect_radio(host)
                if bc is not None:
                    with self.lock:
                        self.sessions[host] = bc
                    self.get_logger().info(f"Connected to {host} (serial {bc.serial})")
            self.stop_event.wait(RECONNECT_INTERVAL)

    def drop_session(self, host):
        with self.lock:
            bc = self.sessions.pop(host, None)
        if bc is not None:
            try:
                bc.connection.close()
            except Exception:
                pass

    def poll_session(self, item):
        host, bc = item
        try:
            st = bc.get_state()
            if not st:
                return host, None
            return host, (bc.serial, st)
        except Exception:
            return host, None

    def tick_rate(self):
        self.rate_n += 1
        elapsed = time.monotonic() - self.rate_t0
        if elapsed >= 1.0:
            self.rate_hz = self.rate_n / elapsed
            self.rate_n = 0
            self.rate_t0 += elapsed

    def should_log(self):
        throttle = self.get_parameter('log_throttle').value
        if throttle <= 0.0:
            return True
        now = time.monotonic()
        if now - self.last_log < throttle:
            return False
        self.last_log = now
        return True

    def check_mesh_links(self):
        # 1. pull state from every radio we already hold a session to, IN PARALLEL
        with self.lock:
            active = list(self.sessions.items())

        if not active:
            self.get_logger().warn("No radios were reachable or authenticated.",
                                   throttle_duration_sec=5.0)
            return

        radios = {}
        for host, result in self.pool.map(self.poll_session, active):
            if result is None:
                self.get_logger().warn(f"Lost session to {host}, will reconnect.")
                self.drop_session(host)
            else:
                radios[host] = result

        if not radios:
            self.get_logger().warn("No radios returned state this cycle.",
                                   throttle_duration_sec=5.0)
            return

        # 2. index every interface MAC so peers can be named
        index = {}
        for host, (serial, st) in radios.items():
            for w in st.wireless: index[macstr(w.mac)] = serial
            for d in st.wired:    index[macstr(d.mac)] = serial

        # Track the weakest link during this iteration
        weakest_link_str = ""
        lowest_snr = float('inf')

        # 3. Process directional links
        for host, (serial, st) in sorted(radios.items()):
            for w in st.wireless:
                for p in w.peer:
                    peer = index.get(macstr(p.mac), p.ipv4Address or macstr(p.mac))
                    snr  = p.signal - w.noise        # == p.rssi

                    link_output = (f"{serial} {w.name} ch{w.channel} -> {peer}: "
                                   f"signal {p.signal}dBm  noise {w.noise}dBm  "
                                   f"SNR {snr}dB  cost {p.cost}  rate {p.rate}")

                    # Log or update lowest SNR
                    if snr < lowest_snr:
                        lowest_snr = snr
                        weakest_link_str = link_output

        # 4. Publish the results to the ROS 2 topic if a link was found
        if weakest_link_str:
            msg = String()
            msg.data = weakest_link_str
            self.publisher_.publish(msg)
            self.tick_rate()
            if self.should_log():
                self.get_logger().info(
                    f"Published Weakest Link (SNR {lowest_snr}dB) [publishing {self.rate_hz:.1f} Hz]")
        else:
            self.get_logger().info("No active wireless links detected.",
                                   throttle_duration_sec=5.0)

    def destroy_node(self):
        self.stop_event.set()
        for host in list(self.sessions):
            self.drop_session(host)
        self.pool.shutdown(wait=False)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = RajantMeshMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean shutdown handling
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
