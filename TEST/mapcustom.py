
#!/usr/bin/env python3

import math
import time
from collections import deque

import numpy as np
import rclpy
import matplotlib.pyplot as plt

from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

from tf2_ros import Buffer, TransformListener, TransformException


class ScanGridVisualizer(Node):

    def __init__(self):
        super().__init__('scan_grid_visualizer')

        # ---------------- Configuration ----------------
        self.map_frame = 'map'

        self.resolution = 0.05
        self.width = 400
        self.height = 400

        self.origin_x = -10.0
        self.origin_y = -10.0

        # How long to wait for a missing transform
        self.transform_wait_seconds = 3.0

        # Maximum number of scans waiting for TF
        self.max_pending_scans = 20

        # -1 = unknown, 0 = free, 100 = occupied
        self.grid = np.full(
            (self.height, self.width),
            -1,
            dtype=np.int8
        )

        # ---------------- TF ----------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------- Pending scan queue ----------------
        # Each entry: (scan message, monotonic enqueue time)
        self.pending_scans = deque()

        # ---------------- ROS publishers/subscribers ----------------
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/custom_map',
            map_qos
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan_levelled',
            self.scan_callback,
            10
        )

        # Retry pending scans without blocking the scan callback
        self.retry_timer = self.create_timer(
            0.05,
            self.process_pending_scans
        )

        # ---------------- Visualization ----------------
        plt.ion()

        self.fig, self.ax = plt.subplots()

        self.image = self.ax.imshow(
            self.grid,
            cmap='gray_r',
            vmin=-1,
            vmax=100,
            origin='lower',
            interpolation='nearest',
            extent=[
                self.origin_x,
                self.origin_x + self.width * self.resolution,
                self.origin_y,
                self.origin_y + self.height * self.resolution
            ]
        )

        self.ax.set_title('Live LiDAR Occupancy Grid')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_aspect('equal')
        self.fig.tight_layout()

        plt.show(block=False)

        self.last_warning_time = 0.0

        self.get_logger().info(
            'Scan grid visualizer started'
        )

        self.get_logger().info(
            f'Grid: {self.width} x {self.height}, '
            f'resolution={self.resolution} m'
        )

        self.get_logger().info(
            f'Using map frame: {self.map_frame}'
        )

    # ---------------------------------------------------------
    # Receive scans and enqueue them for timestamped TF lookup
    # ---------------------------------------------------------
    def scan_callback(self, scan):

        if len(self.pending_scans) >= self.max_pending_scans:
            # Drop the oldest waiting scan to limit latency
            self.pending_scans.popleft()

            self.get_logger().warning(
                'TF queue full; dropping oldest pending scan',
                throttle_duration_sec=2.0
            )

        self.pending_scans.append(
            (scan, time.monotonic())
        )

    # ---------------------------------------------------------
    # Retry pending scans until TF becomes available
    # ---------------------------------------------------------
    def process_pending_scans(self):

        if not self.pending_scans:
            self.update_visualization()
            return

        # Process scans in timestamp order
        while self.pending_scans:
            scan, enqueue_time = self.pending_scans[0]

            try:
                stamp = Time.from_msg(scan.header.stamp)

                # Non-blocking check. Do not use the latest pose
                # as a substitute for the scan's timestamp.
                available = self.tf_buffer.can_transform(
                    self.map_frame,
                    scan.header.frame_id,
                    stamp
                )

            except Exception as e:
                self.pending_scans.popleft()
                self.get_logger().warning(
                    f'Invalid scan timestamp or TF request: {e}'
                )
                continue

            if not available:
                age = time.monotonic() - enqueue_time

                if age >= self.transform_wait_seconds:
                    self.pending_scans.popleft()

                    self.get_logger().warning(
                        'Dropping scan: transform unavailable '
                        f'after {age:.1f} seconds. '
                        f'Scan stamp={scan.header.stamp.sec}.'
                        f'{scan.header.stamp.nanosec:09d}, '
                        f'frame={scan.header.frame_id}',
                        throttle_duration_sec=2.0
                    )

                    # Continue to check the next pending scan
                    continue

                # Preserve ordering and retry on the next timer tick
                break

            self.pending_scans.popleft()

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    scan.header.frame_id,
                    stamp
                )

                self.insert_scan(scan, transform)

            except TransformException as e:
                self.get_logger().warning(
                    f'TF lookup failed: {e}',
                    throttle_duration_sec=2.0
                )

        self.update_visualization()

    # ---------------------------------------------------------
    # Convert world coordinates to grid indices
    # ---------------------------------------------------------
    def world_to_grid(self, x, y):

        col = math.floor(
            (x - self.origin_x) / self.resolution
        )

        row = math.floor(
            (y - self.origin_y) / self.resolution
        )

        if (
            0 <= col < self.width
            and 0 <= row < self.height
        ):
            return int(row), int(col)

        return None

    # ---------------------------------------------------------
    # Bresenham ray tracing
    # Returns cells between sensor and endpoint,
    # excluding the endpoint itself.
    # ---------------------------------------------------------
    def raytrace(self, x0, y0, x1, y1):

        start = self.world_to_grid(x0, y0)
        end = self.world_to_grid(x1, y1)

        if start is None or end is None:
            return []

        row0, col0 = start
        row1, col1 = end

        cells = []

        dx = abs(col1 - col0)
        dy = abs(row1 - row0)

        sx = 1 if col0 < col1 else -1
        sy = 1 if row0 < row1 else -1

        error = dx - dy

        col = col0
        row = row0

        while (col, row) != (col1, row1):

            cells.append((row, col))

            e2 = 2 * error

            if e2 > -dy:
                error -= dy
                col += sx

            if e2 < dx:
                error += dx
                row += sy

        return cells

    # ---------------------------------------------------------
    # Insert a scan using its correctly timestamped transform
    # ---------------------------------------------------------
    def insert_scan(self, scan, transform):

        t = transform.transform.translation
        q = transform.transform.rotation

        # Quaternion -> 3D rotation matrix
        qx, qy, qz, qw = q.x, q.y, q.z, q.w

        R = np.array([
            [
                1 - 2 * (qy*qy + qz*qz),
                2 * (qx*qy - qz*qw),
                2 * (qx*qz + qy*qw)
            ],
            [
                2 * (qx*qy + qz*qw),
                1 - 2 * (qx*qx + qz*qz),
                2 * (qy*qz - qx*qw)
            ],
            [
                2 * (qx*qz - qy*qw),
                2 * (qy*qz + qx*qw),
                1 - 2 * (qx*qx + qy*qy)
            ]
        ])

        # LiDAR origin in map frame
        sensor_x = t.x
        sensor_y = t.y

        ranges = np.asarray(
            scan.ranges,
            dtype=np.float64
        )

        angles = (
            scan.angle_min
            + np.arange(len(ranges)) * scan.angle_increment
        )

        # Accept only finite, valid obstacle hits
        valid = (
            np.isfinite(ranges)
            & (ranges >= max(scan.range_min, 0.05))
            & (ranges < scan.range_max)
        )

        indices = np.flatnonzero(valid)

        for i in indices:

            r = ranges[i]
            a = angles[i]

            # Point in the scan frame
            point_scan = np.array([
                r * math.cos(a),
                r * math.sin(a),
                0.0
            ])

            # Point in the map frame
            point_map = R @ point_scan

            end_x = point_map[0] + t.x
            end_y = point_map[1] + t.y

            endpoint = self.world_to_grid(end_x, end_y)

            # Only clear free cells if the measured endpoint
            # falls inside the grid. Out-of-map rays are skipped.
            if endpoint is None:
                continue

            # Mark cells before the obstacle as free
            free_cells = self.raytrace(
                sensor_x,
                sensor_y,
                end_x,
                end_y
            )

            for row, col in free_cells:
                self.grid[row, col] = 0

            # Mark the endpoint occupied
            row, col = endpoint
            self.grid[row, col] = 100

        self.publish_map(scan)

    # ---------------------------------------------------------
    # Publish ROS OccupancyGrid
    # ---------------------------------------------------------
    def publish_map(self, scan):

        msg = OccupancyGrid()

        msg.header.stamp = scan.header.stamp
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        msg.data = self.grid.ravel().tolist()

        self.map_pub.publish(msg)

    # ---------------------------------------------------------
    # Update Matplotlib image
    # ---------------------------------------------------------
    def update_visualization(self):

        if not plt.fignum_exists(self.fig.number):
            return

        self.image.set_data(self.grid)
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.pause(0.001)


def main(args=None):

    rclpy.init(args=args)

    node = ScanGridVisualizer()

    try:
        while rclpy.ok():

            rclpy.spin_once(
                node,
                timeout_sec=0.02
            )

            if not plt.fignum_exists(node.fig.number):
                break

            plt.pause(0.001)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
