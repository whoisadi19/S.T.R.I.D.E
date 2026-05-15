"""
Kinematics Node — Drone state management for S.T.R.I.D.E

Bridges ROS 2 /cmd_vel commands to Gazebo kinematic movement.
Also simulates battery drain and publishes telemetry:
  /drone/pose       — PoseStamped (position + orientation)
  /drone/battery    — String (JSON: percentage, voltage, state)
  /drone/telemetry  — String (JSON: speed, altitude, heading, etc.)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from std_msgs.msg import String
import math
import json
import time


class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.get_logger().info('━━━ Kinematics Node v2.0 ━━━')

        # Subscribe to /cmd_vel (global-frame velocities)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Gazebo service client
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')

        # Publishers
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        self.battery_pub = self.create_publisher(String, '/drone/battery', 10)
        self.telemetry_pub = self.create_publisher(String, '/drone/telemetry', 10)

        # Internal state (starting pose — matches helipad at 0, -6)
        self.x = 0.0
        self.y = -6.0
        self.z = 1.0
        self.yaw = 0.0

        # Current velocity commands
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        # Velocity timeout
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5

        # ── Battery Simulation ──────────────────────────────────────
        self.battery_percent = 100.0
        self.battery_voltage = 16.8       # 4S LiPo fully charged
        self.battery_min_voltage = 13.2   # 4S LiPo empty
        self.battery_drain_idle = 0.02    # %/sec when hovering
        self.battery_drain_move = 0.08    # %/sec when moving
        self.battery_low_threshold = 20.0
        self.battery_critical_threshold = 10.0
        self.battery_warned = False

        # ── Flight stats ────────────────────────────────────────────
        self.flight_start_time = time.time()
        self.total_distance = 0.0
        self.max_altitude = 0.0
        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_z = self.z

        # Update rate: 20 Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_loop)

        # Telemetry publish at 2 Hz (separate timer)
        self.telem_timer = self.create_timer(0.5, self.publish_telemetry)

        self.service_ready = False

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.last_cmd_time = self.get_clock().now()

    def euler_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def update_loop(self):
        # Velocity timeout
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0

        # Update position
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt

        # Face direction of travel
        if abs(self.vx) > 0.05 or abs(self.vy) > 0.05:
            self.yaw = math.atan2(self.vy, self.vx)

        # Geofencing (expanded for new world layout)
        self.x = max(-20.0, min(30.0, self.x))
        self.y = max(-20.0, min(20.0, self.y))
        self.z = max(0.1, min(25.0, self.z))

        # ── Battery drain ───────────────────────────────────────────
        speed = math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
        if speed > 0.1:
            drain = self.battery_drain_move * self.dt
        else:
            drain = self.battery_drain_idle * self.dt
        self.battery_percent = max(0.0, self.battery_percent - drain)

        # Map percentage to voltage (linear approximation)
        voltage_range = self.battery_voltage - self.battery_min_voltage
        self.battery_voltage = self.battery_min_voltage + (
            voltage_range * self.battery_percent / 100.0)

        # Battery warnings
        if self.battery_percent <= self.battery_low_threshold and not self.battery_warned:
            self.battery_warned = True
            self.get_logger().warn(
                f'⚠ BATTERY LOW: {self.battery_percent:.0f}%')

        # ── Flight stats ────────────────────────────────────────────
        dx = self.x - self.prev_x
        dy = self.y - self.prev_y
        dz = self.z - self.prev_z
        self.total_distance += math.sqrt(dx*dx + dy*dy + dz*dz)
        self.max_altitude = max(self.max_altitude, self.z)
        self.prev_x, self.prev_y, self.prev_z = self.x, self.y, self.z

        # ── Publish pose ────────────────────────────────────────────
        qx, qy, qz, qw = self.euler_to_quaternion(self.yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        # ── Publish battery ─────────────────────────────────────────
        batt_msg = String()
        batt_msg.data = json.dumps({
            'percentage': round(self.battery_percent, 1),
            'voltage': round(self.battery_voltage, 2),
            'state': 'CRITICAL' if self.battery_percent < self.battery_critical_threshold
                     else 'LOW' if self.battery_percent < self.battery_low_threshold
                     else 'OK'
        })
        self.battery_pub.publish(batt_msg)

        # ── Update Gazebo ───────────────────────────────────────────
        if not self.service_ready:
            if self.client.service_is_ready():
                self.service_ready = True
                self.get_logger().info('Connected to Gazebo set_entity_state')
            else:
                return

        req = SetEntityState.Request()
        state = EntityState()
        state.name = 'simple_drone'
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.position.z = self.z
        state.pose.orientation.x = qx
        state.pose.orientation.y = qy
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        req.state = state

        future = self.client.call_async(req)
        future.add_done_callback(self._service_cb)

    def publish_telemetry(self):
        """Publish comprehensive telemetry at 2 Hz."""
        speed = math.sqrt(self.vx**2 + self.vy**2 + self.vz**2)
        flight_time = time.time() - self.flight_start_time
        heading_deg = math.degrees(self.yaw) % 360

        msg = String()
        msg.data = json.dumps({
            'position': {
                'x': round(self.x, 2),
                'y': round(self.y, 2),
                'z': round(self.z, 2)
            },
            'velocity': {
                'vx': round(self.vx, 2),
                'vy': round(self.vy, 2),
                'vz': round(self.vz, 2),
                'speed': round(speed, 2)
            },
            'altitude': round(self.z, 2),
            'heading_deg': round(heading_deg, 1),
            'battery_percent': round(self.battery_percent, 1),
            'flight_time_sec': round(flight_time, 1),
            'total_distance_m': round(self.total_distance, 1),
            'max_altitude_m': round(self.max_altitude, 1)
        })
        self.telemetry_pub.publish(msg)

    def _service_cb(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().error(
                    f'Gazebo rejected move: {resp.status_message}',
                    throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(
                f'Service error: {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
