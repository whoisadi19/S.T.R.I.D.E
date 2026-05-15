"""
Navigation Node — Multi-Phase Inspection Mission for S.T.R.I.D.E

Executes a full autonomous inspection of the multi-section tower:
  Phase 1: TAKEOFF        — Ascend to cruise altitude
  Phase 2: LOWER_ORBIT    — Orbit the wide lower section (alt ~3m, r=4m)
  Phase 3: MID_ORBIT      — Orbit at walkway level (alt ~5.5m, r=5m)
  Phase 4: UPPER_ORBIT    — Orbit the narrow upper section (alt ~8.5m, r=3.5m)
  Phase 5: TOP_SCAN       — Close pass around the antenna/cap (alt ~12m, r=3m)
  Phase 6: VERTICAL_SCAN  — Descend along the tower face for a detailed sweep
  Phase 7: RETURN_TO_LAUNCH — Fly back to start position
  Phase 8: LAND           — Descend and touch down

Subscribes to /drone/pose (from kinematics_node)
Publishes to /cmd_vel
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import math
import time
import json


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('━━━ S.T.R.I.D.E Navigation Node v2.0 ━━━')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Publisher for mission status (for future dashboard)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Subscribe to authoritative pose from kinematics_node
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose', self.pose_callback, 10)

        # Subscribe to LiDAR for obstacle avoidance
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # ── Mission Phases ──────────────────────────────────────────
        self.PHASE_WAITING       = 'WAITING'
        self.PHASE_TAKEOFF       = 'TAKEOFF'
        self.PHASE_LOWER_ORBIT   = 'LOWER_ORBIT'
        self.PHASE_MID_ORBIT     = 'MID_ORBIT'
        self.PHASE_UPPER_ORBIT   = 'UPPER_ORBIT'
        self.PHASE_TOP_SCAN      = 'TOP_SCAN'
        self.PHASE_VERTICAL_SCAN = 'VERTICAL_SCAN'
        self.PHASE_RTL           = 'RETURN_TO_LAUNCH'
        self.PHASE_LAND          = 'LAND'
        self.PHASE_COMPLETE      = 'COMPLETE'

        self.phase = self.PHASE_WAITING
        self.phase_start_time = time.time()

        # ── Build Waypoint Sequences ────────────────────────────────
        # Tower geometry reference:
        #   Foundation: 0-0.5m (3.5x3.5 box)
        #   Lower cylinder: 0.5-5.5m (r=1.5)
        #   Walkway ring: 5.6m (r=2.2)
        #   Upper cylinder: 6-11m (r=1.2)
        #   Top cap: 11m, Antenna: to 14m

        self.HOME = (0.0, -6.0, 1.0)

        self.mission_phases = [
            {
                'name': self.PHASE_TAKEOFF,
                'waypoints': self._generate_takeoff(),
                'description': 'Ascending to cruise altitude'
            },
            {
                'name': self.PHASE_LOWER_ORBIT,
                'waypoints': self._generate_orbit(alt=3.0, radius=4.0, points=12),
                'description': 'Inspecting lower section (weathered concrete)'
            },
            {
                'name': self.PHASE_MID_ORBIT,
                'waypoints': self._generate_orbit(alt=5.5, radius=5.0, points=12),
                'description': 'Inspecting walkway and transition ring'
            },
            {
                'name': self.PHASE_UPPER_ORBIT,
                'waypoints': self._generate_orbit(alt=8.5, radius=3.5, points=12),
                'description': 'Inspecting upper section (aged steel)'
            },
            {
                'name': self.PHASE_TOP_SCAN,
                'waypoints': self._generate_orbit(alt=12.0, radius=3.0, points=8),
                'description': 'Scanning antenna and top cap'
            },
            {
                'name': self.PHASE_VERTICAL_SCAN,
                'waypoints': self._generate_vertical_scan(),
                'description': 'Vertical sweep — detailed face scan'
            },
            {
                'name': self.PHASE_RTL,
                'waypoints': self._generate_rtl(),
                'description': 'Returning to launch position'
            },
            {
                'name': self.PHASE_LAND,
                'waypoints': [(self.HOME[0], self.HOME[1], 0.3)],
                'description': 'Landing'
            },
        ]

        self.current_phase_idx = 0
        self.current_wp = 0

        # ── Drone State ─────────────────────────────────────────────
        self.x = self.HOME[0]
        self.y = self.HOME[1]
        self.z = self.HOME[2]
        self.pose_received = False

        # ── Obstacle Avoidance ──────────────────────────────────────
        self.obstacle_detected = False
        self.obstacle_clear_count = 0

        # ── Control Parameters ──────────────────────────────────────
        self.wp_reached_radius = 0.5   # meters
        self.max_speed = 0.8           # m/s max velocity
        self.approach_gain = 0.5       # P-controller gain
        self.approach_slow_dist = 2.0  # start slowing down

        # ── Mission Stats ───────────────────────────────────────────
        self.mission_start_time = None
        self.waypoints_completed = 0
        self.total_waypoints = sum(
            len(p['waypoints']) for p in self.mission_phases)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Mission loaded: {len(self.mission_phases)} phases, '
            f'{self.total_waypoints} total waypoints')

    # ═══════════════════════════════════════════════════════════════
    # WAYPOINT GENERATORS
    # ═══════════════════════════════════════════════════════════════

    def _generate_takeoff(self):
        """Takeoff straight up, then move to first orbit entry point."""
        alt = 3.0
        return [
            (self.HOME[0], self.HOME[1], alt),          # ascend
            (0.0, -4.0, alt),                            # approach orbit entry
        ]

    def _generate_orbit(self, alt, radius, points):
        """Generate a circular orbit at given altitude and radius."""
        wps = []
        # Start from the south (-Y) and go counter-clockwise
        for i in range(points + 1):  # +1 to close the loop
            angle = -math.pi/2 + (2 * math.pi * i / points)
            wx = radius * math.cos(angle)
            wy = radius * math.sin(angle)
            wps.append((wx, wy, alt))
        return wps

    def _generate_vertical_scan(self):
        """Descend along one face of the tower for a detailed vertical sweep."""
        wps = []
        scan_radius = 3.5  # distance from tower center
        # Descend from 12m to 2m in steps, facing the tower
        for alt in [12.0, 10.0, 8.0, 6.0, 4.0, 2.5]:
            wps.append((scan_radius, 0.0, alt))
        return wps

    def _generate_rtl(self):
        """Return to launch — climb to safe altitude first, then fly home."""
        return [
            (3.5, 0.0, 5.0),                            # clear the tower
            (0.0, -6.0, 5.0),                            # fly above home
        ]

    # ═══════════════════════════════════════════════════════════════
    # CALLBACKS
    # ═══════════════════════════════════════════════════════════════

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        if not self.pose_received:
            self.pose_received = True
            self.phase = self.mission_phases[0]['name']
            self.mission_start_time = time.time()
            self.phase_start_time = time.time()
            self.get_logger().info(
                f'✓ Pose locked at ({self.x:.1f}, {self.y:.1f}, {self.z:.1f})')
            self.get_logger().info(
                '━━━ MISSION START ━━━')
            self._log_phase_start()

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if 0.5 < r < float('inf')]
        if valid and min(valid) < 1.5:
            self.obstacle_detected = True
            self.obstacle_clear_count = 0
        else:
            self.obstacle_clear_count += 1
            if self.obstacle_clear_count > 5:
                self.obstacle_detected = False

    # ═══════════════════════════════════════════════════════════════
    # CONTROL LOOP
    # ═══════════════════════════════════════════════════════════════

    def control_loop(self):
        cmd = Twist()

        # === WAITING: No pose data yet ===
        if self.phase == self.PHASE_WAITING:
            self.cmd_pub.publish(cmd)
            return

        # === COMPLETE: Mission done ===
        if self.phase == self.PHASE_COMPLETE:
            self.cmd_pub.publish(cmd)
            self.get_logger().info(
                '✓ Mission Complete — hovering at home.',
                throttle_duration_sec=10.0)
            return

        # === OBSTACLE: Hold position ===
        if self.obstacle_detected:
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(
                '⚠ Obstacle detected — holding position.',
                throttle_duration_sec=2.0)
            return

        # === FLYING: Execute current phase waypoints ===
        current_mission = self.mission_phases[self.current_phase_idx]
        waypoints = current_mission['waypoints']

        # Check if current phase is done
        if self.current_wp >= len(waypoints):
            self._advance_phase()
            self.cmd_pub.publish(cmd)
            return

        # Current target
        tx, ty, tz = waypoints[self.current_wp]
        dx = tx - self.x
        dy = ty - self.y
        dz = tz - self.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        # Waypoint reached
        if dist < self.wp_reached_radius:
            self.waypoints_completed += 1
            phase_wp_num = self.current_wp + 1
            phase_wp_total = len(waypoints)
            self.get_logger().info(
                f'  ✓ WP {phase_wp_num}/{phase_wp_total} reached '
                f'({tx:.1f}, {ty:.1f}, {tz:.1f})')
            self.current_wp += 1
            self.cmd_pub.publish(cmd)
            self._publish_status()
            return

        # Compute velocity: proportional control with approach deceleration
        speed_scale = min(1.0, dist / self.approach_slow_dist)
        speed = self.max_speed * max(speed_scale, 0.15)

        cmd.linear.x = (dx / dist) * speed
        cmd.linear.y = (dy / dist) * speed
        cmd.linear.z = (dz / dist) * speed

        self.cmd_pub.publish(cmd)

        # Periodic status log
        progress = (self.waypoints_completed / self.total_waypoints) * 100
        self.get_logger().info(
            f'→ [{current_mission["name"]}] WP{self.current_wp+1} '
            f'({tx:.0f},{ty:.0f},{tz:.0f}) | '
            f'pos ({self.x:.1f},{self.y:.1f},{self.z:.1f}) | '
            f'dist {dist:.1f}m | progress {progress:.0f}%',
            throttle_duration_sec=3.0)

    # ═══════════════════════════════════════════════════════════════
    # PHASE MANAGEMENT
    # ═══════════════════════════════════════════════════════════════

    def _advance_phase(self):
        """Move to the next mission phase."""
        elapsed = time.time() - self.phase_start_time
        current_name = self.mission_phases[self.current_phase_idx]['name']
        self.get_logger().info(
            f'━━ Phase [{current_name}] complete ({elapsed:.1f}s) ━━')

        self.current_phase_idx += 1
        self.current_wp = 0

        if self.current_phase_idx >= len(self.mission_phases):
            self.phase = self.PHASE_COMPLETE
            total_time = time.time() - self.mission_start_time
            self.get_logger().info(
                f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            self.get_logger().info(
                f'  ✓ MISSION COMPLETE')
            self.get_logger().info(
                f'  Total time: {total_time:.1f}s')
            self.get_logger().info(
                f'  Waypoints: {self.waypoints_completed}/{self.total_waypoints}')
            self.get_logger().info(
                f'━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━')
            self._publish_status()
            return

        self.phase = self.mission_phases[self.current_phase_idx]['name']
        self.phase_start_time = time.time()
        self._log_phase_start()

    def _log_phase_start(self):
        """Log the start of a new phase."""
        mission = self.mission_phases[self.current_phase_idx]
        self.get_logger().info(
            f'▶ Phase {self.current_phase_idx + 1}/{len(self.mission_phases)}: '
            f'{mission["name"]}')
        self.get_logger().info(
            f'  {mission["description"]} '
            f'({len(mission["waypoints"])} waypoints)')

    def _publish_status(self):
        """Publish mission status as JSON for the dashboard."""
        msg = String()
        progress = (self.waypoints_completed / self.total_waypoints) * 100
        elapsed = time.time() - (self.mission_start_time or time.time())
        payload = {
            'phase': self.phase,
            'phase_index': self.current_phase_idx,
            'total_phases': len(self.mission_phases),
            'waypoint': self.current_wp,
            'waypoints_completed': self.waypoints_completed,
            'total_waypoints': self.total_waypoints,
            'progress_percent': round(progress, 1),
            'elapsed_seconds': round(elapsed, 1),
            'position': {
                'x': round(self.x, 2),
                'y': round(self.y, 2),
                'z': round(self.z, 2)
            },
            'obstacle_detected': self.obstacle_detected
        }
        msg.data = json.dumps(payload)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
