"""
Navigation Node v3.0 — Multi-Structure Inspection Mission for S.T.R.I.D.E

Full autonomous inspection covering ALL infrastructure types:
  Phase 1:  TAKEOFF              — Ascend from helipad
  Phase 2:  TOWER_LOWER          — Orbit tower lower section (r=4m, alt=3m)
  Phase 3:  TOWER_UPPER          — Orbit tower upper section (r=3.5m, alt=8.5m)
  Phase 4:  TOWER_TOP            — Scan antenna area (r=3m, alt=12m)
  Phase 5:  TRANSIT_BRIDGE       — Fly to bridge
  Phase 6:  BRIDGE_INSPECTION    — Fly along bridge deck and underside
  Phase 7:  TRANSIT_PIPELINE     — Fly to pipeline rack
  Phase 8:  PIPELINE_INSPECTION  — Fly along pipeline at pipe level
  Phase 9:  TRANSIT_POWERLINE    — Fly to power line corridor
  Phase 10: POWERLINE_INSPECTION — Fly along power lines
  Phase 11: RETURN_TO_LAUNCH     — Fly back to helipad
  Phase 12: LAND                 — Descend and touch down

World Layout:
  Tower:     (0, 0)       Helipad: (0, -6)
  Bridge:    (20, 0)      Pipeline: (-12, 8)
  Power:     (-8, -12→12) Shed: (8, 3)
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
        self.get_logger().info('━━━ S.T.R.I.D.E Navigation Node v3.0 ━━━')
        self.get_logger().info('    Multi-Structure Inspection Mission')

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/mission/status', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose', self.pose_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.battery_sub = self.create_subscription(
            String, '/drone/battery', self.battery_callback, 10)

        # ── State ───────────────────────────────────────────────────
        self.HOME = (0.0, -6.0, 1.0)
        self.CRUISE_ALT = 5.0

        self.x, self.y, self.z = self.HOME
        self.pose_received = False
        self.obstacle_detected = False
        self.obstacle_clear_count = 0
        self.battery_percent = 100.0

        # ── Control ─────────────────────────────────────────────────
        self.wp_reached_radius = 0.6
        self.max_speed = 0.8
        self.transit_speed = 1.2   # faster between structures
        self.approach_slow_dist = 2.0

        # ── Build Mission ───────────────────────────────────────────
        self.mission_phases = self._build_mission()
        self.current_phase_idx = 0
        self.current_wp = 0
        self.phase = 'WAITING'
        self.phase_start_time = time.time()
        self.mission_start_time = None

        self.waypoints_completed = 0
        self.total_waypoints = sum(
            len(p['waypoints']) for p in self.mission_phases)

        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(
            f'Mission: {len(self.mission_phases)} phases, '
            f'{self.total_waypoints} waypoints')

    # ═══════════════════════════════════════════════════════════════
    # MISSION BUILDER
    # ═══════════════════════════════════════════════════════════════

    def _build_mission(self):
        return [
            # ── TOWER INSPECTION ──
            {
                'name': 'TAKEOFF',
                'waypoints': [
                    (0.0, -6.0, self.CRUISE_ALT),
                    (0.0, -4.0, self.CRUISE_ALT),
                ],
                'description': 'Ascending from helipad',
                'speed': self.max_speed,
            },
            {
                'name': 'TOWER_LOWER',
                'waypoints': self._orbit(cx=0, cy=0, alt=3.0, r=4.0, n=10),
                'description': 'Inspecting tower lower section (concrete)',
                'speed': 0.6,
            },
            {
                'name': 'TOWER_UPPER',
                'waypoints': self._orbit(cx=0, cy=0, alt=8.5, r=3.5, n=10),
                'description': 'Inspecting tower upper section (steel)',
                'speed': 0.6,
            },
            {
                'name': 'TOWER_TOP',
                'waypoints': self._orbit(cx=0, cy=0, alt=12.0, r=3.0, n=8),
                'description': 'Scanning antenna and top cap',
                'speed': 0.5,
            },

            # ── TRANSIT TO BRIDGE ──
            {
                'name': 'TRANSIT_BRIDGE',
                'waypoints': [
                    (5.0, 0.0, self.CRUISE_ALT),
                    (14.0, 0.0, self.CRUISE_ALT),
                ],
                'description': 'Transiting to bridge',
                'speed': self.transit_speed,
            },

            # ── BRIDGE INSPECTION ──
            {
                'name': 'BRIDGE_INSPECT',
                'waypoints': [
                    # Fly along left side at deck level
                    (16.0, -2.5, 3.5),
                    (20.0, -2.5, 3.5),
                    (24.0, -2.5, 3.5),
                    # Cross over to right side
                    (24.0, 2.5, 3.5),
                    # Fly back along right side
                    (20.0, 2.5, 3.5),
                    (16.0, 2.5, 3.5),
                    # Underside pass (below deck level)
                    (16.0, 0.0, 1.5),
                    (20.0, 0.0, 1.5),
                    (24.0, 0.0, 1.5),
                    # Rise back up
                    (24.0, 0.0, self.CRUISE_ALT),
                ],
                'description': 'Inspecting bridge deck, trusses, and underside',
                'speed': 0.5,
            },

            # ── TRANSIT TO PIPELINE ──
            {
                'name': 'TRANSIT_PIPELINE',
                'waypoints': [
                    (10.0, 0.0, self.CRUISE_ALT),
                    (0.0, 5.0, self.CRUISE_ALT),
                    (-9.0, 8.0, self.CRUISE_ALT),
                ],
                'description': 'Transiting to pipeline rack',
                'speed': self.transit_speed,
            },

            # ── PIPELINE INSPECTION ──
            {
                'name': 'PIPELINE_INSPECT',
                'waypoints': [
                    # Fly along pipeline from one end to the other
                    (-15.0, 9.5, 3.5),
                    (-12.0, 9.5, 3.5),
                    (-9.0, 9.5, 3.5),
                    # Return along other side at pipe level
                    (-9.0, 6.5, 2.8),
                    (-12.0, 6.5, 2.8),
                    (-15.0, 6.5, 2.8),
                    # Top-down pass
                    (-12.0, 8.0, 5.0),
                ],
                'description': 'Inspecting pipeline rack and pipe surfaces',
                'speed': 0.5,
            },

            # ── TRANSIT TO POWER LINES ──
            {
                'name': 'TRANSIT_POWERLINE',
                'waypoints': [
                    (-10.0, 4.0, self.CRUISE_ALT),
                    (-10.0, -8.0, self.CRUISE_ALT),
                ],
                'description': 'Transiting to power line corridor',
                'speed': self.transit_speed,
            },

            # ── POWER LINE INSPECTION ──
            {
                'name': 'POWERLINE_INSPECT',
                'waypoints': [
                    # Fly along the power line corridor (3 poles)
                    (-6.0, -12.0, 6.0),
                    (-6.0, -6.0, 7.0),
                    (-6.0, 0.0, 7.5),    # at pole 2, near wire height
                    (-6.0, 6.0, 7.0),
                    (-6.0, 12.0, 6.0),   # at pole 3
                    # Return pass closer to poles
                    (-8.5, 12.0, 5.0),
                    (-8.5, 0.0, 5.0),
                    (-8.5, -12.0, 5.0),
                ],
                'description': 'Inspecting power poles, wires, and insulators',
                'speed': 0.5,
            },

            # ── RETURN TO LAUNCH ──
            {
                'name': 'RETURN_TO_LAUNCH',
                'waypoints': [
                    (-5.0, -8.0, self.CRUISE_ALT),
                    (0.0, -6.0, self.CRUISE_ALT),
                ],
                'description': 'Returning to helipad',
                'speed': self.transit_speed,
            },

            # ── LAND ──
            {
                'name': 'LAND',
                'waypoints': [(0.0, -6.0, 0.3)],
                'description': 'Landing on helipad',
                'speed': 0.3,
            },
        ]

    def _orbit(self, cx, cy, alt, r, n):
        """Generate circular orbit waypoints."""
        wps = []
        for i in range(n + 1):
            angle = -math.pi/2 + (2 * math.pi * i / n)
            wps.append((cx + r * math.cos(angle), cy + r * math.sin(angle), alt))
        return wps

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
                f'✓ Pose locked ({self.x:.1f}, {self.y:.1f}, {self.z:.1f})')
            self.get_logger().info('━━━ MISSION START ━━━')
            self._log_phase()

    def scan_callback(self, msg):
        valid = [r for r in msg.ranges if 0.5 < r < float('inf')]
        if valid and min(valid) < 1.5:
            self.obstacle_detected = True
            self.obstacle_clear_count = 0
        else:
            self.obstacle_clear_count += 1
            if self.obstacle_clear_count > 5:
                self.obstacle_detected = False

    def battery_callback(self, msg):
        data = json.loads(msg.data)
        self.battery_percent = data['percentage']

    # ═══════════════════════════════════════════════════════════════
    # CONTROL LOOP
    # ═══════════════════════════════════════════════════════════════

    def control_loop(self):
        cmd = Twist()

        if self.phase == 'WAITING':
            self.cmd_pub.publish(cmd)
            return

        if self.phase == 'COMPLETE':
            self.cmd_pub.publish(cmd)
            return

        if self.obstacle_detected:
            self.cmd_pub.publish(cmd)
            self.get_logger().warn(
                '⚠ Obstacle — holding.', throttle_duration_sec=2.0)
            return

        mission = self.mission_phases[self.current_phase_idx]
        wps = mission['waypoints']
        speed = mission.get('speed', self.max_speed)

        if self.current_wp >= len(wps):
            self._advance_phase()
            self.cmd_pub.publish(cmd)
            return

        tx, ty, tz = wps[self.current_wp]
        dx, dy, dz = tx - self.x, ty - self.y, tz - self.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)

        if dist < self.wp_reached_radius:
            self.waypoints_completed += 1
            self.get_logger().info(
                f'  ✓ WP {self.current_wp+1}/{len(wps)} '
                f'({tx:.1f}, {ty:.1f}, {tz:.1f})')
            self.current_wp += 1
            self.cmd_pub.publish(cmd)
            self._publish_status()
            return

        # Proportional speed control with approach deceleration
        scale = min(1.0, dist / self.approach_slow_dist)
        spd = speed * max(scale, 0.15)

        cmd.linear.x = (dx / dist) * spd
        cmd.linear.y = (dy / dist) * spd
        cmd.linear.z = (dz / dist) * spd
        self.cmd_pub.publish(cmd)

        progress = (self.waypoints_completed / self.total_waypoints) * 100
        self.get_logger().info(
            f'→ [{mission["name"]}] dist {dist:.1f}m | '
            f'batt {self.battery_percent:.0f}% | '
            f'progress {progress:.0f}%',
            throttle_duration_sec=3.0)

    # ═══════════════════════════════════════════════════════════════
    # PHASE MANAGEMENT
    # ═══════════════════════════════════════════════════════════════

    def _advance_phase(self):
        elapsed = time.time() - self.phase_start_time
        name = self.mission_phases[self.current_phase_idx]['name']
        self.get_logger().info(f'━━ [{name}] done ({elapsed:.1f}s) ━━')

        self.current_phase_idx += 1
        self.current_wp = 0

        if self.current_phase_idx >= len(self.mission_phases):
            self.phase = 'COMPLETE'
            total = time.time() - self.mission_start_time
            self.get_logger().info('━' * 45)
            self.get_logger().info('  ✓ FULL INSPECTION MISSION COMPLETE')
            self.get_logger().info(f'  Time: {total:.0f}s | '
                f'WPs: {self.waypoints_completed}/{self.total_waypoints} | '
                f'Battery: {self.battery_percent:.0f}%')
            self.get_logger().info(
                '  Structures inspected: Tower, Bridge, Pipeline, Power Lines')
            self.get_logger().info('━' * 45)
            self._publish_status()
            return

        self.phase = self.mission_phases[self.current_phase_idx]['name']
        self.phase_start_time = time.time()
        self._log_phase()

    def _log_phase(self):
        m = self.mission_phases[self.current_phase_idx]
        self.get_logger().info(
            f'▶ Phase {self.current_phase_idx+1}/{len(self.mission_phases)}: '
            f'{m["name"]} ({len(m["waypoints"])} WPs)')
        self.get_logger().info(f'  {m["description"]}')

    def _publish_status(self):
        msg = String()
        progress = (self.waypoints_completed / self.total_waypoints) * 100
        elapsed = time.time() - (self.mission_start_time or time.time())
        msg.data = json.dumps({
            'phase': self.phase,
            'phase_index': self.current_phase_idx,
            'total_phases': len(self.mission_phases),
            'waypoints_completed': self.waypoints_completed,
            'total_waypoints': self.total_waypoints,
            'progress_percent': round(progress, 1),
            'elapsed_seconds': round(elapsed, 1),
            'battery_percent': round(self.battery_percent, 1),
            'position': {'x': round(self.x, 2), 'y': round(self.y, 2),
                         'z': round(self.z, 2)},
        })
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
