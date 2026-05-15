import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import math

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.get_logger().info('Navigation Node started')
        
        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscribe to authoritative pose from kinematics_node (NOT gazebo/model_states)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/drone/pose', self.pose_callback, 10)
        
        # Subscribe to LiDAR for obstacle avoidance
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Flight states
        self.STATE_WAITING = 'WAITING'      # Waiting for pose data
        self.STATE_FLYING = 'FLYING'        # Following waypoints
        self.STATE_AVOIDING = 'AVOIDING'    # Obstacle detected
        self.STATE_COMPLETE = 'COMPLETE'    # Mission done
        
        self.state = self.STATE_WAITING
        
        # Inspection waypoints: orbit the tower (at origin) at 4m radius, altitude 5m
        # Tower is at (0, 0, z=0..10), drone spawns at (0, -5, 1)
        radius = 4.0
        alt = 5.0
        n_orbit_points = 8  # 8 points for a smooth orbit
        
        self.waypoints = []
        # Step 1: Takeoff straight up
        self.waypoints.append((0.0, -5.0, alt))
        # Step 2: Fly to first orbit point
        self.waypoints.append((0.0, -radius, alt))
        # Step 3: Orbit the tower (circle of 8 points)
        for i in range(n_orbit_points + 1):
            angle = -math.pi/2 + (2 * math.pi * i / n_orbit_points)
            wx = radius * math.cos(angle)
            wy = radius * math.sin(angle)
            self.waypoints.append((wx, wy, alt))
        # Step 4: Return to launch
        self.waypoints.append((0.0, -5.0, alt))
        # Step 5: Land
        self.waypoints.append((0.0, -5.0, 0.5))
        
        self.current_wp = 0
        
        # Drone position (updated from kinematics_node)
        self.x = 0.0
        self.y = -5.0
        self.z = 1.0
        self.pose_received = False
        
        # Obstacle avoidance
        self.obstacle_detected = False
        self.obstacle_clear_count = 0
        
        # Control parameters
        self.wp_reached_radius = 0.4    # meters — close enough to waypoint
        self.max_speed = 0.8            # m/s max velocity
        self.approach_gain = 0.5        # P-controller gain
        self.approach_slow_dist = 2.0   # start slowing down within this distance
        
        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f'Mission: {len(self.waypoints)} waypoints loaded')

    def pose_callback(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.z = msg.pose.position.z
        if not self.pose_received:
            self.pose_received = True
            self.state = self.STATE_FLYING
            self.get_logger().info(
                f'Pose locked: ({self.x:.1f}, {self.y:.1f}, {self.z:.1f}) — Starting mission!')

    def scan_callback(self, msg):
        # Filter out the drone's own body (< 0.5m) and inf values
        valid = [r for r in msg.ranges if 0.5 < r < float('inf')]
        if valid and min(valid) < 1.5:
            self.obstacle_detected = True
            self.obstacle_clear_count = 0
        else:
            # Require several consecutive "clear" readings before resuming
            self.obstacle_clear_count += 1
            if self.obstacle_clear_count > 5:
                self.obstacle_detected = False

    def control_loop(self):
        cmd = Twist()
        
        # === STATE: WAITING ===
        if self.state == self.STATE_WAITING:
            self.cmd_pub.publish(cmd)  # zero velocity
            return
        
        # === STATE: COMPLETE ===
        if self.state == self.STATE_COMPLETE:
            self.cmd_pub.publish(cmd)  # zero velocity
            self.get_logger().info('Mission Complete — hovering.', throttle_duration_sec=10.0)
            return
        
        # === STATE: AVOIDING ===
        if self.obstacle_detected and self.state == self.STATE_FLYING:
            self.cmd_pub.publish(cmd)  # hover in place
            self.get_logger().warn('Obstacle nearby — holding position.',
                                   throttle_duration_sec=2.0)
            return
        
        # === STATE: FLYING ===
        if self.current_wp >= len(self.waypoints):
            self.state = self.STATE_COMPLETE
            self.cmd_pub.publish(cmd)
            self.get_logger().info('All waypoints reached — Mission Complete!')
            return
        
        # Current target
        tx, ty, tz = self.waypoints[self.current_wp]
        dx = tx - self.x
        dy = ty - self.y
        dz = tz - self.z
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Check if waypoint reached
        if dist < self.wp_reached_radius:
            self.get_logger().info(
                f'✓ WP {self.current_wp}/{len(self.waypoints)-1} reached '
                f'({tx:.1f}, {ty:.1f}, {tz:.1f})')
            self.current_wp += 1
            # Send zero velocity briefly to prevent overshoot
            self.cmd_pub.publish(cmd)
            return
        
        # Compute velocity: proportional control with approach deceleration
        # As we get close, reduce speed proportionally
        speed_scale = min(1.0, dist / self.approach_slow_dist)
        speed = self.max_speed * max(speed_scale, 0.15)  # min 15% speed near waypoint
        
        # Normalize direction and scale to desired speed
        cmd.linear.x = (dx / dist) * speed
        cmd.linear.y = (dy / dist) * speed
        cmd.linear.z = (dz / dist) * speed
        
        self.cmd_pub.publish(cmd)
        
        # Periodic status log
        self.get_logger().info(
            f'→ WP{self.current_wp} ({tx:.0f},{ty:.0f},{tz:.0f}) | '
            f'pos ({self.x:.1f},{self.y:.1f},{self.z:.1f}) | '
            f'dist {dist:.1f}m | spd {speed:.2f}m/s',
            throttle_duration_sec=3.0)

def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
