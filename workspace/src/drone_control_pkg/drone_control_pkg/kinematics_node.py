import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import math

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.get_logger().info('Kinematics Node started')
        
        # Subscribe to /cmd_vel (global-frame velocities)
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # Gazebo service client
        self.client = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # Publish authoritative pose so navigation_node can read it directly
        self.pose_pub = self.create_publisher(PoseStamped, '/drone/pose', 10)
        
        # Internal state (starting pose — matches inspection.world)
        self.x = 0.0
        self.y = -5.0
        self.z = 1.0
        self.yaw = 0.0
        
        # Current velocity commands
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        
        # Velocity timeout: if no cmd_vel received in 0.5s, stop
        self.last_cmd_time = self.get_clock().now()
        self.cmd_timeout = 0.5  # seconds
        
        # Update rate: 20 Hz
        self.dt = 0.05
        self.timer = self.create_timer(self.dt, self.update_loop)
        
        self.service_ready = False

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.vz = msg.linear.z
        self.last_cmd_time = self.get_clock().now()

    def euler_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion (roll=pitch=0)."""
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return (0.0, 0.0, qz, qw)

    def update_loop(self):
        # Check for velocity timeout — auto-stop if no commands
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            self.vx = 0.0
            self.vy = 0.0
            self.vz = 0.0
        
        # Update position (global frame — no rotation needed)
        self.x += self.vx * self.dt
        self.y += self.vy * self.dt
        self.z += self.vz * self.dt
        
        # Face direction of travel
        if abs(self.vx) > 0.05 or abs(self.vy) > 0.05:
            self.yaw = math.atan2(self.vy, self.vx)

        # Geofencing: keep drone within safe bounds
        self.x = max(-15.0, min(15.0, self.x))
        self.y = max(-15.0, min(15.0, self.y))
        self.z = max(0.1, min(20.0, self.z))

        # === Publish pose for navigation_node ===
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'world'
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        pose_msg.pose.position.z = self.z
        qx, qy, qz, qw = self.euler_to_quaternion(self.yaw)
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.pose_pub.publish(pose_msg)

        # === Update Gazebo visualization ===
        if not self.service_ready:
            if self.client.service_is_ready():
                self.service_ready = True
                self.get_logger().info('Connected to Gazebo set_entity_state service')
            else:
                return  # Skip Gazebo update until service is ready
        
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

    def _service_cb(self, future):
        try:
            resp = future.result()
            if not resp.success:
                self.get_logger().error(f'Gazebo rejected move: {resp.status_message}',
                                       throttle_duration_sec=2.0)
        except Exception as e:
            self.get_logger().error(f'Service error: {e}', throttle_duration_sec=2.0)

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
