import rclpy
from gazebo_msgs.srv import SetEntityState

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('test_state_node')
    client = node.create_client(SetEntityState, '/gazebo/set_entity_state')
    
    if not client.wait_for_service(timeout_sec=5.0):
        print("Service not available")
        return
        
    req = SetEntityState.Request()
    req.state.name = 'simple_drone'
    req.state.pose.position.z = 5.0
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    print(f"Success: {response.success}, Message: {response.status_message}")

if __name__ == '__main__':
    main()
