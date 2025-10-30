import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from youbot_interfaces.srv import NavigateToObject
import math

class GoalSendService(Node):
    def __init__(self):
        super().__init__('goal_send_service')

        self.waypoint_dict = {
            'home': [0.0, 0.0, 0.0],
            'ball': [6.3, -5.0, 0.0],
            'apple': [6.5, 4.2, 0.85],
            'duck': [-3.7, -7.7, 0.0],
        }

        self.srv = self.create_service(NavigateToObject, 
                                    'navigate_to_object',
                                    self.navigate_to_object_callback)

        self._action_client = ActionClient(self, 
                                           NavigateToPose, 
                                           'navigate_to_pose')
        
        self.get_logger().info('Named Navigation Service ready!')
        self.get_logger().info(f'Waypoints: {list(self.waypoint_dict.keys())}')

    def navigate_to_object_callback(self, request, response):
        name = request.waypoint_name
        if name not in self.waypoint_dict:
            response.success = False
            response.message = f'Unknown waypoint: {name}'
            self.get_logger().error(response.message)
            return response
        
        x, y, theta = self.waypoint_dict[name]

        response.pose.header.frame_id = 'map'
        response.pose.header.stamp = self.get_clock().now().to_msg()
        response.pose.pose.position.x = x
        response.pose.pose.position.y = y
        response.pose.pose.orientation.z = math.sin(theta / 2.0)
        response.pose.pose.orientation.w = math.cos(theta / 2.0)

        if self.send_goal(x, y, theta, name):
            response.success = True
            response.message = f'Navigating to {name}'
            self.get_logger().info(response.message)
        else:
            response.success = False
            response.message = f'Failed to send goal to {name}'
            self.get_logger().error(response.message)
        
        return response
    
    def send_goal(self, x, y, theta, name):
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return False
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        self.get_logger().info(f'Goal "{name}": [{x:.2f}, {y:.2f}, {theta:.2f}]')
        self._action_client.send_goal_async(goal_msg)
    
        return True
    
def main(args=None):
    rclpy.init(args=args)
    service = GoalSendService()
    rclpy.spin(service)
    service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()