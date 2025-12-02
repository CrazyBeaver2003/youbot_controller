import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
import numpy as np
import math
from collections import deque

class FrontierExplorer(Node):
    def __init__(self):
        super().__init__('frontier_explorer')
        
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription  # prevent unused variable warning

        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.map_data = None
        self.map_info = None
        self.frontiers = []

    def map_callback(self, msg):
        self.get_logger().info('Received map update')
        self.map_info = msg.info
        # Reshape to (height, width)
        # OccupancyGrid data is row-major (y, x)
        self.map_data = np.array(msg.data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    def find_frontiers(self):
        if self.map_data is None:
            self.get_logger().warn('No map received yet')
            return []

        # Constants
        UNKNOWN = -1
        FREE = 0
        
        # Create masks
        # We are looking for FREE cells adjacent to UNKNOWN cells
        unknown_mask = (self.map_data == UNKNOWN)
        free_mask = (self.map_data == FREE)
        
        # Dilate unknown mask to find boundaries
        # Using numpy slicing to simulate 4-connectivity dilation
        dilated_unknown = np.zeros_like(unknown_mask)
        dilated_unknown[:-1, :] |= unknown_mask[1:, :]  # Shift up
        dilated_unknown[1:, :]  |= unknown_mask[:-1, :] # Shift down
        dilated_unknown[:, :-1] |= unknown_mask[:, 1:]  # Shift left
        dilated_unknown[:, 1:]  |= unknown_mask[:, :-1] # Shift right
        
        # Frontiers are free cells that touch unknown cells
        frontier_mask = free_mask & dilated_unknown
        
        # Get indices of frontier points
        frontier_indices = np.argwhere(frontier_mask)
        
        if len(frontier_indices) == 0:
            return []

        # Cluster frontiers
        clusters = self.cluster_frontiers(frontier_indices)
        
        # Calculate centroids
        centroids = []
        for cluster in clusters:
            if len(cluster) < 5: # Filter out small noise
                continue
            
            # Average of indices (r, c)
            centroid = np.mean(cluster, axis=0)
            centroids.append(centroid)
            
        return centroids

    def cluster_frontiers(self, points, distance_threshold=2.0):
        # Simple clustering using BFS/Euclidean distance
        # points is a list of [row, col]
        
        # Since we are on a grid, adjacent pixels have distance 1 or sqrt(2)
        # We can use a simple adjacency check for clustering
        
        clusters = []
        visited = set()
        
        # Convert to set of tuples for fast lookup
        point_set = set(map(tuple, points))
        
        for point in points:
            pt_tuple = tuple(point)
            if pt_tuple in visited:
                continue
            
            # Start new cluster
            cluster = []
            queue = deque([pt_tuple])
            visited.add(pt_tuple)
            
            while queue:
                curr = queue.popleft()
                cluster.append(curr)
                
                # Check 8-neighbors
                r, c = curr
                neighbors = [
                    (r+1, c), (r-1, c), (r, c+1), (r, c-1),
                    (r+1, c+1), (r-1, c-1), (r+1, c-1), (r-1, c+1)
                ]
                
                for nr, nc in neighbors:
                    if (nr, nc) in point_set and (nr, nc) not in visited:
                        visited.add((nr, nc))
                        queue.append((nr, nc))
            
            clusters.append(cluster)
            
        return clusters

    def get_next_frontier(self):
        centroids = self.find_frontiers()
        if not centroids:
            return None
            
        # Strategy: Pick the closest frontier to the robot?
        # For simplicity, let's pick the largest cluster or just the first one found.
        # Or the one closest to the map origin if we don't have robot pose here.
        # Ideally we should transform to map coordinates.
        
        # Let's convert the first centroid to world coordinates
        # Centroid is (row, col) -> (y, x) in grid
        
        # Just picking the first one for now
        target_centroid = centroids[0]
        
        return self.grid_to_world(target_centroid)

    def grid_to_world(self, grid_point):
        # grid_point is (row, col) -> (y, x)
        r, c = grid_point
        
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        resolution = self.map_info.resolution
        
        # x = c * res + origin_x
        # y = r * res + origin_y
        
        x = c * resolution + origin_x
        y = r * resolution + origin_y
        
        return (x, y)

    def send_goal(self, x, y):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0 # No rotation
        
        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending goal to ({x}, {y})')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    explorer = FrontierExplorer()
    
    # Spin in a separate thread or just spin once to get map
    # For testing, we can just spin
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        pass
    finally:
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
