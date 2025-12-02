#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy

from geometry_msgs.msg import Pose, Point, Twist, PoseStamped
from std_msgs.msg import Float64, String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from youbot_interfaces.msg import ObjectLocation

import smach
import math
import time
import random
import numpy as np
import tf2_ros
import tf2_geometry_msgs # Import is needed for do_transform_pose
import json
import os
import subprocess

# =============================================================================
# STATES
# =============================================================================

class ExploreState(smach.State):
    """
    Drives the robot using Frontier Exploration.
    """
    def __init__(self, node):
        super().__init__(outcomes=['succeeded', 'aborted', 'preempted'])
        self.node = node
        self.visited_frontiers = []
        self.unreachable_frontiers = []

    def execute(self, userdata):
        self.node.get_logger().info("Starting Frontier Exploration...")
        
        while rclpy.ok():
            if self.node.latest_map is None:
                self.node.get_logger().info("Waiting for map data...", throttle_duration_sec=2.0)
                time.sleep(1.0)
                continue
                
            # Find Frontiers
            frontier = self.get_next_frontier()
            
            if frontier is None:
                self.node.get_logger().info("No frontiers found. Exploration complete.")
                self.node.save_map_and_objects()
                self.node.stop_robot()
                return 'succeeded'
                
            target_x, target_y = frontier
            self.node.get_logger().info(f"Exploration: Going to frontier ({target_x:.2f}, {target_y:.2f})")
            
            # Send Goal
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = target_x
            goal.pose.pose.position.y = target_y
            goal.pose.pose.orientation.w = 1.0
            
            if not self.node.nav_action_client.wait_for_server(timeout_sec=5.0):
                self.node.get_logger().error("Nav2 action server not available!")
                return 'aborted'
                
            send_goal_future = self.node.nav_action_client.send_goal_async(goal)
            
            # Wait for goal acceptance
            while not send_goal_future.done():
                time.sleep(0.1)
            
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.node.get_logger().info("Goal rejected, trying new point...")
                continue
            
            # Wait for result
            result_future = goal_handle.get_result_async()
            
            while not result_future.done():
                time.sleep(0.5)
            
            # Check result
            status = result_future.result().status
            if status == 4: # SUCCEEDED
                 self.node.get_logger().info("Frontier reached. Spinning to scan...")
                 self.node.spin_360()
                 self.visited_frontiers.append((target_x, target_y))
            else:
                 self.node.get_logger().info("Failed to reach frontier.")
                 self.unreachable_frontiers.append((target_x, target_y))
            
            time.sleep(0.5)
            
        return 'aborted'

    def get_next_frontier(self):
        # Get map data
        grid = self.node.latest_map
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        
        # Convert to numpy
        data = np.array(grid.data, dtype=np.int8).reshape((height, width))
        
        # Constants
        UNKNOWN = -1
        FREE = 0
        OCCUPIED = 100
        
        # Find free cells
        free_mask = (data == FREE)
        unknown_mask = (data == UNKNOWN)
        
        # Find boundaries (Free cells adjacent to Unknown)
        # We can use simple shifting or convolution
        # Dilate unknown mask by 1 pixel
        # A free cell is a frontier if it touches an unknown cell
        
        # Shift unknown mask in 4 directions
        unknown_up = np.roll(unknown_mask, -1, axis=0)
        unknown_down = np.roll(unknown_mask, 1, axis=0)
        unknown_left = np.roll(unknown_mask, -1, axis=1)
        unknown_right = np.roll(unknown_mask, 1, axis=1)
        
        # Combine (logical OR)
        unknown_neighbors = unknown_up | unknown_down | unknown_left | unknown_right
        
        # Frontier is Free AND has Unknown Neighbor
        frontier_mask = free_mask & unknown_neighbors
        
        # Get coordinates of frontier points
        y_indices, x_indices = np.where(frontier_mask)
        
        if len(x_indices) == 0:
            return None
            
        # Safety Check: Filter out frontiers too close to obstacles
        # Radius in pixels (e.g., 0.5m / 0.05 = 10 pixels)
        # Increased safety radius to 0.7m to avoid getting too close to walls
        safety_radius = int(0.7 / resolution) 
        
        # Optimization: Instead of checking every point, we can use a mask
        # But for simplicity, let's just check the candidates we are about to pick
        
        # Convert to world coordinates
        candidates = []
        
        # Pre-calculate occupied indices for faster check? No, too slow.
        # Just check the window around the point in the grid
        
        for x, y in zip(x_indices, y_indices):
            # Check safety window
            x_min = max(0, x - safety_radius)
            x_max = min(width, x + safety_radius)
            y_min = max(0, y - safety_radius)
            y_max = min(height, y + safety_radius)
            
            window = data[y_min:y_max, x_min:x_max]
            if np.any(window == OCCUPIED):
                continue # Skip unsafe frontier
            
            wx = x * resolution + origin_x
            wy = y * resolution + origin_y
            
            # Check if visited (within 0.8m)
            is_visited = False
            for vx, vy in self.visited_frontiers:
                dist = math.hypot(wx - vx, wy - vy)
                if dist < 0.8: # Reduced from 2.0m
                    is_visited = True
                    break
            
            if is_visited:
                continue

            # Check if unreachable (within 0.5m)
            for ux, uy in self.unreachable_frontiers:
                dist = math.hypot(wx - ux, wy - uy)
                if dist < 0.5:
                    is_visited = True
                    break
            
            if not is_visited:
                candidates.append((wx, wy))
        
        if not candidates:
            # If all visited or unsafe, clear visited list and try again (maybe map changed)
            if len(self.visited_frontiers) > 0:
                self.node.get_logger().info("All candidates visited. Clearing visited list to re-check.")
                self.visited_frontiers = []
                # Recursion might be dangerous if no safe frontiers exist at all
                # Let's just return None and wait
                return None
            return None
            
        # Sort candidates by distance to robot to explore closest areas first (Room completion)
        try:
            # Get robot pose
            trans = self.node.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            rx = trans.transform.translation.x
            ry = trans.transform.translation.y
            
            # Sort by distance
            candidates.sort(key=lambda p: math.hypot(p[0]-rx, p[1]-ry))
            
            # Pick one of the closest (e.g. top 10) to add some randomness/robustness
            # or just the closest
            best_candidate = candidates[0]
            
            return best_candidate
            
        except Exception as e:
            self.node.get_logger().warn(f"Could not get robot pose: {e}")
            return random.choice(candidates)

class WaitState(smach.State):
    """
    Waits for a command on /mission_command topic.
    """
    def __init__(self, node):
        super().__init__(outcomes=['succeeded', 'aborted'], output_keys=['target_object'])
        self.node = node
        self.command_received = None

    def execute(self, userdata):
        self.node.get_logger().info("Waiting for command (publish to /mission_command)...")
        self.node.latest_command = None # Reset
        
        while rclpy.ok():
            if self.node.latest_command:
                cmd = self.node.latest_command
                if cmd in self.node.found_objects:
                    self.node.get_logger().info(f"Command received: {cmd}")
                    userdata.target_object = cmd
                    return 'succeeded'
                else:
                    self.node.get_logger().warn(f"Unknown object or not found yet: {cmd}")
                    self.node.latest_command = None
            
            time.sleep(0.5)
        return 'aborted'

class NavigateToTargetState(smach.State):
    """
    Uses Nav2 to go to the object's saved location.
    Strategy:
    1. Go to the 'Sighting Pose' (where robot first saw the object).
    2. Go to the 'Approach Pose' (0.4m from object).
    """
    def __init__(self, node):
        super().__init__(outcomes=['succeeded', 'aborted'], input_keys=['target_object'])
        self.node = node

    def execute(self, userdata):
        target_name = userdata.target_object
        if target_name not in self.node.found_objects:
            self.node.get_logger().error(f"Object {target_name} not found!")
            return 'aborted'
            
        info = self.node.found_objects[target_name]
        sighting_pose = info['robot']
        
        # 1. Navigate to Sighting Pose ONLY
        # The user requested to avoid navigating "through points" to the object.
        # We go to where we first saw it, then switch to visual servoing.
        self.node.get_logger().info(f"Navigating to Sighting Pose for {target_name}...")
        if not self.navigate_to_pose(sighting_pose):
            self.node.get_logger().warn("Failed to reach sighting pose, but will try to approach anyway.")
            
        return 'succeeded'

    def navigate_to_pose(self, pose_stamped):
        goal = NavigateToPose.Goal()
        goal.pose = pose_stamped
        
        self.node.nav_action_client.wait_for_server()
        future = self.node.nav_action_client.send_goal_async(goal)
        
        while not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected")
            return False
        
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.1)
            
        result = res_future.result()
        return result.status == 4 # SUCCEEDED

class ApproachObjectState(smach.State):
    """
    Approaches the object using live coordinates.
    If object is lost, spins to find it.
    Stops when distance < 0.6m.
    """
    def __init__(self, node):
        super().__init__(outcomes=['succeeded', 'aborted'], input_keys=['target_object'], output_keys=['object_pose'])
        self.node = node

    def execute(self, userdata):
        target_name = userdata.target_object
        self.node.get_logger().info(f"Approaching {target_name}...")
        
        # State: 'SEARCHING' or 'APPROACHING'
        state = 'SEARCHING'
        start_time = time.time()
        
        while rclpy.ok():
            if time.time() - start_time > 120.0:
                self.node.get_logger().warn("Approach timed out!")
                self.node.stop_robot()
                return 'aborted'

            # 1. Check visibility
            is_visible = False
            current_msg = None
            
            if target_name in self.node.latest_detections:
                msg, rx_time = self.node.latest_detections[target_name]
                
                # Check freshness based on reception time (wall clock)
                # This avoids issues with Simulation Time vs System Time
                if (time.time() - rx_time) < 1.0:
                    is_visible = True
                    current_msg = msg
                else:
                    if int(time.time()) % 2 == 0:
                        self.node.get_logger().warn(f"Stale detection for {target_name} (age: {time.time() - rx_time:.1f}s)")

            # 2. State Machine Logic
            if state == 'SEARCHING':
                if is_visible:
                    self.node.get_logger().info(f"Target {target_name} found! Switching to APPROACH.")
                    self.node.stop_robot()
                    state = 'APPROACHING'
                else:
                    # Spin to find
                    cmd = Twist()
                    cmd.angular.z = 0.5
                    self.node.cmd_vel_pub.publish(cmd)
                    time.sleep(0.1)

            elif state == 'APPROACHING':
                if not is_visible:
                    self.node.get_logger().warn(f"Target {target_name} lost! Switching to SEARCH.")
                    self.node.stop_robot()
                    state = 'SEARCHING'
                    continue
                
                # Calculate distance using x and y from the message (base_link frame)
                x = current_msg.pose.position.x
                y = current_msg.pose.position.y
                
                dist = math.hypot(x, y)
                yaw_err = math.atan2(y, x)
                
                if int(time.time()) % 2 == 0:
                    self.node.get_logger().info(f"Dist={dist:.3f}, Yaw={yaw_err:.3f}")

                # Stop condition
                if dist < 0.45:
                    self.node.stop_robot()
                    self.node.get_logger().info(f"Reached target distance (dist={dist:.3f}). Stabilizing...")
                    state = 'STABILIZING'
                    stabilize_start = time.time()
                    continue
                
                # Control
                cmd = Twist()
                
                # Rotate to face object
                cmd.angular.z = min(max(yaw_err * 2.0, -0.5), 0.5)
                
                # Move forward if roughly aligned
                if abs(yaw_err) < 0.2:
                    cmd.linear.x = 0.15
                else:
                    cmd.linear.x = 0.0
                
                self.node.cmd_vel_pub.publish(cmd)
                time.sleep(0.05)

            elif state == 'STABILIZING':
                if not is_visible:
                    self.node.get_logger().warn("Lost object during stabilization! Switching to SEARCH.")
                    state = 'SEARCHING'
                    continue

                if time.time() - stabilize_start > 1.5:
                    # Take the final measurement
                    self.node.get_logger().info(f"Stabilized. Final pose: x={current_msg.pose.position.x:.3f}, y={current_msg.pose.position.y:.3f}")
                    userdata.object_pose = current_msg.pose
                    return 'succeeded'
                
                # Keep robot stopped
                self.node.stop_robot()
                time.sleep(0.1)
                
        return 'aborted'

class ReturnHomeState(smach.State):
    def __init__(self, node):
        super().__init__(outcomes=['succeeded', 'aborted'])
        self.node = node

    def execute(self, userdata):
        self.node.get_logger().info("Returning Home (0,0)...")
        
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = 0.0
        goal.pose.pose.position.y = 0.0
        goal.pose.pose.orientation.w = 1.0
        
        self.node.nav_action_client.wait_for_server()
        future = self.node.nav_action_client.send_goal_async(goal)
        
        while not future.done():
            time.sleep(0.1)
            
        goal_handle = future.result()
        if not goal_handle.accepted:
            return 'aborted'
        
        res_future = goal_handle.get_result_async()
        while not res_future.done():
            time.sleep(0.1)
            
        result = res_future.result()
        if result.status == 4:
            return 'succeeded'
        else:
            return 'aborted'

class DriveForwardState(smach.State):
    """
    Drives the robot forward (or backward) for a specific distance using open-loop control.
    """
    def __init__(self, node, distance=0.15, speed=0.1):
        super().__init__(outcomes=['succeeded', 'aborted'])
        self.node = node
        self.distance = distance
        self.speed = speed

    def execute(self, userdata):
        direction = "forward" if self.distance > 0 else "backward"
        self.node.get_logger().info(f"Driving {direction} {abs(self.distance)}m...")
        
        duration = abs(self.distance) / abs(self.speed)
        cmd = Twist()
        cmd.linear.x = math.copysign(self.speed, self.distance)
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if not rclpy.ok(): return 'aborted'
            self.node.cmd_vel_pub.publish(cmd)
            time.sleep(0.1)
            
        self.node.stop_robot()
        return 'succeeded'

# Reuse Arm/Gripper states logic (simplified)
class SimpleArmState(smach.State):
    def __init__(self, node, action_type):
        super().__init__(outcomes=['succeeded', 'aborted'], input_keys=['object_pose'])
        self.node = node
        self.action_type = action_type # 'pickup', 'lift', 'release', 'home'
        self.ARM_LINK_OFFSET = {
            'x': 0.16,
            'y': 0.0,
            'z': 0.144 - 0.253
        }

    def execute(self, userdata):
        self.node.get_logger().info(f"Arm Action: {self.action_type}")
        
        target_pose = Pose()
        target_pose.orientation.w = 1.0
        
        if self.action_type == 'pickup':
            if hasattr(userdata, 'object_pose'):
                # Calculate target pose in arm frame
                obj_x = userdata.object_pose.position.x
                obj_y = userdata.object_pose.position.y
                obj_z = userdata.object_pose.position.z
                
                target_pose.position.x = obj_x - self.ARM_LINK_OFFSET['x'] + 0.02 # Add 3cm margin to reach deeper
                target_pose.position.y = obj_y - self.ARM_LINK_OFFSET['y']
                target_pose.position.z = obj_z - self.ARM_LINK_OFFSET['z'] - 0.01 # Lower by 2cm
                
                self.node.get_logger().info(f"Pickup Target: x={target_pose.position.x:.3f}, y={target_pose.position.y:.3f}, z={target_pose.position.z:.3f}")
            else:
                self.node.get_logger().warn("No object pose found in userdata, using default")
                target_pose.position.x = 0.28
                target_pose.position.y = 0.0
                target_pose.position.z = 0.00
        elif self.action_type == 'lift':
            target_pose.position.x = 0.2
            target_pose.position.y = 0.0
            target_pose.position.z = 0.3
        elif self.action_type == 'release':
            target_pose.position.x = 0.3
            target_pose.position.y = 0.0
            target_pose.position.z = 0.5 # High release for bucket
        elif self.action_type == 'home':
            target_pose.position.x = 0.1
            target_pose.position.y = 0.0
            target_pose.position.z = 0.4

        self.node.arm_target_publisher.publish(target_pose)
        time.sleep(5.0) # Wait longer for arm to settle
        return 'succeeded'

class SimpleGripperState(smach.State):
    def __init__(self, node, state):
        super().__init__(outcomes=['succeeded', 'aborted'])
        self.node = node
        self.state = state # 'open', 'close'

    def execute(self, userdata):
        self.node.get_logger().info(f"Gripper: {self.state}")
        msg = Float64()
        msg.data = 0.07 if self.state == 'open' else 0.025
        self.node.gripper_target_gap_publisher.publish(msg)
        time.sleep(1.0)
        return 'succeeded'

# =============================================================================
# MAIN NODE
# =============================================================================

class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        self.cb_group = ReentrantCallbackGroup()
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Data
        self.found_objects = {}
        self.latest_detections = {} # Stores the most recent message for each object
        self.target_objects = ['apple', 'ball', 'cube']
        self.latest_command = None
        self.latest_scan = None
        self.latest_map = None
        
        # Subs
        self.create_subscription(ObjectLocation, '/object_locations', self.object_callback, 10, callback_group=self.cb_group)
        self.create_subscription(String, '/mission_command', self.command_callback, 10, callback_group=self.cb_group)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10, callback_group=self.cb_group)
        
        # Map subscription with Transient Local durability to receive static maps
        map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos, callback_group=self.cb_group)
        
        # Pubs
        self.arm_target_publisher = self.create_publisher(Pose, 'arm_target_pose', 10)
        self.gripper_target_gap_publisher = self.create_publisher(Float64, 'gripper_target_gap', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Clients
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group)
        
        # Persistence
        self.json_file_path = os.path.expanduser('~/webots_ws/src/youbot_control/maps/objects.json')
        self.load_objects_from_file()
        
        self.get_logger().info("Mission Controller Initialized")

    def load_objects_from_file(self):
        if os.path.exists(self.json_file_path):
            try:
                with open(self.json_file_path, 'r') as f:
                    data = json.load(f)
                    for name, info in data.items():
                        # Reconstruct PoseStamped for Object
                        obj_ps = PoseStamped()
                        obj_ps.header.frame_id = 'map'
                        obj_ps.pose.position.x = info['object']['x']
                        obj_ps.pose.position.y = info['object']['y']
                        obj_ps.pose.position.z = info['object']['z']
                        obj_ps.pose.orientation.x = info['object']['qx']
                        obj_ps.pose.orientation.y = info['object']['qy']
                        obj_ps.pose.orientation.z = info['object']['qz']
                        obj_ps.pose.orientation.w = info['object']['qw']
                        
                        # Reconstruct PoseStamped for Robot (Sighting Pose)
                        robot_ps = PoseStamped()
                        robot_ps.header.frame_id = 'map'
                        robot_ps.pose.position.x = info['robot']['x']
                        robot_ps.pose.position.y = info['robot']['y']
                        robot_ps.pose.position.z = info['robot']['z']
                        robot_ps.pose.orientation.x = info['robot']['qx']
                        robot_ps.pose.orientation.y = info['robot']['qy']
                        robot_ps.pose.orientation.z = info['robot']['qz']
                        robot_ps.pose.orientation.w = info['robot']['qw']
                        
                        self.found_objects[name] = {'object': obj_ps, 'robot': robot_ps}
                        
                self.get_logger().info(f"Loaded {len(self.found_objects)} objects from file.")
            except Exception as e:
                self.get_logger().error(f"Failed to load objects: {e}")

    def save_objects_to_file(self):
        data = {}
        for name, info in self.found_objects.items():
            obj_ps = info['object']
            robot_ps = info['robot']
            
            data[name] = {
                'object': {
                    'x': obj_ps.pose.position.x,
                    'y': obj_ps.pose.position.y,
                    'z': obj_ps.pose.position.z,
                    'qx': obj_ps.pose.orientation.x,
                    'qy': obj_ps.pose.orientation.y,
                    'qz': obj_ps.pose.orientation.z,
                    'qw': obj_ps.pose.orientation.w
                },
                'robot': {
                    'x': robot_ps.pose.position.x,
                    'y': robot_ps.pose.position.y,
                    'z': robot_ps.pose.position.z,
                    'qx': robot_ps.pose.orientation.x,
                    'qy': robot_ps.pose.orientation.y,
                    'qz': robot_ps.pose.orientation.z,
                    'qw': robot_ps.pose.orientation.w
                }
            }
        
        try:
            os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
            with open(self.json_file_path, 'w') as f:
                json.dump(data, f, indent=4)
            self.get_logger().info("Objects saved to file.")
        except Exception as e:
            self.get_logger().error(f"Failed to save objects: {e}")

    def save_map_and_objects(self):
        # Save Objects
        self.save_objects_to_file()
        
        # Save Map
        map_dir = os.path.dirname(self.json_file_path)
        map_path = os.path.join(map_dir, 'my_map')
        
        self.get_logger().info(f"Saving map to {map_path}...")
        try:
            # Using map_saver_cli
            subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', map_path], check=True)
            self.get_logger().info("Map saved successfully!")
        except Exception as e:
            self.get_logger().error(f"Failed to save map: {e}")

    def map_callback(self, msg):
        self.latest_map = msg

    def scan_callback(self, msg):
        self.latest_scan = msg

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def spin_360(self):
        twist = Twist()
        twist.angular.z = 1.0
        # Spin for ~6.5 seconds (2*pi / 1.0 = 6.28)
        for _ in range(65):
            if not rclpy.ok(): break
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self.stop_robot()

    def object_callback(self, msg):
        # Always update latest detection for visual servoing
        # Store tuple (message, reception_time)
        self.latest_detections[msg.name] = (msg, time.time())
        
        # Debug log to verify we are receiving data
        # self.get_logger().info(f"Received detection for {msg.name}")

        if msg.name in self.target_objects:
            try:
                # Transform pose to map frame
                # Create a PoseStamped from the message
                p_stamped = PoseStamped()
                p_stamped.header = msg.header
                p_stamped.pose = msg.pose
                
                # Wait for transform
                if self.tf_buffer.can_transform('map', msg.header.frame_id, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)):
                    map_pose = self.tf_buffer.transform(p_stamped, 'map')
                    
                    # Get Robot Pose (Sighting Pose)
                    robot_transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                    robot_pose = PoseStamped()
                    robot_pose.header.frame_id = 'map'
                    robot_pose.pose.position.x = robot_transform.transform.translation.x
                    robot_pose.pose.position.y = robot_transform.transform.translation.y
                    robot_pose.pose.position.z = robot_transform.transform.translation.z
                    robot_pose.pose.orientation = robot_transform.transform.rotation
                    
                    # Calculate distance between robot and object
                    dist = math.hypot(
                        map_pose.pose.position.x - robot_pose.pose.position.x,
                        map_pose.pose.position.y - robot_pose.pose.position.y
                    )
                    
                    # Check if we should update
                    should_update = False
                    
                    if dist < 1.0:
                        # Too close to be a good sighting pose
                        if int(time.time()) % 5 == 0:
                            self.get_logger().info(f"Ignored {msg.name} sighting: dist {dist:.2f}m < 1.0m")
                        return

                    if msg.name not in self.found_objects:
                        self.get_logger().info(f"New object found: {msg.name} at {dist:.2f}m")
                        should_update = True
                    else:
                        # Check if this new sighting is closer than the old one
                        old_info = self.found_objects[msg.name]
                        old_robot_pose = old_info['robot']
                        old_object_pose = old_info['object']
                        
                        old_dist = math.hypot(
                            old_object_pose.pose.position.x - old_robot_pose.pose.position.x,
                            old_object_pose.pose.position.y - old_robot_pose.pose.position.y
                        )
                        
                        if dist < old_dist:
                            self.get_logger().info(f"Updating sighting for {msg.name}: New dist {dist:.2f}m < Old dist {old_dist:.2f}m")
                            should_update = True

                    if should_update:
                        self.found_objects[msg.name] = {'object': map_pose, 'robot': robot_pose}
                        self.save_objects_to_file()
                        
                        found_list = list(self.found_objects.keys())
                        self.get_logger().info(f"✓ Saved {msg.name}. Progress: {len(found_list)}/3 {found_list}")
                        self.get_logger().info(f"  Object Loc: {map_pose.pose.position.x:.2f}, {map_pose.pose.position.y:.2f}")
                        self.get_logger().info(f"  Sighting Loc: {robot_pose.pose.position.x:.2f}, {robot_pose.pose.position.y:.2f}")
            except Exception as e:
                self.get_logger().warn(f"TF Error: {e}")

    def command_callback(self, msg):
        self.latest_command = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    # SMACH
    sm = smach.StateMachine(outcomes=['mission_completed', 'mission_failed'])
    sm.userdata.target_object = ""
    
    with sm:
        # 1. Explore
        smach.StateMachine.add('EXPLORE', ExploreState(node),
                               transitions={'succeeded':'WAIT_FOR_COMMAND', 'aborted':'mission_failed', 'preempted':'mission_failed'})
        
        # 2. Wait
        smach.StateMachine.add('WAIT_FOR_COMMAND', WaitState(node),
                               transitions={'succeeded':'NAVIGATE_TO_OBJECT', 'aborted':'mission_failed'})
        
        # 3. Fetch Sequence
        smach.StateMachine.add('NAVIGATE_TO_OBJECT', NavigateToTargetState(node),
                               transitions={'succeeded':'APPROACH_OBJECT', 'aborted':'mission_failed'})
        
        smach.StateMachine.add('APPROACH_OBJECT', ApproachObjectState(node),
                               transitions={'succeeded':'OPEN_GRIPPER', 'aborted':'mission_failed'})
        
        smach.StateMachine.add('OPEN_GRIPPER', SimpleGripperState(node, 'open'),
                               transitions={'succeeded':'ARM_PICKUP', 'aborted':'mission_failed'})
                               
        smach.StateMachine.add('ARM_PICKUP', SimpleArmState(node, 'pickup'),
                               transitions={'succeeded':'CLOSE_GRIPPER', 'aborted':'mission_failed'})
                               
        smach.StateMachine.add('CLOSE_GRIPPER', SimpleGripperState(node, 'close'),
                               transitions={'succeeded':'ARM_LIFT', 'aborted':'mission_failed'})
                               
        smach.StateMachine.add('ARM_LIFT', SimpleArmState(node, 'lift'),
                               transitions={'succeeded':'RETURN_HOME', 'aborted':'mission_failed'})
        
        # 4. Return
        smach.StateMachine.add('RETURN_HOME', ReturnHomeState(node),
                               transitions={'succeeded':'ARM_PRE_RELEASE', 'aborted':'mission_failed'})
                               
        smach.StateMachine.add('ARM_PRE_RELEASE', SimpleArmState(node, 'release'),
                               transitions={'succeeded':'NUDGE_FORWARD', 'aborted':'mission_failed'})
        
        smach.StateMachine.add('NUDGE_FORWARD', DriveForwardState(node, 0.2),
                               transitions={'succeeded':'GRIPPER_RELEASE', 'aborted':'mission_failed'})
                               
        smach.StateMachine.add('GRIPPER_RELEASE', SimpleGripperState(node, 'open'),
                               transitions={'succeeded':'BACK_OFF', 'aborted':'mission_failed'})
        
        smach.StateMachine.add('BACK_OFF', DriveForwardState(node, -0.2),
                               transitions={'succeeded':'ARM_HOME', 'aborted':'mission_failed'})
                               
        smach.StateMachine.add('ARM_HOME', SimpleArmState(node, 'home'),
                               transitions={'succeeded':'WAIT_FOR_COMMAND', 'aborted':'mission_failed'})

    # Run SMACH in a separate thread so ROS callbacks work
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    import threading
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    outcome = sm.execute()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
