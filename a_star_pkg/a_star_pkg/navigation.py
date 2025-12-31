#!/usr/bin/env python3
# Navigation process with global costmap

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from math import atan2, sqrt, sin, pi
import heapq
import numpy as np

#QoS for map topic(Reliable & Transient Local)
map_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1
)

class NodeAStar:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = self.h = self.f = 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f

class IntegratedNavigation(Node):
    def __init__(self):
        super().__init__('integrated_navigation')

        # 1. Get parameters from .yaml
        self.declare_parameter('lookahead_dist', 0.40)
        self.declare_parameter('linear_vel', 0.15)
        self.declare_parameter('stop_tolerance', 0.15)
        self.declare_parameter('yaw_tolerance', 0.08)
        self.declare_parameter('obstacle_distance', 0.45)
        self.declare_parameter('safety_range', 0.30)
        self.declare_parameter('OCCUPIED', 80)

        self.lookahead_dist = self.get_parameter('lookahead_dist').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.stop_tolerance = self.get_parameter('stop_tolerance').value
        self.yaw_tolerance = self.get_parameter('yaw_tolerance').value
        self.obstacle_distance = self.get_parameter('obstacle_distance').value
        self.safety_range = self.get_parameter('safety_range').value
        self.OCCUPIED = self.get_parameter('OCCUPIED').value

        # safety value in param_callback()
        self.max_linear = 0.5

        self.add_on_set_parameters_callback(self.param_callback)

        # 2. Initializing variables
        self.map_data = None
        self.current_pose = None
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.global_path = []
        self.path_index = 0
        self.DECIDE = True
        self.latest_ranges = None
        self.turn_start_time = None
        self.min_turn_time = 0.6
        self.past_angular = 0.0
        self.stop_by_yolo = False

        # 3. subscriber & publisher
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/planned_path', 10)

        self.sub_map = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.sub_pose = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, 10)
        self.sub_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.stop_sub = self.create_subscription(Bool, '/stop_signal', self.stop_callback, 10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("integrated_navigation Node started")

    def param_callback(self, params):

        for param in params:
            if param.name == 'linear_vel':
                if 0.0 <= param.value <= self.max_linear:
                    self.linear_vel = param.value
                    self.get_logger().info(f"Updated linear_vel: {self.linear_vel}")
                else:
                    return SetParametersResult(successful=False, reason="Out of speed range")
            
            elif param.name == 'lookahead_dist':
                if 0.1 <= param.value <= 2.0:
                    self.lookahead_dist = param.value
                    self.get_logger().info(f"Updated lookahead_dist: {self.lookahead_dist}")
            
            elif param.name == 'obstacle_distance':
                self.obstacle_distance = param.value
                self.get_logger().info(f"Updated obstacle_distance: {self.obstacle_distance}")

            elif param.name == 'OCCUPIED':
                if 0 <= param.value <= 100 and param.value.isdigit():
                    self.OCCUPIED = param.value
                    self.get_logger().info(f"Updated OCCUPIED: {self.OCCUPIED}")
                else:
                    return SetParametersResult(successful=False, reason="Out of OccupancyGrid range")

            elif param.name == 'yaw_tolerance':
                self.yaw_tolerance = param.value
                self.get_logger().info(f"Updated yaw_tolerance: {self.yaw_tolerance}")

        return SetParametersResult(successful=True)

    def map_callback(self, msg):
        try:
            self.map_resolution = msg.info.resolution
            self.map_width = msg.info.width
            self.map_height = msg.info.height
            self.map_origin = [msg.info.origin.position.x, msg.info.origin.position.y]
            
            raw_map = np.array(msg.data, dtype=np.int8).reshape((self.map_height, self.map_width))
            obstacle_mask = (raw_map >= self.OCCUPIED).astype(np.int8)
            
            m_range = int(self.safety_range // self.map_resolution)
            if m_range < 1: m_range = 1
            
            # Numpy slicing to preprocess metamapdata (global map costing)
            inflated_mask = obstacle_mask.copy()
            for d in range(1, m_range + 1):
                inflated_mask[d:, :] |= obstacle_mask[:-d, :]
                inflated_mask[:-d, :] |= obstacle_mask[d:, :]
                inflated_mask[:, d:] |= obstacle_mask[:, :-d]
                inflated_mask[:, :-d] |= obstacle_mask[:, d:]

            self.map_data = raw_map.copy()
            self.map_data[inflated_mask > 0] = self.OCCUPIED
            self.get_logger().info(f"Map Updated (Inflation: {m_range} cells)")
        except Exception as e:
            self.get_logger().error(f"Map Error: {e}")

    def pose_callback(self, msg):
        self.current_pose = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        q = msg.pose.pose.orientation
        self.current_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def goal_callback(self, msg):
        if self.map_data is None or self.current_pose is None:
            self.get_logger().warn("Waiting for Map/Pose...")
            return

        goal_pose = [msg.pose.position.x, msg.pose.position.y]
        q = msg.pose.orientation
        self.target_yaw = atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

        start_grid = self.world_to_grid(self.current_pose)
        goal_grid = self.world_to_grid(goal_pose)
        
        self.get_logger().info(f"Planning to {goal_pose}...")
        path_grid = self.run_astar(start_grid, goal_grid)
        
        if path_grid:
            self.global_path = [self.grid_to_world(p) for p in path_grid]
            self.path_index = 0
            self.publish_path_viz()
            self.get_logger().info("Path Found!")
        else:
            self.get_logger().warn("A* Failed.")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges, dtype=np.float32)
        self.latest_ranges = np.where(np.isfinite(ranges), ranges, 3.5)

    def run_astar(self, start, goal):
        if self.map_data[start[0], start[1]] >= self.OCCUPIED:
            return None
            
        start_node = NodeAStar(None, start)
        end_node = NodeAStar(None, goal)
        open_list = [start_node]
        visited = set()
        moves = [(0,1), (0,-1), (1,0), (-1,0), (1,1), (1,-1), (-1,1), (-1,-1)]

        while open_list:
            current_node = heapq.heappop(open_list)
            if current_node.position in visited: continue
            visited.add(current_node.position)

            if current_node.position == end_node.position:
                path = []
                while current_node:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return path[::-1]

            for move in moves:
                ny, nx = current_node.position[0] + move[0], current_node.position[1] + move[1]
                if not (0 <= ny < self.map_height and 0 <= nx < self.map_width): continue
                if self.map_data[ny, nx] >= self.OCCUPIED: continue
                
                new_node = NodeAStar(current_node, (ny, nx))
                new_node.g = current_node.g + 1
                new_node.h = sqrt((ny - goal[0])**2 + (nx - goal[1])**2)
                new_node.f = new_node.g + new_node.h
                heapq.heappush(open_list, new_node)
        return None

    def control_loop(self):
        if not self.global_path or self.latest_ranges is None: return
        if self.stop_by_yolo:
            self.stop_robot()
            return

        final_goal = self.global_path[-1]
        dist_to_final = sqrt((final_goal[0]-self.current_pose[0])**2 + (final_goal[1]-self.current_pose[1])**2)

        # 1. after heading logic (match yaw)
        if dist_to_final < self.stop_tolerance:
            yaw_error = self.target_yaw - self.current_yaw

            # yaw_error nomalization
            while yaw_error > pi: yaw_error -= 2*pi
            while yaw_error < -pi: yaw_error += 2*pi

            if abs(yaw_error) > self.yaw_tolerance:
                cmd = Twist()
                cmd.angular.z = 0.5 if yaw_error > 0 else -0.5
                self.pub_cmd.publish(cmd)
            else:
                self.global_path = []
                self.stop_robot()
                self.get_logger().info("Goal Reached!")
            return

        # 2. following node with pure pursuit & obstacle avoidance with FSM
        ranges = self.latest_ranges
        front_dist = np.min(np.concatenate((ranges[:20], ranges[-20:])))
        
        if self.DECIDE:

            target_x, target_y = self.global_path[-1]
            for i in range(self.path_index, len(self.global_path)):
                px, py = self.global_path[i]
                if sqrt((px-self.current_pose[0])**2 + (py-self.current_pose[1])**2) >= self.lookahead_dist:
                    target_x, target_y = px, py
                    self.path_index = i
                    break

            alpha = atan2(target_y - self.current_pose[1], target_x - self.current_pose[0]) - self.current_yaw
            while alpha > pi: alpha -= 2*pi
            while alpha < -pi: alpha += 2*pi

            ang_vel = (2.0 * self.linear_vel * sin(alpha)) / self.lookahead_dist
            
            if front_dist < self.obstacle_distance:
                self.DECIDE = False
                self.turn_start_time = self.get_clock().now()
                self.past_angular = 0.8 if alpha > 0 else -0.8
                self.get_logger().warn("Obstacle! Avoiding...")
            else:
                cmd = Twist()
                cmd.linear.x = self.linear_vel
                cmd.angular.z = max(min(ang_vel, 1.0), -1.0)
                self.pub_cmd.publish(cmd)

        else: # Avoidance
            elapsed = (self.get_clock().now() - self.turn_start_time).nanoseconds * 1e-9
            if elapsed > self.min_turn_time and front_dist > self.obstacle_distance:
                self.DECIDE = True
            else:
                cmd = Twist()
                cmd.angular.z = self.past_angular
                self.pub_cmd.publish(cmd)

    def stop_callback(self, msg: Bool): self.stop_by_yolo = msg.data
    def world_to_grid(self, w): return (int((w[1]-self.map_origin[1])/self.map_resolution), int((w[0]-self.map_origin[0])/self.map_resolution))
    def grid_to_world(self, g): return [(g[1]*self.map_resolution)+self.map_origin[0], (g[0]*self.map_resolution)+self.map_origin[1]]
    
    def publish_path_viz(self):
        msg = Path()
        msg.header.frame_id = 'map'
        for p in self.global_path:
            ps = PoseStamped()
            ps.pose.position.x, ps.pose.position.y = p[0], p[1]
            msg.poses.append(ps)
        self.pub_path.publish(msg)

    def stop_robot(self): self.pub_cmd.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedNavigation()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__': main()