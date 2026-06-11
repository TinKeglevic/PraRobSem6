#!/usr/bin/env python3

import os
import json
import rclpy
from rclpy.node import Node
import numpy as np
import time
import threading

from geometry_msgs.msg import Point
from std_msgs.msg import String

from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.heuristic import euclidean

import matplotlib.pyplot as plt


class PathPlannerNode(Node):
    def __init__(self):
        super().__init__('path_planner_node')

        self.task_subscriber = self.create_subscription(
            String,
            '/path_request',
            self.task_callback,
            10
        )

        self.waypoint_pub = self.create_publisher(Point, '/marker_target', 10)

        self.Z_DRAW = -0.02
        self.Z_LIFT = 0.05

        self.inference_id = 0

        self.get_logger().info("Path Planner Node has been started.")

    def plot_grid(self, grid, path=None, start=None, goal=None):
        grid_array = np.array(grid)
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.imshow(grid_array, cmap='gray_r', origin='upper')
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            ax.plot(path_x, path_y, color='blue', linewidth=2, label='Path')
        if start:
            ax.plot(start[0], start[1], 'go', markersize=8, label='Start')
        if goal:
            ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')
        ax.set_title('Path Planning Grid')
        ax.set_xlabel('Column Index')
        ax.set_ylabel('Row Index')
        ax.legend()
        plt.grid(True)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        temp_folder = os.path.join(script_dir, '..', 'temp')
        os.makedirs(temp_folder, exist_ok=True)
        image_path = os.path.join(temp_folder, f'path_plot_{self.inference_id}.png')
        plt.savefig(image_path, bbox_inches='tight')
        plt.show()
        self.inference_id += 1

    def clamp_to_grid(self, num, limit):
        return max(0, min(num, limit))

    def plan_path(self, start_world, goal_world, obstacles_world):
        grid_resolution = 0.005
        grid_origin_x = -0.25
        grid_origin_y = 0.35
        num_rows = int(0.4 / grid_resolution)
        num_cols = int(0.4 / grid_resolution)
        grid = np.ones((num_rows, num_cols))

        # Oznaci prepreke
        obs_radius = int(0.04 / grid_resolution)  # 4 cm radius
        for ox, oy in obstacles_world:
            cx = self.clamp_to_grid(int((ox - grid_origin_x) / grid_resolution), num_cols - 1)
            cy = self.clamp_to_grid(int((grid_origin_y - oy) / grid_resolution), num_rows - 1)
            for dr in range(-obs_radius, obs_radius + 1):
                for dc in range(-obs_radius, obs_radius + 1):
                    r = self.clamp_to_grid(cy + dr, num_rows - 1)
                    c = self.clamp_to_grid(cx + dc, num_cols - 1)
                    grid[r][c] = 0

        grid_object = Grid(matrix=grid.tolist())

        start_node = grid_object.node(
            self.clamp_to_grid(int((start_world[0] - grid_origin_x) / grid_resolution), num_cols - 1),
            self.clamp_to_grid(int((grid_origin_y - start_world[1]) / grid_resolution), num_rows - 1)
        )
        goal_node = grid_object.node(
            self.clamp_to_grid(int((goal_world[0] - grid_origin_x) / grid_resolution), num_cols - 1),
            self.clamp_to_grid(int((grid_origin_y - goal_world[1]) / grid_resolution), num_rows - 1)
        )

        self.get_logger().info(f"Start node: {start_node}, Goal node: {goal_node}")

        finder = AStarFinder(diagonal_movement=DiagonalMovement.always, heuristic=euclidean)
        path, runs = finder.find_path(start_node, goal_node, grid_object)

        if not path:
            self.get_logger().warn("No path found.")
            return None

        self.get_logger().info(f"Path found with {len(path)} nodes and {runs} runs.")

        grid_copy = [[1 - int(cell) for cell in row] for row in grid]
        path_for_plot = [(node.x, node.y) for node in path]
        self.plot_grid(grid_copy, path=path_for_plot, start=(start_node.x, start_node.y), goal=(goal_node.x, goal_node.y))

        robot_path_coordinates = []
        for node in path:
            robot_x = grid_origin_x + node.x * grid_resolution - grid_resolution / 2
            robot_y = grid_origin_y - node.y * grid_resolution - grid_resolution / 2
            robot_path_coordinates.append((robot_x, robot_y))

        return robot_path_coordinates

    def task_callback(self, msg):
        try:
            data = json.loads(msg.data)
            start_world = data['start']
            goal_world = data['goal']
            obstacles_world = data.get('obstacles', [])
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Neispravan /path_request: {e}')
            return

        self.get_logger().info(f"Received task: start={start_world}, goal={goal_world}, obstacles={obstacles_world}")

        robot_path_coordinates = self.plan_path(start_world, goal_world, obstacles_world)
        if robot_path_coordinates is None:
            return

        threading.Thread(target=self._execute, args=(robot_path_coordinates,), daemon=True).start()

    def _execute(self, robot_path_coordinates):
        # pen up na prvoj tocki
        pt = Point()
        pt.x = float(robot_path_coordinates[0][0])
        pt.y = float(robot_path_coordinates[0][1])
        pt.z = float(self.Z_LIFT)
        self.waypoint_pub.publish(pt)
        time.sleep(2.0)

        # pen down — idi po putu
        for x, y in robot_path_coordinates:
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(self.Z_DRAW)
            self.waypoint_pub.publish(pt)
            time.sleep(0.5)

        # pen up na zadnjoj tocki
        pt = Point()
        pt.x = float(robot_path_coordinates[-1][0])
        pt.y = float(robot_path_coordinates[-1][1])
        pt.z = float(self.Z_LIFT)
        self.waypoint_pub.publish(pt)
        time.sleep(1.0)

        # vrati na home
        pt = Point()
        pt.x = 0.0
        pt.y = 0.25
        pt.z = float(self.Z_LIFT)
        self.waypoint_pub.publish(pt)

        self.get_logger().info("Izvrsavanje zavrseno.")


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()