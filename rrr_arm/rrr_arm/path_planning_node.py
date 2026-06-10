#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

import json
import heapq
import math
import time
import threading
import numpy as np


# ─────────────────────────────────────────────
#  A* planer — nepromijenjeno
# ─────────────────────────────────────────────

class AStarPlanner:
    WORKSPACE    = 0.35
    GRID         = 100
    OBS_RADIUS_M = 0.04

    def __init__(self):
        self.cell_size = self.WORKSPACE / self.GRID

    def _to_grid(self, x, y):
        gx = max(0, min(self.GRID - 1, int(x / self.cell_size)))
        gy = max(0, min(self.GRID - 1, int(y / self.cell_size)))
        return gx, gy

    def _to_world(self, gx, gy):
        return (gx + 0.5) * self.cell_size, (gy + 0.5) * self.cell_size

    def _build_obstacle_grid(self, obstacles):
        grid = np.zeros((self.GRID, self.GRID), dtype=bool)
        r_cells = int(self.OBS_RADIUS_M / self.cell_size) + 1
        for ox, oy in obstacles:
            cx, cy = self._to_grid(ox, oy)
            for dx in range(-r_cells, r_cells + 1):
                for dy in range(-r_cells, r_cells + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < self.GRID and 0 <= ny < self.GRID:
                        wx, wy = self._to_world(nx, ny)
                        if math.hypot(wx - ox, wy - oy) <= self.OBS_RADIUS_M:
                            grid[nx][ny] = True
        return grid

    def plan(self, start, goal, obstacles):
        obs_grid = self._build_obstacle_grid(obstacles)
        sg = self._to_grid(*start)
        gg = self._to_grid(*goal)

        open_heap = []
        heapq.heappush(open_heap, (0.0, sg))
        came_from = {sg: None}
        g_score   = {sg: 0.0}
        dirs = [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == gg:
                break
            cx, cy = current
            for dx, dy in dirs:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.GRID and 0 <= ny < self.GRID):
                    continue
                if obs_grid[nx][ny]:
                    continue
                ng = g_score[current] + math.hypot(dx, dy)
                nb = (nx, ny)
                if nb not in g_score or ng < g_score[nb]:
                    g_score[nb] = ng
                    h = math.hypot(nx - gg[0], ny - gg[1])
                    heapq.heappush(open_heap, (ng + h, nb))
                    came_from[nb] = current

        if gg not in came_from:
            return None

        path = []
        node = gg
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()
        path = self._simplify(path)
        return [self._to_world(gx, gy) for gx, gy in path]

    def _simplify(self, path):
        if len(path) <= 2:
            return path
        result = [path[0]]
        for i in range(1, len(path) - 1):
            prev, curr, nxt = result[-1], path[i], path[i+1]
            if (curr[0]-prev[0], curr[1]-prev[1]) != (nxt[0]-curr[0], nxt[1]-curr[1]):
                result.append(curr)
        result.append(path[-1])
        return result


# ─────────────────────────────────────────────
#  FK — iste dimenzije kao kinematika.py
# ─────────────────────────────────────────────

def fk(q1, q2, q3, L1=0.084, L2=0.200, L3=0.19):
    r = L2 * math.sin(q2) + L3 * math.sin(q2 + q3)
    x = r * math.cos(q1)
    y = r * math.sin(q1)
    return x, y


# ─────────────────────────────────────────────
#  ROS2 node
# ─────────────────────────────────────────────

class PathPlanningNode(Node):

    Z_DRAW = 0.0
    Z_LIFT = 0.05
    REACH_TOL = 0.008   # 8mm tolerancija — podesi ako treba
    REACH_TIMEOUT = 5.0  # max sekundi čekanja po waypoint

    def __init__(self):
        super().__init__('path_planning_node')

        self.planner = AStarPlanner()

        self.waypoint_pub = self.create_publisher(Point, '/marker_target', 10)
        self.status_pub   = self.create_publisher(String, '/path_status', 10)

        self.create_subscription(String,     '/path_request', self._on_request,     10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)

        # Trenutna XY pozicija end-effectora (iz FK na /joint_states)
        self.current_ee_xy = None
        self._executing    = False

        self.get_logger().info('PathPlanningNode spreman.')

    def _on_joint_states(self, msg):
        if len(msg.position) >= 3:
            x, y = fk(msg.position[0], msg.position[1], msg.position[2])
            self.current_ee_xy = (x, y)

    def _on_request(self, msg):
        if self._executing:
            self.get_logger().warn('Već izvršavam, ignoriram novi zahtjev.')
            return
        try:
            data      = json.loads(msg.data)
            start     = data['start']
            goal      = data['goal']
            obstacles = data.get('obstacles', [])
        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f'Neispravan /path_request: {e}')
            self._publish_status('ERROR: neispravan zahtjev')
            return

        self._publish_status('PLANNING')
        waypoints = self.planner.plan(start, goal, obstacles)

        if waypoints is None:
            self.get_logger().error('A*: nema puta!')
            self._publish_status('ERROR: nema puta')
            return

        self.get_logger().info(f'Pronađen put: {len(waypoints)} waypointa')
        threading.Thread(target=self._execute, args=(waypoints,), daemon=True).start()

    def _execute(self, waypoints):
        self._executing = True
        self._publish_status(f'EXECUTING {len(waypoints)}')

        for i, (x, y) in enumerate(waypoints):
            # Pošalji waypoint kinematika nodu
            pt = Point()
            pt.x = float(x)
            pt.y = float(y)
            pt.z = float(self.Z_DRAW)
            self.waypoint_pub.publish(pt)
            self.get_logger().info(f'Waypoint {i+1}/{len(waypoints)}: x={x:.3f}, y={y:.3f}')

            # Čekaj dok robot ne stigne
            self._wait_until_reached(x, y)

        self._publish_status('DONE')
        self.get_logger().info('Izvršavanje završeno.')
        self._executing = False

    def _wait_until_reached(self, target_x, target_y):
        deadline = time.time() + self.REACH_TIMEOUT
        while time.time() < deadline:
            if self.current_ee_xy is not None:
                dist = math.hypot(
                    self.current_ee_xy[0] - target_x,
                    self.current_ee_xy[1] - target_y
                )
                if dist < self.REACH_TOL:
                    return
            time.sleep(0.05)
        self.get_logger().warn(
            f'Timeout za waypoint ({target_x:.3f}, {target_y:.3f}), '
            f'trenutna pozicija: {self.current_ee_xy}'
        )

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()