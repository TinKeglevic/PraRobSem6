import rclpy
from rclpy.node import Node
import math
from math import atan2, sin, cos, sqrt
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from visualization_msgs.msg import Marker
from rclpy.duration import Duration


class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__("kinematics_node")

        self.subscription = self.create_subscription(
            Point, "/marker_target", self.target_callback, 10
        )
        self.actual_sub = self.create_subscription(
            JointState, "/joint_states", self.actual_callback, 10
        )
        self.robot_publisher = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )
        self.target_joint_publisher = self.create_publisher(
            JointState, "/target_joint_states", 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, "/visualization_marker", 10
        )

        self.d1 = 0.085       # visina od stola do joint2
        self.l1 = 0.200        # L2 — gornji link
        self.l2 = 0.155        # L3_eff — donji link + olovka
        self.q2_offset = math.radians(0.0)
        self.q3_offset = math.radians(85.5)

        self.actual_angles = [0.0, 0.0, 0.0]
        self.target_angles = [0.0, 0.0, 0.0]

        self.get_logger().info("Kinematics node pokrenut.")

    def actual_callback(self, msg):
        if len(msg.position) >= 3:
            self.actual_angles = list(msg.position)
            self._log_tracking_error()
            x, y, z = self.compute_fk(*self.actual_angles)
            self.publish_marker(x, y, z, ID=0, r=1.0, g=0.0, b=0.0)

    def target_callback(self, msg):
        x, y, z = msg.x, msg.y, msg.z
        try:
            q1, q2, q3 = self.compute_ik(x, y, z)
            self.target_angles = [q1, q2, q3]

            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = ["joint1", "joint2", "joint3"]
            point = JointTrajectoryPoint()
            point.positions = [float(q1), float(q2), float(q3)]
            point.time_from_start = Duration(seconds=2.0).to_msg()
            traj_msg.points.append(point)
            self.robot_publisher.publish(traj_msg)

            target_msg = JointState()
            target_msg.header.stamp = self.get_clock().now().to_msg()
            target_msg.name = ["joint1", "joint2", "joint3"]
            target_msg.position = [float(q1), float(q2), float(q3)]
            self.target_joint_publisher.publish(target_msg)

            self.publish_marker(x, y, z, ID=1, r=0.0, g=1.0, b=0.0)

            self.get_logger().info(
                f"IK ({x:.3f}, {y:.3f}, {z:.3f}) -> "
                f"q1={math.degrees(q1):.1f}° q2={math.degrees(q2):.1f}° q3={math.degrees(q3):.1f}°"
            )

        except ValueError as e:
            self.get_logger().error(f"IK greška: {e}")

    def compute_ik(self, x, y, z):
        q0 = self.actual_angles

        q1 = atan2(-x, y)
        r = sqrt(x**2 + y**2)
        z_p = z - self.d1

        cos_q3 = (r**2 + z_p**2 - self.l1**2 - self.l2**2) / (2.0 * self.l1 * self.l2)
        cos_q3 = max(min(cos_q3, 1.0), -1.0)
        sin_q3 = sqrt(max(0.0, 1.0 - cos_q3**2))

        q3_candidates = [atan2(sin_q3, cos_q3), atan2(-sin_q3, cos_q3)]
        q2_candidates = []
        for q3 in q3_candidates:
            q2_eff = atan2(z_p, r) - atan2(self.l2 * sin(q3), self.l1 + self.l2 * cos(q3))
            q2 = -q2_eff + self.q2_offset
            q2_candidates.append(q2)

        # Odaberi granu najbližu trenutnoj poziciji
        best = None
        best_cost = float("inf")
        for q2, q3 in zip(q2_candidates, q3_candidates):
            q3_with_offset = q3 + self.q3_offset
            dq = np.array([q1 - q0[0], q2 - q0[1], q3_with_offset - q0[2]])
            cost = float(dq @ dq)
            if cost < best_cost:
                best_cost = cost
                best = (q1, q2, q3_with_offset)

        if best is None:
            raise ValueError("IK nije pronašla rješenje!")

        return best

    def compute_fk(self, q1, q2, q3):
        r = self.l1 * math.sin(-q2) + self.l2 * math.sin(-q2 + -q3)
        z = self.d1 + self.l1 * math.cos(-q2) + self.l2 * math.cos(-q2 + -q3)
        x = r * math.sin(q1)
        y = r * math.cos(q1)
        return x, y, z

    def _log_tracking_error(self):
        for i, name in enumerate(["joint1", "joint2", "joint3"]):
            err = math.degrees(self.target_angles[i] - self.actual_angles[i])
            self.get_logger().debug(
                f"{name}: cmd={math.degrees(self.target_angles[i]):.1f}° "
                f"actual={math.degrees(self.actual_angles[i]):.1f}° err={err:.2f}°"
            )

    def publish_marker(self, x, y, z, ID, r, g, b):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "start_goal_markers"
        marker.id = ID
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r = float(r)
        marker.color.g = float(g)
        marker.color.b = float(b)
        marker.color.a = 1.0
        self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()