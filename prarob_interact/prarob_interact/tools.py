"""Custom tools to help ROSA with ROS2 service/message introspection."""

import subprocess
import os
import json
from langchain.agents import tool
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import time

JOINT_NAMES = ['joint1', 'joint2', 'joint3']
JOINT_MIN = [-3.14, -3.14, -3.14]
JOINT_MAX = [ 3.14,  3.14,  3.14]


@tool
def ros2_interface_show(interface_type: str) -> str:
    """Show the full definition of a ROS2 message, service, or action type.

    Use this as a fallback when you need to check an interface that was NOT
    included in the pre-scanned environment snapshot (e.g. a node that
    started after the agent was created).

    Args:
        interface_type: Full interface type, e.g. 'geometry_msgs/msg/Point'
                        or 'crazyflie_interfaces/srv/GoTo'.
    """
    try:
        result = subprocess.run(
            ["ros2", "interface", "show", interface_type],
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode != 0:
            return f"Error: {result.stderr.strip()}"
        return result.stdout.strip()
    except Exception as e:
        return f"Error running ros2 interface show: {e}"


@tool
def move_robot_joints(positions: list[float], duration: float = 2.0):
    """
    Moves the robot arm joints to specific positions (radians).
    Args:
        positions: A list of 3 floats representing target angles for joint1, joint2, joint3.
        duration: Time in seconds to reach the target.
    """
    if not rclpy.ok():
        rclpy.init()

    node_name = f'rosa_mover_{int(time.time())}'
    node = rclpy.create_node(node_name)

    try:
        publisher = node.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = np.clip(positions, JOINT_MIN, JOINT_MAX).tolist()

        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        msg.points.append(point)

        time.sleep(0.1)
        publisher.publish(msg)
        time.sleep(0.1)

        return f"Successfully sent joint command via {node_name}."

    except Exception as e:
        return f"Error: {str(e)}"
    finally:
        node.destroy_node()


@tool
def get_tool_pose(timeout_sec: float = 2.0):
    """
    Retrieves the current end-effector position by reading /joint_states and computing FK.

    Args:
        timeout_sec: How long to wait for a message before giving up.
    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('rosa_joint_state_fetcher')
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    node.create_subscription(JointState, '/joint_states', callback, 10)

    from rrr_arm.kinematika import RRRKinematics
    kin = RRRKinematics(L1=0.084, L2=0.200, L3=0.19)

    try:
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return "Error: Timeout reached. No messages received on /joint_states."

        q = received_msg.position
        x, y, z = kin.compute_fk(q[0], q[1], q[2])

        return (
            f"X: {x:.4f}\n"
            f"Y: {y:.4f}\n"
            f"Z: {z:.4f}\n"
        )

    except Exception as e:
        return f"Error retrieving joint states: {str(e)}"
    finally:
        node.destroy_node()


@tool
def move_to_pose(x: float, y: float, z: float, roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0, duration: float = 3.0):
    """
    Moves the robot end-effector to a specific 3D position (X, Y, Z in meters).
    Calculates required joint angles using Inverse Kinematics.
    Note: roll, pitch, yaw are ignored — this is a planar RRR robot.

    Args:
        x, y, z: Target position in meters.
        roll, pitch, yaw: Ignored for this robot.
        duration: Time in seconds to complete the movement.
    """
    if not rclpy.ok():
        rclpy.init()

    node_name = f'rosa_ik_mover_{int(time.time())}'
    node = rclpy.create_node(node_name)

    try:
        from rrr_arm.kinematika import RRRKinematics
        kin = RRRKinematics(L1=0.084, L2=0.200, L3=0.19)
        joint_angles = kin.calculate_joint_angles(x, y, z)

        if joint_angles is None:
            return f"Error: IK nije uspio za ({x:.3f}, {y:.3f}, {z:.3f})"

        publisher = node.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = np.clip(joint_angles, JOINT_MIN, JOINT_MAX).tolist()

        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        msg.points.append(point)

        time.sleep(0.1)
        publisher.publish(msg)
        time.sleep(0.1)

        return (
            f"IK Success. Moving to: X={x:.3f}, Y={y:.3f}, Z={z:.3f}. "
            f"Joints: {point.positions}"
        )

    except Exception as e:
        return f"Error during IK movement: {str(e)}"
    finally:
        node.destroy_node()


@tool
def get_yolo_boxes(yolo_detections_topic: str, search_for: list):
    """
    Gets the bounding boxes of detected objects from the YOLO detections topic.
    Searches for objects named in search_for and returns their bounding box centers.

    Args:
        yolo_detections_topic: The topic reporting YOLO detections (e.g. '/yolo/detections').
        search_for: A list of object class names to search for.
    """
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('rosa_yolo_fetcher')
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    from yolo_msgs.msg import DetectionArray
    node.create_subscription(DetectionArray, yolo_detections_topic, callback, 10)

    try:
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > 3e9:
                return "Error: Timeout. Nema detekcija na topicu."

        results = {}
        for target in search_for:
            results[target] = None
            for det in received_msg.detections:
                if target.lower() in det.class_name.lower():
                    results[target] = {
                        'cx': det.bbox.center.position.x,
                        'cy': det.bbox.center.position.y,
                        'class_name': det.class_name
                    }
                    break

        return results

    except Exception as e:
        return f"Error: {str(e)}"
    finally:
        node.destroy_node()


@tool
def send_path_request(start_object: str, goal_object: str, avoid_objects: list):
    """
    Plans and executes a drawing path between two detected objects while avoiding others.
    Detects objects via YOLO, converts pixel coordinates to robot coordinates,
    and sends the path request to the path planning node.

    Args:
        start_object: Name of the object to start from.
        goal_object: Name of the object to draw to.
        avoid_objects: List of object names to avoid.
    """
    from std_msgs.msg import String
    from prarob_calib.camera_to_world import image2world

    # 1. Dohvati detekcije
    all_objects = [start_object, goal_object] + avoid_objects
    boxes = get_yolo_boxes.func('/yolo/detections', all_objects)

    if isinstance(boxes, str):
        return boxes

    if boxes[start_object] is None:
        return f"Objekt '{start_object}' nije detektiran."
    if boxes[goal_object] is None:
        return f"Objekt '{goal_object}' nije detektiran."

    # 2. Učitaj kalibracijsku matricu
    T_path = os.path.expanduser('~/ros2_ws/src/PraRobSem6/T_cam_world.npy')
    if not os.path.exists(T_path):
        return "Error: T_cam_world.npy nije pronađen. Pokreni kalibraciju kamere."
    T_cam_world = np.load(T_path)

    # 3. Dohvati K matricu i pretvori pixel → robot koordinate
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('rosa_path_sender')

    try:
        from sensor_msgs.msg import CameraInfo
        k_matrix = None

        def cam_cb(msg):
            nonlocal k_matrix
            k_matrix = np.reshape(np.array(msg.k), (3, 3))

        node.create_subscription(CameraInfo, '/camera1/camera_info', cam_cb, 10)
        start_t = node.get_clock().now()
        while k_matrix is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.get_clock().now() - start_t).nanoseconds > 3e9:
                return "Error: Timeout čekanja na CameraInfo."

        def to_world(det):
            px = np.array([det['cx'], det['cy']])
            pt = image2world(px, k_matrix, T_cam_world, z=0.0)
            return [float(pt[0]), float(pt[1])]

        start_w = to_world(boxes[start_object])
        goal_w  = to_world(boxes[goal_object])
        obstacles_w = []
        for name in avoid_objects:
            if boxes.get(name):
                obstacles_w.append(to_world(boxes[name]))

        # 4. Pošalji path request
        pub = node.create_publisher(String, '/path_request', 10)
        request = {'start': start_w, 'goal': goal_w, 'obstacles': obstacles_w}
        msg = String()
        msg.data = json.dumps(request)
        time.sleep(0.1)
        pub.publish(msg)
        time.sleep(0.1)

        return (
            f"Path request poslan: {start_object} → {goal_object}, "
            f"izbjegavam: {avoid_objects}. "
            f"Start: {start_w}, Goal: {goal_w}"
        )

    except Exception as e:
        return f"Error: {str(e)}"
    finally:
        node.destroy_node()


TOOLS = [ros2_interface_show, get_tool_pose, move_robot_joints, move_to_pose, get_yolo_boxes, send_path_request]