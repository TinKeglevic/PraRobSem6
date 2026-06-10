"""Custom tools to help ROSA with ROS2 service/message introspection."""

import subprocess
from langchain.agents import tool
import rclpy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from prarob_interact.kinematics import Kinematics
import numpy as np
import time

## TODO ##
# Implement your own global variables, these are used for demonstrative purposes only
JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']
# Define Limits
JOINT_MIN = [-3.0, -1.5, -1.4, -1.5]
JOINT_MAX = [3.0, 1.5, 1.0, 1.5]
## ##

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
        positions: A list of 4 floats representing target angles.
        duration: Time in seconds to reach the target.
    """
    if not rclpy.ok():
        rclpy.init()

    # FIX 1: Use a unique node name to avoid "Ghost Node" collisions in the graph
    node_name = f'rosa_mover_{int(time.time())}'
    node = rclpy.create_node(node_name)
    
    try:
        publisher = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Prepare Message
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = np.clip(positions, JOINT_MIN, JOINT_MAX).tolist()
        
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        msg.points.append(point)

        # FIX 2: Give the discovery mechanism a tiny moment to see the publisher
        # and another moment after publishing to ensure the data hits the wire.
        time.sleep(0.1) 
        publisher.publish(msg)
        time.sleep(0.1) # The "Grace Period"
        
        return f"Successfully sent command to {node_name}."

    except Exception as e:
        return f"Error: {str(e)}"
    
    finally:
        # Properly cleanup the node
        node.destroy_node()

@tool
def get_tool_pose(timeout_sec: float = 2.0):
    """
    Retrieves the most recent message from the /joint_states topic.
    Use this to get the current positions, velocities, or efforts of the robot's joints.
    
    Args:
        timeout_sec: How long to wait for a message before giving up.
    """
    # Initialize rclpy if it hasn't been initialized yet
    if not rclpy.ok():
        rclpy.init()

    node = rclpy.create_node('rosa_joint_state_fetcher')
    received_msg = None

    def callback(msg):
        nonlocal received_msg
        received_msg = msg

    # Create a subscription
    subscription = node.create_subscription(
        JointState,
        '/joint_states',
        callback,
        10
    )
    # TODO you will implement your own kinematics solution in kinematics.py
    kinematics_node = Kinematics()

    try:
        # Spin until message is received or timeout occurs
        start_time = node.get_clock().now()
        while received_msg is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            
            elapsed = node.get_clock().now() - start_time
            if elapsed.nanoseconds > (timeout_sec * 1e9):
                return "Error: Timeout reached. No messages received on /joint_states."
            
        w = kinematics_node.get_dk(received_msg.position)

        # Format the output for the agent
        output = (
            f"X: {w[0]}\n"
            f"Y: {w[1]}\n"
            f"Z: {w[2]}\n"
            f"Roll: {w[3]}\n"
            f"Pitch: {w[4]}\n"
            f"Yaw: {w[5]}\n"

        )
        return output

    except Exception as e:
        return f"Error retrieving joint states: {str(e)}"
    finally:
        node.destroy_node()

@tool
def move_to_pose(x: float, y: float, z: float, roll: float, pitch: float, yaw: float, duration: float = 3.0):
    """
    Moves the robot end-effector to a specific 3D pose (X, Y, Z in meters, Roll, Pitch, Yaw in radians).
    This tool calculates the required joint angles using Inverse Kinematics.
    
    Args:
        x, y, z: Target position in meters.
        roll, pitch, yaw: Target orientation in radians.
        duration: Time in seconds to complete the movement.
    """
    if not rclpy.ok():
        rclpy.init()

    # Use unique node name to avoid graph collisions
    node_name = f'rosa_ik_mover_{int(time.time())}'
    node = rclpy.create_node(node_name)
    
    try:
        # 1. Calculate Joint Angles via IK
        # TODO you will implement your own kinematics solution in kinematics.py
        kinematics_node = Kinematics()
        target_pose = [x, y, z, roll, pitch, yaw]
        joint_angles = kinematics_node.get_ik(target_pose)

        # Basic error handling if IK fails (returns None or empty)
        if joint_angles is None or len(joint_angles) == 0:
            return f"Error: The pose {target_pose} is out of reach or mathematically impossible for this robot."

        # 2. Setup Publisher
        publisher = node.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        
        # 3. Prepare Trajectory Message
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        
        # Clip the IK results for safety
        point.positions = np.clip(joint_angles, JOINT_MIN, JOINT_MAX).tolist()
        
        # Set timing
        seconds = int(duration)
        nanoseconds = int((duration - seconds) * 1e9)
        point.time_from_start.sec = seconds
        point.time_from_start.nanosec = nanoseconds
        msg.points.append(point)

        # 4. Execute with Discovery Buffers
        time.sleep(0.1)  # Allow publisher discovery
        publisher.publish(msg)
        time.sleep(0.1)  # Ensure message delivery before node destruction
        
        return (f"IK Success. Moving to Pose: X:{x:.3f}, Y:{y:.3f}, Z:{z:.3f}. "
                f"Computed Joints: {point.positions}")

    except Exception as e:
        return f"Hardware error during IK movement: {str(e)}"
    
    finally:
        node.destroy_node()

@tool
def get_yolo_boxes(yolo_detections_topic: str, search_for: list):
    """
    Gets the bounding boxes of detected objects from the detections topic.
    The method searches for objects named in the search_for list of strings and returns their bounding boxes.
    
    Args:
        yolo_detections_topic: The topic reporting the bounding boxes of the YOLO detections.
        search_for: A list of strings, each representing an object class that YOLO should search for.
    """

    ## TODO ##
    # This tool is meant to be an interface for ROSA to use YOLO from the previous exercise.
    # Your task is to implement the tool.
    # It should return a detection for each search_for key (or None for any that aren't found).
    return "This method is not yet implemented."
    ## ##
    

TOOLS = [ros2_interface_show, get_tool_pose, move_robot_joints, move_to_pose, get_yolo_boxes]