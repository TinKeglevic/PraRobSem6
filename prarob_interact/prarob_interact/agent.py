from rosa import ROSA, RobotSystemPrompts
from prarob_interact.config import get_llm
from prarob_interact.tools import TOOLS as BUILTIN_TOOLS
from prarob_interact.ros2_introspection import (
    ROS2State,
    scan_ros2_environment,
    format_state_for_prompt,
)


def create_agent(
    tools=None,
    tool_packages=None,
    prompts=None,
    streaming=True,
    verbose=False,
    ros2_state: ROS2State | None = None,
):
    """Create a configured ROSA agent for ROS2.

    Args:
        tools: Optional list of additional LangChain @tool functions.
        tool_packages: Optional list of Python packages containing tools.
        prompts: Optional RobotSystemPrompts instance.
        streaming: Enable streaming responses (default True).
        verbose: Enable debug output (default False).
        ros2_state: Pre-scanned ROS2 environment state. If None, a scan is
            performed automatically.

    Returns:
        A configured ROSA instance.
    """
    llm = get_llm()

    # Pre-fetch ROS2 environment if not already provided
    if ros2_state is None:
        ros2_state = scan_ros2_environment()

    env_description = format_state_for_prompt(ros2_state)

    if prompts is None:
        prompts = RobotSystemPrompts(
            embodiment_and_persona=(
                "You are a helpful ROS2 robot assistant. "
                "You can inspect and interact with the ROS2 system "
                "on behalf of the operator."
            ),
            about_your_operators=(
                "Your operators are robotics engineers and researchers. "
                "Be concise and technical in your responses."
            ),
            about_your_environment=(
                "The following is a COMPLETE live snapshot of the ROS2 "
                "environment taken at startup. This data is ALREADY KNOWN "
                "to you — treat it as ground truth.\n\n"
                + env_description
            ),
            critical_instructions=(
                "*** MANDATORY: DO NOT CALL DISCOVERY TOOLS ***\n"
                "You have a COMPLETE pre-scanned snapshot of all ROS2 topics, "
                "services, actions, and their full interface definitions in "
                "your 'about_your_environment' context above.\n\n"
                "NEVER call these tools — the data is already in your context:\n"
                "  - ros2_node_list\n"
                "  - ros2_topic_list\n"
                "  - ros2_service_list\n"
                "  - ros2_service_info\n"
                "  - ros2_service_type\n"
                "  - ros2_interface_show\n"
                "  - ros2_action_list\n"
                "Instead, look up the topic/service/action name and its type "
                "directly from the environment snapshot above.\n"
                "Only call a discovery tool if the operator asks about "
                "something NOT listed in the snapshot (e.g. a node that "
                "started after boot).\n\n"
                "When the operator asks to call a service, go STRAIGHT to "
                "ros2_service_call — you already know the service name, type, "
                "and exact field definitions from the snapshot.\n\n"
                "SERVICE CALL FORMAT:\n"
                "The 'request' argument to ros2_service_call must be a YAML "
                "dictionary string. Use YAML syntax with colons, NOT equals signs.\n\n"
                "CORRECT format:\n"
                "  {{group_mask: 1, height: 1.0, duration: {{sec: 5, nanosec: 0}}}}\n\n"
                "WRONG formats (will fail):\n"
                "  group_mask=1, height=1.0         <-- equals signs, not a dict\n"
                "  {{group_mask=1, height=1.0}}       <-- equals signs\n"
                "  {{'group_mask': 1}}                 <-- Python dict with quotes\n\n"
                "Nested messages use nested YAML dicts:\n"
                "  {{goal: {{x: 1.0, y: 2.0, z: 0.5}}, duration: {{sec: 5, nanosec: 0}}}}\n\n"
                "Common field facts:\n"
                "- geometry_msgs/msg/Point has ONLY: x, y, z (float64). NO yaw.\n"
                "- geometry_msgs/msg/Quaternion has: x, y, z, w.\n"
                "- builtin_interfaces/msg/Duration has: sec (int32), nanosec (uint32).\n"
                "- If a call fails, re-inspect the interface definition above and fix format."
            ),
        )

    kwargs = dict(
        ros_version=2,
        llm=llm,
        prompts=prompts,
        streaming=streaming,
        verbose=verbose,
    )

    all_tools = list(BUILTIN_TOOLS)
    if tools:
        all_tools.extend(tools)
    kwargs["tools"] = all_tools
    if tool_packages:
        kwargs["tool_packages"] = tool_packages

    return ROSA(**kwargs)
