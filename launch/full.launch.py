from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    EmitEvent,
    RegisterEventHandler,
    LogInfo,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events import lifecycle
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs


def generate_launch_description():
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("wiimote_teleop"), "urdf", "test_robot.urdf.xacro"]
            ),
        ]
    )

    controller_config = PathJoinSubstitution(
        [FindPackageShare("wiimote_teleop"), "config", "controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_content},
            controller_config,
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller"],
    )

    wiimote_handler = Node(
        package="wiimote_teleop",
        executable="wiimote_handler",
        parameters=[
            {
                "robot_description": robot_description_content,
                "end_link": LaunchConfiguration("end_link"),
            }
        ],
        output="both",
    )

    wiimote_node = LifecycleNode(
        package="wiimote",
        executable="wiimote_node",
        namespace="",
        name="wiimote",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("wiimote_teleop"), "config", "wiimote.yaml"]
            )
        ],
    )

    configure_wiimote = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=lifecycle.matches_node_name("/wiimote"),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            position_controller_spawner,
            wiimote_handler,
            wiimote_node,
            configure_wiimote,
        ],
    )

    activate_wiimote = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=lifecycle.matches_node_name("/wiimote"),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
        )
    )

    on_configure_wiimote = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=wiimote_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="wiimote successfully configured. Proceeding to activate."),
                activate_wiimote,
            ],
        )
    )

    on_activate_wiimote = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=wiimote_node,
            start_state="inactive",
            goal_state="active",
            entities=[
                LogInfo(
                    msg="wiimote successfully activated. Proceeding to call handler."
                ),
                wiimote_handler,
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("end_link", default_value="tool0"),
            robot_state_publisher,
            controller_manager,
            delayed_spawners,
            on_configure_wiimote,
            on_activate_wiimote,
        ]
    )
