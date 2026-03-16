from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    EmitEvent,
    RegisterEventHandler,
    LogInfo,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events import lifecycle, matches_node_name
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs


def generate_launch_description():
    wiimote_teleop_pck = FindPackageShare("wiimote_teleop")
    sim = LaunchConfiguration("rviz")
    grip = LaunchConfiguration("gripper")

    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    wiimote_teleop_pck,
                    "description",
                    "urdf",
                    "6dofexample.urdf.xacro",
                ]
            ),
            " sim:=",
            sim,
            " gripper:=",
            grip,
            " end_link:=",
            LaunchConfiguration("end_link"),
        ]
    )

    controller_config = PathJoinSubstitution(
        [wiimote_teleop_pck, "config", "controllers.yaml"]
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_content, "use_sim_time": sim}
        ],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_config],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
        condition=UnlessCondition(sim),
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

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="gripper",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
        ],
    )

    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            position_controller_spawner,
            gripper_controller_spawner,
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    wiimote_teleop_pck,
                    "description",
                    "6dofWithGripper.rviz",
                ]
            ),
        ],
        condition=IfCondition(sim),
    )

    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("ros_gz_sim"),
                        "launch",
                        "gz_sim.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_args": ["-r -v4 empty.sdf"],
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(sim),
    )

    gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        wiimote_teleop_pck,
                        "launch",
                        "gripper.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"sim": sim}.items(),
    )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "my_bot", "-z", "0.1"],
        output="screen",
        condition=IfCondition(sim),
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    wiimote_handler = Node(
        package="wiimote_teleop",
        executable="wiimote_handler",
        parameters=[
            {
                "robot_description": robot_description_content,
                "end_link_name": LaunchConfiguration("end_link"),
                "gripper": grip,
            }
        ],
        output="both",
    )

    controller_handler = Node(
        package="wiimote_teleop",
        executable="controller_handler",
        parameters=[
            {
                "robot_description": robot_description_content,
                "end_link_name": LaunchConfiguration("end_link"),
                "gripper": grip,
            }
        ],
        output="both",
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[
            PathJoinSubstitution([wiimote_teleop_pck, "config", "joystick.yaml"])
        ],
        condition=UnlessCondition(LaunchConfiguration("wiimote")),
    )

    wiimote_node = LifecycleNode(
        package="wiimote",
        executable="wiimote_node",
        namespace="",
        name="wiimote",
        output="screen",
        parameters=[
            PathJoinSubstitution([wiimote_teleop_pck, "config", "wiimote.yaml"])
        ],
        condition=IfCondition(LaunchConfiguration("wiimote")),
    )
    configure_wiimote = EmitEvent(
        event=lifecycle.ChangeState(
            lifecycle_node_matcher=matches_node_name("/wiimote"),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
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
            DeclareLaunchArgument(
                "rviz", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "gripper", default_value="true", choices=["true", "false"]
            ),
            DeclareLaunchArgument(
                "wiimote", default_value="true", choices=["true", "false"]
            ),
            robot_state_publisher,
            controller_manager,
            delayed_spawners,
            delay_rviz_after_joint_state_broadcaster_spawner,
            gazebo,
            spawn_entity,
            # gripper,
            gz_bridge,
            joy_node,  # lauch joy_node based on the launch argument
            RegisterEventHandler(  # handle controller when launched
                OnProcessStart(
                    target_action=joy_node,
                    on_start=[
                        LogInfo(msg="Joy node started, launching handler"),
                        controller_handler,
                    ],
                )
            ),
            wiimote_node,  # launch wiimote_node based on the launch argument
            RegisterEventHandler(  # configure wiimote when launched
                OnProcessStart(
                    target_action=wiimote_node,
                    on_start=[
                        LogInfo(msg="Wiimote started, configuring..."),
                        configure_wiimote,
                    ],
                )
            ),
            on_configure_wiimote,  # when configured, activate it
            on_activate_wiimote,  # when activated, start the handler
        ]
    )
