from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    # Get package paths
    pkg_delto_description = FindPackageShare("dg_description").find("dg_description")
    model_path = os.path.join(pkg_delto_description, "meshes")

    # Set Gazebo model path
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (
            os.environ["IGN_GAZEBO_RESOURCE_PATH"] + ":" + model_path
        )
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = model_path

    # Get URDF via xacro
    gripper_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("wiimote_teleop"),
                    "description",
                    "urdf",
                    "gripper.urdf.xacro",
                ]
            ),
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("wiimote_teleop"),
            "config",
            "gripper_controllers.yaml",
        ]
    )

    gripper_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="gripper",
        output="screen",
        parameters=[
            {"robot_description": gripper_description_content, "use_sim_time": sim}
        ],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="gripper",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/gripper/robot_description")],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="gripper",
        arguments=[
            "joint_state_broadcaster",
            "-c",
            "/gripper/controller_manager",
        ],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="gripper",
        arguments=[
            "position_controller",
            "-c",
            "/gripper/controller_manager",
        ],
    )

    delayed_spawners = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            position_controller_spawner,
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            "/gripper/robot_description",
            "-name",
            "dg3f_m",
            "-allow_renaming",
            "true",
            # "-x", "0.0",
            # "-y", "0.0",
            # "-z", "0.0",
        ],
        condition=IfCondition(sim),
    )

    return LaunchDescription(
        [
            gripper_state_publisher,
            control_node,
            delayed_spawners,
            gz_spawn_entity,
        ]
    )
