from launch import LaunchDescription, LaunchContext
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    robot_description_path = PathJoinSubstitution(
        [FindPackageShare("description"), "urdf", "athena_arm.urdf.xacro"]
    )
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("arm_bringup"), "config", "athena_arm_controllers.yaml"]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("description"), "rviz", "rviz_config.rviz"]
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", robot_description_path,
        " use_mock_hardware:=true",
    ])
    robot_description = {"robot_description": robot_description_content}

    moveit_config = (
        MoveItConfigsBuilder("athena_arm", package_name="arm_moveit")
        .robot_description(file_path=robot_description_path.perform(LaunchContext()))
        .robot_description_semantic("srdf/athena_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=False, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"],
            default_planning_pipeline="ompl",
        )
        .to_moveit_configs()
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[robot_controllers],
        remappings=[("~/robot_description", "/robot_description")],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    hello_moveit_node = Node(
        package="arm_moveit",
        executable="hello_moveit",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    jtc_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    delay_jsb = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=control_node,
            on_start=[TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner])],
        )
    )

    delay_jtc = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[jtc_spawner],
        )
    )

    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_hello_moveit = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jtc_spawner,
            on_exit=[TimerAction(period=5.0, actions=[hello_moveit_node])],
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_publisher,
        move_group_node,
        delay_jsb,
        delay_jtc,
        delay_rviz,
        delay_hello_moveit,
    ])
