import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def launch_setup(context):
    share = get_package_share_directory("fr3_compliance")
    # 初期姿勢は moveit 設定側（init_pose.csv の値）と同じものを使う
    init_yaml = os.path.join(get_package_share_directory("fairino3_v6_moveit2_config"), "config", "initial_positions.yaml")
    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(share, "urdf", "fr3_compliance.urdf.xacro"), " initial_positions_file:=", init_yaml]),
        value_type=str,
    )
    controllers = os.path.join(share, "config", "controllers.yaml")

    def spawner(name, extra=()):
        return Node(package="controller_manager", executable="spawner", arguments=[name, "--controller-manager", "/controller_manager", *extra])

    mode = context.launch_configurations.get("mode", "wrist")
    common = [
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": robot_description}]),
        Node(package="controller_manager", executable="ros2_control_node", parameters=[{"robot_description": robot_description}, controllers], output="screen"),
        spawner("joint_state_broadcaster"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", os.path.join(share, "rviz", "compliance.rviz")]),
    ]
    if mode == "joint":
        # 関節モード: リンクの途中にかけた力で、関節がばねのように動く（push_link.sh で力をかける）
        return common + [
            spawner("forward_position_controller"),
            Node(package="fr3_compliance", executable="link_compliance.py", output="screen",
                 parameters=[{"model": context.launch_configurations.get("model", "fairino")}]),
        ]
    # 手先モード（既定）: 手先の偽の力覚センサ + admittance_controller
    return common + [
        spawner("fake_ft_command"),
        spawner("admittance_controller"),
        # 基準姿勢（= 力がないときに戻る姿勢）を流し続ける。初期姿勢と同じ値
        ExecuteProcess(cmd=["ros2", "topic", "pub", "-r", "20", "/admittance_controller/joint_references",
                            "trajectory_msgs/msg/JointTrajectoryPoint",
                            "{positions: [-0.19073000463318587,-1.6277927678125543,1.3478830856525392,-1.2907574813220124,-1.570728016712936,-1.7617578932848648], velocities: [0,0,0,0,0,0]}"],
                       output="log"),
        Node(package="fr3_compliance", executable="force_arrow.py", output="log"),
    ]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("mode", default_value="wrist"), DeclareLaunchArgument("model", default_value="fairino"), OpaqueFunction(function=launch_setup)])
