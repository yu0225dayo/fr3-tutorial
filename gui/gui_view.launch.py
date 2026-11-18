"""RViz(腕の表示) + robot_state_publisher。関節角は fr3_tk.py が /joint_states に流す。"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("fr3_compliance")
    init_yaml = os.path.join(get_package_share_directory("fairino3_v6_moveit2_config"), "config", "initial_positions.yaml")
    rd = ParameterValue(Command(["xacro ", os.path.join(share, "urdf", "fr3_compliance.urdf.xacro"), " initial_positions_file:=", init_yaml]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": rd}]),
        # 比較用: 初期姿勢(init_pose.csv)の腕。フレーム名に home/ を付けて、今の腕と重ねて表示する。関節角は fr3_tk.py が /home/joint_states に流す
        Node(package="robot_state_publisher", executable="robot_state_publisher", namespace="home",
             parameters=[{"robot_description": rd, "frame_prefix": "home/"}]),
        Node(package="tf2_ros", executable="static_transform_publisher",
             arguments=["--frame-id", "base_link", "--child-frame-id", "home/base_link"]),
        Node(package="rviz2", executable="rviz2", arguments=["-d", "/work/gui/fr3_gui.rviz"]),
    ])
