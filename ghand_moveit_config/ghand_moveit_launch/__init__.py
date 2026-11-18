"""Shared planning nodes for preview, simulation, and physical hardware."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def planning_nodes(product, side, description, use_rviz, *, use_sim_time=False, execute=True):
    prefix = "dual_" if side == "dual" else ""
    model = Path(get_package_share_directory("ghand_description")) / f"urdf/{product}/{product}_{side}_description.urdf"
    config = (MoveItConfigsBuilder(f"{product}-{side}", package_name="ghand_moveit_config")
        .robot_description(file_path=str(model))
        .robot_description_semantic(file_path=f"config/{product}/{product}_{side}.srdf")
        .trajectory_execution(file_path=f"config/{product}/{prefix}moveit_controllers.yaml")
        .joint_limits(file_path=f"config/{product}/{prefix}joint_limits.yaml")
        .robot_description_kinematics(file_path=f"config/{product}/{prefix}kinematics.yaml")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .to_moveit_configs())
    config.robot_description = description
    parameters = [config.to_dict(), {"use_sim_time": use_sim_time,
        "allow_trajectory_execution": execute, "moveit_manage_controllers": False,
        "publish_robot_description_semantic": True}]
    share = Path(get_package_share_directory("ghand_moveit_config"))
    return [Node(package="moveit_ros_move_group", executable="move_group",
                 parameters=parameters, output="screen"),
            Node(package="rviz2", executable="rviz2",
                 arguments=["-d", str(share / f"config/{product}/{prefix}moveit.rviz")],
                 parameters=parameters, condition=IfCondition(use_rviz), output="screen")]
