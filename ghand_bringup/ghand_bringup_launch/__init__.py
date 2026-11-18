"""Launch actions shared by the hardware and simulation scenes."""

from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def ros_log_arguments():
    return ["--log-level", LaunchConfiguration("log_level", default="info")]


def controller_spawner(name, *options, params_file):
    return Node(
        package="controller_manager", executable="spawner",
        arguments=[name, "-c", "/controller_manager",
                   "--controller-manager-timeout", "30",
                   "--switch-timeout", "30", "--service-call-timeout", "30",
                   "--param-file", params_file, *options],
        ros_arguments=ros_log_arguments(), output="screen",
    )


def wait_for_control(stage, name):
    return Node(
        package="ghand_bringup", executable="wait_for_control",
        arguments=[stage, name], ros_arguments=ros_log_arguments(), output="screen",
    )


def trajectory_stages(*trajectories, params_file):
    stages = [
        (f"activate {name}", controller_spawner(name, params_file=params_file))
        for name in ("joint_state_broadcaster", *trajectories)
    ]
    stages.extend(
        (f"wait for {name} action", wait_for_control("trajectory", name))
        for name in trajectories
    )
    return stages


def start_in_order(stages, ready=()):
    """Register the exit handlers before starting the first required stage."""
    def after(stage, next_actions):
        def on_exit(event, context):
            if context.is_shutdown:
                return []
            if event.returncode == 0:
                return list(next_actions)
            # A launch exception also propagates to the CLI exit status.
            raise RuntimeError(f"GHand {stage} failed (exit {event.returncode})")
        return on_exit

    handlers = []
    for index, (stage, action) in enumerate(stages):
        next_actions = [stages[index + 1][1]] if index + 1 < len(stages) else ready
        handlers.append(RegisterEventHandler(OnProcessExit(
            target_action=action, on_exit=after(stage, next_actions),
        )))
    return [*handlers, stages[0][1]]


def planning_after_start(product, side, description, use_rviz, *, use_sim_time=False):
    from ghand_moveit_launch import planning_nodes

    return planning_nodes(product, side, description, use_rviz, use_sim_time=use_sim_time)
