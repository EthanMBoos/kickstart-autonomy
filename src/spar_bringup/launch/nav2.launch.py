"""Nav2, configured from this repo.

Speed limits, costmaps, and controller tuning live in
spar_bringup/config/nav2.yaml (vx_max is the top-speed knob; see the
comments there).
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap

from nav2_common.launch import RewrittenYaml

ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time", default_value="true", choices=["true", "false"]
    ),
    DeclareLaunchArgument("namespace", default_value="husky"),
]


def launch_setup(context, *args, **kwargs):
    pkg_spar_bringup = get_package_share_directory("spar_bringup")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace").perform(context)

    rewritten_parameters = RewrittenYaml(
        source_file=os.path.join(pkg_spar_bringup, "config", "nav2.yaml"),
        # The only *.topic parameters in the file are the costmap scan topics.
        param_rewrites={"topic": f"/{namespace}/sensors/lidar2d_0/scan"},
        convert_types=True,
    )

    nav2 = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap(f"/{namespace}/odom", f"/{namespace}/platform/odom"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [pkg_nav2_bringup, "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("params_file", rewritten_parameters),
                    ("use_composition", "False"),
                    ("namespace", namespace),
                ],
            ),
        ]
    )

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
