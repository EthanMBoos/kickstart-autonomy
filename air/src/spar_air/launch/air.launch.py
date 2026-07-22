"""The air stack: the TF publisher, the detector, and the behavior tree,
all under the drone's namespace. PX4 SITL runs separately (started by the
smoke script or by hand, see the README); this launch is only the ROS
side, so a BT or config change is a Ctrl-C and rerun away, PX4 untouched.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    world = LaunchConfiguration("world")

    autonomy_params = PathJoinSubstitution(
        [FindPackageShare("spar_air"), "config", ["autonomy_", world, ".yaml"]]
    )

    def behavior_node(executable):
        return Node(
            package="spar_air",
            executable=executable,
            namespace=namespace,
            parameters=[autonomy_params, {"use_sim_time": True}],
            # TransformListener subscribes to the absolute /tf; this stack
            # publishes TF namespaced. Remap it in.
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            output="screen",
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="skydio"),
            DeclareLaunchArgument("world", default_value="blank"),
            behavior_node("tf_from_px4"),
            behavior_node("anomaly_detector"),
            behavior_node("bt_executive"),
        ]
    )
