import os
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition

# Packages
CHARGE_PKG: Final[str] = "charger"

# Dirs
CONFIG_DIR: Final[str] = "config"

# Files
CHARGER_CONFIG: Final[str] = "charger.yaml"

# File paths
charger_params_path = os.path.join(
    get_package_share_directory(CHARGE_PKG), CONFIG_DIR, CHARGER_CONFIG
)

# Launch Arguments
ARGUMENTS: List[DeclareLaunchArgument] = [
    DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="use simulation time",
    ),
    DeclareLaunchArgument(
        name="namespace",
        default_value="",
        description="Robot namespace",
    ),
]

tf_remappings: Final[list] = [("/tf", "tf"), ("/tf_static", "tf_static")]


def launch_setup(context: LaunchContext):
    namespace = LaunchConfiguration("namespace").perform(context)

    charger_service_node = Node(
        package="charger",
        executable="charger",
        output="screen",
        name="charger_node",
        namespace=namespace,
        remappings=tf_remappings,
        emulate_tty=True,
        parameters=[charger_params_path],
    )
    
    return [
        charger_service_node,
    ]
    
def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
