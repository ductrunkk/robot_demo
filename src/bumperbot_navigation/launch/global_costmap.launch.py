import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    global_costmap_config = LaunchConfiguration("global_costmap_config")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time if true"
    )

    global_costmap_config_arg = DeclareLaunchArgument(
        "global_costmap_config",
        default_value=os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "config",
            "global_costmap.yaml"
        ),
        description="Full path to the global costmap yaml file"
    )

    # Node for the global costmap
    global_costmap_node = Node(
        package="nav2_costmap_2d",
        executable="costmap_2d_node",
        name="global_costmap",
        output="screen",
        emulate_tty=True,
        parameters=[
            global_costmap_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        global_costmap_config_arg,
        global_costmap_node,
    ])
