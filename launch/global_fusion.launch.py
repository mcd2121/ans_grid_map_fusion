import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    ans_grid_map_fusion_dir = get_package_share_directory('ans_grid_map_fusion')
    global_fusion_config_file = LaunchConfiguration('config_file')
    # Arguments 
    declare_fusion_config_file_cmd = DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(
                            ans_grid_map_fusion_dir, 'config', 'global_fusion_params.yaml'), # Specify the path to your YAML file
            description='Path to the parameter file'
        )

    # nodes 
    ans_grid_map_fusion_node = Node(
            package='ans_grid_map_fusion',
            executable='global_fusion_node',
            name='global_fusion_node',
            parameters=[LaunchConfiguration('config_file')],
            output='screen'
        )
    
    # launch description
    ld = LaunchDescription()
    ld.add_action(declare_fusion_config_file_cmd)
    ld.add_action(ans_grid_map_fusion_node)
    return ld