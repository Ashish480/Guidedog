from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('clifford_sim_1'),
        'urdf',
        'clifford_model.urdf'
    )

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': robot_desc, 
                 'controller_config_file': os.path.join(
                     get_package_share_directory('clifford_sim_1'),
                     'config',
                     'clifford_controllers.yaml')}
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2 run controller_manager spawner joint_state_controller'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2 run controller_manager spawner joint_trajectory_controller'],
            output='screen'
        )
    ])

