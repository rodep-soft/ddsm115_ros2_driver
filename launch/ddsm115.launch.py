from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('ddsm115_ros2_driver')
    
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for DDSM115 motor controller'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    motor_id_arg = DeclareLaunchArgument(
        'motor_id',
        default_value='1',
        description='Motor ID for DDSM115 controller'
    )
    
    # Create the node
    ddsm115_node = Node(
        package='ddsm115_ros2_driver',
        executable='ddsm115_node',
        name='ddsm115_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'motor_id': LaunchConfiguration('motor_id'),
            'publish_rate': 10.0
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        motor_id_arg,
        ddsm115_node
    ])
