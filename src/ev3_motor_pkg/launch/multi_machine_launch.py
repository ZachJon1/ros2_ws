from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ROS_DOMAIN_ID for network configuration
    domain_id_arg = DeclareLaunchArgument(
        'domain_id', 
        default_value='0', 
        description='ROS_DOMAIN_ID for network configuration'
    )

    # Camera node
    camera_node = Node(
        package='camera_pkg',
        executable='camera_node',
        name='camera_publisher',
        parameters=[{'domain_id': LaunchConfiguration('domain_id')}]
    )

    # Marker detection node
    marker_detector_node = Node(
        package='marker_detection_pkg',
        executable='marker_detector',
        name='marker_position_detector',
        parameters=[{'domain_id': LaunchConfiguration('domain_id')}]
    )

    # EV3 Motor controller node
    motor_controller_node = Node(
        package='ev3_motor_pkg',
        executable='motor_controller',
        name='ev3_motor_control',
        parameters=[{'domain_id': LaunchConfiguration('domain_id')}]
    )

    return LaunchDescription([
        domain_id_arg,
        camera_node,
        marker_detector_node,
        motor_controller_node
    ])