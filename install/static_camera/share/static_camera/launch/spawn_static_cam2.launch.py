
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node

    
def generate_launch_description():
    # Get Package Description and Directory
    static_camera_dir = get_package_share_directory('static_camera')
    
    spawn_camera = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-param', 'robot_description',],
        parameters=[{'robot_description': Command(['xacro ', 
                                                   PathJoinSubstitution([static_camera_dir, "models", "camera.sdf.xacro"])])}],
        output='screen'
    )
    
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@gz.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
        output='screen'
    )
    
    camera_subscriber = Node(
        package='static_camera',
        executable='camera_subscriber',
        output='screen'
    )
       
    return LaunchDescription([camera_bridge, camera_subscriber, spawn_camera])
    
    


