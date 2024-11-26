import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import (Command, LaunchConfiguration)
from launch_ros.actions import (Node, SetParameter)
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter

# ROS2 Launch System will look for this function definition #
def generate_launch_description():

    # Get Package Description and Directory #
    package_description = "robot_description"
    package_directory = get_package_share_directory(package_description)
    
    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, "meshes")
    pkg_models_path = os.path.join(package_directory, "models") # add local models path
    gazebo_resource_paths = [install_dir_path, robot_meshes_path, pkg_models_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))
    
    
    world_file = "project_world.sdf"
    world_file_path = os.path.join(package_directory, "worlds", world_file)
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument("world",
                                              default_value=[world_file_path],
                                              description="SDF World File")
    
    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": world_config}.items(),
    )
    # Load URDF File #
    urdf_file = 'robot.xacro'
    robot_desc_path_0 = os.path.join(package_directory, "urdf", urdf_file)
    robot_desc_path_1 = os.path.join(package_directory, "urdf", "robot_2.xacro")
    print("URDFs Loaded !")

    # Robot State Publisher (RSP) #
    robot_state_publisher_node_0 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace='robot_0',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': Command(['xacro ', robot_desc_path_0])}]
    )
    
    robot_control_node_0 = Node(
        package = 'robot_description',
        namespace = 'robot_0',
        executable = 'multi_robot_control',
        name = 'robot_0_controller',
        parameters=[
            {'robot_id': '12'},
        ],
        arguments=['12']
    )
    
    ign_bridge_node= Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ign_bridge",
        arguments=[
            "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
            "/robot_12/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/robot_13/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
            "/robot_12/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/robot_13/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
            "/robot_12/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            "/robot_13/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
            "/robot_12/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            "/robot_13/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
            "/robot_12/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
            "/robot_13/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
            "/camera@sensor_msgs/msg/Image@gz.msgs.Image",
            "/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
        ],
        remappings=[

        ],
        output="screen",
    )
    
    robot_state_publisher_node_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        namespace='robot_1',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 
                     'robot_description': Command(['xacro ', robot_desc_path_1])}]
    )
    
    robot_control_node_1 = Node(
        package='robot_description',
        namespace='robot_1',
        executable='multi_robot_control',
        name='robot_1_controller',
        parameters=[
            {'robot_id': '13'},
        ],
        arguments=['13']
    )
    
    spawn_robot_0 = Node(
        package="ros_gz_sim",
        executable="create",
        name="robot_0_spawn",
        arguments=[
            "-name", "robot_0",
            "-allow_renaming", "false",
            "-topic", "robot_0/robot_description",
            "-x", "-3.0",
            "-y", "0.0",
            "-z", "0.05",
        ],
        output="screen",
    )
    
    spawn_robot_1 = Node(
        package="ros_gz_sim",
        executable="create",
        name="robot_1_spawn",
        arguments=[
            "-name", "robot_1",
            "-allow_renaming", "false",
            "-topic", "robot_1/robot_description",
            "-x", "3.0",
            "-y", "-1.5",
            "-z", "0.05",
        ],
        output="screen",
    )
    
    multi_robot_control_node = Node(
        package='robot_description',
        executable='multi_robot_control',
        name='multi_robot_control',
        parameters=[
            {'robot_ids': '12 13'},
        ],
        arguments=['12 13']
    )
    
    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),
        declare_world_arg,
        gz_sim,
        robot_state_publisher_node_0,
        robot_control_node_0,
        ign_bridge_node,
        robot_state_publisher_node_1,
        robot_control_node_1,
        spawn_robot_0,
        spawn_robot_1,
        multi_robot_control_node
    ])