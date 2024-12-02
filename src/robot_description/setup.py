import os
from glob import glob
from setuptools import setup

package_name = 'robot_description'

def generate_paths(dir_name):
    paths = glob(dir_name + '/**', recursive=True)
    paths = [path for path in paths if os.path.isfile(path)]
    paths = sorted(list(set(['/'.join(path.split('/')[0:-1]) for path in paths])))
    paths = [(os.path.join('share', package_name, path), glob(path + '/*.*')) for path in paths]
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ] + generate_paths(dir_name='models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'obstacle_avoidance = robot_description.obstacle_avoidance:main',
            # 'camera_subscriber = robot_description.camera_subscriber:main',
            # 'aruco_detection_pose_estimation = robot_description.aruco_detection_pose_estimation:main',
            # 'vision_node = robot_description.vision_node:main',
            # 'monitor_node = robot_description.monitor_node:main',
            # 'analyze_node = robot_description.analyze_node:main',
            # 'plan_node = robot_description.plan_node:main',
            # 'execute_node = robot_description.execute_node:main',
            # 'vision_node = robot_description.vision_node_v3:main',
            # 'monitor_node_v3 = robot_description.monitor_node_v3:main',
            # 'analyze_node_v3 = robot_description.analyze_node_v3:main',
            # 'plan_node_v3 = robot_description.plan_node_v3:main',
            # 'execute_node_v3 = robot_description.execute_node_v3:main',
            # 'monitor_new_node = robot_description.monitor_new_node:main',
            # 'analyze_new_node = robot_description.analyze_new_node:main',
            # 'plan_new_node = robot_description.plan_new_node:main',
            # 'execute_new_node = robot_description.execute_new_node:main',
            'mape_node = robot_description.mape_node:main',
            'mape_node_v2 = robot_description.mape_node_v2:main',
            'mape_node_v3 = robot_description.mape_node_v3:main',
            'mape_node_v4 = robot_description.mape_node_v4:main',
            'mape_node_v5 = robot_description.mape_node_v5:main',
            'mape_node_v6 = robot_description.mape_node_v6:main',
            'mape_node_v7 = robot_description.mape_node_v7:main',
        ],
    },
)