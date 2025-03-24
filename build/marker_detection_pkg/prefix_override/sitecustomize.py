import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/azakaria/Documents/sim_v2/ros2_ws/install/marker_detection_pkg'
