import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shanti/WayneHacks3/Lidar_Interpolation/install/lidar_interpolation_node'
