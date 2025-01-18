import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/blaine/WayneHacks3/360_Cam_Lidar_Fusion/install/fusion'
