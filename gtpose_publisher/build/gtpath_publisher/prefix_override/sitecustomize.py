import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shameer/Documents/SLAM/gtpose_publisher/install/gtpath_publisher'
