import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/shameer/sjtu_drone-ros2/sjtu_drone_control/install/sjtu_drone_control'
