import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arz-1024/arm_ws/src/arm_gazebo/install/arm_gazebo'
