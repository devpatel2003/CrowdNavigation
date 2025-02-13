import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/devpatel/crowd_nav_ws/install/yahboomcar_description'
