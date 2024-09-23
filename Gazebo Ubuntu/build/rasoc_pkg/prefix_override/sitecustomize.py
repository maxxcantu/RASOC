import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/maxx/Workspaces/rasoc2_ws/install/rasoc_pkg'
