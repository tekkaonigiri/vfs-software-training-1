import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kenan/Code/VFS/vfs-software-training-1/install/navigation_py'
