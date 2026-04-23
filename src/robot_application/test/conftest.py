import os
import sys
import types


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)


class _StubString:
    def __init__(self):
        self.data = ''


std_msgs_module = types.ModuleType('std_msgs')
std_msgs_msg_module = types.ModuleType('std_msgs.msg')
std_msgs_msg_module.String = _StubString
std_msgs_module.msg = std_msgs_msg_module
sys.modules.setdefault('std_msgs', std_msgs_module)
sys.modules.setdefault('std_msgs.msg', std_msgs_msg_module)


ament_module = types.ModuleType('ament_index_python')
ament_packages_module = types.ModuleType('ament_index_python.packages')
ament_packages_module.get_package_share_directory = lambda name: os.path.join(PACKAGE_ROOT)
ament_module.packages = ament_packages_module
sys.modules.setdefault('ament_index_python', ament_module)
sys.modules.setdefault('ament_index_python.packages', ament_packages_module)