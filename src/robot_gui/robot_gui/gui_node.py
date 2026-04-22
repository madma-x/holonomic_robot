"""Entry point for the robot GUI application."""

from __future__ import annotations

import sys

import rclpy
from PySide6.QtWidgets import QApplication

from robot_gui.main_window import MainWindow
from robot_gui.ros_interface import RobotGuiRosInterface, start_executor_thread


def main(args=None):
    rclpy.init(args=args)

    ros_node = RobotGuiRosInterface()
    executor, spin_thread = start_executor_thread(ros_node)

    app = QApplication.instance() or QApplication(sys.argv)
    window = MainWindow(ros_node)

    fullscreen = bool(ros_node.get_parameter('fullscreen').value) if ros_node.has_parameter('fullscreen') else False
    title = str(ros_node.get_parameter('window_title').value) if ros_node.has_parameter('window_title') else 'Robot GUI'
    min_width = int(ros_node.get_parameter('min_width').value) if ros_node.has_parameter('min_width') else 720
    min_height = int(ros_node.get_parameter('min_height').value) if ros_node.has_parameter('min_height') else 1080

    window.setWindowTitle(title)
    window.setMinimumSize(min_width, min_height)

    if fullscreen:
        window.showFullScreen()
    else:
        window.show()

    exit_code = app.exec()

    executor.shutdown()
    ros_node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    if spin_thread.is_alive():
        spin_thread.join(timeout=0.5)

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
