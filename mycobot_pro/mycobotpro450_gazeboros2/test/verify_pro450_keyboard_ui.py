"""Offscreen GUI event check; does not construct a ROS node or robot client."""

import os
import sys

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from python_qt_binding.QtCore import Qt  # noqa: E402
from python_qt_binding.QtTest import QTest  # noqa: E402
from python_qt_binding.QtWidgets import QApplication  # noqa: E402
from teleop_keyboard_gazebo import SimulationHoldWindow  # noqa: E402


class FakeNode:
    def __init__(self):
        self.gear = 2
        self.commands = []

    def hold_speed_status(self):
        return self.gear, float(self.gear)

    def set_hold_speed_gear(self, gear):
        self.gear = gear

    def press_hold(self, axis, direction):
        self.commands.append((axis, direction))
        return "accepted"

    def release_hold(self):
        pass

    def hold_tick(self):
        pass

    def hold_idle(self):
        return True

    def stop(self):
        pass


def main():
    app = QApplication.instance() or QApplication(["pro450_ui_check"])
    node = FakeNode()
    window = SimulationHoldWindow(node)
    window.show()
    QTest.keyClick(window.window, Qt.Key_5)
    assert node.gear == 5, "number key did not set speed gear"
    QTest.keyClick(window.window, Qt.Key_Minus)
    assert node.gear == 4, "minus key did not reduce speed"
    QTest.keyClick(window.window, Qt.Key_Plus)
    assert node.gear == 5, "plus key did not increase speed"
    QTest.keyClick(window.window, Qt.Key_R)
    QTest.keyClick(window.window, Qt.Key_F)
    assert node.commands[-2:] == [(2, 1), (2, -1)], node.commands
    window.request_exit("test")
    app.processEvents()
    print("PASS: Qt speed keys, J3 +/- events, and window exit")


if __name__ == "__main__":
    main()
