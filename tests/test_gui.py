import pytest
from PySide6.QtWidgets import QApplication
from src.rokisim_revival.gui import JointControlGUI
from src.rokisim_revival.sender import RobotDefinition, RoKiSimSender


@pytest.fixture(scope="module")
def qapp():
    return QApplication([])


def test_gui_init(qapp):
    sender = RoKiSimSender()
    gui = JointControlGUI(sender)
    assert gui.tabs.count() == 2
    assert gui.tabs.tabText(0) == "Joint Control"
    assert gui.tabs.tabText(1) == "Program Editor"
    # Further tests would require QTest for interactions
