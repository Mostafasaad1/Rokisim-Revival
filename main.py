# main.py
import sys
import logging
from pathlib import Path

# Add src/ to Python path so 'rokisim_revival' is importable
SRC_DIR = Path(__file__).parent / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from PySide6.QtWidgets import QApplication
from rokisim_revival.gui import JointControlGUI
from rokisim_revival.sender import RoKiSimSender
from rokisim_revival.config import Config

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    
    app = QApplication(sys.argv)
    
    config = Config()
    sender = RoKiSimSender(ip=config.ip, port=config.port)
    gui = JointControlGUI(sender)
    
    gui.resize(1000, 800)
    gui.show()
    
    sys.exit(app.exec())