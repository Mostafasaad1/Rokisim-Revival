# [file name]: main.py
# [file content begin]
import sys
import logging
from PySide6.QtWidgets import QApplication  # Add this import
from src.rokisim_revival.gui import JointControlGUI
from src.rokisim_revival.sender import RoKiSimSender
from src.rokisim_revival.config import Config

if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s"
    )
    
    # Create QApplication FIRST
    app = QApplication(sys.argv)
    
    config = Config()  # Load defaults or from file
    sender = RoKiSimSender(ip=config.ip, port=config.port)
    gui = JointControlGUI(sender)  # Create the GUI widget
    
    gui.resize(1000, 800)
    gui.show()  # Make sure to show the window
    
    sys.exit(app.exec())
