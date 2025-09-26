import sys
import logging
from rokisim_revival.gui import JointControlGUI
from rokisim_revival.sender import RoKiSimSender
from rokisim_revival.config import Config

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    config = Config()  # Load defaults or from file
    sender = RoKiSimSender(ip=config.ip, port=config.port)
    app = JointControlGUI(sender)
    app.resize(1000, 800)
    sys.exit(app.exec())