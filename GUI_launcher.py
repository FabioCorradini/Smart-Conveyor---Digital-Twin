import logging
from digtwin.gui.p3d_gui import P3dGui
from digtwin.gui.qt_gui import QtGui
from PyQt5.QtWidgets import QApplication
from pathlib import Path
import signal
import sys

_logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)


def main():
    _logger.info("gui launched")

    _logger.info("3D env init...")

    gui = P3dGui()
    gui.load_objects()
    gui.start_communication()

    signal.signal(signal.SIGINT, gui.exit_program)
    signal.signal(signal.SIGTERM, gui.exit_program)

    _logger.info("UI init...")
    interface_app = QApplication(sys.argv)
    inter_gui = QtGui(gui)


    try:
        ret = interface_app.exec_()
    except Exception as e:  # qt does not raise exception automatically
        _logger.error(e)
        ret = -1
    finally:
        _logger.warning("Closing main...")
        gui.exit_program()

    sys.exit(ret)

if __name__ == '__main__':
    main()
