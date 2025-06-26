import time
from multiprocessing import SimpleQueue, Process, Queue
from queue import Empty
from dataclasses import dataclass
from collections.abc import Callable
import logging
from digtwin.gui.nodes.dt_nodes import DTNode
from digtwin.gui.models.dt_models import DTStatefulModel
import signal

from digtwin.gui.qt_nodes.dt_chek_nodes import DTCheckNode

_logger = logging.getLogger(__name__)

@dataclass
class ThetaMessage:
    target_name: str
    values: list[float | int]


class DtComQueue:
    def __init__(self):
        self._queue = Queue()  # cannot inherit multiprocessing queue

    def put_message(self, target_name: str, values: list[float | int]) -> None:
        self._queue.put(ThetaMessage(target_name, values))

    def empty(self) -> bool:
        return self._queue.empty()

    def get(self, block=True, timeout=None) -> ThetaMessage:
        return self._queue.get(block, timeout)

    def close(self) -> None:
        self._queue.close()


class DtComCmd:
    CLOSE_CMD = "X"
    CONNECT_CMD = "C"
    DISCONNECT_CMD = "D"
    WATCHDOG_CMD = "W"


    def __init__(self):
        self._queue = SimpleQueue()
        self.conn_address = ""
        self.conn_port = 0
        self._close_callback = None
        self._connect_callback = None
        self._disconnect_callback = None
        self._watchdog_callback = None

    def send_close_cmd(self):
        self._queue.put(self.CLOSE_CMD)

    def send_connect_cmd(self):
        self._queue.put(self.CONNECT_CMD)

    def send_disconnect_cmd(self):
        self._queue.put(self.DISCONNECT_CMD)

    def send_watchdog_cmd(self):
        self._queue.put(self.WATCHDOG_CMD)

    def set_close_callback(self, callback: Callable[[], None]):
        self._close_callback = callback

    def set_connect_callback(self, callback: Callable[[], None]):
        self._connect_callback = callback

    def set_disconnect_callback(self, callback: Callable[[], None]):
        self._disconnect_callback = callback

    def set_watchdog_callback(self, callback: Callable[[], None]):
        self._watchdog_callback = callback

    def read_commands(self):
        if not self._queue.empty():
            cmd = self._queue.get()
            if cmd == self.CLOSE_CMD:
                _logger.warning("Received close command")
                self._close_callback()
            elif cmd == self.CONNECT_CMD:
                _logger.warning("Received connect command")
                self._connect_callback()
            elif cmd == self.DISCONNECT_CMD:
                _logger.warning("Received disconnect command")
                self._disconnect_callback()
            elif cmd == self.WATCHDOG_CMD:
                self._watchdog_callback()
            else:
                _logger.error(f"Received unknown command: {cmd}")


    def close(self) -> None:
        self._queue.close()




class GUISideCommunicationProcess(Process):

    def __init__(self, dt_name, gui_to_dt: DtComQueue, dt_to_gui: DtComQueue, cmd_queue: DtComCmd):
        super().__init__()
        self.gui_to_dt = gui_to_dt
        self.dt_to_gui = dt_to_gui
        self.cmd_queue = cmd_queue
        self._closing = False
        self.cmd_queue.set_close_callback(self.close_all)
        self.cmd_queue.set_connect_callback(self.connect_to_dt)
        self.cmd_queue.set_disconnect_callback(self.disconnect_from_dt)
        self.cmd_queue.set_watchdog_callback(self._reset_watchdog)
        self._target_models: dict[str, str] = {}
        self._watchdog_timeout = 5.0
        self._watchdog_timer = None


    def add_target_model(self, target: DTNode | DTStatefulModel | DTCheckNode):
        if not target.name in self._target_models:
            self._target_models[target.name] =  target.source_dev
        else:
            raise ValueError(f"Target model: {target.name} already exists")

    def _reset_watchdog(self):
        # _logger.info("Resetting watchdog timer")
        self._watchdog_timer = time.time()

    def ping_watchdog(self):
        self.cmd_queue.send_watchdog_cmd()

    def connect_to_dt(self):
        raise NotImplementedError()

    def disconnect_from_dt(self):
        raise NotImplementedError()

    def close_all(self):
        self._closing = True

    def is_connected(self) -> bool:
        raise NotImplementedError()

    def _cleanup(self):
        self.gui_to_dt.close()
        self.dt_to_gui.close()
        self.cmd_queue.close()

    def send_to_dt(self, msg: ThetaMessage):
        raise NotImplementedError()

    def run(self):
        logging.basicConfig(level=logging.INFO)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        while not self._closing:
            try:
                msg = self.gui_to_dt.get(True, 1.0)
                if self.is_connected():
                    self.send_to_dt(msg)
                else:
                    _logger.warning(f"Could not send message to {msg.target_name}")
            except Empty:
                pass

            self.cmd_queue.read_commands()
            if self._watchdog_timer is not None and time.time() - self._watchdog_timer > self._watchdog_timeout:
                _logger.error("Watchdog timeout")
                self.close_all()


        _logger.info("End of main loop")
        self._cleanup()