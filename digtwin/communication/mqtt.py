import paho.mqtt.client as mqtt
from digtwin.communication.common import GUISideCommunicationProcess, ThetaMessage, DtComQueue, DtComCmd
import logging
import json

from digtwin.gui.nodes.dt_nodes import DTNode

_logger = logging.getLogger(__name__)


class MqttGUIComProc(GUISideCommunicationProcess):
    def __init__(self, gui_to_dt_q: DtComQueue, dt_to_gui_q: DtComQueue,
                 cmd_queue_q: DtComCmd, mqtt_address: str = "localhost", mqtt_port: int = 1883):
        super().__init__(self, gui_to_dt_q, dt_to_gui_q, cmd_queue_q)
        self._mqtt_client: mqtt.Client | None = None

        self.mqtt_address = mqtt_address
        self.mqtt_port = mqtt_port

    def connect_to_dt(self):
        _logger.info("Initializing mqtt")
        self._mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=f"client_{__name__}")
        self._mqtt_client.connect(self.mqtt_address, self.mqtt_port)
        for model_name, device_name in self._target_models.items():
            self._mqtt_client.subscribe(f"{device_name}/{model_name}/to_gui")
            _logger.info(f"Subscribed to {device_name}/{model_name}/to_gui")
        self._mqtt_client.on_message = self._on_mqtt_message
        self._mqtt_client.loop_start()
        _logger.info("Mqtt started.")

    def disconnect_from_dt(self):
        _logger.warning("Disconnecting mqtt")
        if self.is_connected():
            for model_name, device_name in self._target_models.items():
                self._mqtt_client.unsubscribe(f"{device_name}/{model_name}/to_gui/+")
            self._mqtt_client.disconnect()
            self._mqtt_client = None
        _logger.warning("Disconnected")
        # self._mqtt_client.loop_stop()

    def _on_mqtt_message(self, client, userdata, message):
        device_name, model_name, direction = message.topic.split("/")
        if direction == "to_gui":
            self.dt_to_gui.put_message(model_name, json.loads(message.payload.decode("utf-8")))

    def send_to_dt(self, msg: ThetaMessage):
        _logger.info(f"Sending {msg.values} to {msg.target_name}")
        self._mqtt_client.publish(f"{self._target_models[msg.target_name]}/{msg.target_name}/to_dt", json.dumps(msg.values))

    def is_connected(self) -> bool:
        return self._mqtt_client is not None and self._mqtt_client.is_connected()

    def _cleanup(self):
        self.disconnect_from_dt()
        super()._cleanup()



if __name__ == '__main__':
    import time
    from queue import Empty
    cmd_queue = DtComCmd()
    gui_to_dt = DtComQueue()
    dt_to_gui = DtComQueue()
    test_node = DTNode("test_model", "Smart Conveyor Panel")

    com_proc = MqttGUIComProc(gui_to_dt, dt_to_gui, cmd_queue)
    com_proc.add_target_model(test_node)
    com_proc.start()

    print("Connecting to dt.")
    cmd_queue.send_connect_cmd()
    print("Connected")


    time.sleep(2.0)
    print("Accodo messaggio")
    gui_to_dt.put_message("test_model", [1,2,3])
    time.sleep(2.0)

    print("Waiting for data...")
    try:
        msg = dt_to_gui.get(True, 10)
        print(msg)
    except Empty:
        print("No data received")

    print("Sending close command.")
    cmd_queue.send_close_cmd()
    print("Closing")
    time.sleep(2.0)




    com_proc.join()
    com_proc.close()
    print("Closed")













