import time
from asyncua import Server, Node, ua, uamethod
from digtwin.physics.robot import Robot, Command
import paho.mqtt.client as mqtt
import asyncio
import logging
import signal
import numpy as np
import json

_logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.INFO)
logging.getLogger("asyncua").setLevel(logging.WARNING)



class OPCUARobot:

    _data_node = Node | None
    _go_to_node: Node | None
    _rotate_to_node: Node | None
    _move_paddle_node: Node | None
    _is_running_node: Node | None
    _command_queue_node: Node | None
    _status_node: Node | None
    _current_position_node: Node | None
    _current_rotation_node: Node | None
    _current_paddle_position_node: Node | None
    _current_command_node: Node | None
    _current_feedrate_node: Node | None

    _pause_node: Node | None
    _resume_node: Node | None
    _stop_node: Node | None

    _action_node: Node | None

    _set_feedrate_node: Node | None

    _robot_task = None | asyncio.Task
    _server_task = None | asyncio.Task



    def __init__(self,name: str,  opcua_host: str = "0.0.0.0", opcua_port: int = 4840, mqtt_host: str= "127.0.0.1", mqtt_port: int = 1883):
        self._robot = Robot()
        self._opcua_server = Server()
        self.opcua_host = opcua_host
        self.opcua_port = opcua_port
        self.mqtt_host = mqtt_host
        self.mqtt_port = mqtt_port
        self.name: str = name
        self._idx = 0
        self._feedrate = 1.0
        self._default_shift_speed = self._robot.shift_speed
        self._default_rot_speed = self._robot.rot_speed
        self._default_paddle_speed = self._robot.paddle_speed
        self.closing = False

        self._data_node = None
        self._action_node = None
        self._go_to_node = None
        self._rotate_to_node = None
        self._move_paddle_node = None
        self._is_running_node = None
        self._command_queue_node = None
        self._status_node = None
        self._current_position_node = None
        self._current_rotation_node = None
        self._current_paddle_position_node = None
        self._current_command_node = None
        self._current_feedrate_node = None
        self._pause_node = None
        self._resume_node = None
        self._stop_node = None
        self._action_node = None
        self._set_feedrate_node = None
        self._robot_task = None
        self._server_task = None
        self.sub = None
        self.sub_handle = None

        self._old_robot_theta = None
        self._old_paddle_theta = None
        self._old_is_running = None
        self._old_command_queue = None
        self._old_status = None
        self._status = ""
        self._old_command_str = ""
        self._old_feedrate = None

        self._mqtt_client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=name)


    async def init(self):
        await self._opcua_server.init()
        self._opcua_server.set_endpoint(f"opc.tcp://{self.opcua_host}:{self.opcua_port:d}/{self.name}/server")
        self._opcua_server.set_server_name(self.name)
        self._idx = await self._opcua_server.register_namespace(f"http://{self.name}.io")
        self._data_node = await self._opcua_server.nodes.objects.add_object(self._idx, "Data")
        self._action_node = await self._opcua_server.nodes.objects.add_object(self._idx, "Action")

        self._go_to_node = await self._data_node.add_variable(self._idx, "go to", [0.0, 0.0])
        await self._go_to_node.set_writable(True)
        self._rotate_to_node = await self._data_node.add_variable(self._idx, "rotate to", 0.0)
        await self._rotate_to_node.set_writable(True)
        self._move_paddle_node = await self._data_node.add_variable(self._idx, "move paddle", 0.0)
        await self._move_paddle_node.set_writable(True)
        self._is_running_node = await self._data_node.add_variable(self._idx, "is running", False)
        self._command_queue_node = await self._data_node.add_variable(self._idx, "command queue", 0)
        self._status_node = await self._data_node.add_variable(self._idx, "Status", "")
        self._current_position_node = await self._data_node.add_variable(self._idx, "current position", [0.0, 0.0])
        self._current_rotation_node = await self._data_node.add_variable(self._idx, "current rotation", 0.0)
        self._current_paddle_position_node = await self._data_node.add_variable(self._idx, "current paddle position", 0.0)
        self._current_command_node = await self._data_node.add_variable(self._idx, "current command", "")
        self._current_feedrate_node = await self._data_node.add_variable(self._idx, "current feedrate", self._feedrate)

        self._pause_node = await self._action_node.add_method(self._idx, "pause", self.set_pause)
        self._resume_node = await self._action_node.add_method(self._idx, "resume", self.set_resume)
        self._stop_node = await self._action_node.add_method(self._idx, "stop", self.set_stop)

        self._set_feedrate_node = await self._action_node.add_method(self._idx, "set feedrate", self.set_feedrate, [ua.VariantType.Double], [ua.VariantType.Double, ua.VariantType.Double, ua.VariantType.Double])

        self.sub = await self._opcua_server.create_subscription(500, self)
        self.sub_handle = await self.sub.subscribe_data_change([self._go_to_node, self._rotate_to_node, self._move_paddle_node])

        self._robot_task = asyncio.create_task(self.robot_main())
        self._server_task = asyncio.create_task(self.server_main())

        _logger.info("Initializing mqtt")
        self._mqtt_client.connect(self.mqtt_host, self.mqtt_port)
        self._mqtt_client.loop_start()

    @uamethod
    def set_pause(self, parent):
        self._robot.pause()
        self._status = "Paused"

    @uamethod
    def set_resume(self, parent):
        self._robot.resume()
        self._status = "Resumed"

    @uamethod
    def set_stop(self, parent):
        self._robot.stop()
        self._status = "Stopped"

    @uamethod
    def set_feedrate(self, parent, new_feedrate: float):
        self._feedrate = min(max(new_feedrate, 0.1), 5.0)
        self._robot.shift_speed = self._feedrate * self._default_shift_speed
        self._robot.rot_speed = self._feedrate * self._default_rot_speed
        self._robot.paddle_speed = self._feedrate * self._default_paddle_speed
        return self._robot.shift_speed, self._robot.rot_speed*180/np.pi , self._robot.paddle_speed


    def _send_to_gui(self, data: list | tuple, target_device: str):
        self._mqtt_client.publish(f"{self.name}/{target_device}/to_gui", json.dumps(data))

    def datachange_notification(self, node, val, data):
        if str(node) == str(self._go_to_node):
            self._robot.move_to(val[0], val[1])
        elif str(node) == str(self._rotate_to_node):
            self._robot.rotate(val*np.pi/180)
        elif str(node) == str(self._move_paddle_node):
            self._robot.move_paddle(val)
        else:
            _logger.warning("Python: unknown change event %s %s", node, val)


    async def robot_main(self):
        _logger.info("Robot main started")
        while not self.closing:
            self._robot.run(time.time())

            if self._old_robot_theta != self._robot.get_robot_theta():
                self._send_to_gui(self._robot.get_robot_theta(), "smart_helper_node")
                await self._current_position_node.write_value(self._robot.get_robot_position())
                await self._current_rotation_node.write_value(self._robot.get_robot_angle() * 180 / np.pi)
                self._old_robot_theta = self._robot.get_robot_theta()

            if self._old_paddle_theta != self._robot.get_paddle_theta():
                self._send_to_gui([self._robot.get_paddle_theta()], "smart_helper_paddle_node")
                await self._current_paddle_position_node.write_value(self._robot.get_paddle_theta())
                self._old_paddle_theta = self._robot.get_paddle_theta()

            if self._old_is_running != self._robot.is_busy():
                if self._robot.is_busy() and not self._robot.is_paused():
                    self._status = "Running"
                elif not self._robot.is_busy() and not self._robot.is_paused():
                    self._status = "idle"

                await self._is_running_node.write_value(self._robot.is_busy())
                self._old_is_running = self._robot.is_busy()

            if self._old_command_queue != self._robot.command_queue_len():
                await self._command_queue_node.write_value(self._robot.command_queue_len())
                self._old_is_running = self._robot.command_queue_len()

            if self._old_status != self._status:
                await self._status_node.write_value(self._status)
                self._old_status = self._status

            new_cmd_str = self._robot.get_state()
            if self._old_command_str != new_cmd_str:
                await self._current_command_node.write_value(new_cmd_str)
                self._old_command_str = new_cmd_str

            if self._old_feedrate != self._feedrate:
                await self._current_feedrate_node.write_value(self._feedrate)
                self._old_feedrate = self._feedrate


            await asyncio.sleep(0.05)
        _logger.info("Robot main stopped")

    async def server_main(self):
        _logger.info("OPCUA server main started")
        async with self._opcua_server:
            while not self.closing:
                await asyncio.sleep(1.0)
        _logger.info("OPCUA server main stopped")

    def close_all(self, signal, frame):
        self.closing = True
        self._mqtt_client.disconnect()

    async def wait_for_server(self):
        await self._server_task
        await self._robot_task



async def main():
    opcua_robot = OPCUARobot("smart_conveyor_helper")
    signal.signal(signal.SIGINT, opcua_robot.close_all)
    signal.signal(signal.SIGTERM, opcua_robot.close_all)
    await opcua_robot.init()
    await opcua_robot.wait_for_server()


if __name__ == '__main__':
    asyncio.run(main())









