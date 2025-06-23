import asyncio
import time

import paho.mqtt.client as mqtt
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.server import StartAsyncTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
from pymodbus import __version__ as pymodbus_version
from dataclasses import dataclass
from enum import Enum
import logging
import json

_logger = logging.getLogger(__name__)

class ModVarType(Enum):
    BOOLEAN = "BOOLEAN"
    INT16 = "INT16"

class ModRegType(Enum):
    DISCRETE_INPUT = "DISCRETE_INPUT"
    COILS = "COILS"
    INPUT_REGISTER = "INPUT_REGISTER"
    HOLDING_REGISTER = "HOLDING_REGISTER"


@dataclass
class ModVar:
    name: str
    address: int
    default_value: int | float
    reg_type: ModRegType
    changed: bool = False

@dataclass
class InternalVar:
    name: str
    target_device: str
    value: list[float | int | bool]
    changed: bool = False



class PLCSubsystem:
    COILS_START = 1
    DISCRETE_INPUT_START =   1001
    INPUT_REGISTER_START =   3001
    HOLDING_REGISTER_START = 4001

    _discrete_input_memory: ModbusSequentialDataBlock | None
    _coils_memory: ModbusSequentialDataBlock | None
    _holding_register_memory: ModbusSequentialDataBlock | None
    _input_registers_memory: ModbusSequentialDataBlock | None
    _modbus_context: ModbusServerContext | None
    _server_task: asyncio.Task | None

    def __init__(self, name: str, modbus_address: str | None = None, modbus_port: int = 5020, mqtt_address: str = "localhost", mqtt_port: int = 1883) -> None:
        self.name = name
        self._external_input_vars: dict[str, ModVar] = {}
        self._external_output_vars: dict[str, ModVar] = {}
        self._internal_input_vars: dict[str, InternalVar] = {}
        self._internal_output_vars: dict[str, InternalVar] = {}

        self._mqtt_client =  mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id=name)
        self._modbus_identification = ModbusDeviceIdentification(
            info_name={
                "VendorName": "SUPSI - DTI",
                "ProductCode": "DTI 001",
                "VendorUrl": "https://moodle.msengineering.ch/mod/data/view.php?d=127&advanced=0&paging&filter=1&page=0&rid=5804",
                "ProductName": self.name,
                "ModelName": self.name,
                "MajorMinorRevision": pymodbus_version
            }
        )

        self._discrete_input_memory = None
        self._coils_memory = None
        self._holding_register_memory = None
        self._input_registers_memory = None
        self._modbus_context = None
        self._server_task = None
        self.modbus_address = modbus_address
        self.modbus_port = modbus_port
        self._closing = False
        self.mqtt_address = mqtt_address
        self.mqtt_port = mqtt_port
        self.event_loop = asyncio.get_event_loop()

        # debug

        self._main_loop_timer = time.time()
        self._loop_time_limit = 0.05


    async def init(self):
        _logger.info("Initializing Modbus...")
        di_var = [var.default_value for var in self.order_var_by_type(ModRegType.DISCRETE_INPUT)]
        co_var = [var.default_value for var in self.order_var_by_type(ModRegType.COILS)]
        hr_var = [var.default_value for var in self.order_var_by_type(ModRegType.HOLDING_REGISTER)]
        ir_var = [var.default_value for var in self.order_var_by_type(ModRegType.INPUT_REGISTER)]


        self._discrete_input_memory = ModbusSequentialDataBlock(self.DISCRETE_INPUT_START, di_var if di_var else [False])
        self._coils_memory = ModbusSequentialDataBlock(self.COILS_START, co_var if co_var else [False])
        self._holding_register_memory = ModbusSequentialDataBlock(self.HOLDING_REGISTER_START, hr_var if hr_var else [0])
        self._input_registers_memory = ModbusSequentialDataBlock(self.INPUT_REGISTER_START, ir_var if ir_var else [0])
        slave_context = ModbusSlaveContext(
            di=self._discrete_input_memory,
            co=self._coils_memory,
            ir=self._input_registers_memory,
            hr=self._holding_register_memory
        )

        self._modbus_context = ModbusServerContext(slave_context, True)
        self._server_task = asyncio.create_task(
            StartAsyncTcpServer(
                context=self._modbus_context,  # Data storage
                identity=self._modbus_identification,  # server identify
                address=(self.modbus_address, self.modbus_port)
            )
        )
        _logger.info("Modbus server started.")

        _logger.info("Initializing mqtt")
        self._mqtt_client.connect(self.mqtt_address, self.mqtt_port)
        self._mqtt_client.loop_start()
        for target_device, internal_var in self._internal_input_vars.items():
            self._mqtt_client.subscribe(self._internal_var_to_topic(internal_var, False))
        self._mqtt_client.on_message = self._on_mqtt_message

        for target_device, internal_var in self._internal_output_vars.items():
            self._mqtt_client.publish(self._internal_var_to_topic(internal_var, True), json.dumps(internal_var.value))
        _logger.info("Mqtt started.")

    def _on_mqtt_message(self, client, userdata, message):
        device_name, target_name, direction = message.topic.split("/")
        try:
            internal_var = self._internal_input_vars[target_name]
            internal_var.value = json.loads(message.payload.decode("utf-8"))
        except KeyError as e:
            _logger.error(f"Key {target_name} not found (received topic {message.topic}).")
        except json.decoder.JSONDecodeError as e:
            _logger.error(f"JSON decoding error (received topic {message.topic}). Message: {message.payload.decode('utf8')}")

    def main_plc_task(self):
        raise NotImplementedError()

    async def run(self):
        _logger.info(f"Starting {self.name}")
        while not self._closing:
            _main_task_time = time.time()
            self.main_plc_task()
            _main_task_duration = time.time() - _main_task_time
            if _main_task_duration < self._loop_time_limit:
                await asyncio.sleep(self._loop_time_limit -_main_task_duration)
            else:
                _logger.warning(f"Loop time exceeds limit {_main_task_duration} s on {self._loop_time_limit} s")

        await self._cleanup()


    async def _cleanup(self):
        if self._server_task is not None:
            self._server_task.cancel()
            await self._server_task
        self._mqtt_client.disconnect()

    def close(self, sig_num, frame):
        self._closing = True

    def _register_external_variable(self,var_name: str,  var_type: ModVarType, writable: bool, default_value: bool | int):
        if var_name in self._external_input_vars or var_name in self._external_output_vars:
            raise ValueError(f"Variable {var_name} already exists")

        if var_type == ModVarType.BOOLEAN:
            if writable:
                reg_type = ModRegType.DISCRETE_INPUT
            else:
                reg_type = ModRegType.COILS

        else:
            if writable:
                reg_type = ModRegType.INPUT_REGISTER
            else:
                reg_type = ModRegType.HOLDING_REGISTER


        var_num = len(self.order_var_by_type(reg_type))
        new_addr = self._get_start_address(reg_type) + var_num
        _logger.info(f"{self.name}: registered variable {var_name} as {reg_type} at address {new_addr}")

        new_var = ModVar(
                name=var_name,
                address=new_addr,
                default_value=default_value,
                reg_type=reg_type
            )

        if writable:
            self._external_output_vars[var_name] = new_var
        else:
            self._external_input_vars[var_name] = new_var

    def _register_internal_variable(self, var_name: str, writable: bool, default_value: list[float | int | bool],  target_device: str):
        if target_device in self._internal_input_vars or target_device in self._internal_output_vars:
            raise ValueError(f"Target device {target_device} already exists")

        internal_var = InternalVar(
            name = var_name,
            target_device = target_device,
            value= default_value
        )

        if writable:
            self._internal_output_vars[target_device] = internal_var
        else:
            self._internal_input_vars[target_device] = internal_var

    def _internal_var_to_topic(self, internal_var: InternalVar, writable: bool):
        if writable:
            return f"{self.name}/{internal_var.target_device}/to_gui"
        else:
            return f"{self.name}/{internal_var.target_device}/to_dt"

    def _variable_changed(self, var_name: str, internal=True) -> bool:
        if internal:
            if var_name in self._internal_input_vars:
                var_obj = self._internal_input_vars[var_name]
            else:
                var_obj = self._internal_output_vars[var_name]
            is_changed = var_obj.changed
            var_obj.changed = False
            return is_changed
        else:
            if var_name in self._external_output_vars:
                var_obj = self._external_output_vars[var_name]
            else:
                var_obj = self._external_input_vars[var_name]
            is_changed = var_obj.changed
            var_obj.changed = False
            return is_changed





    def order_var_by_type(self, var_type: ModRegType) -> list[ModVar]:
        if var_type == ModRegType.DISCRETE_INPUT or var_type == ModRegType.INPUT_REGISTER:
            out_list = [mod_var for var_name, mod_var in self._external_output_vars.items() if var_type == mod_var.reg_type]
        else:
            out_list = [mod_var for var_name, mod_var in self._external_input_vars.items() if var_type == mod_var.reg_type]

        return sorted(out_list, key=lambda x: x.address)


    def _read_external_variable(self, var_name: str) -> int | bool:
        if var_name in self._external_output_vars:
            mod_var = self._external_output_vars[var_name]
        elif var_name in self._external_input_vars:
            mod_var = self._external_input_vars[var_name]
        else:
            raise KeyError(var_name)

        if mod_var.reg_type == ModRegType.COILS:
            return self._coils_memory.values[mod_var.address - self.COILS_START]
        elif mod_var.reg_type == ModRegType.DISCRETE_INPUT:
            return self._discrete_input_memory.values[mod_var.address - self.DISCRETE_INPUT_START]
        elif mod_var.reg_type == ModRegType.HOLDING_REGISTER:
            return self._holding_register_memory.values[mod_var.address - self.HOLDING_REGISTER_START]
        else:
            return self._input_registers_memory.values[mod_var.address - self.INPUT_REGISTER_START]

    def _write_external_variable(self, var_name: str, value: int | bool):
        if var_name in self._external_output_vars:
            mod_var = self._external_output_vars[var_name]
        elif var_name in self._external_input_vars:
            _logger.error(f"Input variable {var_name} is not writable")
            raise KeyError(var_name)
        else:
            raise KeyError(var_name)

        if mod_var.reg_type == ModRegType.DISCRETE_INPUT:
            self._discrete_input_memory.values[mod_var.address - self.DISCRETE_INPUT_START] = value
        elif mod_var.reg_type == ModRegType.INPUT_REGISTER:
            self._input_registers_memory.values[mod_var.address - self.INPUT_REGISTER_START] = value%65536 # only values between 0 and 65535 are allowed
        else:
            _logger.error(f"{mod_var.reg_type} for variable {var_name} is not a valid output type")
            raise KeyError(var_name)

        mod_var.changed = True

    def _read_internal_variable(self, target_device: str) -> list[int | bool | float]:
        if target_device in self._internal_output_vars:
            int_var = self._internal_output_vars[target_device]
            return int_var.value
        elif target_device in self._internal_input_vars:
            int_var = self._internal_input_vars[target_device]
            return int_var.value
        else:
            raise KeyError(target_device)

    def _write_internal_variable(self, target_device: str, value: list[int | bool | float]):
        if target_device in self._internal_output_vars:
            int_var = self._internal_output_vars[target_device]
            if value != int_var.value:
                int_var.value = value
                self._mqtt_client.publish(self._internal_var_to_topic(int_var, True), json.dumps(value))
            int_var.changed = True
        else:
            raise KeyError(target_device)


    @classmethod
    def _get_start_address(cls, var_type: ModRegType) -> int:
        if var_type == ModRegType.DISCRETE_INPUT:
            return cls.DISCRETE_INPUT_START
        elif var_type == ModRegType.INPUT_REGISTER:
            return cls.INPUT_REGISTER_START
        elif var_type == ModRegType.HOLDING_REGISTER:
            return cls.HOLDING_REGISTER_START
        else:
            return cls.COILS_START

    @classmethod
    def get_type_by_address(cls, address: int) -> ModRegType:
        if cls.COILS_START <= address < cls.DISCRETE_INPUT_START:
            return ModRegType.COILS
        elif cls.DISCRETE_INPUT_START <= address < cls.INPUT_REGISTER_START:
            return ModRegType.DISCRETE_INPUT
        elif cls.INPUT_REGISTER_START <= address < cls.HOLDING_REGISTER_START:
            return ModRegType.INPUT_REGISTER
        elif cls.HOLDING_REGISTER_START <= address:
            return ModRegType.HOLDING_REGISTER
        else:
            raise ValueError(f"{address} is not a valid address")

    def print_registers(self) -> str:
        s = [f"Device: {self.name}", "Modbus", "Inputs:"]

        input_list = sorted(self._external_input_vars.values(), key=lambda x: x.address)
        output_list = sorted(self._external_output_vars.values(), key=lambda x: x.address)

        for mod_var in input_list:
            s.append(f"{mod_var.name} - {mod_var.reg_type.name}: {mod_var.address}")
        s.append("Outputs:")

        for mod_var in output_list:
            s.append(f"{mod_var.name} - {mod_var.reg_type.name}: {mod_var.address}")

        return "\n".join(s)

    def print_topics(self):
        s = [f"Device: {self.name}", "Mqtt","Inputs:"]

        for int_var in self._internal_input_vars.values():
            s.append(f"{int_var.name} - {self._internal_var_to_topic(int_var, False)}")

        s.append("Outputs:")

        for int_var in self._internal_output_vars.values():
            s.append(f"{int_var.name} - {self._internal_var_to_topic(int_var, True)}")

        return "\n".join(s)









