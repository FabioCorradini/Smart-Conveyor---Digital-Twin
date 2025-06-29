from digtwin.control.plc_subsystem import PLCSubsystem, ModVarType
from digtwin.physics.motor import Motor
from digtwin.physics.crank_drive import CrankDrive
from pymodbus.client import ModbusTcpClient
import numpy as np
import time
import asyncio
import signal
import logging

_logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)
logging.getLogger("pymodbus").setLevel(logging.INFO)

class SmartConveyorPanel(PLCSubsystem):
    def __init__(self):
        super(SmartConveyorPanel, self).__init__("smart_conveyor_panel")
        # inputs
        # panel
        self._register_internal_variable("emergency", False, [False], "mushroom")
        self._register_internal_variable("switch", False, [False], "festo_switch")
        self._register_internal_variable("red_button", False, [False], "pushbutton_red")
        self._register_internal_variable("green_button", False, [False], "pushbutton_green")

        # outputs
        self._register_internal_variable("green_light", True, [False], "lightpanel_green")
        self._register_internal_variable("red_light", True, [False], "lightpanel_red")

        # externals
        # inputs
        self._register_external_variable("panel_alarm_1", ModVarType.BOOLEAN ,False, False)
        self._register_external_variable("panel_alarm_2", ModVarType.BOOLEAN ,False, False)
        self._register_external_variable("panel_running", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("panel_state", ModVarType.INT16, False, 0)

        # outputs
        self._register_external_variable("panel_alarm_out", ModVarType.BOOLEAN ,True, False)
        self._register_external_variable("panel_mode", ModVarType.BOOLEAN ,True, False)
        self._register_external_variable("panel_cylinder_on", ModVarType.BOOLEAN ,True, False)
        self._register_external_variable("panel_motor_on", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("panel_motor_speed", ModVarType.INT16, True, False)

        self._error_panel_reg_len = 35
        self._add_string_registers("error_panel", self._error_panel_reg_len)

        # memory

        # triggers ref

        self._old_switch = False
        self._old_green_button = False
        self._old_red_button = False
        self._old_panel_motor_speed = False
        self._old_panel_motor_on = False
        self._old_panel_cylinder_on = False
        self._old_emergency = False
        self._old_panel_state = None

        # timer refs

        self._g_button_timer = time.time()
        self._light_blink_timer = time.time()
        self._light_blink_timeout = 0

        # physics

    def _add_string_registers(self, variable_name: str, n_registers: int) -> None:
        for i in range(n_registers):
            self._register_external_variable( f"{variable_name}_{i}", ModVarType.INT16, True, 0)

    def _panel_state_to_error_string(self, panel_state: int) -> str:
        s = []
        if panel_state == 0:
            s.append("No error")
        else:
            if panel_state & 0x01:
                s.append("cylinder_stuck")

            if panel_state & 0x02:
                s.append("motor_stuck")

            if panel_state & 0x04:
                s.append("part_stuck")

            if panel_state & 0x08:
                s.append("no_pressure_alarm")

            if panel_state & 0x10:
                s.append("leakage_alarm")

        return " ".join(s)


    def main_plc_task(self):

        # panel
        if self.int_emergency:
            self.ext_panel_motor_on = False
            self.ext_panel_cylinder_on = False
            self.int_green_light = False
            self.int_red_light = False
            self.ext_panel_alarm_out = True
        else:
            self.ext_panel_alarm_out = False
            if self.int_switch:
                if self._old_switch != self.int_switch:
                    _logger.info("Auto mode enabled")
                    self._old_switch = self.int_switch

                if self._old_green_button != self.int_green_button:
                    if self.int_green_button:
                        _logger.info("Activating conveyor")
                        self.ext_panel_motor_on = True
                        self._g_button_timer = time.time()
                    self._old_green_button = self.int_green_button

                if self._old_red_button != self.int_red_button:
                    if self.int_red_button:
                        _logger.info("Deactivating conveyor")
                        self.ext_panel_motor_on = False
                    self._old_red_button = self.int_red_button

                if self.int_green_button:
                    # controlling timer
                    self.ext_panel_motor_speed = (int(time.time() - self._g_button_timer) % 3) + 1

            else:
                if self._old_switch != self.int_switch:
                    _logger.info("Manual mode enabled")
                    self._old_switch = self.int_switch

                self.ext_panel_motor_on = self.int_green_button and not self.int_emergency
                self.ext_panel_cylinder_on = self.int_red_button and not self.int_emergency

            self.int_green_light = self.ext_panel_motor_on and not (
                        self.ext_panel_alarm_1 or self.ext_panel_alarm_2)

        if not self.ext_panel_alarm_1 and not self.ext_panel_alarm_2:
            self._light_blink_timeout = 0.0
        elif self.ext_panel_alarm_1:
            self._light_blink_timeout = 1.0
        elif self.ext_panel_alarm_2:
            self._light_blink_timeout = 2.0

        if self._light_blink_timeout:
            if (time.time() - self._light_blink_timer) > self._light_blink_timeout:
                self.int_red_light = not self.int_red_light
                self._light_blink_timer = time.time()
        else:
            self.int_red_light = False

        self.ext_panel_mode = self.int_switch

        if self._old_panel_state != self.ext_panel_state:
            self.ext_error_panel_message = self._panel_state_to_error_string(self.ext_panel_state)
            self._old_panel_state = self.ext_panel_state

        self.debug_print()

    def debug_print(self):
        if self._old_panel_motor_on != self.ext_panel_motor_on:
            _logger.info(f"Motor on switched to: {self.ext_panel_motor_on}")
            self._old_panel_motor_on = self.ext_panel_motor_on

        if self._old_panel_cylinder_on != self.ext_panel_cylinder_on:
            _logger.info(f"Cylinder on switched to: {self.ext_panel_cylinder_on}")
            self._old_panel_cylinder_on = self.ext_panel_cylinder_on

        if self._old_panel_motor_speed != self.ext_panel_motor_speed:
            _logger.info(f"Motor speed se to: {self._old_panel_motor_speed}")
            self._old_panel_motor_speed = self.ext_panel_motor_speed

        if self._old_emergency != self.int_emergency:
            if self.int_emergency:
                _logger.info(f"Emergency called")
            else:
                _logger.info(f"Emergency decativated")
            self._old_emergency = self.int_emergency

    @property
    def int_green_light(self) -> bool:
        return self._read_internal_variable("lightpanel_green")[0]

    @int_green_light.setter
    def int_green_light(self, value: bool):
        self._write_internal_variable("lightpanel_green", [value])

    @property
    def int_red_light(self) -> bool:
        return self._read_internal_variable("lightpanel_red")[0]

    @int_red_light.setter
    def int_red_light(self, value: bool):
        self._write_internal_variable("lightpanel_red", [value])

    @property
    def int_emergency(self) -> bool:
        return self._read_internal_variable("mushroom")[0]

    @property
    def int_switch(self) -> bool:
        return self._read_internal_variable("festo_switch")[0]

    @property
    def int_red_button(self) -> bool:
        return self._read_internal_variable("pushbutton_red")[0]

    @property
    def int_green_button(self) -> bool:
        return self._read_internal_variable("pushbutton_green")[0]

    @property
    def ext_panel_alarm_1(self) -> bool:
        return self._read_external_variable("panel_alarm_1")

    @property
    def ext_panel_alarm_2(self) -> bool:
        return self._read_external_variable("panel_alarm_2")

    @property
    def ext_panel_running(self) -> bool:
        return self._read_external_variable("panel_running")

    @property
    def ext_panel_state(self) -> int:
        return self._read_external_variable("panel_state")

    @property
    def ext_panel_alarm_out(self) -> bool:
        return self._read_external_variable("panel_alarm_out")
    @ext_panel_alarm_out.setter
    def ext_panel_alarm_out(self, value: bool):
        self._write_external_variable("panel_alarm_out", value)

    @property
    def ext_panel_mode(self) -> bool:
        return self._read_external_variable("panel_mode")
    @ext_panel_mode.setter
    def ext_panel_mode(self, value: bool):
        self._write_external_variable("panel_mode", value)

    @property
    def ext_panel_cylinder_on(self) -> bool:
        return self._read_external_variable("panel_cylinder_on")
    @ext_panel_cylinder_on.setter
    def ext_panel_cylinder_on(self, value: bool):
        self._write_external_variable("panel_cylinder_on", value)

    @property
    def ext_panel_motor_on(self) -> bool:
        return self._read_external_variable("panel_motor_on")
    @ext_panel_motor_on.setter
    def ext_panel_motor_on(self, value: bool):
        self._write_external_variable("panel_motor_on", value)

    @property
    def ext_panel_motor_speed(self) -> bool:
        return self._read_external_variable("panel_motor_speed")
    @ext_panel_motor_speed.setter
    def ext_panel_motor_speed(self, value: bool):
        self._write_external_variable("panel_motor_speed", value)

    @property
    def ext_error_panel_message(self) -> str:
        registers = [self._read_external_variable(f"error_panel_{i}") for i in range(self._error_panel_reg_len)]
        return ModbusTcpClient.convert_from_registers([c for c in registers if c !=0], ModbusTcpClient.DATATYPE.STRING)

    @ext_error_panel_message.setter
    def ext_error_panel_message(self, value: str):
        registers = ModbusTcpClient.convert_to_registers(value, ModbusTcpClient.DATATYPE.STRING)
        registers = registers + [0] * (self._error_panel_reg_len - len(registers))
        for i in range(self._error_panel_reg_len):
            self._write_external_variable(f"error_panel_{i}", registers[i])


class SmartConveyorMotor(PLCSubsystem):
    def __init__(self):
        super(SmartConveyorMotor, self).__init__("smart_conveyor_motor", modbus_port=5021)
        # inputs
        # motor
        self._register_internal_variable("prox_1", False, [False], "prox1")
        self._register_internal_variable("prox_2", False, [False], "prox2")
        self._register_internal_variable("motor_unpowered", False, [False], "motor_unpowered")

        # outputs

        # motor
        self._register_internal_variable("phys_encoder_pos", True, [0], "encoder_node")
        self._register_internal_variable("phys_motor_pos_0", True, [0], "conveyor_node_0")
        self._register_internal_variable("phys_motor_pos_1", True, [0], "conveyor_node_1")
        self._register_internal_variable("phys_motor_pos_2", True, [0], "conveyor_node_2")
        self._register_internal_variable("phys_motor_pos_3", True, [0], "conveyor_node_3")
        self._register_internal_variable("phys_motor_pos_4", True, [0], "conveyor_node_4")
        self._register_internal_variable("phys_motor_pos_5", True, [0], "conveyor_node_5")
        self._register_internal_variable("phys_motor_pos_6", True, [0], "conveyor_node_6")
        self._register_internal_variable("phys_motor_pos_7", True, [0], "conveyor_node_7")
        self._register_internal_variable("phys_motor_pos_8", True, [0], "conveyor_node_8")

        # externals
        # inputs
        self._register_external_variable("motor_run_cmd", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("motor_dir_cmd", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("motor_speed_cmd", ModVarType.INT16, False, 1)
        self._register_external_variable("reset_counters_cmd", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("motor_alarm_1 ", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("motor_alarm_2", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("reset_alarm_cmd", ModVarType.BOOLEAN, False, False)

        # outputs
        self._register_external_variable("motor_position", ModVarType.INT16, True, 0)
        self._register_external_variable("produced_part_counter", ModVarType.INT16, True, 0)
        self._register_external_variable("part_counter", ModVarType.INT16, True, 0)
        self._register_external_variable("part_stuck", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("motor_stuck", ModVarType.BOOLEAN, True, False)

        self._register_external_variable("long_encoder_pos_reg_0", ModVarType.INT16, True, 0)
        self._register_external_variable("long_encoder_pos_reg_1", ModVarType.INT16, True, 0)
        self._register_external_variable("long_encoder_pos_reg_2", ModVarType.INT16, True, 0)
        self._register_external_variable("long_encoder_pos_reg_3", ModVarType.INT16, True, 0)

        self._register_external_variable("float_motor_speed_reg_0", ModVarType.INT16, True, 0)
        self._register_external_variable("float_motor_speed_reg_1", ModVarType.INT16, True, 0)

        # memory

        self._part_on_flap = False

        # triggers ref

        self._old_prox_1 = self.int_prox_1
        self._old_prox_2 = self.int_prox_2
        self._old_motor_unpowered = self.int_motor_unpowered
        self._old_motor_position = 0
        self._old_motor_stuck = False
        self._old_reset_alarm_cmd = False
        self._old_reset_counters_cmd = False
        self._old_theta = 0

        # timer refs

        self._motor_speed_mes_timer = time.time()
        self._stuck_timer = 0


        # physics
        self._motor = Motor()


    def main_plc_task(self):
        self._motor.run(time.time())
        # motor

        self.int_motor_en_port = self.ext_motor_run_cmd and not self.int_motor_unpowered
        self.int_motor_speed_port = self.ext_motor_speed_cmd
        self.int_motor_dir_port = not self.ext_motor_dir_cmd

        self.phys_motor_pos = self._motor.theta

        encoder_position = int(self._motor.theta * 400 * 27.0/(17.5*2*np.pi))

        self.ext_motor_position = encoder_position
        self.ext_long_encoder_position = encoder_position

        if self._old_prox_1 != self.int_prox_1:
            if self.int_prox_1:
                self.ext_produced_part_counter += 1
                self.ext_part_counter -= 1
                _logger.info(f"Produced part counter: {self.ext_produced_part_counter}")
                _logger.info(f"Part counter: {self.ext_part_counter}")
            self._old_prox_1 = self.int_prox_1

        if self._old_prox_2 != self.int_prox_2:
            if self.int_prox_2:
                self.ext_part_counter += 1
                _logger.info(f"Part counter: {self.ext_part_counter}")
                self._stuck_timer = time.time()
                self._part_on_flap = True
            else:
                self._part_on_flap = False
            self._old_prox_2 = self.int_prox_2


        if self._part_on_flap and not self.ext_part_stuck and time.time() - self._stuck_timer > 5.0:
            _logger.warning("PART STUCK ALARM")
            self.ext_part_stuck = True

        if time.time() - self._motor_speed_mes_timer > 1.0:
            average_speed = self.ext_motor_position - self._old_motor_position
            self.ext_float_motor_speed = (self._motor.theta - self._old_theta)/(2*np.pi*(time.time() - self._motor_speed_mes_timer))

            if not average_speed and self.ext_motor_run_cmd: # zero speed but motor enabled
                _logger.warning("Motor is about to be stuck")
                if self._old_motor_stuck:  # second time
                    _logger.warning("Motor STUCK!")
                    self.ext_motor_stuck = True
                self._old_motor_stuck = True
            else:
                self._old_motor_stuck = False


            self._motor_speed_mes_timer = time.time()
            self._old_motor_position = self.ext_motor_position
            self._old_theta = self._motor.theta

        if self._old_reset_alarm_cmd != self.ext_reset_alarm_cmd:
            if self.ext_reset_alarm_cmd:
                _logger.info("RESETTING ALARMS")
                self.ext_motor_stuck = False
                self._old_motor_stuck = False
                self.ext_part_stuck = False
            self._old_reset_alarm_cmd = self.ext_reset_alarm_cmd

        if self._old_reset_counters_cmd != self.ext_reset_counters_cmd:
            if self.ext_reset_counters_cmd:
                _logger.info("RESETTING COUNTERS")
                self.ext_produced_part_counter = 0
                self.ext_part_counter = 0
            self._old_reset_counters_cmd = self.ext_reset_counters_cmd


        self.debug_print()


    def debug_print(self):
        if self._old_motor_unpowered != self.int_motor_unpowered:
            _logger.info(f"Motor power set to: {not self.int_motor_unpowered}")
            self._old_motor_unpowered = self.int_motor_unpowered

    @property
    def int_prox_1(self) -> bool:
        return self._read_internal_variable("prox1")[0]

    @property
    def int_prox_2(self) -> bool:
        return self._read_internal_variable("prox2")[0]

    @property
    def int_motor_unpowered(self) -> bool:
        return self._read_internal_variable("motor_unpowered")[0]

    @property
    def phys_motor_pos(self) -> float:
        return self._motor.theta

    @phys_motor_pos.setter
    def phys_motor_pos(self, value: float):
        self._write_internal_variable("encoder_node", [-value])
        self._write_internal_variable("conveyor_node_0", [value])
        self._write_internal_variable("conveyor_node_1", [value])
        self._write_internal_variable("conveyor_node_2", [value])
        self._write_internal_variable("conveyor_node_3", [value])
        self._write_internal_variable("conveyor_node_4", [value])
        self._write_internal_variable("conveyor_node_5", [value])
        self._write_internal_variable("conveyor_node_6", [value])
        self._write_internal_variable("conveyor_node_7", [value])
        self._write_internal_variable("conveyor_node_8", [value])

    @property
    def int_motor_en_port(self) -> bool:
        return self._motor.running

    @int_motor_en_port.setter
    def int_motor_en_port(self, value: bool):
        self._motor.running = value

    @property
    def int_motor_dir_port(self):
        return self._motor.moving_forward

    @int_motor_dir_port.setter
    def int_motor_dir_port(self, value: bool):
        self._motor.moving_forward = value

    @property
    def int_motor_speed_port(self) -> float:
        return self._motor.movement_speed

    @int_motor_speed_port.setter
    def int_motor_speed_port(self, value: float):
        self._motor.movement_speed = value * 2*np.pi

    @property
    def ext_motor_run_cmd(self) -> bool:
        return self._read_external_variable("motor_run_cmd")

    @property
    def ext_motor_dir_cmd(self) -> bool:
        return self._read_external_variable("motor_dir_cmd")

    @property
    def ext_motor_speed_cmd(self) -> int:
        return self._read_external_variable("motor_speed_cmd")

    @property
    def ext_reset_counters_cmd(self) -> bool:
        return self._read_external_variable("reset_counters_cmd")

    @property
    def ext_motor_alarm_1(self) -> bool:
        return self._read_external_variable("motor_alarm_1")

    @property
    def ext_motor_alarm_2(self) -> bool:
        return self._read_external_variable("motor_alarm_2")

    @property
    def ext_reset_alarm_cmd(self) -> bool:
        return self._read_external_variable("reset_alarm_cmd")

    @property
    def ext_motor_position(self) -> int:
        return self._read_external_variable("motor_position")
    @ext_motor_position.setter
    def ext_motor_position(self, value: int):
        self._write_external_variable("motor_position", value)

    @property
    def ext_produced_part_counter(self) -> int:
        return self._read_external_variable("produced_part_counter")
    @ext_produced_part_counter.setter
    def ext_produced_part_counter(self, value: int):
        self._write_external_variable("produced_part_counter", value)

    @property
    def ext_part_counter(self) -> int:
        return self._read_external_variable("part_counter")
    @ext_part_counter.setter
    def ext_part_counter(self, value: int):
        self._write_external_variable("part_counter", value)

    @property
    def ext_part_stuck(self):
        return self._read_external_variable("part_stuck")
    @ext_part_stuck.setter
    def ext_part_stuck(self, value: bool):
        self._write_external_variable("part_stuck", value)

    @property
    def ext_motor_stuck(self):
        return self._read_external_variable("motor_stuck")
    @ext_motor_stuck.setter
    def ext_motor_stuck(self, value: bool):
        self._write_external_variable("motor_stuck", value)

    @property
    def ext_long_encoder_position(self) -> int:
        registers = [
            self._read_external_variable("long_encoder_pos_reg_0"),
            self._read_external_variable("long_encoder_pos_reg_1"),
            self._read_external_variable("long_encoder_pos_reg_2"),
            self._read_external_variable("long_encoder_pos_reg_3")
        ]
        return ModbusTcpClient.convert_from_registers(registers, ModbusTcpClient.DATATYPE.INT64)

    @ext_long_encoder_position.setter
    def ext_long_encoder_position(self, value: int):
        registers = ModbusTcpClient.convert_to_registers(value, ModbusTcpClient.DATATYPE.INT64)
        self._write_external_variable("long_encoder_pos_reg_0", registers[0])
        self._write_external_variable("long_encoder_pos_reg_1", registers[1])
        self._write_external_variable("long_encoder_pos_reg_2", registers[2])
        self._write_external_variable("long_encoder_pos_reg_3", registers[3])

    @property
    def ext_float_motor_speed(self) -> float:
        registers =  [
            self._read_external_variable("float_motor_speed_reg_0"),
            self._read_external_variable("float_motor_speed_reg_1")
        ]

        return ModbusTcpClient.convert_from_registers(registers, ModbusTcpClient.DATATYPE.FLOAT32)

    @ext_float_motor_speed.setter
    def ext_float_motor_speed(self, value: float):
        registers = ModbusTcpClient.convert_to_registers(value, ModbusTcpClient.DATATYPE.FLOAT32)
        self._write_external_variable("float_motor_speed_reg_0", registers[0])
        self._write_external_variable("float_motor_speed_reg_1", registers[1])

class SmartConveyorCylinder(PLCSubsystem):
    def __init__(self):
        super(SmartConveyorCylinder, self).__init__("smart_conveyor_cylinder", modbus_port=5022)

        # inputs

        # cylinder
        self._register_internal_variable("bck_switch", False, [False], "piston_sens2")
        self._register_internal_variable("frw_switch", False, [False], "piston_sens1")
        self._register_internal_variable("low_pressure", False, [False], "low_pressure")
        self._register_internal_variable("stuck_state", False, [False], "stuck_state")

        # outputs

        # motor

        self._register_internal_variable("phys_piston_plunger", True, [0], "piston_in_node")
        self._register_internal_variable("phys_piston_ext", True, [0], "piston_ext_node")
        self._register_internal_variable("phys_gate", True, [0], "gate_node")

        # externals

        # inputs
        self._register_external_variable("flap_cmd", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("reset_counters_cmd", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("reset_alarm_cmd", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("auto_mode", ModVarType.BOOLEAN, False, False)

        _default_regs = ModbusTcpClient.convert_to_registers(int(5e5), ModbusTcpClient.DATATYPE.UINT32, "little")
        self._register_external_variable("pressure_controller_reg_0", ModVarType.INT16, False, _default_regs[0])
        self._register_external_variable("pressure_controller_reg_1", ModVarType.INT16, False, _default_regs[1])

        #output
        self._register_external_variable("movement_count", ModVarType.INT16, True, 0)
        self._register_external_variable("pressure_alarm", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("leakage_alarm", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("cylinder_alarm_out", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("cylinder_stuck_alarm", ModVarType.BOOLEAN, True, False)

        # memory

        self.cylinder_moving_forward = False
        self.cylinder_moving_backward = False

        # triggers ref

        self._old_frw_switch = self.int_frw_switch
        self._old_bck_switch = self.int_bck_switch
        self._old_cmd_port = False
        self._old_stuck_state = False
        self._old_low_pressure = self.int_low_pressure
        self._old_reset_alarm_cmd = False
        self._old_reset_counters_cmd = False
        self._old_pressure_controller = 5e5

        # timer refs

        self._gate_timer = time.time()

        # physics

        self._crank_drive = CrankDrive()


    def main_plc_task(self):
        self._crank_drive.run(time.time())

        #cylinder

        self.ext_cylinder_alarm_out = self.ext_pressure_alarm or self.ext_cylinder_stuck_alarm or self.ext_leakage_alarm

        self.int_valve_port = self.ext_flap_cmd and not self.ext_cylinder_alarm_out

        if (self._old_low_pressure != self.int_low_pressure) or (self._old_pressure_controller != self.ext_pressure_controller):
            if self.int_low_pressure:
                self._crank_drive.pressure = min(1.2e5, float(self.ext_pressure_controller))
            else:
                self._crank_drive.pressure = min(5e5, float(self.ext_pressure_controller))

            if self._crank_drive.pressure < 2e5:
                print("Pressure is too Low!")
                self.ext_pressure_alarm = True

            _logger.info(f"New pressure: {int(self._crank_drive.pressure)} Pa")

            self._old_low_pressure = self.int_low_pressure
            self._old_pressure_controller = self.ext_pressure_controller


        if self._old_cmd_port != self.int_valve_port:
            if self.int_valve_port:
                self.ext_movement_count += 1
                self.cylinder_moving_forward = True
            else:
                self.cylinder_moving_backward = False

            self._gate_timer = time.time()
            self._old_cmd_port = self.int_valve_port

        if self.cylinder_moving_forward and not self.ext_cylinder_stuck_alarm and time.time() - self._gate_timer > 5.0:
            _logger.info("Cylinder stuck during forward movement")
            self.ext_cylinder_stuck_alarm = True

        if self.cylinder_moving_backward and not self.ext_cylinder_stuck_alarm and time.time() - self._gate_timer > 5.0:
            _logger.info("Cylinder stuck during backward movement")
            self.ext_cylinder_stuck_alarm = True

        if self._old_frw_switch != self.int_frw_switch:
            _logger.info(f"Frw switch to: {self.int_frw_switch}")
            if self.int_frw_switch:
                self.cylinder_moving_forward = False
                self.cylinder_moving_backward = False
            self._old_frw_switch = self.int_frw_switch

        if self._old_bck_switch != self.int_bck_switch:
            _logger.info(f"Bck switch to: {self.int_bck_switch}")
            if self.int_bck_switch:
                self.cylinder_moving_forward = False
                self.cylinder_moving_backward = False
            self._old_bck_switch = self.int_bck_switch

        if self._old_stuck_state != self.int_stuck_state:
            _logger.info(f"Stuck state to: {self.int_stuck_state}")
            self._crank_drive.set_stuck_state(self.int_stuck_state)
            self._old_stuck_state = self.int_stuck_state

        if self._old_reset_alarm_cmd != self.ext_reset_alarm_cmd:
            if self.ext_reset_alarm_cmd:
                _logger.info("RESETTING ALARMS")
                self.ext_cylinder_stuck_alarm = False
                self.ext_pressure_alarm = False
                self.cylinder_moving_forward = False
                self.cylinder_moving_backward = False
            self._old_reset_alarm_cmd = self.ext_reset_alarm_cmd

        if self._old_reset_counters_cmd != self.ext_reset_counters_cmd:
            if self.ext_reset_counters_cmd:
                _logger.info("RESETTING COUNTERS")
                self.ext_movement_count = 0
            self._old_reset_counters_cmd = self.ext_reset_counters_cmd


        self.send_cylinder_pos()


        self.debug_print()


    def debug_print(self):
        pass

    def send_cylinder_pos(self):
        psi, alpha = self._crank_drive.get_d_psi_d_alpha(self._crank_drive.theta)
        self._write_internal_variable("gate_node", [-self._crank_drive.theta])
        self._write_internal_variable("piston_ext_node", [psi])
        self._write_internal_variable("piston_in_node", [alpha])

    @property
    def int_bck_switch(self) -> bool:
        return self._read_internal_variable("piston_sens2")[0]

    @property
    def int_frw_switch(self) -> bool:
        return self._read_internal_variable("piston_sens1")[0]

    @property
    def int_low_pressure(self) -> float:
        return self._read_internal_variable("low_pressure")[0]

    @property
    def int_stuck_state(self):
        return self._read_internal_variable("stuck_state")[0]

    @property
    def int_valve_port(self) -> bool:
        return self._crank_drive.moving_forward

    @int_valve_port.setter
    def int_valve_port(self, value: bool):
        self._crank_drive.moving_forward = value

    @property
    def ext_flap_cmd(self):
        return self._read_external_variable("flap_cmd")

    @property
    def ext_reset_counters_cmd(self):
        return self._read_external_variable("reset_counters_cmd")

    @property
    def ext_auto_mode(self):
        return self._read_external_variable("auto_mode")

    @property
    def ext_movement_count(self):
        return self._read_external_variable("movement_count")
    @ext_movement_count.setter
    def ext_movement_count(self, value: int):
        self._write_external_variable("movement_count", value)

    @property
    def ext_pressure_alarm(self):
        return self._read_external_variable("pressure_alarm")
    @ext_pressure_alarm.setter
    def ext_pressure_alarm(self, value: bool):
        self._write_external_variable("pressure_alarm", value)

    @property
    def ext_leakage_alarm(self):
        return self._read_external_variable("leakage_alarm")
    @ext_leakage_alarm.setter
    def ext_leakage_alarm(self, value: bool):
        self._write_external_variable("leakage_alarm", value)

    @property
    def ext_cylinder_stuck_alarm(self):
        return self._read_external_variable("cylinder_stuck_alarm")
    @ext_cylinder_stuck_alarm.setter
    def ext_cylinder_stuck_alarm(self, value: bool):
        self._write_external_variable("cylinder_stuck_alarm", value)

    @property
    def ext_cylinder_alarm_out(self):
        return self._read_external_variable("cylinder_alarm_out")
    @ext_cylinder_alarm_out.setter
    def ext_cylinder_alarm_out(self, value: bool):
        self._write_external_variable("cylinder_alarm_out", value)

    @property
    def ext_reset_alarm_cmd(self) -> bool:
        return self._read_external_variable("reset_alarm_cmd")

    @property
    def ext_pressure_controller(self) -> int:
        registers = [
            self._read_external_variable("pressure_controller_reg_0"),
            self._read_external_variable("pressure_controller_reg_1")
        ]

        return ModbusTcpClient.convert_from_registers(registers, ModbusTcpClient.DATATYPE.UINT32, "little")

    @ext_pressure_controller.setter
    def ext_pressure_controller(self, value: float):
        registers = ModbusTcpClient.convert_to_registers(value, ModbusTcpClient.DATATYPE.UINT32, "little")
        self._write_external_variable("pressure_controller_reg_0", registers[0])
        self._write_external_variable("pressure_controller_reg_1", registers[1])



async def main():
    panel_plc = SmartConveyorPanel()
    motor_plc = SmartConveyorMotor()
    cylinder_plc = SmartConveyorCylinder()

    def close(a, b):
        panel_plc.close(a,b)
        motor_plc.close(a,b)
        cylinder_plc.close(a,b)

    await panel_plc.init()
    await motor_plc.init()
    await cylinder_plc.init()

    plc_task = asyncio.create_task(panel_plc.run())
    motor_task = asyncio.create_task(motor_plc.run())
    cylinder_task = asyncio.create_task(cylinder_plc.run())

    signal.signal(signal.SIGINT, close)
    signal.signal(signal.SIGTERM, close)

    print(panel_plc.print_registers())
    print(panel_plc.print_topics())
    print(motor_plc.print_registers())
    print(motor_plc.print_topics())
    print(cylinder_plc.print_registers())
    print(cylinder_plc.print_topics())

    try:
        while not plc_task.done() and not motor_task.done():
            await asyncio.sleep(1.0)
    finally:
        await plc_task
        await motor_task
        await cylinder_task

if __name__ == '__main__':
    asyncio.run(main())




