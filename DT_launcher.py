from digtwin.control.plc_subsystem import PLCSubsystem, ModVarType
from digtwin.physics.motor import Motor
import numpy as np
import time
import asyncio
import signal
import logging

_logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.DEBUG)

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

        # outputs
        self._register_external_variable("panel_alarm_out", ModVarType.BOOLEAN ,True, False)
        self._register_external_variable("panel_mode", ModVarType.BOOLEAN ,True, False)
        self._register_external_variable("panel_cylinder_on", ModVarType.BOOLEAN ,True, False)
        self._register_external_variable("panel_motor_on", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("panel_motor_speed", ModVarType.INT16, True, False)

        # self.ext_panel_motor_on = False
        # self.ext_panel_cylinder_on = False
        # self.ext_panel_motor_speed = 1.0
        # self.ext_panel_mode = False
        # self.ext_panel_alarm_out = False

        # memory

        # triggers ref

        self._old_switch = False
        self._old_green_button = False
        self._old_red_button = False
        self._old_panel_motor_speed = False
        self._old_panel_motor_on = False
        self._old_panel_cylinder_on = False
        self._old_emergency = False

        # timer refs

        self._g_button_timer = time.time()
        self._light_blink_timer = time.time()
        self._light_blink_timeout = 0

        # physics

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

        # timer refs

        self._g_button_timer = time.time()
        self._light_blink_timer = time.time()
        self._light_blink_timeout = 0
        self._gate_timer = time.time()
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

        self.ext_motor_position = int(self._motor.theta * 400 * 27.0/(17.5*2*np.pi))

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
            if not average_speed and self.ext_motor_run_cmd: # zero speed but motor enabled
                self._old_motor_stuck = True
                _logger.warning("Motor is about to be stuck")
                if self._old_motor_stuck:  # second time
                    _logger.warning("Motor STUCK!")
                    self.ext_motor_stuck = True
            else:
                self._old_motor_stuck = False


            self._motor_speed_mes_timer = time.time()
            self._old_motor_position = self.ext_motor_position

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


async def main():
    panel_plc = SmartConveyorPanel()
    motor_plc = SmartConveyorMotor()

    def close(a, b):
        panel_plc.close(a,b)
        motor_plc.close(a,b)

    await panel_plc.init()
    await motor_plc.init()
    plc_task = asyncio.create_task(panel_plc.run())
    motor_task = asyncio.create_task(motor_plc.run())
    signal.signal(signal.SIGINT, close)
    signal.signal(signal.SIGTERM, close)
    print(panel_plc.print_registers())
    print(panel_plc.print_topics())
    print(motor_plc.print_registers())
    print(motor_plc.print_topics())
    try:
        while not plc_task.done() and not motor_task.done():
            await asyncio.sleep(1.0)
    finally:
        await plc_task
        await motor_task

if __name__ == '__main__':
    asyncio.run(main())




