import setuptools

from digtwin.control.plc_subsystem import PLCSubsystem, ModVarType
from digtwin.physics.motor import Motor
from digtwin.physics.crank_drive import CrankDrive
import time
import asyncio
import signal
import logging
import numpy as mp

from gen_models import piston_ext_node

_logger = logging.getLogger(__name__)
logging.basicConfig(level=logging.DEBUG)
logging.basicConfig(level=logging.DEBUG)

class GlobalPlc(PLCSubsystem):
    def __init__(self):
        super(GlobalPlc, self).__init__("global_plc")

        # inputs
        # panel
        self._register_internal_variable("emergency", False, [True], "mushroom")
        self._register_internal_variable("switch", False, [False], "festo_switch")
        self._register_internal_variable("red_button", False, [False], "pushbutton_red")
        self._register_internal_variable("green_button", False, [False], "pushbutton_green")
        # motor
        self._register_internal_variable("prox_1", False, [False], "prox1")
        self._register_internal_variable("prox_2", False, [False], "prox2")
        # cylinder
        self._register_internal_variable("bck_switch", False, [False], "piston_sens2")
        self._register_internal_variable("frw_switch", False, [False], "piston_sens1")

        # outputs
        self._register_internal_variable("green_light", True, [False], "lightpanel_green")
        self._register_internal_variable("red_light", True, [False], "lightpanel_red")
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
        self._register_internal_variable("phys_piston_plunger", True, [0], "piston_in_node")
        self._register_internal_variable("phys_piston_ext", True, [0], "piston_ext_node")
        self._register_internal_variable("phys_gate", True, [0], "gate_node")

        # externals

        self.ext_panel_motor_on = False
        self.ext_panel_cylinder_on = False
        self.ext_panel_motor_speed = 1.0
        self.ext_panel_alarm_1 = False
        self.ext_panel_alarm_2 = False
        self.ext_panel_mode = False
        self.ext_panel_alarm_out = False
        self.ext_produced_part_counter = 0
        self.ext_part_counter = 0
        self.ext_cylinder_stuck = False
        self.ext_pressure_alarm = False
        self.ext_leakage_alarm = False

        # memory

        self.cylinder_moving_forward = False
        self.cylinder_moving_backward = False

        # triggers ref

        self._old_switch = self.int_switch
        self._old_green_button = self.int_green_button
        self._old_red_button = self.int_red_button
        self._old_panel_motor_speed = self.ext_panel_motor_speed
        self._old_panel_motor_on = self.ext_panel_motor_on
        self._old_panel_cylinder_on = self.ext_panel_cylinder_on
        self._old_emergency = self.int_emergency
        self._old_prox_1 = self.int_prox_1
        self._old_prox_2 = self.int_prox_2

        self._old_frw_switch = self.int_frw_switch
        self._old_bck_switch = self.int_bck_switch

        # timer refs

        self._g_button_timer = time.time()
        self._light_blink_timer = time.time()
        self._light_blink_timeout = 0
        self._gate_timer = time.time()


        # physics

        self._crank_drive = CrankDrive()
        self._motor = Motor()


    def main_plc_task(self):
        self._crank_drive.run(time.time())
        self._motor.run(time.time())

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

            self.int_green_light = self.ext_panel_motor_on and not (self.ext_panel_alarm_1 or self.ext_panel_alarm_2)

        if not self.ext_panel_alarm_1 and not self.ext_panel_alarm_2:
            self._light_blink_timeout = 0.0
        elif self.ext_panel_alarm_1:
            self._light_blink_timeout = 1.0
        elif self.ext_panel_alarm_2:
            self._light_blink_timeout = 2.0

        if self._light_blink_timeout:
            if (time.time() - self._light_blink_timer) > self._light_blink_timeout:
                self.int_red_light = not self.int_green_light
                self._light_blink_timer = time.time()
        else:
            self.int_red_light = False

        self.ext_panel_mode = self.int_green_light

        # motor

        self.int_motor_en_port = self.ext_panel_motor_on
        self.int_motor_speed_port = self.ext_panel_motor_speed

        self.phys_motor_pos = self._motor.theta

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
            self._old_prox_2 = self.int_prox_2

        #cylinder

        if self.int_switch:
            if self.ext_panel_motor_on:
                if (time.time() - self._gate_timer) > 5.0 or self.int_prox_2:
                    self.cylinder_moving_forward = True
                    self._gate_timer = time.time()
        else:
            self.int_valve_port = self.ext_panel_cylinder_on

        if self.cylinder_moving_forward:
            if self.int_switch:
                self.int_valve_port = True
            if self.int_frw_switch:
                self.cylinder_moving_forward = False
                self.cylinder_moving_backward = True

        if self.cylinder_moving_backward:
            if self.int_switch:
                self.int_valve_port = False
            if self.int_bck_switch:
                self.cylinder_moving_backward = False


        self.send_cylinder_pos()


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

        if self._old_frw_switch != self.int_frw_switch:
            _logger.info(f"FRw switch to: {self.int_frw_switch}")
            self._old_frw_switch = self.int_frw_switch

        if self._old_bck_switch != self.int_bck_switch:
            _logger.info(f"Bck switch to: {self.int_bck_switch}")
            self._old_bck_switch = self.int_bck_switch


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
    def int_prox_1(self) -> bool:
        return self._read_internal_variable("prox1")[0]

    @property
    def int_prox_2(self) -> bool:
        return self._read_internal_variable("prox2")[0]

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

    def send_cylinder_pos(self):
        psi, alpha = self._crank_drive.get_d_psi_d_alpha(self._crank_drive.theta)
        self._write_internal_variable("gate_node", [-self._crank_drive.theta])
        self._write_internal_variable("piston_ext_node", [psi])
        self._write_internal_variable("piston_in_node", [alpha])

    @property
    def int_bck_switch(self) -> bool:
        return bool(self._read_internal_variable("piston_sens2")[0])

    @property
    def int_frw_switch(self) -> bool:
        return bool(self._read_internal_variable("piston_sens1")[0])

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
        self._motor.movement_speed = value * 2*3.14

    @property
    def int_valve_port(self) -> bool:
        return self._crank_drive.moving_forward

    @int_valve_port.setter
    def int_valve_port(self, value: bool):
        self._crank_drive.moving_forward = value


async def main():
    panel_plc = GlobalPlc()
    await panel_plc.init()
    plc_task = asyncio.create_task(panel_plc.run())
    signal.signal(signal.SIGINT, panel_plc.close)
    signal.signal(signal.SIGTERM, panel_plc.close)
    print(panel_plc.print_registers())
    print(panel_plc.print_topics())
    try:
        while not plc_task.done():
            await asyncio.sleep(1.0)
    finally:
        await plc_task

if __name__ == '__main__':
    asyncio.run(main())




