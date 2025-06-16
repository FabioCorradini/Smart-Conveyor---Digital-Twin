from digtwin.control.plc_subsystem import PLCSubsystem, ModVarType
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
        self._register_internal_variable("emergency", False, [True], "mushroom")
        self._register_internal_variable("switch", False, [False], "festo_switch")
        self._register_internal_variable("red_button", False, [False], "pushbutton_red")
        self._register_internal_variable("green_button", False, [False], "pushbutton_green")

        self._register_external_variable("system_alarm_1", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("system_alarm_2", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("system_alarm_3", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("motor_status", ModVarType.BOOLEAN, False, False)  # new
        self._register_external_variable("system_state", ModVarType.INT16, False, 0)

         # outputs

        self._register_internal_variable("green_light", True, [False], "lightpanel_green")
        self._register_internal_variable("red_light", True, [False], "lightpanel_red")


        self._register_external_variable("panel_alarm", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("panel_running", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("panel_mode", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("cylinder_on", ModVarType.BOOLEAN, True, False) # new
        self._register_external_variable("motor_on", ModVarType.BOOLEAN, True, False) # new
        self._register_external_variable("panel_state", ModVarType.INT16, True, 0)
        self._register_external_variable("panel_motor_speed", ModVarType.INT16, True, 0) # new

        # self._old_panel_alarm = False
        # self._old_system_alarm_1 = False
        # self._old_system_alarm_2 = False
        # self._old_system_alarm_3 = False
        #
        # self._old_panel_running = False
        # self._old_panel_state = 0
        # self._old_system_state = 0

        self._old_motor_on = False
        self._old_green_button = False
        self._old_panel_motor_speed = 0

        self._old_emergency = True

        self._speed_timer = time.time()
        self.loop_time =  time.time()


    def main_plc_task(self):
        self.debug_print()

        self.ext_panel_mode = self.int_switch

        self.ext_motor_on = (self.int_green_button or self.ext_motor_on) and not self.int_switch and self.int_emergency
        self.ext_cylinder_on = self.int_red_button and not self.int_switch and self.int_emergency

        if self._old_green_button != self.int_green_button:
            if self.int_green_button:
                _logger.info("Green button pressed")
                self._speed_timer = time.time()
            else:
                _logger.info("Green button released")
            self._old_green_button = self.int_green_button

        if self.int_green_button:
            if 1.0 <= (time.time() - self._speed_timer) < 2.0:
                self.ext_panel_motor_speed = 1
            elif 2.0 <= (time.time() - self._speed_timer) < 3.0:
                self.ext_panel_motor_speed = 2
            elif 3.0 <= (time.time() - self._speed_timer) < 4.0:
                self.ext_panel_motor_speed = 3
            elif 4.0 <= (time.time() - self._speed_timer):
                self.ext_panel_motor_speed = 0

        if self._old_emergency != self.int_emergency:
            if not self.int_emergency:
                self.ext_panel_alarm = True
                print("Emergency alarm!")
                self.ext_panel_motor_speed = 0
            else:
                print("Emergency realeased!")
                self.ext_panel_alarm = False
            self._old_emergency = self.int_emergency


    def debug_print(self):

        if self._old_motor_on != self.ext_motor_on:
            print(f"motor on: {self.ext_motor_on}")
            self._old_motor_on = self.ext_motor_on

        if self._old_panel_motor_speed != self.ext_panel_motor_speed:
            print(f"panel motor speed: {self.ext_panel_motor_speed}")
            self._old_panel_motor_speed = self.ext_panel_motor_speed



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
    def ext_panel_alarm(self) -> bool:
        return self._read_external_variable("panel_alarm")

    @ext_panel_alarm.setter
    def ext_panel_alarm(self, value: bool):
        self._write_external_variable("panel_alarm", value)

    @property
    def ext_system_alarm_1(self) -> bool:
        return self._read_external_variable("system_alarm_1")

    @property
    def ext_system_alarm_2(self) -> bool:
        return self._read_external_variable("system_alarm_2")
    @property
    def ext_system_alarm_3(self) -> bool:
        return self._read_external_variable("system_alarm_3")

    @property
    def ext_panel_running(self) -> bool:
        return self._read_external_variable("panel_running")

    @ext_panel_running.setter
    def ext_panel_running(self, value: bool):
        self._write_external_variable("panel_running", value)

    @property
    def ext_panel_state(self) -> int:
        return self._read_external_variable("panel_state")

    @ext_panel_state.setter
    def ext_panel_state(self, value: int):
        self._write_external_variable("panel_state", value)

    @property
    def ext_system_state(self) -> int:
        return self._read_external_variable("system_state")

    @property
    def ext_motor_status(self) -> bool:
        return self._read_external_variable("motor_status")

    @property
    def ext_cylinder_on(self):
        return self._read_external_variable("cylinder_on")

    @ext_cylinder_on.setter
    def ext_cylinder_on(self, value: bool):
        self._write_external_variable("cylinder_on", value)

    @property
    def ext_motor_on(self):
        return self._read_external_variable("motor_on")

    @ext_motor_on.setter
    def ext_motor_on(self, value: bool):
        self._write_external_variable("motor_on", value)

    @property
    def ext_panel_motor_speed(self) -> int:
        return self._read_external_variable("panel_motor_speed")

    @ext_panel_motor_speed.setter
    def ext_panel_motor_speed(self, value: int):
        self._write_external_variable("panel_motor_speed", value)

    @property
    def ext_panel_mode(self) -> bool:
        return self._read_external_variable("panel_mode")

    @ext_panel_mode.setter
    def ext_panel_mode(self, value: bool):
        self._write_external_variable("panel_mode", value)

async def main():
    panel_plc = SmartConveyorPanel()
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




