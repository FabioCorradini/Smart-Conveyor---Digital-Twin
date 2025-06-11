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
        super(SmartConveyorPanel, self).__init__("Smart Conveyor Panel")
        self._register_internal_variable("green_light", True, [False], "lightpanel_green")
        self._register_internal_variable("red_light", True, [False], "lightpanel_red")
        self._register_internal_variable("emergency", False, [True], "mushroom")
        self._register_internal_variable("switch", False, [False], "festo_switch")
        self._register_internal_variable("red_button", False, [False], "pushbutton_red")
        self._register_internal_variable("green_button", False, [False], "pushbutton_green")

        self._register_external_variable("panel_alarm", ModVarType.BOOLEAN, True, False)
        self._register_external_variable("system_alarm_1", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("system_alarm_2", ModVarType.BOOLEAN, False, False)
        self._register_external_variable("system_alarm_3", ModVarType.BOOLEAN, False, False)

        self._register_external_variable("panel_running", ModVarType.BOOLEAN, True, False)

        self._register_external_variable("panel_state", ModVarType.INT16, True, 0)
        self._register_external_variable("system_state", ModVarType.INT16, False, 0)

        self._old_panel_alarm = False
        self._old_system_alarm_1 = False
        self._old_system_alarm_2 = False
        self._old_system_alarm_3 = False

        self._old_panel_running = False
        self._old_panel_state = 0
        self._old_system_state = 0

        self._blink_timer1 = time.time()
        self.loop_time =  time.time()

    @property
    def int_green_light(self) -> bool:
        return self._read_internal_variable("green_light")[0]
    @int_green_light.setter
    def int_green_light(self, value: bool):
        self._write_internal_variable("green_light", [value])
    @property
    def int_red_light(self) -> bool:
        return self._read_internal_variable("red_light")[0]
    @int_red_light.setter
    def int_red_light(self, value: bool):
        self._write_internal_variable("red_light", [value])
    @property
    def int_emergency(self) -> bool:
        return self._read_internal_variable("emergency")[0]
    @property
    def int_switch(self) -> bool:
        return self._read_internal_variable("switch")[0]
    @property
    def int_red_button(self) -> bool:
        return self._read_internal_variable("red_button")[0]
    @property
    def int_green_button(self) -> bool:
        return self._read_internal_variable("green_button")[0]

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

    def main_plc_task(self):
        self.debug_print()
        self.ext_panel_alarm = not self.int_emergency
        any_alarm = self.ext_panel_alarm  or self.ext_system_alarm_1 or self.ext_system_alarm_2 or self.ext_system_alarm_3
        self.ext_panel_running = not any_alarm and self.int_switch

        if not any_alarm:
            if self.ext_panel_running:
                self.int_green_light = True
                self.int_red_light = False
            else:
                self.int_green_light = False
                self.int_red_light = True

        if any_alarm:
            if time.time() - self._blink_timer1 > 2.0:
                self.int_green_light = not self.int_green_light
                self.int_red_light = not self.int_red_light
                self._blink_timer1 = time.time()

                # current_time = time.time()
                # print(f"loop time: {current_time - self.loop_time}")
                # self.loop_time = current_time

    def debug_print(self):
        if self._old_panel_alarm != self.ext_panel_alarm:
            print(f"panel_alarm: {self.ext_panel_alarm}")
            self._old_panel_alarm = self.ext_panel_alarm

        if self._old_system_alarm_1 != self.ext_system_alarm_1:
            print(f"system_alarm_1: {self.ext_system_alarm_1}")
            self._old_system_alarm_1 = self.ext_system_alarm_1

        if self._old_system_alarm_2 != self.ext_system_alarm_2:
            print(f"system_alarm_2: {self.ext_system_alarm_2}")
            self._old_system_alarm_2 = self.ext_system_alarm_2

        if self._old_system_alarm_3 != self.ext_system_alarm_3:
            print(f"system_alarm_3: {self.ext_system_alarm_3}")
            self._old_system_alarm_3 = self.ext_system_alarm_3

        if self._old_panel_running != self.ext_panel_running:
            print(f"panel_running: {self.ext_panel_running}")
            self._old_panel_running = self.ext_panel_running

        if self._old_panel_state != self.ext_panel_state:
            print(f"panel_state: {self.ext_panel_state}")
            self._old_panel_state = self.ext_panel_state


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




