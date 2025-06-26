from digtwin.gui.dt_loadable import DTLoadable
from direct.stdpy.threading import Event
from PyQt5 import QtWidgets


class DTCheckNode(DTLoadable):

    def __init__(self,
                 name: str,
                 source_dev: str,
                 display_name: str,
                 default_state: bool = False,
                 ):

        super().__init__(name, None)
        self.display_name = display_name
        self.source_dev = source_dev
        self._current_state = default_state
        self.default_state = default_state
        self.changed_event = Event()

    @property
    def state_id(self):
        return self._current_state

    @state_id.setter
    def state_id(self, value: bool) -> None:
        if value != self._current_state:
            self._current_state = value
            self.changed_event.set()

    def to_dict(self):
        out_dict = super().to_dict()
        out_dict["source_dev"] = self.source_dev
        out_dict['display_name'] = self.display_name
        out_dict['default_state'] = self.default_state
        return out_dict

    def on_click(self):
        if self.state_id:
            self.state_id = False
        else:
            self.state_id = True

    def get_widget(self) -> QtWidgets.QAbstractButton:
        check_box = QtWidgets.QCheckBox()
        check_box.setText(self.display_name)
        check_box.setChecked(self.state_id)
        check_box.toggled.connect(self.on_click)
        return check_box

    @staticmethod
    def from_dict(in_dict: dict) -> 'DTCheckNode':
        return DTCheckNode(
            name = in_dict['name'],
            source_dev = in_dict['source_dev'],
            display_name = in_dict['display_name'],
            default_state = in_dict['default_state']
        )
