from PyQt5 import QtWidgets as qtw
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import Qt, QTimer, pyqtSignal

from digtwin.gui.p3d_gui import P3dGui, PandaBox

class QtGui(qtw.QMainWindow):
    def __init__(self, gui_3d: P3dGui):
        super(QtGui, self).__init__()
        self.setGeometry(50, 50, 800, 600)
        self.title = "Digital Twin - demo"
        self.setWindowTitle(self.title)
        # self.setWindowIcon(QIcon(getIcoPath("main_w-24px.svg")))
        main_widget = qtw.QWidget()
        main_layout = qtw.QVBoxLayout()
        layout_3d = qtw.QHBoxLayout()
        control_3d_scene = qtw.QHBoxLayout()
        self.gui_3d = gui_3d
        layout_3d.addWidget(PandaBox(gui_3d))

        spawn_actor = qtw.QPushButton("Spawn object")
        despawn_actor = qtw.QPushButton("Remove all objects")
        default_view = qtw.QPushButton("Restore view")

        #clean_plate.clicked.connect(self.gui_3d.clear_printing_plate)

        default_view.clicked.connect(self.restore_cam_default)
        spawn_actor.clicked.connect(self.spawn_actors)
        despawn_actor.clicked.connect(self.despawn_actors)

        control_3d_scene.addStretch()

        for qt_nodes in self.gui_3d.qt_nodes_list:
            control_3d_scene.addWidget(qt_nodes.get_widget())

        control_3d_scene.addWidget(spawn_actor)
        control_3d_scene.addWidget(despawn_actor)
        control_3d_scene.addWidget(default_view)



        main_layout.addLayout(layout_3d)
        main_layout.addLayout(control_3d_scene)


        main_widget.setLayout(main_layout)

        self.setCentralWidget(main_widget)

        # menubar
        menubar = self.menuBar()
        file = menubar.addMenu("File")
        tools = menubar.addMenu("Tools")


        # toolbar
        tb = self.addToolBar("My tools")
        tb.setToolButtonStyle(Qt.ToolButtonTextUnderIcon)

        self.show()

    def restore_cam_default(self):
        """
            reset camera view
        """
        self.gui_3d.camera_preset()

    def spawn_actors(self):
        self.gui_3d.spawn_actors()

    def despawn_actors(self):
        self.gui_3d.despawn_actors()

