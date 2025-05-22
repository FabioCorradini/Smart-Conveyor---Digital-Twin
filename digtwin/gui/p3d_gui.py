import QPanda3D.Panda3DWorld as p3dw
from QPanda3D.QPanda3DWidget import QPanda3DWidget, QMouseEvent
from PyQt5.QtCore import Qt, pyqtSignal
from panda3d.core import NodePath, LVecBase3
import numpy as np
from pathlib import Path
import logging
from digtwin.gui.models.dt_models import DTModel, DTStatefulModel
from digtwin.gui.nodes.dt_nodes import DTNode, DTNodeState
from digtwin.gui.dt_loadable import DTLoadable
from digtwin.utils import constants
from direct.stdpy.threading import Thread
from digtwin.gui.qt.action_window import DTActionWindow
import time

_logger = logging.getLogger(__name__)

class PandaBox(QPanda3DWidget):
    """
        A Qt5 widget containing the 3d environment and accepting mouse scrolling
    """
    ROTATION_MAX_VAL = [100, 100]
    SHIFT_MAX_VAL = [1000, 1000]
    generate_part_menu = pyqtSignal(DTLoadable, int, int)

    def __init__(self, panda_world: 'P3dGui'):
        """
        :param panda_world: "Panda3D environment"
        """
        super(PandaBox, self).__init__(panda_world)
        self.active_rotation = False
        self.active_shifting = False

        self.start_x_movement = 0
        self.start_y_movement = 0
        self.active_aw = None
        self.panda_world = panda_world
        self.generate_part_menu.connect(self.pop_part_menu)

    def pop_part_menu(self, obj: DTLoadable, pos_x, pos_y):
        if isinstance(obj, DTStatefulModel):
            aw = DTActionWindow(obj)
            aw.move(pos_x, pos_y)
            aw.show()
            # only one active at time
            if (self.active_aw is not None) and (not self.active_aw.isHidden()):
                self.active_aw.close()
            self.active_aw = aw


    def mouseDoubleClickEvent(self, evt: QMouseEvent) -> None:
        if evt.button() == Qt.LeftButton:
            w_size = self.size()
            x_lens = evt.x() / w_size.width() * 2 - 1.0
            y_lens = - evt.y() / w_size.height() * 2 + 1.0
            part = self.panda3DWorld.get_clicked_obj_name(x_lens, y_lens)
            if part is not None:
                self.generate_part_menu.emit(part, evt.globalX(), evt.globalY())

    def mousePressEvent(self, evt):

        if evt.button() == Qt.LeftButton:
            self.active_rotation = True
            self.start_x_movement, self.start_y_movement = self.get_norm_input(evt.x(), evt.y(), True)

        elif evt.button() == Qt.RightButton:
            self.active_shifting = True
            self.start_x_movement, self.start_y_movement = self.get_norm_input(evt.x(), evt.y(), False)

    def mouseMoveEvent(self, evt):
        if self.active_rotation:
            new_x, new_y = self.get_norm_input(evt.x(), evt.y(), True)
            self.panda_world.rotate_camera(- new_x + self.start_x_movement, 'z')
            self.panda_world.rotate_camera(- new_y + self.start_y_movement, 'x')
            self.start_x_movement = new_x
            self.start_y_movement = new_y

        elif self.active_shifting:
            new_x, new_y = self.get_norm_input(evt.x(), evt.y(), False)
            self.panda_world.shift_camera(-new_x + self.start_x_movement, +new_y - self.start_y_movement)
            self.start_x_movement = new_x
            self.start_y_movement = new_y

    def mouseReleaseEvent(self, evt):
        self.active_shifting = False
        self.active_rotation = False

    def wheelEvent(self, evt):
        self.panda_world.zoom_camera(-evt.angleDelta().y())


    def get_norm_input(self, x_pixel:int, y_pixel:int, rotation:bool=True)->tuple[float, float]:
        """Normalize the mouse movement so it is not dependent from window size
        :param x_pixel: the pixel of the movement in x direction
        :param y_pixel: the pixel of the movement in y direction
        :param rotation: if set to True ROTATION_MAX_VAL is used to determine the max output value,
            if false SHIFT_MAX_VAL is used instead
        :returns: "A tuple of float containing x and y dimension normalized"
        """

        w_size = self.size()
        if rotation:
            max_size = PandaBox.ROTATION_MAX_VAL
        else:
            max_size = PandaBox.SHIFT_MAX_VAL

        return x_pixel / w_size.width() * max_size[0], y_pixel / w_size.height() * max_size[1]





class P3dGui(p3dw.Panda3DWorld):

    AXIS_ROT_ID = {'z': 0, 'y': 2, 'x': 1}
    model_list: list[DTModel]
    stateful_model_list: list[DTStatefulModel]
    nodes_list: list[DTNode]
    loadable_dict: dict[str,DTLoadable]
    cam_base_node: NodePath
    cam: NodePath

    def __init__(self, const_vect: np.ndarray | None = None, const_mat: np.ndarray | None = None):
        """
            3D environment based on panda3d

            :param const_vect: an array representing the constant vector (C) in the fuction f(x) = [M] * (x+C) = theta
                where x is the position in the printing space
            :type const_vect: np.ndarray

            :param const_mat: a matrix representing the constant matrix [M] in the function f(x) = [M] * (x+C) = theta
                where x is the position in the printing space
            :type const_mat: np.ndarray
        """

        self.model_list = []
        self.stateful_model_list = []
        self.nodes_list = []
        self.loadable_dict = {}

        self.constVect = const_vect
        self.constMat = const_mat
        self.theta = np.zeros(3)

        super().__init__()

        self.exiting = False

        # setting camera

        self.cam_base_node = self.render.attach_new_node("cam_base_node")
        self.cam_base_node.setPos(0, 0, 500)
        self.cam_base_node.setHpr(-90, -45, 0)
        self.cam.reparentTo(self.cam_base_node)

        # self.cam.setHpr(-45, 0, -0)
        self.cam.setPos(0, -3000, 0)

        self.cam_fixed_node = self.render.attach_new_node("cam_fixed_node")
        self.tool_node = self.render.attach_new_node("cam_moving_node")

        # settings lights

        dir_light = p3dw.DirectionalLight("light")
        dir_light.setColor((0.5, 0.5, 0.5, 0.5))
        light_np = self.cam.attach_new_node(dir_light)
        light_np.set_hpr(-20., -20., 0.)
        self.render.set_light(light_np)

        alight = p3dw.AmbientLight('alight')
        alight.setColor((0.1, 0.1, 0.1, 0.5))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        # dir_light.setShadowCaster(True, 512, 512)
        self.render.setShaderAuto()

        # setting collisions
        self.my_traverser = p3dw.CollisionTraverser()
        picker_node = p3dw.CollisionNode("mouseRay")
        picker_np = self.cam.attach_new_node(picker_node)
        self.picker_ray = p3dw.CollisionRay()
        picker_node.addSolid(self.picker_ray)
        self.collision_handler = p3dw.CollisionHandlerQueue()
        self.my_traverser.addCollider(picker_np, self.collision_handler)

        # debug tools

        self._debug_routine = Thread(target=self.debug_routine)
        self._debug_routine.start()


    def debug_routine(self):
        theta = 0
        fps = 30
        increment = 2*np.pi/fps

        start_time = time.time()

        while not self.exiting:
            theta += increment
            for node in self.nodes_list:
                node.set_theta((theta,))
                node.to_position()


            time.sleep(1/fps)

        _logger.info("Closing debug routine")

    def get_clicked_obj_name(self, pos_x: float, pos_y: float) -> DTLoadable | None:
        self.picker_ray.setFromLens(self.camNode, pos_x, pos_y)
        self.my_traverser.traverse(self.render)

        if self.collision_handler.getNumEntries() > 0:
            self.collision_handler.sortEntries()
            picked_obj = self.collision_handler.getEntry(0).getIntoNodePath()
            picked_obj = picked_obj.findNetTag("clickable")
            if not picked_obj.isEmpty():
                obj_path = str(picked_obj)
                node_name = obj_path.split("/")[-2]
                if node_name in self.loadable_dict:
                    return self.loadable_dict[node_name]
                else:
                    try:
                        node_name = node_name.split("_")[-2]
                        if node_name in self.loadable_dict:
                            return self.loadable_dict[node_name]
                        else:
                            _logger.warning(f"Node could not parse {obj_path}")
                    except IndexError:
                        _logger.warning(f"Node could not parse {obj_path}")

        return None


    def set_positions(self):
        for name, loadable in self.loadable_dict.items():
            if isinstance(loadable, DTStatefulModel):
                loadable.to_state()
            else:
                if isinstance(loadable, DTNode):
                    loadable.set_theta((0.0,))
                loadable.to_position()
                loadable.to_color()


    def load_objects(self):

        if constants.MODELS_DIR.is_dir():
            for dt_model_path in constants.MODELS_DIR.glob("**/*.json"):
                dt_model = DTModel.load(dt_model_path)
                self._add_model(dt_model)
                self.loadable_dict[dt_model.name] = dt_model

        if constants.NODES_DIR.is_dir():
            for dt_node_path in constants.NODES_DIR.glob("**/*.json"):
                dt_node = DTNode.load(dt_node_path)
                self._add_node(dt_node)
                self.loadable_dict[dt_node.name] = dt_node

        if constants.STATE_MODELS_DIR.is_dir():
            for dt_model_path in constants.STATE_MODELS_DIR.glob("**/*.json"):
                dt_stateful_model = DTStatefulModel.load(dt_model_path)
                self._add_stateful_model(dt_stateful_model)
                self.loadable_dict[dt_stateful_model.name] = dt_stateful_model


        for name, dt_loadable in self.loadable_dict.items():
            if dt_loadable.parent is None:
                dt_loadable.reparent(self.render)
            else:
                parent_node = self.loadable_dict[dt_loadable.parent].node_path_reference
                dt_loadable.reparent(parent_node)
                _logger.info(f"Model {name} reparented to {parent_node.name}")

        self.set_positions()

    def _add_model(self, model_obj: DTModel):
        """
            add a solid object to the simulation
        """
        model_obj.node_path_reference = self.loader.load_model(model_obj.model_path)
        model_obj.node_path_reference.name = model_obj.name
        self.model_list.append(model_obj)
        _logger.info(f"Added model {model_obj.node_path_reference.name}")

    def _add_stateful_model(self, model_obj: DTStatefulModel):
        """
            add a solid object to the simulation
        """
        model_obj.populate(self.loader)
        self.stateful_model_list.append(model_obj)
        _logger.info(f"Added stateful model {model_obj.name}")

    def _add_node(self, node_obj: DTNode):
        """
            add a reference node to the simulation
        """
        node_obj.node_path_reference = self.render.attach_new_node(node_obj.name)
        self.nodes_list.append(node_obj)
        _logger.info(f"Added node {node_obj.node_path_reference.name}")


    #camera methods

    def shift_camera(self, x, z):
        """
        Move the camera up/down or left/right

        :param x: the amount of left/right movement
        :type x: float

        :param z: the amount of up/down movement
        :type z: float
        """

        shift_vec = self.render.get_relative_point(self.cam_base_node, LVecBase3(x, 0, z))
        self.cam_base_node.setPos(shift_vec)


    def rotate_camera(self, rot, axis='z'):
        """
        Rotate camera around an axis

        :param rot: the amount of rotation
        :type rot: float

        :param axis: axis of rotation, can be 'x','y', or 'z'
        :type axis: str
        """

        index = P3dGui.AXIS_ROT_ID[axis]

        hpr = self.cam_base_node.getHpr()
        hpr[index] = hpr[index] + rot

        self.cam_base_node.setHpr(hpr)

    def zoom_camera(self, adv):
        """
        Move the camera forward or backward

        :param adv: the amount of frw/bcw movement
        :type adv: float
        """
        pos = self.cam.getPos()
        self.cam.setPos(pos[0], pos[1] - adv, pos[2])


    def set_fixed_camera(self):
        """Link the camera to a fixed point """
        self.cam_base_node.reparentTo(self.cam_fixed_node)

    def camera_preset(self):
        """
        Move the camera to its starting position
        """
        self.cam_base_node.setPos(0, 0, 500)
        self.cam_base_node.setHpr(-90, -45, 0)
        self.set_fixed_camera()
        # self.cam.setHpr(-45, 0, -0)
        self.cam.setPos(0, -3000, 0)


    def userExit(self):
        """Catch the closure of the application"""
        self.exit_program()
        # if not self.simulation:
        #     self.exitGUI()

    def exit_gui(self):
        _logger.info("Closing GUI")
        super().userExit()

    def exit_program(self):
        """Tells to the diagnosis process that user requested the closure of the application """
        self.exiting = True
        _logger.info("Waiting for debug routine")
        self._debug_routine.join()
