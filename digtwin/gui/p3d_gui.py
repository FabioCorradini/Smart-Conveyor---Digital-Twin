import QPanda3D.Panda3DWorld as p3dw
from QPanda3D.QPanda3DWidget import QPanda3DWidget, QMouseEvent
from PyQt5.QtCore import Qt, pyqtSignal
from panda3d.core import NodePath, LVecBase3, BitMask32
from panda3d.physics import ActorNode, ForceNode, LinearVectorForce, PhysicsCollisionHandler
import numpy as np
from pathlib import Path
import logging
from dataclasses import dataclass
import random
from digtwin.gui.models.dt_actors import DTActor
from digtwin.gui.models.dt_models import DTModel, DTStatefulModel
from digtwin.gui.models.dt_sensors import DTSensor
from digtwin.gui.nodes.dt_nodes import DTNode, DTNodeState
from digtwin.gui.dt_loadable import DTLoadable
from digtwin.utils import constants
from direct.stdpy.threading import Thread
from digtwin.gui.qt.action_window import DTActionWindow
import time
from digtwin.communication.common import GUISideCommunicationProcess, ThetaMessage, DtComQueue, DtComCmd
from digtwin.communication.mqtt import MqttGUIComProc
import queue
# from panda3d.core import loadPrcFileData
# #
# loadPrcFileData('', 'clock-mode limited')
# loadPrcFileData('', 'clock-frame-rate 60')
# loadPrcFileData('', 'show-frame-rate-meter 1')


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



@dataclass
class LivingActorNode:
    model: NodePath
    actor: ActorNode
    physics: NodePath

class P3dGui(p3dw.Panda3DWorld):

    AXIS_ROT_ID = {'z': 0, 'y': 2, 'x': 1}
    model_list: list[DTModel]
    stateful_model_list: list[DTStatefulModel]
    nodes_list: list[DTNode]
    actor_list: list[DTActor]
    living_actors: dict[str, LivingActorNode]
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
        self.actor_list = []
        self.loadable_dict = {}

        self.living_actors = {}

        self.constVect = const_vect
        self.constMat = const_mat
        self.theta = np.zeros(3)

        super().__init__()

        self.exiting = False

        self._ready = False

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

        # setting mouse collisions
        self.mouse_traverser = p3dw.CollisionTraverser()
        picker_node = p3dw.CollisionNode("mouseRay")
        picker_np = self.cam.attach_new_node(picker_node)
        self.picker_ray = p3dw.CollisionRay()
        picker_node.addSolid(self.picker_ray)
        picker_node.setFromCollideMask(BitMask32(0x01))
        self.collision_handler = p3dw.CollisionHandlerQueue()
        self.mouse_traverser.addCollider(picker_np, self.collision_handler)

        # setting physics
        self.cTrav = p3dw.CollisionTraverser()

        self.enable_particles()
        self.spawner_node = self.render.attach_new_node("spawner_node")
        self.spawner_node.setPos(0, -400, 1000)

        gravityFN = ForceNode('world-forces')
        gravityFNP = self.render.attachNewNode(gravityFN)
        gravityForce = LinearVectorForce(0, 0, -9810)  # gravity acceleration
        gravityFN.addForce(gravityForce)

        self.pusher = PhysicsCollisionHandler()
        c_node = p3dw.CollisionNode("collision_floor")
        c_node.addSolid(p3dw.CollisionPlane(p3dw.Plane(p3dw.Vec3(0,0,1), p3dw.Point3(0, 0, 0))))
        c_node.setCollideMask(BitMask32(0x02))
        self.floor_plane = self.render.attach_new_node(c_node)

        self.physicsMgr.addLinearForce(gravityForce)

        self.pusher.addInPattern("into_%in")
        self.pusher.addOutPattern("out_%in")

        # debug tools

        # self._debug_routine = Thread(target=self.debug_routine)
        # self.messenger.toggle_verbose()

        # communication tools
        self._cmd_queue = DtComCmd()
        self._gui_to_dt = DtComQueue()
        self._dt_to_gui = DtComQueue()

        self._com_proc = MqttGUIComProc(self._gui_to_dt, self._dt_to_gui, self._cmd_queue,
                                        mqtt_address=constants.DIGITAL_TWIN_MQTT_ADDRESS,
                                        mqtt_port=constants.DIGITAL_TWIN_MQTT_PORT)

        self._communication_routine = Thread(target=self.communication_read_routine)
        self._sensors_thread_list: list[Thread] = []


    def start_communication(self):
        # every node is input
        for node in self.nodes_list:
            self._com_proc.add_target_model(node)

        for stateful_model in self.stateful_model_list:
            self._com_proc.add_target_model(stateful_model)
            if not stateful_model.is_input():
                self._sensors_thread_list.append(Thread(name=f"{stateful_model.name}_write_thread", target= self.communication_write_routine, args=(stateful_model,)))

        self._com_proc.start()
        self._cmd_queue.send_connect_cmd()

        self._communication_routine.start()

        for th in self._sensors_thread_list:
            th.start()



    def debug_routine(self):

        def get_alpha_delta(in_theta: float) -> tuple[float, float]:
            A = np.array([[+34.3], [-58.1]])
            B = np.array([[+197.2], [+59.0]])
            V = (B - A).flatten()
            psi = np.atan2(V[0], V[1])
            phi = np.arctan2(A[0], A[1])

            c, s = np.cos(in_theta), np.sin(in_theta)
            R = np.array(((c, -s), (s, c)))
            A_1 = np.matmul(R, A).flatten()
            V_1 = B.flatten() - A_1
            psi_1 = np.atan2(V_1[0], V_1[1])
            phi_1 = np.arctan2(A_1[0], A_1[1])

            return psi_1 - psi, - phi - psi + phi_1 + psi_1


        theta = 0
        fps = 30
        increment = 2*np.pi/fps

        while not self._ready:
            time.sleep(1/fps)
            if self.exiting:
                break

        start_time = time.time()
        red_light_model = self.loadable_dict["lightpanel_red"]
        green_light_model = self.loadable_dict["lightpanel_green"]
        red_light_model: DTStatefulModel
        green_light_model: DTStatefulModel
        red_light_model.state_id = 1

        gate_direction = True
        gate_angle = np.pi/4
        old_theta_gate = 0
        alpha = 0
        delta = 0

        while not self.exiting:
            theta += increment
            for node in self.nodes_list:
                if node.name.count("conveyor_node") or node.name == "encoder_node":
                    node.set_theta((theta,))
                    node.to_position()
                if node.name == "gate_node":
                    theta_gate = (theta/3) % gate_angle
                    if old_theta_gate > theta_gate:
                        gate_direction = not gate_direction

                    if gate_direction:
                        node.set_theta((-theta_gate,))
                        alpha, delta = get_alpha_delta(-theta_gate)
                    else:
                        node.set_theta((-gate_angle+theta_gate,))
                        alpha, delta = get_alpha_delta(-gate_angle+theta_gate)

                    node.to_position()
                    old_theta_gate = theta_gate

                if node.name == "piston_ext_node":
                    node.set_theta((alpha,))
                    node.to_position()

                if node.name == "piston_in_node":
                    node.set_theta((delta,))
                    node.to_position()

            if time.time() - start_time > 1.0:

                if red_light_model.state_id == 0:
                    red_light_model.state_id =1
                else:
                    red_light_model.state_id =0

                if green_light_model.state_id == 0:
                    green_light_model.state_id =1
                else:
                    green_light_model.state_id =0

                red_light_model.to_state()
                green_light_model.to_state()

                start_time = time.time()


            time.sleep(1/fps)

        _logger.info("Closing debug routine")

    def communication_read_routine(self):
        while not self.exiting:
            try:
                in_msg = self._dt_to_gui.get(True, 2.0)
                # _logger.warning(f"Received message: {in_msg}")
                if in_msg.target_name in self.loadable_dict:
                    loadable = self.loadable_dict[in_msg.target_name]
                    if isinstance(loadable, DTNode):
                        loadable.set_theta(in_msg.values)
                        loadable.to_position()
                    elif isinstance(loadable, DTStatefulModel):
                        loadable.state_id = in_msg.values[0]
                        loadable.to_state()
                    else:
                        _logger.error(f"Cannot pass values to loadable {loadable.name} or type {type(loadable)}")
                else:
                    _logger.error(f"Unknown loadable {in_msg.target_name}")
            except queue.Empty:
                pass

    def communication_write_routine(self, dt_sensor: DTSensor):
        _logger.info(f"Started reading routine for {dt_sensor.name}")
        while not self.exiting:
            dt_sensor.changed_event.wait(2.0)
            if not self.exiting and dt_sensor.changed_event.is_set():
                # _logger.info(f"sensor {dt_sensor.name} changed")
                self._gui_to_dt.put_message(dt_sensor.name, [dt_sensor.state_id])
                dt_sensor.changed_event.clear()

        _logger.info(f"Stopping reading routine for {dt_sensor.name}")


    def get_clicked_obj_name(self, pos_x: float, pos_y: float) -> DTLoadable | None:
        self.picker_ray.setFromLens(self.camNode, pos_x, pos_y)
        self.mouse_traverser.traverse(self.render)

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
                        node_name = "_".join(node_name.split("_")[:-1])
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

        if constants.ACTOR_DIR.is_dir():
            for dt_actor_path in constants.ACTOR_DIR.glob("**/*.json"):
                act_model = DTActor.load(dt_actor_path)
                self._add_actor(act_model)
                # actor are not reparented

        if constants.SENSORS_DIR.is_dir():
            for dt_sensor_path in constants.SENSORS_DIR.glob("**/*.json"):
                dt_sensor = DTSensor.load(dt_sensor_path)
                self._add_stateful_model(dt_sensor)
                self.loadable_dict[dt_sensor.name] = dt_sensor


        for name, dt_loadable in self.loadable_dict.items():
            if dt_loadable.parent is None:
                dt_loadable.reparent(self.render)
            else:
                parent_node = self.loadable_dict[dt_loadable.parent].node_path_reference
                dt_loadable.reparent(parent_node)
                _logger.info(f"Model {name} reparented to {parent_node.name}")
                if isinstance(dt_loadable, DTModel) and dt_loadable.detectable and dt_loadable.collision_node_reference is not None:
                    _logger.info(f"Model {dt_loadable.name} clickable, added to traverser")
                    self.cTrav.addCollider(dt_loadable.collision_node_reference, self.pusher)

        self.set_positions()
        self._ready = True

    def _add_model(self, model_obj: DTModel):
        """
            add a solid object to the simulation
        """
        model_obj.node_path_reference = self.loader.load_model(model_obj.model_path)
        model_obj.node_path_reference.name = model_obj.name
        self.model_list.append(model_obj)
        _logger.info(f"Added model {model_obj.node_path_reference.name}")

    def _add_actor(self, actor_obj: DTActor):
        """
            add a solid object to the simulation
        """
        actor_obj.node_path_reference = self.loader.load_model(actor_obj.model_path)
        actor_obj.node_path_reference.name = actor_obj.name
        actor_obj.to_color()
        self.actor_list.append(actor_obj)
        _logger.info(f"Added actor {actor_obj.node_path_reference.name}")

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

    def spawn_actors(self):
        num = len(self.living_actors)
        new_actor: DTActor = random.choice(self.actor_list)
        node = NodePath(f"physics_{new_actor.name}_{num}")
        node.reparentTo(self.spawner_node)
        an = ActorNode(f"actor_{new_actor.name}_{num}")
        anp = node.attachNewNode(an)
        self.physicsMgr.attachPhysicalNode(an)
        actor: NodePath = new_actor.node_path_reference.copyTo(anp)
        actor.name = f"model_{new_actor.name}_{num}"
        self.living_actors[f"{new_actor.name}_{num}"] = LivingActorNode(actor, an, node)

        from_object: NodePath = anp.attachNewNode(p3dw.CollisionNode(f"col_node_{new_actor.name}_{num}"))
        from_object.node().addSolid(new_actor.build_collision_solid())
        new_actor.align_collision_solid(from_object)
        from_object.node().setFromCollideMask(BitMask32(0x06)) # solid anf detectable
        self.pusher.addCollider(from_object, anp)

        self.cTrav.addCollider(from_object, self.pusher)

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

    def exit_program(self, signum=0, frame=None):
        """Tells to the diagnosis process that user requested the closure of the application """
        _logger.warning(f"Closing GUI with signal {signum}")
        self.exiting = True
        if self._communication_routine.is_alive():
            _logger.info("Waiting for debug routine")
            self._communication_routine.join()
        self._cmd_queue.send_close_cmd() # for closing communication proc
        self._com_proc.join()
        self._com_proc.close()
        for th in self._sensors_thread_list:
            th.join()

