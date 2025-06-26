from digtwin.gui.models.dt_models import DTModel, DTStatefulModel
from digtwin.gui.models.dt_actors import DTActor
from digtwin.gui.nodes.dt_nodes import DTNode
from digtwin.gui.models.dt_sensors import DTSensor
from digtwin.gui.qt_nodes.dt_chek_nodes import DTCheckNode

from digtwin.utils import constants
import numpy as np
from panda3d.core import decode_sRGB_float, LQuaternion, LVector3f
import argparse

# PLC subsystems

parser = argparse.ArgumentParser(
                    prog='Data generator for the GUI',
                    description='It generates GUI configuration files',
                    epilog='Text at the bottom of help')

parser.add_argument('-g', '--global_plc', action='store_true', help="Set the configuration to work with a single PLC")

args = parser.parse_args()

if args.global_plc:
    sc_panel = "global_plc"
    sc_motor = "global_plc"
    sc_cylinder = "global_plc"
else:
    sc_panel = "smart_conveyor_panel"
    sc_motor = "smart_conveyor_motor"
    sc_cylinder = "smart_conveyor_cylinder"

# models

white_color = (0.8, 0.8, 0.8, 1)
black_color = (0.05, 0.05, 0.05, 1)
orange_color = (decode_sRGB_float(0xff), decode_sRGB_float(0x99), decode_sRGB_float(0x00), 1)

main_conveyor = DTModel(
    "main_frame",
    r"./Data/obj/fixed_conveyor_scaled.stl",
    collision_center=np.array([0.0, 332.0, 548.0]),
    collision_sides = (100.0, 460.0, 30.0),
    collision_rotation_axis = np.array([1.0, 0.0, 0.0]),
    collision_rotation_angle= np.pi/4,
    solid=True
)

main_conveyor.color = white_color

main_conveyor.save(constants.MODELS_DIR)

main_frame= DTModel(
    "main_conveyor",
    r"./Data/obj/Fixed_frame_lite_scaled.stl",
    collision_center=np.array([0.0, -280.0, 410.0]),
    collision_sides = (125.0, 290.0, 20.0),
    collision_rotation_axis = np.array([1.0, 0.0, 0.0]),
    collision_rotation_angle= -np.pi/6,
    solid=True
)

main_frame.color = (decode_sRGB_float(0x4b), decode_sRGB_float(0x73), decode_sRGB_float(0x9f), 1.0)

main_frame.save(constants.MODELS_DIR)

plate = DTModel(
    "sliding_plate",
    r"./Data/obj/plate_scaled.stl"
)

plate.color = white_color

plate.save(constants.MODELS_DIR)

plate_axis = DTModel(
    "plate_axis",
    r"./Data/obj/plate_axis_scaled.stl"
)

plate_axis.color = black_color

plate_axis.save(constants.MODELS_DIR)


# nodes

pa = np.array((-15, -1 , 231.5)) #center of rotation at the center of conveyor
pb = np.array((-15,642,874.5)) # center of rotation at the opposite site of the conveyor

L = np.linalg.norm(pb-pa)

alpha = np.arctan2((pb-pa)[2],(pb-pa)[1])

r = 17.5

p1 = pa + np.array([0, - r * np.cos(np.pi/2-alpha), +r * np.sin(np.pi/2-alpha)])
p2 = pb + np.array([0, - r * np.cos(np.pi/2-alpha), +r * np.sin(np.pi/2-alpha)])
p3 = pb + np.array([0, + r * np.cos(np.pi/2-alpha), -r * np.sin(np.pi/2-alpha)])
p4 = pa + np.array([0, + r * np.cos(np.pi/2-alpha), -r * np.sin(np.pi/2-alpha)])

rot1 = 0.0
rot2 = L/r
rot3 = rot2 + np.pi
rot4 = rot3 + L/r
rot0 = rot4 + np.pi

p_diff = pb - pa

conveyor_main_node = DTNode("conveyor_node", sc_motor ,theta_period=np.array([[rot0]]))

conveyor_main_node.add_node_state(
    state_name="p12",
    exit_theta_condition=np.array([[rot2]]),
    const_shift_matrix=np.array([pa]).transpose(),
    var_shift_matrix=np.array([[p_diff[0]/L*r], [p_diff[1]/L*r], [p_diff[2]/L*r]])
)

conveyor_main_node.add_node_state(
    state_name="p23",
    exit_theta_condition=np.array([[rot3]]),
    const_shift_matrix=np.array([pb]).transpose(),
    const_rotation_axis_matrix = np.array([[1],[0],[0]]),
    var_rotation_angle_matrix= np.array([[-1]])
)

conveyor_main_node.add_node_state(
    state_name="p34",
    exit_theta_condition=np.array([[rot4]]),
    const_shift_matrix=np.array([pb]).transpose(),
    var_shift_matrix=np.array([[-p_diff[0]/L*r], [-p_diff[1]/L*r], [-p_diff[2]/L*r]]),
    const_rotation_axis_matrix = np.array([[1],[0],[0]]),
    const_rotation_angle_matrix = np.array([[np.pi]]),
)

conveyor_main_node.add_node_state(
    state_name="p41",
    exit_theta_condition=np.array([[rot0]]),
    const_shift_matrix=np.array([pa]).transpose(),
    const_rotation_axis_matrix = np.array([[1],[0],[0]]),
    const_rotation_angle_matrix = np.array([[np.pi]]),
    var_rotation_angle_matrix= np.array([[-1]])
)


palette= DTModel(
    "palette",
    model_path=r"./Data/obj/palette_scaled.stl",
    parent= conveyor_main_node,
    position = p1 - pa,
    rotation_axis = np.array([1,0,0]),
    rotation_angle = np.array([alpha]),
    collision_center=np.array([0.0, 0.0, 22.0]),
    collision_sides=(70.0, 3.0, 26.0),
    collision_rotation_axis=np.array([0.0, 0.0, 0.0]),
    collision_rotation_angle=0,
    solid=True
    )

for i in range(9):
    p_node = conveyor_main_node.copy(f"conveyor_node_{i:d}")
    p_node.delta_theta = np.array([[rot0/9 * i]])
    pal = palette.copy(f"palette_{i:d}")
    pal.color = white_color
    pal.parent = p_node
    p_node.save(constants.NODES_DIR)
    pal.save(constants.MODELS_DIR)


gate_main_node = DTNode("gate_node", sc_cylinder)

gate_main_node.add_node_state(
    state_name="state1",
    const_shift_matrix=np.array([[0],[-116.1],[250.5]]),
    const_rotation_axis_matrix=np.array([[1],[0],[0]]),
    var_rotation_angle_matrix=np.array([[1]])
)

gate_main_node.save(constants.NODES_DIR)

gate = DTModel(
    "gate",
    r"./Data/obj/gate_scaled.stl",
    gate_main_node,
    position = np.array((0.0, 116.1, -250.5)),
    collision_center=np.array([-5.0, -158.0, 299.0]),
    collision_sides=(60.0, 75.0, 50.0),
    collision_rotation_axis=np.array([1.0, 0.0, 0.0]),
    collision_rotation_angle=-np.pi/6,
    solid=True
)

gate.color = black_color

gate.save(constants.MODELS_DIR)

encoder_support = DTModel(
    "encoder_support",
    r"./Data/obj/encoder_support_scaled.stl"
)

encoder_support.color = black_color

encoder_support.save(constants.MODELS_DIR)

pe = np.array([[-87.0], [500.0], [795.0]])

enc_r = 27.0

encoder_main_node = DTNode("encoder_node", sc_motor)

encoder_main_node.add_node_state(
    "state1",
    const_shift_matrix=pe,
    const_rotation_axis_matrix=np.array([[1],[0],[0]]),
    var_rotation_angle_matrix=np.array([[-r/enc_r]]),
)

encoder_main_node.save(constants.NODES_DIR)


encoder = DTModel(
    "encoder",
    r"./Data/obj/encoder_relative.stl",
    encoder_main_node,
    rotation_axis = np.array([0,1,0]),
    rotation_angle = np.array([-np.pi/2])
)

encoder.color = orange_color

encoder.save(constants.MODELS_DIR)

panel_box = DTModel(
    "panel_box",
    r"./Data/obj/panel_box_scaled.stl"
)
panel_box.color = orange_color
panel_box.save(constants.MODELS_DIR)


mushroom = DTStatefulModel("mushroom", sc_panel)

mushroom.add_state(
    model_path=r"./Data/obj/mushroom_OFF_scaled.stl",
    color=(1.0,0.0,0.0, 1.0),
    collision_center=np.array([-160.0, 630.0, 494.0]),
    collision_radius=30.0
)

mushroom.add_state(
    model_path=r"./Data/obj/mushroom_ON_scaled.stl",
    color=(1.0,0.0,0.0, 1.0),
    collision_center=np.array([-150.0, 630.0, 494.0]),
    collision_radius=30.0
)

mushroom.save(constants.STATE_MODELS_DIR)

festo_switch = DTStatefulModel("festo_switch", sc_panel)

festo_switch.add_state(
    model_path=r"./Data/obj/switch_festo_scaled.stl",
    color=black_color,
    position=np.array([-147.5, 629.5, 445.0]),
    collision_center=np.array([0., 0., 0.]),
    collision_radius=30.0
)

festo_switch.add_state(
    model_path=r"./Data/obj/switch_festo_scaled.stl",
    color=black_color,
    position=np.array([-147.5, 629.5, 445.0]),
    rotation_axis=np.array([1,0,0]),
    rotation_angle=np.array([-np.pi/2]),
    collision_center=np.array([0., 0., 0.]),
    collision_radius=30.0
)

festo_switch.save(constants.STATE_MODELS_DIR)

pushbutton_red = DTStatefulModel("pushbutton_red", sc_panel)

pushbutton_red.add_state(
    model_path=r"./Data/obj/pushbutton_OFF_scaled.stl",
    color=(0.5, 0.0, 0.0, 1.0),
    collision_center=np.array([-150.0, 680.0, 444.0]),
    collision_radius=20.0
)

pushbutton_red.add_state(
    model_path=r"./Data/obj/pushbutton_ON_scaled.stl",
    color=(1.0, 0.0, 0.0, 1.0),
    collision_center=np.array([-150.0, 680.0, 444.0]),
    collision_radius=20.0
)

pushbutton_red.save(constants.STATE_MODELS_DIR)

pushbutton_green = DTStatefulModel("pushbutton_green", sc_panel)

pushbutton_green.add_state(
    model_path=r"./Data/obj/pushbutton_OFF_scaled.stl",
    color=(0.0, 0.5, 0.0, 1.0),
    position=np.array([0.0, 34.0, 0.0]),
    collision_center=np.array([-150.0, 680.0, 444.0]),
    collision_radius=20.0
)

pushbutton_green.add_state(
    model_path=r"./Data/obj/pushbutton_ON_scaled.stl",
    color=(0.0, 1.0, 0.0, 1.0),
    position=np.array([0.0, 34.0, 0.0]),
    collision_center=np.array([-150.0, 680.0, 444.0]),
    collision_radius=20.0
)

pushbutton_green.save(constants.STATE_MODELS_DIR)

light_panel_red = DTStatefulModel("lightpanel_red", sc_panel)

light_panel_red.add_state(
    model_path=r"./Data/obj/panel_light_scaled.stl",
    color=(0.1, 0.0, 0.0, 1.0)
)

light_panel_red.add_state(
    model_path=r"./Data/obj/panel_light_scaled.stl",
    color=(1.0, 0.0, 0.0, 1.0)
)

light_panel_red.save(constants.STATE_MODELS_DIR)

light_panel_green = DTStatefulModel("lightpanel_green", sc_panel)

light_panel_green.add_state(
    model_path=r"./Data/obj/panel_light_scaled.stl",
    position=np.array([0.0, 34.0, 0.0]),
    color=(0.0, 0.1, 0.0, 1.0)
)

light_panel_green.add_state(
    model_path=r"./Data/obj/panel_light_scaled.stl",
    position=np.array([0.0, 34.0, 0.0]),
    color=(0.0, 1.0, 0.0, 1.0)
)

light_panel_green.save(constants.STATE_MODELS_DIR)


box = DTActor("box",
              r"./Data/obj/box.stl",
              color=(0.0, 0.0, 1.0, 1.0),
              collision_center=np.array([0.0, 0.0, 0.0]),
              collision_sides=(50.0, 50.0, 50.0)
              )

box.save(constants.ACTOR_DIR)


prox_plate_support = DTModel(
    "prox_plate_support",
    r"./Data/obj/prox_plate_support_scaled.stl",
    color=white_color
)

prox_plate_support.save(constants.MODELS_DIR)

prox_support = DTModel(
    "prox_support",
    r"./Data/obj/prox_support_scaled.stl",
    color=orange_color
)

prox_support.save(constants.MODELS_DIR)

prox1 = DTSensor("prox1", sc_motor)

prox1.add_state(
    model_path=r"./Data/obj/prox_switch.stl",
    position=np.array([88.0, 609.5, 915.5]),
    rotation_axis = np.array([1.0, 0.0, 0.0]),
    rotation_angle=  np.array([-np.pi / 4]),
    collision_center=np.array([-94.0, 15.0, 0.0]),
    collision_sides=(75.0, 10.0, 10.0)
)

prox1.add_state(
    model_path=r"./Data/obj/prox_switch.stl",
    color=(1.0, 0.0, 0.0, 1.0),
    position=np.array([88.0, 609.5, 915.5]),
    rotation_axis = np.array([1.0, 0.0, 0.0]),
    rotation_angle= np.array([-np.pi / 4]),
    collision_center=np.array([-94.0, 15.0, 0.0]),
    collision_sides=(75.0, 10.0, 10.0)
)

prox1.save(constants.SENSORS_DIR)

prox2_rot1_axis = (1, 0, 0)
prox2_rot1_angle = 150 / 180 * np.pi

prox2_q1 = LQuaternion()

prox2_q1.set_from_axis_angle_rad(prox2_rot1_angle, LVector3f(*prox2_rot1_axis))


prox2_rot2_axis = (0, 0, 1)
prox2_rot2_angle = 210 / 180 * np.pi

prox2_q2 = LQuaternion(  )

prox2_q2.set_from_axis_angle_rad(prox2_rot2_angle, LVector3f(*prox2_rot2_axis))

prox2_q = prox2_q2 * prox2_q1
prox2_q.normalize()


prox2 = DTSensor( "prox2", sc_motor)

prox2.add_state(
    model_path=r"./Data/obj/prox_switch.stl",
    position=np.array([-114.0, -77.0, 328.0]),
    rotation_axis = np.array(prox2_q.get_axis_normalized()),
    rotation_angle=  np.array([prox2_q.get_angle()/180*np.pi]),
    collision_center=np.array([-94.0, 15.0, 0.0]),
    collision_sides=(75.0, 10.0, 10.0)
)

prox2.add_state(
    model_path=r"./Data/obj/prox_switch.stl",
    color=(1.0, 0.0, 0.0, 1.0),
    position=np.array([-114.0, -77.0, 328.0]),
    rotation_axis = np.array(prox2_q.get_axis_normalized()),
    rotation_angle=  np.array([prox2_q.get_angle()/180*np.pi]),
    collision_center=np.array([-94.0, 15.0, 0.0]),
    collision_sides=(75.0, 10.0, 10.0)
)

prox2.save(constants.SENSORS_DIR)


piston_axis = DTModel(
    "piston_axis",
    r"./Data/obj/piston_axis_scaled.stl",
    color=(decode_sRGB_float(0x4b), decode_sRGB_float(0x73), decode_sRGB_float(0x9f), 1.0)
)

piston_axis.save(constants.MODELS_DIR)

piston_ext_node = DTNode("piston_ext_node", sc_cylinder)

piston_ext_node.add_node_state(
    "state1",
    const_shift_matrix=np.array([[0.0],[-313.3],[309.5]]),
    const_rotation_axis_matrix=np.array([[1],[0],[0]]),
    var_rotation_angle_matrix=np.array([[1]]),
)

piston_ext_node.save(constants.NODES_DIR)

Pd = np.array([0, -313.3, 309.5])
Pe = np.array([0, -150.4, 192.4])
Pv = Pd - Pe
s_rot = np.atan2(-Pv[1],Pv[2])


piston_ext_model = DTModel(
    name="piston_ext",
    parent=piston_ext_node,
    model_path=r"./Data/obj/piston_ext_scaled.stl",
    rotation_axis=np.array([1.0, 0.0, 0.0]),
    rotation_angle= np.array([s_rot])
)

piston_ext_model.save(constants.MODELS_DIR)


piston_in_node = DTNode(
    "piston_in_node",
    sc_cylinder,
    parent=gate_main_node
)


piston_in_node.add_node_state(
    "state1",
    const_shift_matrix=np.array([  [0.] , [-34.3], [-58.1]]),
    const_rotation_axis_matrix=np.array([[1],[0],[0]]),
    var_rotation_angle_matrix=np.array([[1]])
)

piston_in_node.save(constants.NODES_DIR)


piston_in_model = DTModel(
    name="piston_in_model",
    parent=piston_in_node,
    model_path=r"./Data/obj/piston_int_scaled.stl"
)

piston_in_model.save(constants.MODELS_DIR)

plunger_box = DTModel(
    name = "plunger_box",
    color=(1.0, 0.0, 0.0, 1.0),
    parent=piston_in_node,
    model_path=r"./Data/obj/plunger_box.stl",
    position=np.array([-6, -99.3594, 70.0106]),
    rotation_axis=np.array([1.0, 0.0, 0.0]),
    rotation_angle= np.array([54.2897 / 180 * np.pi]),
    collision_center=np.array([0.0, 0.0, 0.0]),
    # collision_sides=(2.5, 2.5, 5.0),
    collision_sides=(25, 25, 5.0),
    detectable=True
)

plunger_box.save(constants.MODELS_DIR)

piston_sens1 = DTSensor(
    name = "piston_sens1",
    source_dev=sc_cylinder,
    parent=piston_ext_model
)

piston_sens1.add_state(
    model_path=r"./Data/obj/piston_sens.stl",
    color=None,
    position=np.array([-17.5, 0, -30]),
    collision_center=np.array([11.5, 0.0, 0.0]),
    collision_sides=(9, 9, 12.5)
)

piston_sens1.add_state(
    model_path=r"./Data/obj/piston_sens.stl",
    color=(1.0, 0.0, 0.0, 1.0),
    position=np.array([-17.5, 0, -30]),
    collision_center=np.array([11.5, 0.0, 0.0]),
    collision_sides=(9, 9, 12.5)
)

piston_sens1.save(constants.SENSORS_DIR)

piston_sens2 = DTSensor(
    name = "piston_sens2",
    source_dev=sc_cylinder,
    parent=piston_ext_model
)

piston_sens2.add_state(
    model_path=r"./Data/obj/piston_sens.stl",
    color=None,
    position=np.array([-17.5, 0, -84]),
    collision_center=np.array([11.5, 0.0, 0.0]),
    collision_sides=(9, 9, 12.5)
)

piston_sens2.add_state(
    model_path=r"./Data/obj/piston_sens.stl",
    color=(1.0, 0.0, 0.0, 1.0),
    position=np.array([-17.5, 0, -84]),
    collision_center=np.array([11.5, 0.0, 0.0]),
    collision_sides=(9, 9, 12.5)
)

piston_sens2.save(constants.SENSORS_DIR)

motor_unpowered = DTCheckNode(
    name = "motor_unpowered",
    source_dev=sc_motor,
    display_name="Motor stuck"
)

motor_unpowered.save(constants.QT_NODES_DIR)