from digtwin.gui.models.dt_models import DTModel, DTStatefulModel
from digtwin.gui.models.dt_actors import DTActor
from digtwin.gui.nodes.dt_nodes import DTNode
from digtwin.utils import constants
import numpy as np
from panda3d.core import decode_sRGB_float

# models

white_color = (0.8, 0.8, 0.8, 1)
black_color = (0.05, 0.05, 0.05, 1)
orange_color = (decode_sRGB_float(0xff), decode_sRGB_float(0x99), decode_sRGB_float(0x00), 1)

main_conveyor = DTModel(
    "main_frame",
    r"./Data/obj/fixed_conveyor_scaled.stl")

main_conveyor.color = white_color

main_conveyor.save(constants.MODELS_DIR)

main_frame= DTModel(
    "main_conveyor",
    r"./Data/obj/Fixed_frame_lite_scaled.stl")

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

conveyor_main_node = DTNode("conveyor_node", theta_period=np.array([[rot0]]))

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
    rotation_angle = np.array([alpha])
    )

for i in range(9):
    p_node = conveyor_main_node.copy(f"conveyor_node_{i:d}")
    p_node.delta_theta = np.array([[rot0/9 * i]])
    pal = palette.copy(f"palette_{i:d}")
    pal.color = white_color
    pal.parent = p_node
    p_node.save(constants.NODES_DIR)
    pal.save(constants.MODELS_DIR)


gate_main_node = DTNode("gate_node")

gate_main_node.add_node_state(
    state_name="state1",
    const_shift_matrix=np.array([[0],[-116.1],[250.5]]),
    const_rotation_axis_matrix=np.array([[1],[0],[0]]),
    var_rotation_angle_matrix=np.array([[1]]),
)

gate_main_node.save(constants.NODES_DIR)

gate = DTModel(
    "gate",
    r"./Data/obj/gate_scaled.stl",
    gate_main_node,
    position = np.array((0.0, 116.1, -250.5))
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

encoder_main_node = DTNode("encoder_node")

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


mushroom = DTStatefulModel(
    "mushroom" )

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

festo_switch = DTStatefulModel(
    "festo_switch",
)

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

pushbutton_red = DTStatefulModel(
    "pushbutton_red"
)

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

pushbutton_green = DTStatefulModel(
    "pushbutton_green"
)

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

light_panel_red = DTStatefulModel(
    "lightpanel_red"
)

light_panel_red.add_state(
    model_path=r"./Data/obj/panel_light_scaled.stl",
    color=(0.1, 0.0, 0.0, 1.0)
)

light_panel_red.add_state(
    model_path=r"./Data/obj/panel_light_scaled.stl",
    color=(1.0, 0.0, 0.0, 1.0)
)

light_panel_red.save(constants.STATE_MODELS_DIR)

light_panel_green = DTStatefulModel(
    "lightpanel_green"
)

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

