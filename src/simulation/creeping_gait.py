'''
What this file does:
- Visualize the Megabot inside the Placo simulation
- Move one leg after the other following the creeping gait
- Move base after the 4 legs
- Loop

Purpose: Move the Megabot in one of the cardinal directions in the simulation, following the creeping gait.

Developer:
- This script is based on the one_leg_creeping_gait script.
  It is advisable to look at this script if you wish to modify this one.
'''


import time
import numpy as np
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz, point_viz, robot_frame_viz, frame_viz
import megabot
import global_functions as gf


##### Constants #####

# length of the cylindes
cylinder_length = 0.2 # m

# offset of the base on the z axis to have the legs on the ground
z_base_offset = 0.535 # m

# time between each target creation
refresh_target_time = 0.5

# distances between positions of the creepings gait
leg_x_or_y_move_dst = 0.1
leg_z_move_dst = 0.1
base_translation = 0
# creeping gait cts
leg_nb_possible_pos = 6
creeping_gait_pos = 0
current_leg_moving = 4
is_changing_leg = False
nb_moved_legs_in_this_cycle = 0
nb_cycles = 0
is_init_stage = True
need_to_calculate_interpolation_steps = True

# Init the leg at the barycenter of the action surface of the leg (calculated with the one_leg_range_on_ground script)
x_barycenter_leg_1 = 0.890                  # 0.8905177687943348
y_barycenter_leg_1 = -0.984                 # -0.9844437782269537
x_barycenter_leg_2 = -x_barycenter_leg_1    # -1.0142640970103076
y_barycenter_leg_2 = y_barycenter_leg_1     # -0.9482419773195867
x_barycenter_leg_3 = -x_barycenter_leg_1    # -0.9407225838565039
y_barycenter_leg_3 = -y_barycenter_leg_1    # -1.0381144044394623
x_barycenter_leg_4 = x_barycenter_leg_1     # 0.9735852564573991
y_barycenter_leg_4 = -y_barycenter_leg_1    # 1.008839175784754
barycenters_legs = [np.array([x_barycenter_leg_1, y_barycenter_leg_1]), np.array([x_barycenter_leg_2, y_barycenter_leg_2]), np.array([x_barycenter_leg_3, y_barycenter_leg_3]), np.array([x_barycenter_leg_4, y_barycenter_leg_4])]

# interpolation target cts
interpolation_steps = 0
current_interpolation_step = 0

# max errors to consider target reached
z_err_max_ground = 0.01 # m
x_and_y_err_max_ground = 0.05 # m
z_err_max_elevated = 0.05 # m
x_and_y_err_max_elevated = 0.07 # m
# target reached cts
target_is_reached_flag = False

# velocity limits
vlim_c2 = 0.1
vlim_c1_c3 = 0.3

# Moving direction +x, -x, +y, -y
moving_direction = "+x"
# moving_direction = "-x"
# moving_direction = "+y"
# moving_direction = "-y"

# Define a mapping of current leg to next leg
def set_legs_mapping(moving_direction):
    match moving_direction:
        case "+x":
            legs_mapping = {
                1: 3,
                2: 1,
                3: 4,
                4: 2
            }
        case "-x":
            legs_mapping = {
                1: 2,
                2: 4,
                3: 1,
                4: 3
            }
        case "+y":
            legs_mapping = {
                1: 4,
                2: 3,
                3: 1,
                4: 2
            }
        case "-y":
            legs_mapping = {
                1: 3,
                2: 4,
                3: 2,
                4: 1
            }
    return legs_mapping
legs_mapping = set_legs_mapping(moving_direction)


##### Init robot and solver #####

# Creation
robot = megabot.load()
static_solver = megabot.make_solver(robot)

# Set joints and velocity limits for the all robot
robot, static_solver = gf.set_joint_limits_all_robot(robot, static_solver, vlim_c2, vlim_c1_c3)

# Init T_world_base
T_world_base = np.eye(4)
T_world_base[2, 3] = z_base_offset # To have the legs on the floor

# Initialization
robot = gf.init_robot_in_world(robot, T_world_base)

## Init base task ##
base_task = gf.set_base_task_soft_static_solver(static_solver, T_world_base)

# Base copies
base_start_pos = T_world_base.copy()
base_target_pos = T_world_base.copy()
base_current_pos = base_start_pos.copy()
base_task.T_world_frame = base_current_pos

## Init legs tasks & init pos ##
init_pos_legs = []
pos_legs = []
legs_tasks = []
legs_tasks, init_pos_legs, pos_legs = gf.set_legs_tasks_hard_static_solver_with_positions(robot, static_solver, legs_tasks, init_pos_legs, pos_legs)

def set_current_leg_moving_from_direction(moving_direction):
    match moving_direction:
        case "+x":
            current_leg_moving = 4
        case "-x":
            current_leg_moving = 2
        case "+y":
            current_leg_moving = 3
        case "-y":
            current_leg_moving = 1
    return current_leg_moving

current_leg_moving = set_current_leg_moving_from_direction(moving_direction)

tasks_types = ["hard", "hard", "hard", "hard"]
tasks_types[current_leg_moving - 1] = "soft"
legs_tasks = gf.configure_four_legs_tasks(legs_tasks, tasks_types)


##### Sustentation polygon #####

def update_polygon_from_current_leg_moving(polygon, current_leg_moving):
    if current_leg_moving == 1:
        polygon = np.array([
            [pos_legs[1][0] - 0.01, pos_legs[1][1] - 0.01], # l2
            [pos_legs[2][0] - 0.01, pos_legs[2][1] + 0.01], # l3
            [pos_legs[3][0] + 0.01, pos_legs[3][1] + 0.01]  # l4
        ])
    elif current_leg_moving == 2:
        polygon = np.array([
            [pos_legs[2][0] - 0.01, pos_legs[2][1] + 0.01], # l3
            [pos_legs[3][0] + 0.01, pos_legs[3][1] + 0.01], # l4
            [pos_legs[0][0] + 0.01, pos_legs[0][1] - 0.01]  # l1
        ])
    elif current_leg_moving == 3:
        polygon = np.array([
            [pos_legs[3][0] + 0.01, pos_legs[3][1] + 0.01], # l4
            [pos_legs[0][0] + 0.01, pos_legs[0][1] - 0.01], # l1
            [pos_legs[1][0] - 0.01, pos_legs[1][1] - 0.01]  # l2
        ])
    elif current_leg_moving == 4:
        polygon = np.array([
            [pos_legs[0][0] + 0.01, pos_legs[0][1] - 0.01], # l1
            [pos_legs[1][0] - 0.01, pos_legs[1][1] - 0.01], # l2
            [pos_legs[2][0] - 0.01, pos_legs[2][1] + 0.01]  # l3
        ])
    return polygon

def update_base_task_from_polygon(polygon):
    polygon_barycenter_x = np.mean(polygon[:, 0])
    polygon_barycenter_y = np.mean(polygon[:, 1])
    T_world_base[0, 3] = polygon_barycenter_x
    T_world_base[1, 3] = polygon_barycenter_y
    
    return T_world_base

# Create polygon constraint for the com at the polygon barycenter
polygon = np.array([
    [init_pos_legs[0][0] + 0.001, init_pos_legs[0][1] - 0.001],
    [init_pos_legs[1][0] - 0.001, init_pos_legs[1][1] - 0.001],
    [init_pos_legs[2][0] - 0.001, init_pos_legs[2][1] + 0.001]
])

polygon = update_polygon_from_current_leg_moving(polygon, current_leg_moving)
previous_T_world_frame = base_task.T_world_frame
base_task.T_world_frame = update_base_task_from_polygon(polygon)


##### Target to reach #####

def create_target(leg_number, creeping_gait_pos, moving_direction):
    if is_init_stage:
        x = barycenters_legs[leg_number - 1][0]
        y = barycenters_legs[leg_number - 1][1]
        z = init_pos_legs[leg_number - 1][2]
    else:
        x = pos_legs[leg_number - 1][0]
        y = pos_legs[leg_number - 1][1]
        z = init_pos_legs[leg_number - 1][2]
    
        dx_or_dy = 0
        multi_factor = 1
        if moving_direction == "-x" or moving_direction == "-y":   
            multi_factor = -1 
        
        if creeping_gait_pos == 1:
            z += leg_z_move_dst
        elif creeping_gait_pos == 2 or creeping_gait_pos == 3:
            z += leg_z_move_dst
            dx_or_dy += leg_x_or_y_move_dst
        elif creeping_gait_pos == 4:
            dx_or_dy += leg_x_or_y_move_dst
        elif creeping_gait_pos == 5:
            dx_or_dy += -leg_x_or_y_move_dst
            
        if moving_direction == "+x" or moving_direction == "-x":   
            x += dx_or_dy * multi_factor
        elif moving_direction == "+y" or moving_direction == "-y":   
            y += dx_or_dy * multi_factor
    
    t = np.array([x, y, z])
    return t

target = create_target(current_leg_moving, creeping_gait_pos, moving_direction)
current_interpolation_target = init_pos_legs[current_leg_moving - 1]

def calculate_distance(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)

def linear_interpolate(start_pos, end_pos, alpha):
    return (1 - alpha) * start_pos + alpha * end_pos

def target_is_reached(target, leg, creeping_gait_pos):
    T_world_leg = robot.get_T_world_frame("leg_" + str(leg))
    pos_leg = T_world_leg[:3, 3]
    
    x_and_y_err_max = x_and_y_err_max_elevated
    z_err_max = z_err_max_elevated
    
    if (creeping_gait_pos == 0) or (creeping_gait_pos == 4):
        x_and_y_err_max = x_and_y_err_max_ground
        z_err_max = z_err_max_ground
    
    x_is_valid = abs(target[0] - pos_leg[0]) <= x_and_y_err_max
    y_is_valid = abs(target[1] - pos_leg[1]) <= x_and_y_err_max
    z_is_valid = abs(target[2] - pos_leg[2]) <= z_err_max

    pos_is_valid = (x_is_valid and y_is_valid and z_is_valid)
    
    return pos_is_valid


##### Init viewer #####

viz = robot_viz(robot)


##### Loop parts #####

def update_all_vizualisation_and_solve():
     # Display target
    point_viz("target", current_interpolation_target, color=0x00FF00)

    # Display action surface and support surface
    dx = 0
    dy = 0
    if moving_direction == '+x':
        dx = leg_x_or_y_move_dst * 3 * nb_cycles
    elif moving_direction == '-x':
        dx = - leg_x_or_y_move_dst * 3 * nb_cycles
    elif moving_direction == '+y':
        dy = leg_x_or_y_move_dst * 3 * nb_cycles
    elif moving_direction == '-y':
        dy = - leg_x_or_y_move_dst * 3 * nb_cycles
    gf.display_info(pos_legs, current_leg_moving, gf.points, gf.barycentre, dx, dy)

    # Showing the center of mass (on the ground)
    com_world = robot.com_world()
    com_world[2] = 0.0
    point_viz("com", com_world, color=0xFF0000)

    # Update vizualisation of the base
    robot_frame_viz(robot, "base")
    frame_viz("base_target", base_task.T_world_frame, 0.5)
    
    # Update the kinematics and solve the IK
    robot.update_kinematics()
    static_solver.solve(True)
    
    # Display the robot state
    viz.display(robot.state.q)

def move_base_com_to_barycenter(previous_T_world_frame, current_T_world_frame):
    counter = 0
    print("Base moving.")
    current_pos = previous_T_world_frame[:2, 3]
    current_target = current_T_world_frame[:2, 3]
    steps = calculate_interpolation_steps_from_target_and_current(current_pos, current_target)
    while counter < steps:
        legs_tasks[current_leg_moving - 1].target_world = current_interpolation_target # Update the current leg task
        
        update_all_vizualisation_and_solve()
        
        counter += 1

def calculate_interpolation_steps_from_target_and_current(current_interpolation_target, target):
    # Calculating the distance between the current position and the new target
    distance = calculate_distance(current_interpolation_target, target)
    
    # Setting the number of interpolation points based on distance
    interpolation_steps = max(int(distance / (min(vlim_c2, vlim_c1_c3) * static_solver.dt)), 1)
    
    return interpolation_steps


##### Loop #####

time.sleep(1) # to refresh the page

print(f"\nStart init stage....")

move_base_com_to_barycenter(previous_T_world_frame, base_task.T_world_frame)

while nb_moved_legs_in_this_cycle < 4:
    if need_to_calculate_interpolation_steps:
        print(f"Leg {current_leg_moving} moving to its action surface barycenter.")
        need_to_calculate_interpolation_steps = False
        current_interpolation_step = 0
        target = create_target(current_leg_moving, creeping_gait_pos, moving_direction)
        interpolation_steps = calculate_interpolation_steps_from_target_and_current(current_interpolation_target, target)

    if current_interpolation_step < interpolation_steps:
        alpha = current_interpolation_step / interpolation_steps
        current_interpolation_target = linear_interpolate(current_interpolation_target, target, alpha)
        current_interpolation_step += 1
    else:
        current_interpolation_target = target
    
    # Update the current leg task
    legs_tasks[current_leg_moving - 1].target_world = current_interpolation_target
    
    # Update the legs positions
    pos_legs[current_leg_moving - 1][0] = current_interpolation_target[0]
    pos_legs[current_leg_moving - 1][1] = current_interpolation_target[1]
    
    update_all_vizualisation_and_solve()
    
    # gf.display_all_joint(robot, cylinder_length)
    
    if current_interpolation_step >= interpolation_steps:
        nb_moved_legs_in_this_cycle += 1
        
        need_to_calculate_interpolation_steps = True
        creeping_gait_pos = 0
        current_interpolation_step = 0
        
        tasks_types = ["hard", "hard", "hard", "hard"]
        current_leg_moving = legs_mapping[current_leg_moving]
        legs_tasks = gf.configure_four_legs_tasks(legs_tasks, tasks_types) # Update the four legs tasks (hard or soft)

        current_interpolation_target = init_pos_legs[current_leg_moving - 1] # Update current_interpolation_target to the position of the current leg
        
        polygon = update_polygon_from_current_leg_moving(polygon, current_leg_moving)
        previous_T_world_frame = base_task.T_world_frame.copy()
        base_task.T_world_frame = update_base_task_from_polygon(polygon)
        
        move_base_com_to_barycenter(previous_T_world_frame, base_task.T_world_frame)

        tasks_types[current_leg_moving - 1] = "soft" # Edit the configuration of the next leg to move to be able to change its position
        legs_tasks = gf.configure_four_legs_tasks(legs_tasks, tasks_types) # Update the four legs tasks (hard or soft)
        

print(f"End init stage.\n\nStart creeping gait loop.")

is_init_stage = False
creeping_gait_pos = 1 # To not loose time at the current position

while True:
    if need_to_calculate_interpolation_steps:
        print(f"Leg {current_leg_moving} moving to creeping gait position {creeping_gait_pos}")
        need_to_calculate_interpolation_steps = False
        current_interpolation_step = 0
        target = create_target(current_leg_moving, creeping_gait_pos, moving_direction)
        interpolation_steps = calculate_interpolation_steps_from_target_and_current(current_interpolation_target, target)
 
    if current_interpolation_step < interpolation_steps:
        alpha = current_interpolation_step / interpolation_steps
        current_interpolation_target = linear_interpolate(current_interpolation_target, target, alpha)
        current_interpolation_step += 1
    else:
        current_interpolation_target = target
    
    # Update the current leg task
    legs_tasks[current_leg_moving - 1].target_world = current_interpolation_target
    
    # Update the legs positions
    pos_legs[current_leg_moving - 1][0] = current_interpolation_target[0]
    pos_legs[current_leg_moving - 1][1] = current_interpolation_target[1]
    
    update_all_vizualisation_and_solve()
        
    # gf.display_all_joint(robot, cylinder_length)
    
    if current_interpolation_step >= interpolation_steps:
        need_to_calculate_interpolation_steps = True
        creeping_gait_pos = (creeping_gait_pos + 1) % leg_nb_possible_pos
        
        if creeping_gait_pos == 5:
            current_leg_moving = legs_mapping[current_leg_moving]
            nb_moved_legs_in_this_cycle = (nb_moved_legs_in_this_cycle + 1) % 4
            
            if nb_moved_legs_in_this_cycle == 0:
                nb_cycles += 1
            
            creeping_gait_pos = 1
            current_interpolation_step = 0
            
            tasks_types = ["hard", "hard", "hard", "hard"]
            tasks_types[current_leg_moving - 1] = "soft" # Edit the configuration of the next leg to move to be able to change its position
            legs_tasks = gf.configure_four_legs_tasks(legs_tasks, tasks_types) # Update the four legs tasks (hard or soft)

            # Update current_interpolation_target to the position of the current leg
            current_interpolation_target = np.array([pos_legs[current_leg_moving - 1][0], pos_legs[current_leg_moving - 1][1], init_pos_legs[current_leg_moving - 1][2]])
            
            polygon = update_polygon_from_current_leg_moving(polygon, current_leg_moving)
            previous_T_world_frame = base_task.T_world_frame.copy()
            base_task.T_world_frame = update_base_task_from_polygon(polygon)
        
            move_base_com_to_barycenter(previous_T_world_frame, base_task.T_world_frame)
