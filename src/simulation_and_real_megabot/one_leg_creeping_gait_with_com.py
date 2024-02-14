'''
What this file does:
- Visualize the Megabot inside the Placo simulation
- 3 of the 4 legs are fixed to the ground (the effectors can't move)
- Create a target in the following list:
  ([x, y, z], [x, y, z + 0.05], [x + 0.03, y, z + 0.05], [x + 0.06, y, z + 0.05], [x + 0.06, y, z], [x + 0.03, y, z])
- Tells the leg 1 (the one not fixed) to reach the current target (with interpolation steps in between) in the simulation
- Send all the joints values (in the simulation) to the Megabot microcontroller
- Move to the next target (following the order)
- Loop on the 6 positions

Purpose: See the creeping gait movement for just one leg in loop.

Developer:
- The interpolation_factor can be change to have a quicker or slower and more or less fluid movement
- controller = com.ControlerHandler("one_leg_creeping_gait_with_com.log") 
  permits to generate the logs in the one_leg_creeping_gait_with_com.log file.
  It contains the position that the microcontroller recieved compared to the actual positions of the cylinders
  The log specification in more detail can be found in the com.py file
- The name of the log file (one_leg_creeping_gait_with_com.log) can be changed if you want

/!\ The leg 1 is the one in the +x, -y frame
    Look at the state of the art ~/docs/report for information on the creeping gait
'''


import time
import numpy as np
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz, point_viz, robot_frame_viz, frame_viz
import megabot
import global_functions as gf
import com


##### Constants #####

# length of the cylindes
cylinder_length = 0.2 # m

# offset of the base on the z axis to have the legs on the ground
z_base_offset = 0.535 # m

# distances between positions of the creepings gait
leg_x_move_dst = 0.1
leg_y_move_dst = 0.1
leg_z_move_dst = 0.1
# creeping gait cts
leg_nb_possible_pos = 6
creeping_gait_pos = 0
need_to_calculate_interpolation_steps = True

# leg to test
current_leg_moving = 1
# current_leg_moving = 2
# current_leg_moving = 3
# current_leg_moving = 4

# Init the leg at the barycenter of the action surface of the leg (calculated with the one_leg_range_on_ground script)
x_barycenter_leg_1 = 0.890 # 0.8905177687943348
y_barycenter_leg_1 = -0.984 # -0.9844437782269537
x_barycenter_leg_2 = -x_barycenter_leg_1
y_barycenter_leg_2 = y_barycenter_leg_1
x_barycenter_leg_3 = -x_barycenter_leg_1
y_barycenter_leg_3 = -y_barycenter_leg_1
x_barycenter_leg_4 = x_barycenter_leg_1
y_barycenter_leg_4 = -y_barycenter_leg_1
barycenters_legs = [np.array([x_barycenter_leg_1, y_barycenter_leg_1]), np.array([x_barycenter_leg_2, y_barycenter_leg_2]), np.array([x_barycenter_leg_3, y_barycenter_leg_3]), np.array([x_barycenter_leg_4, y_barycenter_leg_4])]

# interpolation target cts
interpolation_factor = 1000
interpolation_steps = 0
current_step = 0

# velocity limits
vlim_c2 = 0.1
vlim_c1_c3 = 0.3


##### Controller #####

controller = com.ControlerHandler("one_leg_creeping_gait_with_com.log")
controller.send_info(0.005)


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

## Init legs tasks & init pos ##
init_pos_legs = []
pos_legs = []
legs_tasks = []
legs_tasks, init_pos_legs, pos_legs = gf.set_legs_tasks_hard_static_solver_with_positions(robot, static_solver, legs_tasks, init_pos_legs, pos_legs)

tasks_types = ["hard", "hard", "hard", "hard"]
tasks_types[current_leg_moving - 1] = "soft"
legs_tasks = gf.configure_four_legs_tasks(legs_tasks, tasks_types)


##### Target to reach #####
        
def create_target(creeping_gait_pos):    
    x = barycenters_legs[current_leg_moving - 1][0]
    y = barycenters_legs[current_leg_moving - 1][1]
    z = init_pos_legs[current_leg_moving - 1][2]
    
    if (creeping_gait_pos == 1):
        z += leg_z_move_dst
    elif (creeping_gait_pos == 2):
        z += leg_z_move_dst
        x += leg_x_move_dst
    elif (creeping_gait_pos == 3):
        z += leg_z_move_dst
        x += leg_x_move_dst * 2
    elif (creeping_gait_pos == 4):
        x += leg_x_move_dst * 2
    elif (creeping_gait_pos == 5):
        x += leg_x_move_dst
    
    t = np.array([x, y, z])
    return t

target = create_target(creeping_gait_pos)
current_target = target.copy()

def calculate_distance(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)

def linear_interpolate(start_pos, end_pos, alpha):
    return (1 - alpha) * start_pos + alpha * end_pos


##### Init viewer #####

viz = robot_viz(robot)


##### Loop #####

try:
    while True:
        if need_to_calculate_interpolation_steps:
            need_to_calculate_interpolation_steps = False
            creeping_gait_pos = (creeping_gait_pos + 1) % leg_nb_possible_pos
            new_target = create_target(creeping_gait_pos)
            
            # Calculating the distance between the current position and the new target
            distance = calculate_distance(current_target, new_target)
            
            # Setting the number of interpolation points based on distance
            interpolation_steps = max(int(distance / (min(vlim_c2, vlim_c1_c3) * static_solver.dt)), 1)        
            current_step = 0
            target = new_target
            
            print(f"interpolation_steps {interpolation_steps}")
            
        if current_step < interpolation_steps:
            alpha = current_step / interpolation_steps
            current_target = linear_interpolate(current_target, target, alpha)
            current_step += 1
        else:
            current_target = target
            
        # Display target
        point_viz("target", current_target, color=0x00FF00)
        legs_tasks[current_leg_moving - 1].target_world = current_target

        # Display action surface and support surface
        gf.display_info(pos_legs, current_leg_moving, gf.points, gf.barycentre, 0, 0)

        # Update vizualisation
        robot_frame_viz(robot, "base")
        frame_viz("base_target", base_task.T_world_frame, 0.5)
        
        # Update the kinematics and solve the IK
        robot.update_kinematics()
        static_solver.solve(True)
            
        gf.display_all_joint(robot, controller, cylinder_length)

        # Display the robot state
        viz.display(robot.state.q)
        
        if current_step >= interpolation_steps:
            need_to_calculate_interpolation_steps = True
except:
    pass

controller.stop()
controller.join()

