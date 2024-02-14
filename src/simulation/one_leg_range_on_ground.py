'''
What this file does:
- Visualize the Megabot inside the Placo simulation
- 3 of the 4 legs are fixed to the ground (the effectors can't move)
- Creates targets in the interval [[-0.65, -0.65, 0.0], [0.66, 0.66, 0]] with a 0.01 step
- Tells the leg 1 (the one not fixed) to reach the current target
- See if the leg can/has reach it and save the result (Boolean) and the target position in an array
- Move to the next target
- Once all the range has been scanned, a figure is created to visualize the action range of the leg on the ground

Purpose: Visualize the range of action of one leg on the ground with the base at initial height (urdf height).

What the figure contains:
- In blue: the targets reached by the leg
- In red: the targets not reached by the leg
- In green: the starting position of the leg effector
- In yellow: the barycenter of the reachable targets surface
- In black: a convex approximation of the reacheable surface

Developer:
- The surface depends on the robot base height.
  Edit the robot base height (lower) to have a larger surface.
- The tested leg can be changed with the leg_moving variable.

/!\ The leg 1 is the one in the +x, -y frame
    The visualization has the world as a frame of reference,
    but this code also has the targets in the leg effector frame of reference (action_surface_leg). 
'''


import time
import numpy as np
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz, arrow_viz, point_viz, robot_frame_viz, frame_viz, contacts_viz
import megabot
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import global_functions as gf


##### Constants #####

# length of the cylindes
cylinder_length = 0.2 # m

# offset of the base on the z axis to have the legs on the ground
z_base_offset = 0.535 # m

# time between each target creation
refresh_target_time = 0.1

# translation constants
x_y_translation = 0.0 # m
dx_dy_translation = 0.01 # m

# max errors to consider the target reached
z_err_max = 0.01 # m
x_and_y_err_max = 0.05 # m

# flag for reach/not reached target
target_is_reached_flag = True
# flag for the switch of target
target_has_changed = False

# leg to test
leg_moving = 1
# leg_moving = 2
# leg_moving = 3
# leg_moving = 4

# all scanned targets with according reach/not reached flag
action_surface_leg = []
action_surface_world = []

# Constants for grid scanning
grid_x_min, grid_x_max = -0.66, 0.66  # Range for x
grid_y_min, grid_y_max = -0.66, 0.66  # Range for y
grid_step_size = 0.02  # Step size for scanning
# grid_step_size = 0.06  # Quick test

# Initialize x and y for grid scanning
x = grid_x_min
y = grid_y_min

# velocity limits
vlim_c2 = 0.1
vlim_c1_c3 = 0.3

# time constants
t: float = 0.0
dt: float = 0.01
start_time = time.time()
last_sample_t = 0.0


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

# Set the leg leg_moving to soft
leg_name = f"leg_{leg_moving}"
legs_tasks[leg_moving - 1].configure(leg_name, "soft", 1.0)


##### Target to reach #####

def create_target(x, y):
    x += init_pos_legs[leg_moving - 1][0]
    y += init_pos_legs[leg_moving - 1][1]
    z = init_pos_legs[leg_moving - 1][2] # Keep z constant as before
    
    target = np.array([x, y, z])
    return target

target = np.array([init_pos_legs[leg_moving - 1][0], init_pos_legs[leg_moving - 1][1], init_pos_legs[leg_moving - 1][2]])

def target_is_reached(target, leg_moving):
    leg_name = f"leg_{leg_moving}"
    T_world_leg = robot.get_T_world_frame(leg_name)
    pos_leg = T_world_leg[:3, 3]
    
    x_is_valid = abs(target[0] - pos_leg[0]) <= x_and_y_err_max
    y_is_valid = abs(target[1] - pos_leg[1]) <= x_and_y_err_max
    z_is_valid = abs(target[2] - pos_leg[2]) <= z_err_max
    
    pos_is_valid = (x_is_valid and y_is_valid and z_is_valid)
    
    return pos_is_valid


##### Init viewer #####

viz = robot_viz(robot)


##### Display action surface #####

def barycentre_calculus(points):
    if len(points) == 0:
        return None  # Renvoie None si la liste de points est vide
    
    sum_x = sum(point[0] for point in points)
    sum_y = sum(point[1] for point in points)
    nb_points = len(points)
    
    return (sum_x / nb_points, sum_y / nb_points)


def display_action_surface(action_surface, title):
    points = np.array([point[0] for point in action_surface if point[1]])
    points_2d = points[:, :2]
    hull_can_be_drawed = True
    try:
        hull = ConvexHull(points_2d)
    except:
        print("There is not enought reached ponts to draw the convex approximation.")
        hull_can_be_drawed = False

    x_true = points[:, 0]
    y_true = points[:, 1]
    x_false = [point[0][0] for point in action_surface if not point[1]]
    y_false = [point[0][1] for point in action_surface if not point[1]]
    
    # Starting point
    x_first, y_first = action_surface[0][0][0], action_surface[0][0][1]
    
    # Barycenter
    true_points = np.column_stack((x_true, y_true))
    barycenter = barycentre_calculus(true_points)
    
    # Add info to title
    title +=  "\nstarting point = " + str(x_first) + "," + str(y_first)
    title +=  "\nbarycenter = " + str(barycenter[0]) + "," + str(barycenter[1])

    plt.figure(figsize=(8, 8))
    plt.scatter(x_true, y_true, color='blue', label='Reachable')
    plt.scatter(x_false, y_false, color='red', label='Unreachable')  
    if hull_can_be_drawed:  
        plt.plot(points[hull.vertices,0], points[hull.vertices,1], 'k', lw=2, label='Convex Hull')
    plt.scatter(x_first, y_first, color='green', label='Starting position')
    plt.scatter(barycenter[0], barycenter[1], color='yellow', label='Barycenter')
    
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.title(title)
    plt.legend()
    plt.grid(True)
    plt.show()


##### Loop #####

while x <= grid_x_max:
    while y <= grid_y_max:
        # Update time
        t += dt
        
        # Updating target every refresh_target_time seconds
        if last_sample_t + refresh_target_time < t:
            last_sample_t = t
            target_has_changed = True
            
            target_is_reached_flag = target_is_reached(target, leg_moving)
            print(f"Target: x={x}, y={y}, Reached: {target_is_reached_flag}")
            
            target_in_leg = [target[0] - init_pos_legs[leg_moving - 1][0], target[1] - init_pos_legs[leg_moving - 1][1], target[2] - init_pos_legs[leg_moving - 1][2]]
            action_surface_leg.append((target_in_leg, target_is_reached_flag))
            action_surface_world.append((target, target_is_reached_flag))
            
            # Sample new target based on current x, y
            target = create_target(x, y)

        # Display target
        point_viz("target", target, color=0x00FF00)
        legs_tasks[leg_moving - 1].target_world = target

        # Update visualization
        robot_frame_viz(robot, "base")
        frame_viz("base_target", base_task.T_world_frame, 0.5)
        
        # Update the kinematics and solve the IK
        robot.update_kinematics()
        static_solver.solve(True)

        # Display the robot state
        viz.display(robot.state.q)

        # Increment y
        if target_has_changed:
            target_has_changed = False
            y += grid_step_size

    # Once y loop is complete, reset y and increment x
    y = grid_y_min
    x += grid_step_size

# Add the initial position of the robot leg
action_surface_leg.insert(0, (np.array([0.0, 0.0, init_pos_legs[leg_moving - 1][2]]), True))
action_surface_world.insert(0, (np.array([init_pos_legs[leg_moving - 1][0], init_pos_legs[leg_moving - 1][1], init_pos_legs[leg_moving - 1][2]]), True))

# Print the array of targets
print(action_surface_world)

# Display the figure
display_action_surface(action_surface_world, f"Surface d'action de la patte {leg_moving} dans le repère monde avec la base à hauteur initiale (urdf ref).\nx_err = {x_and_y_err_max}m, y_err = {x_and_y_err_max}m, z_err = {z_err_max}m")
