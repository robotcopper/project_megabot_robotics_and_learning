'''
What this file does:
- Visualize the Megabot inside the Placo simulation
- 3 of the 4 legs are fixed to the ground (the effectors can't move)
- Create a random target in the interval [[1, -1.5, 0.0], [1.5, -1, 0]]
- Tells the leg 1 (the one not fixed) to reach the target
- Regenerate a new target every once in a while

Purpose: Visualize where one leg can go on the ground (x, y range).  

Developer:
- The create_target function can be modified to have a larger range (for the z axis for example).

/!\ The leg 1 is the one in the +x, -y frame 
'''


import time
import numpy as np
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz, arrow_viz, point_viz, robot_frame_viz, frame_viz, contacts_viz
import megabot
import global_functions as gf


##### Constants #####

# length of the cylindes
cylinder_length = 0.2 # m

# offset of the base on the z axis to have the legs on the ground
z_base_offset = 0.535 # m

# time between each target creation
refresh_target_time = 0.5

# time constants
t: float = 0.0
dt: float = 0.01
start_time = time.time()
last_sample_t = 0.0


##### Init robot and solver #####

# Creation
robot = megabot.load()
static_solver = megabot.make_solver(robot)

# Init T_world_base
T_world_base = np.eye(4)
T_world_base[2, 3] = z_base_offset # To have the legs on the floor

# Initialization
robot = gf.init_robot_in_world(robot, T_world_base)

## Init base task ##
base_task = gf.set_base_task_soft_static_solver(static_solver, T_world_base)

## Init legs tasks ##
legs_tasks = []
legs_tasks = gf.set_legs_tasks_hard_static_solver(robot, static_solver, legs_tasks)

# Set the leg 1 to soft
legs_tasks[0].configure("leg_1", "soft", 1.0)


##### Target to reach #####
        
def create_target():
    return np.random.uniform(np.array([1, -1.5, 0.0]), np.array([1.5, -1, 0]))

target = create_target()


##### Init viewer #####

viz = robot_viz(robot)


##### Loop #####

print(f"\ntarget = {target}")

while True:
    # Update time
    t += dt
    
    # Updating target every refresh_target_time seconds
    if last_sample_t + refresh_target_time < t:
        last_sample_t = t
        target = create_target()
        print(f"target = {target}")
        
    # Dispkay target
    point_viz("target", target, color=0x00FF00)
    legs_tasks[0].target_world = target

    # Update vizualisation
    robot_frame_viz(robot, "base")
    frame_viz("base_target", base_task.T_world_frame, 0.5)
    
    # Update the kinematics and solve the IK
    robot.update_kinematics()
    static_solver.solve(True)

    # gf.display_all_joint(robot, cylinder_length)

    # Display the robot state
    viz.display(robot.state.q)
