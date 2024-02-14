import numpy as np
from placo_utils.visualization import point_viz, line_viz

##### Robot joints #####

def get_all_joint(robot):
    all_joint = []
    
    for leg in range(1, 5):
        joint_current_leg = []
        for cylinder in range(1, 4):
            joint_current_leg.append(robot.get_joint("l" + str(leg) + "_c" + str(cylinder)))
        all_joint.append(joint_current_leg)
        
    return all_joint

def display_all_joint(robot, controller, cylinder_length):
    all_joint = get_all_joint(robot)
    
    for leg in range(1, 5):
        for cylinder in range(1, 4):
            joint_value = all_joint[leg - 1][cylinder - 1]
            absolute_joint_value = joint_value + cylinder_length / 2
            msg = f"L{leg}{cylinder}#{absolute_joint_value:0.6f}##"
            # print(msg)
            controller.send_move(leg, cylinder, absolute_joint_value, 0.9)
            

##### Init robot #####

def init_robot_in_world(robot, T_world_base):
    # Initializing T_world_base to identity
    robot.set_T_world_frame("base", T_world_base)
    robot.update_kinematics()
    
    return robot

def set_base_task_soft_static_solver(solver, T_world_base):
    # Setting the position of the base in the world
    base_task = solver.add_frame_task("base", T_world_base)
    # base_task.configure("base", "soft", 1.0, 10.0)
    base_task.configure("base", "soft", 0.01, 10.0)
    
    return base_task

def set_legs_tasks_hard_static_solver(robot, solver, legs_tasks):
    for leg in range(1, 5):
        name = f"leg_{leg}"
        T_world_leg = robot.get_T_world_frame(name)
        leg_task = solver.add_position_task(name, T_world_leg[:3, 3])
        leg_task.configure(name, "hard", 1.0)
        legs_tasks.append(leg_task)
        
    return legs_tasks

def set_legs_tasks_hard_static_solver_with_positions(robot, solver, legs_tasks, init_pos_legs, pos_legs):
    for i in range(1, 5):
        name = f"leg_{i}"
        T_world_leg = robot.get_T_world_frame(name)
        init_pos_leg = T_world_leg[:3, 3]
        leg_task = solver.add_position_task(name, init_pos_leg)
        leg_task.configure(name, "hard", 1.0)

        # Append the initial position and task to their respective lists
        init_pos_legs.append(init_pos_leg)
        pos_legs.append(init_pos_leg)
        legs_tasks.append(leg_task)
        
    return legs_tasks, init_pos_legs, pos_legs

def init_dynamic_solver(dynamic_solver):
    dynamic_solver.set_static(True)
    dynamic_solver.enable_torque_limits(False)
    
    return dynamic_solver

def set_legs_tasks_hard_dynamic_solver(robot, dynamic_solver, legs_tasks):
    for leg in range(1, 5):
        name = f"leg_{leg}"
        T_world_leg = robot.get_T_world_frame(name)
        leg_task = dynamic_solver.add_position_task(name, T_world_leg[:3, 3])
        leg_task.configure(name, "hard", 1.0)
        contact = dynamic_solver.add_unilateral_point_contact(leg_task)
        contact.weight_forces = 1e3
        legs_tasks.append(leg_task)
        
    return legs_tasks
        
    

##### Tasks #####
            
def configure_four_legs_tasks(tasks, tasks_types):
    for index in range(len(tasks)):
        name = f"leg_{index}"
        tasks[index].configure(name, tasks_types[index], 1.0)
    return tasks

##### Limits #####
            
def set_joint_limits_all_robot(robot, solver, vlim_c2, vlim_c1_c3):
    solver.enable_velocity_limits(True)
    solver.dt = 0.01
    # solver.dt = 0.15 # Higher -> megabot goes quicker but less accurate
    for leg in range(1, 5):
        for c in range(1, 4):
            security_margin = 0.02 # [m]
            robot.set_joint_limits(f"l{leg}_c{c}", -0.1 + security_margin, 0.1 - security_margin)
            if c == 2:
                robot.set_velocity_limit(f"l{leg}_r{c}", vlim_c2)
            else:
                robot.set_velocity_limit(f"l{leg}_r{c}", vlim_c1_c3)
    
    return robot, solver


##### Action surface #####

points =  [[0.5444894, -1.02407853, 0.03206501], [0.5444894, -0.99407853, 0.03206501], [0.5544894, -1.06407853, 0.03206501], [0.5544894, -0.99407853, 0.03206501], [0.5644894, -1.10407853, 0.03206501], [0.5644894, -0.98407853, 0.03206501], [0.5644894, -0.92407853, 0.03206501], [0.5744894, -1.14407853, 0.03206501], [0.5744894, -0.98407853, 0.03206501], [0.5744894, -0.94407853, 0.03206501], [0.5744894, -0.93407853, 0.03206501], [0.5844894, -1.18407853, 0.03206501], [0.5844894, -0.93407853, 0.03206501], [0.5944894, -1.21407853, 0.03206501], [0.5944894, -0.93407853, 0.03206501], [0.6044894, -1.25407853, 0.03206501], [0.6044894, -0.93407853, 0.03206501], [0.6144894, -1.29407853, 0.03206501], [0.6144894, -0.93407853, 0.03206501], [0.6244894, -1.34407853, 0.03206501], [0.6244894, -0.93407853, 0.03206501], [0.6344894, -1.33407853, 0.03206501], [0.6344894, -0.93407853, 0.03206501], [0.6444894, -1.33407853, 0.03206501], [0.6444894, -0.92407853, 0.03206501], [0.6544894, -1.32407853, 0.03206501], [0.6544894, -0.92407853, 0.03206501], [0.6644894, -1.32407853, 0.03206501], [0.6644894, -0.91407853, 0.03206501], [0.6744894, -1.31407853, 0.03206501], [0.6744894, -0.91407853, 0.03206501], [0.6844894, -1.30407853, 0.03206501], [0.6844894, -0.90407853, 0.03206501], [0.6944894, -1.30407853, 0.03206501], [0.6944894, -0.89407853, 0.03206501], [0.7044894, -1.30407853, 0.03206501], [0.7044894, -0.89407853, 0.03206501], [0.7144894, -1.29407853, 0.03206501], [0.7144894, -0.89407853, 0.03206501], [0.7244894, -1.29407853, 0.03206501], [0.7244894, -0.88407853, 0.03206501], [0.7344894, -1.29407853, 0.03206501], [0.7344894, -0.87407853, 0.03206501], [0.7444894, -1.28407853, 0.03206501], [0.7444894, -0.86407853, 0.03206501], [0.7544894, -1.28407853, 0.03206501], [0.7544894, -0.85407853, 0.03206501], [0.7644894, -1.28407853, 0.03206501], [0.7644894, -0.85407853, 0.03206501], [0.7744894, -1.27407853, 0.03206501], [0.7744894, -0.84407853, 0.03206501], [0.7844894, -1.27407853, 0.03206501], [0.7844894, -0.83407853, 0.03206501], [0.7944894, -1.27407853, 0.03206501], [0.7944894, -0.81407853, 0.03206501], [0.8044894, -1.26407853, 0.03206501], [0.8044894, -0.80407853, 0.03206501], [0.8144894, -1.26407853, 0.03206501], [0.8144894, -0.79407853, 0.03206501], [0.8244894, -1.25407853, 0.03206501], [0.8244894, -0.77407853, 0.03206501], [0.8344894, -1.25407853, 0.03206501], [0.8344894, -0.76407853, 0.03206501], [0.8444894, -1.25407853, 0.03206501], [0.8444894, -0.75407853, 0.03206501], [0.8544894, -1.25407853, 0.03206501], [0.8544894, -0.74407853, 0.03206501], [0.8644894, -1.24407853, 0.03206501], [0.8644894, -0.72407853, 0.03206501], [0.8744894, -1.23407853, 0.03206501], [0.8744894, -0.72407853, 0.03206501], [0.8844894, -1.23407853, 0.03206501], [0.8844894, -0.72407853, 0.03206501], [0.8944894, -1.23407853, 0.03206501], [0.8944894, -0.70407853, 0.03206501], [0.9044894, -1.22407853, 0.03206501], [0.9044894, -0.69407853, 0.03206501], [0.9044894, -0.67407853, 0.03206501], [0.9044894, -0.66407853, 0.03206501], [0.9144894, -1.21407853, 0.03206501], [0.9144894, -0.66407853, 0.03206501], [0.9244894, -1.21407853, 0.03206501], [0.9244894, -0.66407853, 0.03206501], [0.9344894, -1.20407853, 0.03206501], [0.9344894, -0.66407853, 0.03206501], [0.9444894, -1.19407853, 0.03206501], [0.9444894, -0.66407853, 0.03206501], [0.9544894, -1.19407853, 0.03206501], [0.9544894, -0.66407853, 0.03206501], [0.9644894, -1.18407853, 0.03206501], [0.9644894, -0.67407853, 0.03206501], [0.9744894, -1.18407853, 0.03206501], [0.9744894, -0.66407853, 0.03206501], [0.9844894, -1.17407853, 0.03206501], [0.9844894, -0.66407853, 0.03206501], [0.9944894, -1.16407853, 0.03206501], [0.9944894, -0.66407853, 0.03206501], [1.0044894, -1.16407853, 0.03206501], [1.0044894, -0.67407853, 0.03206501], [1.0144894, -1.15407853, 0.03206501], [1.0144894, -0.67407853, 0.03206501], [1.0244894, -1.14407853, 0.03206501], [1.0244894, -0.67407853, 0.03206501], [1.0344894, -1.13407853, 0.03206501], [1.0344894, -0.68407853, 0.03206501], [1.0444894, -1.13407853, 0.03206501], [1.0444894, -0.68407853, 0.03206501], [1.0544894, -1.11407853, 0.03206501], [1.0544894, -0.69407853, 0.03206501], [1.0644894, -1.11407853, 0.03206501], [1.0644894, -0.69407853, 0.03206501], [1.0744894, -1.09407853, 0.03206501], [1.0744894, -0.70407853, 0.03206501], [1.0844894, -1.09407853, 0.03206501], [1.0844894, -0.70407853, 0.03206501], [1.0944894, -1.08407853, 0.03206501], [1.0944894, -0.71407853, 0.03206501], [1.1044894, -1.07407853, 0.03206501], [1.1044894, -0.71407853, 0.03206501], [1.1144894, -1.06407853, 0.03206501], [1.1144894, -0.72407853, 0.03206501], [1.1244894, -1.05407853, 0.03206501], [1.1244894, -0.72407853, 0.03206501], [1.1344894, -1.04407853, 0.03206501], [1.1344894, -0.73407853, 0.03206501], [1.1444894, -1.03407853, 0.03206501], [1.1444894, -0.73407853, 0.03206501], [1.1544894, -1.02407853, 0.03206501], [1.1544894, -0.74407853, 0.03206501], [1.1644894, -1.00407853, 0.03206501], [1.1644894, -0.74407853, 0.03206501], [1.1744894, -0.99407853, 0.03206501], [1.1744894, -0.75407853, 0.03206501], [1.1844894, -0.98407853, 0.03206501], [1.1844894, -0.75407853, 0.03206501], [1.1944894, -0.96407853, 0.03206501], [1.1944894, -0.76407853, 0.03206501], [1.2044894, -0.95407853, 0.03206501], [1.2044894, -0.76407853, 0.03206501], [1.2144894, -0.93407853, 0.03206501], [1.2144894, -0.77407853, 0.03206501], [1.2244894, -0.92407853, 0.03206501], [1.2244894, -0.77407853, 0.03206501], [1.2344894, -0.90407853, 0.03206501], [1.2344894, -0.78407853, 0.03206501], [1.2444894, -0.88407853, 0.03206501], [1.2444894, -0.78407853, 0.03206501], [1.2544894, -0.86407853, 0.03206501], [1.2544894, -0.79407853, 0.03206501], [1.2644894, -0.84407853, 0.03206501], [1.2644894, -0.79407853, 0.03206501], [1.2744894, -0.82407853, 0.03206501], [1.2744894, -0.80407853, 0.03206501], [1.2844894, -0.81407853, 0.03206501], [1.2844894, -0.80407853, 0.03206501], [1.2944894, -0.80407853, 0.03206501]]

barycentre = [ 0.89051777, -0.98444378,  0.03206501]

def display_info(pos_legs, current_leg_moving, points, barycentre, dx=0, dy=0):
        points_= [[x + dx, y + dy, 0] for x, y, z in points]
        points_ymoins = [[x + dx, y * -1 + dy, 0] for x, y, z in points]
        points_xmoins = [[x * -1 + dx, y + dy, 0] for x, y, z in points]
        points_xymoins = [[x * -1 + dx, y * -1 + dy, 0] for x, y, z in points]

        line_viz("polygon_xy", np.array(points_xymoins), color=0xFF11AA)
        line_viz("polygon_x", np.array(points_xmoins), color=0xFF11AA)
        line_viz("polygon_y", np.array(points_ymoins), color=0xFF11AA)
        line_viz("polygon", np.array(points_), color=0xFF11AA)

        barycentre_= [barycentre[0] + dx, barycentre[1] + dy, barycentre[2]]
        barycentre_ymoins = [barycentre[0] + dx, barycentre[1]*-1 + dy, barycentre[2]]
        barycentre_xmoins = [barycentre[0]*-1 + dx, barycentre[1] + dy, barycentre[2]]
        barycentre_xymoins = [barycentre[0]*-1 + dx, barycentre[1]*-1 + dy, barycentre[2]]

        point_viz("barycentre_", barycentre_, color=0x0000FF)
        point_viz("barycentre_ymoins", barycentre_ymoins, color=0x0000FF)
        point_viz("barycentre_xmoins", barycentre_xmoins, color=0x0000FF)
        point_viz("barycentre_xymoins", barycentre_xymoins, color=0x0000FF)

        #print(pos_legs)
        legs=[]
        for i in range(len(pos_legs)):  # Parcourir les indices de pos_legs
            if i != current_leg_moving-1:  # Si l'indice n'est pas celui de la jambe en mouvement
                legs.append(pos_legs[i])
        #print(legs)
        legs += [legs[0], legs[1], legs[2]]

        line_viz("ZMP", np.array(legs), color=0x00AA00)
            

##### #####
            

##### #####
            

##### #####