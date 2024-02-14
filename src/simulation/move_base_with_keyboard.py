'''
What this file does:
- Visualize the Megabot inside the Placo simulation
- The 4 legs are fixed to the ground (the effectors can't move)
- Move the robot base with keyboard inputs

Purpose: Make the Megabot "dance" by just moving its base.
    (With this code, the problem of the necessary amount of energy to rise a leg is not addressed.)

Keyboard inputs:
- ← : translate the base in the -x direction
- → : translate the base in the +x direction
- ↓ : translate the base in the -y direction
- ↑ : translate the base in the +y direction
- d : rotate the base in a circle motion in the clockwise direction
- q : rotate the base in a circle motion in the trigonometric direction
- s : stop the base rotation

/!\ The radius of the circle motion is defined by 
    the distance between the robot base center
    and the origin of the world.
    Therefore, you have to translate at least once
    the base before stating the circle motion.
'''


import time
import numpy as np
import meshcat.transformations as tf
from placo_utils.visualization import robot_viz, arrow_viz, robot_frame_viz, frame_viz, contacts_viz
import megabot
import pygame
import os
import sys
import math
import global_functions as gf


##### Constants #####

# length of the cylindes
cylinder_length = 0.2 # m

# time constants
t: float = 0.0
positive_t = 0.0
dt: float = 0.01
start_time = time.time()

# robot movement variables
rotation_direction = "CLOCKWISE"
rotation_factor = 1
translation_direction = ""
translation_y = 0
translation_x = 0
is_rotating = False
has_to_set_time = True
need_to_calcul_t = True
translation_step = 0.03
pi_factor = 2 * math.pi


##### Init robot and solver #####

# Creation
robot = megabot.load()
static_solver = megabot.make_solver(robot)
dynamic_solver = megabot.make_dynamics_solver(robot)

# Init T_world_base
T_world_base = np.eye(4)

# Initialization
robot = gf.init_robot_in_world(robot, T_world_base)

## Init base task ##
base_task = gf.set_base_task_soft_static_solver(static_solver, T_world_base)

## Init legs tasks ##
legs_tasks = []
legs_tasks = gf.set_legs_tasks_hard_static_solver(robot, static_solver, legs_tasks)

# Init dynamics solver
dynamic_solver = gf.init_dynamic_solver(dynamic_solver)

## Init legs tasks ##
legs_tasks = gf.set_legs_tasks_hard_dynamic_solver(robot, dynamic_solver, legs_tasks)


##### Init viewer #####

viz = robot_viz(robot)


##### Pygame #####

pygame.init()

# Get screen dimensions
width_screen, height_screen = pygame.display.Info().current_w, pygame.display.Info().current_h

# Set pygame window dimensions
width_pygame = width_screen
height_pygame = 50

# Set the window position in screen
pos_x = width_screen - width_pygame
pos_y = height_screen - height_pygame

# Set the pygame position
os.environ['SDL_VIDEO_WINDOW_POS'] = "%d,%d" % (pos_x, pos_y)

# Create window to detect press on keyboard (cette fenêtre n'a pas besoin d'être visible)
window = pygame.display.set_mode((width_pygame - 70, height_pygame))
pygame.display.set_caption("Keyboard press")

# Text to explain the keyboard inputs
font = pygame.font.Font(None, 20)
explaination_text = "Appuyez sur les flèches gauche, droite, haut et bas pour translater la base du robot. Appuyez sur q ou d pour effectuer une rotation de la base dans un sens ou dans l'autre. Appuyer sur s pour arrêter la rotation. Laissez cette fenêtre en premier plan."
text = font.render(explaination_text, True, (255, 255, 255))


##### Loop #####

while True:
    window.fill((0, 0, 0)) # Clean screen
    window.blit(text, (10, 10)) # Add text
    pygame.display.flip() # Update display
    
    # To exit
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Detect key press
    keys = pygame.key.get_pressed()
    
    # Rotation
    if keys[pygame.K_q]:
        rotation_direction = "TRIGONOMETRIC"
        rotation_factor = -1
        is_rotating = True
    if keys[pygame.K_d]:
        rotation_direction = "CLOCKWISE"
        rotation_factor = 1
        is_rotating = True
    if keys[pygame.K_s]:
        rotation_direction = "NONE"
        need_to_calcul_t = True
        is_rotating = False
        has_to_set_time = True
    
    # Translation
    if keys[pygame.K_LEFT] and (is_rotating == False):
        translation_direction = "LEFT"
        translation_x -= translation_step
    if keys[pygame.K_RIGHT] and (is_rotating == False):
        translation_direction = "RIGHT"
        translation_x += translation_step
    if keys[pygame.K_UP] and (is_rotating == False):
        translation_direction = "UP"
        translation_y += translation_step
    if keys[pygame.K_DOWN] and (is_rotating == False):
        translation_direction = "DOWN"
        translation_y -= translation_step
        
    T = T_world_base.copy()
    
    if  (is_rotating == False):
        # Update the body target frame (translation)
        T[0, 3] = translation_x # x translation (red)
        T[1, 3] = translation_y # y translation (green)
        base_task.T_world_frame = T
        
    if (is_rotating == True):
        # Get rotation radius
        translation_radius = math.sqrt(translation_x**2 + translation_y**2)
        
        if (need_to_calcul_t == True):
            t = math.atan2(translation_y, translation_x) / pi_factor
            translation_x = np.cos(t * pi_factor) * translation_radius
            translation_y = np.sin(t * pi_factor) * translation_radius
            need_to_calcul_t = False
        
        # Update x and y position
        translation_x = np.cos(t * pi_factor) * translation_radius
        translation_y = np.sin(t * pi_factor) * translation_radius
        
        # Update the body target frame (rotation)
        T[0, 3] = translation_x # x translation (red)
        T[1, 3] = translation_y # y translation (green)
        base_task.T_world_frame = T
        
        gf.display_all_joint(robot, cylinder_length)

    # Updating the kinematics and solving the IK
    robot.update_kinematics()
    static_solver.solve(True)

    # Computing static
    result = dynamic_solver.solve()
    if result.success:
        contacts_viz(dynamic_solver, ratio=1e-3, radius=0.03)

    # Displaying
    viz.display(robot.state.q)
    robot_frame_viz(robot, "base")
    frame_viz("base_target", T, 0.5)

    t += rotation_factor * dt
    positive_t += dt
    while time.time() - start_time < positive_t:
        time.sleep(1e-4)
