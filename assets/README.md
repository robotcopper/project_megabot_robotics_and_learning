# Assests of the MEGABOT project

**Description:** 
This folder contains images and videos of the MEGABOT as well as results of different scripts. Thoses results are either figures or text files containing data.


## 📄 Details
In more detail, the folder contains the following files:
- Figures of the action surface of each leg of the Megabot with its joint et velocity limits set:
  - [leg_1_action_surface_on_ground_with_limits_0.01_step.png](leg_1_action_surface_on_ground_with_limits_0.01_step.png)
  - [leg_1_action_surface_on_ground_with_limits_0.02_step.png](leg_1_action_surface_on_ground_with_limits_0.02_step.png)
  - [leg_2_action_surface_on_ground_with_limits_0.02_step.png](leg_2_action_surface_on_ground_with_limits_0.02_step.png)
  - [leg_3_action_surface_on_ground_with_limits_0.02_step.png](leg_3_action_surface_on_ground_with_limits_0.02_step.png)
  - [leg_4_action_surface_on_ground_with_limits_0.02_step.png](leg_4_action_surface_on_ground_with_limits_0.02_step.png)

  Thoses figures were generated with the [one_leg_range_on_ground](../src/simulation/one_leg_range_on_ground.py) script.
  These figures are used to determine the **field of action** of a leg of the megabot on the ground and thus **define the distance of the largest possible step** for a leg.

- Text files containing the data used to create the previous figures:
  - [leg_1_action_surface_on_ground_with_limits_0.01_step.txt](leg_1_action_surface_on_ground_with_limits_0.01_step.txt)
  - [leg_1_action_surface_on_ground_with_limits_0.02_step.txt](leg_1_action_surface_on_ground_with_limits_0.02_step.txt)
  - [leg_2_action_surface_on_ground_with_limits_0.02_step.txt](leg_2_action_surface_on_ground_with_limits_0.02_step.txt)
  - [leg_3_action_surface_on_ground_with_limits_0.02_step.txt](leg_3_action_surface_on_ground_with_limits_0.02_step.txt)
  - [leg_4_action_surface_on_ground_with_limits_0.02_step.txt](leg_4_action_surface_on_ground_with_limits_0.02_step.txt)

  Thoses files can be used to process the data differently and were generated with the [one_leg_range_on_ground](../src/simulation/one_leg_range_on_ground.py) script.

- Figures of the cylinders characterization resulting into the curve of the cylinder speed according to the mass that its supporting:
  - [cylinder_22_speed_mass.png](cylinder_22_speed_mass.png) (cylinder 2 of leg 2)
  - [cylinder_23_speed_mass.png](cylinder_23_speed_mass.png) (cylinder 3 of leg 2)

  Thoses figures were generated with the [vitesses_22](../src/simulation_and_real_megabot/Verin22/vitesses_22.py) and [vitesses_23](../src/simulation_and_real_megabot/Verin23/vitesses_23.py) scripts.
  These figures are used to determine the **speed limits** of the megabot's actuators (cylinders). It is thus possible to **add** these **limits** in the **Placo solver** in order to have a simulation as close as possible to reality.

- Logs figures representing the positions (commands) recieved by the microcontroller compared to the actual real cylinder positions:
  - [log_cylinder_41_oneLegCreepingGait.png](log_cylinder_41_oneLegCreepingGait.png) (cylinder 1 of leg 4 with the creeping gait motion)
  - [log_cylinder_42_oneLegCreepingGait.png](log_cylinder_42_oneLegCreepingGait.png) (cylinder 2 of leg 4 with the creeping gait motion)
  - [log_cylinder_43_oneLegCreepingGait.png](log_cylinder_43_oneLegCreepingGait.png) (cylinder 3 of leg 4 with the creeping gait motion)
  - [log_cylinder_41_sin.png](log_cylinder_41_sin.png) (cylinder 1 of leg 4 with a sinus function)
  - [log_cylinder_42_sin.png](log_cylinder_42_sin.png) (cylinder 2 of leg 4 with a sinus function)
  - [log_cylinder_43_sin.png](log_cylinder_43_sin.png) (cylinder 3 of leg 4 with a sinus function)

  These logs make it possible to **compare** the **position commands received** and the **actual positions** of each cylinder. There is a **delay** in the **actual positions (in purple)** compared to the **recieved positions (in green)**. The logs are automatically saved in the **one_leg_creeping_gait_with_com.log** file, in the same folder as the lauched scipt: [one_leg_creeping_gait_with_com](../src/simulation_and_real_megabot/one_leg_creeping_gait_with_com.py). Afterward, the figures are created with the following commands in a terminal :
  ```
  src/simulation_and_real_megabot$ grep '^<number of the leg>;<number of the cylinder>;' one_leg_creeping_gait_with_com.log > <choose a name for the resulting file>
  src/simulation_and_real_megabot$ gnuplot
  gnuplot> set datafile separator ";"
  gnuplot> plot "<choose a name for the resulting file>" using (column(9)):(column(4) "<choose a name for the resulting file>" using (column(9)):(column(5))
  ```

- Picture of the Megabot:
  - [MEGABOT_Real.png](MEGABOT_Real.png)

- Videos of the Megabot:
  - [move_base_with_keyboard_base_translation_simulation.mp4](move_base_with_keyboard_base_translation_simulation.mp4): from the [move_base_with_keyboard.py](../src/simulation/move_base_with_keyboard.py) script
  - [move_base_with_keyboard_base_rotation_simulation.mp4](move_base_with_keyboard_base_rotation_simulation.mp4): from the [move_base_with_keyboard.py](../src/simulation/move_base_with_keyboard.py) script
  - [one_leg_creeping_gait_real_megabot.mp4](one_leg_creeping_gait_real_megabot.mp4): from the [creeping_gait_with_com.py](../src/simulation_and_real_megabot/move_base_with_keyboard.py) script
  - [creeping_gait_simulation_megabot.mp4](creeping_gait_simulation_megabot.mp4): from the [creeping_gait.py](../src/simulation/creeping_gait.py) script
  - [creeping_gait_quick_simulation_megabot.mp4](creeping_gait_quick_simulation_megabot.mp4): from the [creeping_gait.py](../src/simulation/creeping_gait.py) script
