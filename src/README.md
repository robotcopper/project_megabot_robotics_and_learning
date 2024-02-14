# Scripts of the MEGABOT project

**Description:** 
This folder contains all the python scripts created during this project. It is divided into two subfolders: [simulation](simulation/) and [simulation_and_real_megabot](simulation_and_real_megabot/).

The first one contains the scripts that are **only** related to the **simulation** on the Placo solver. The second one contains scripts which **combine simulation and validation** on the real Megabot as well as the **cylinders caracterization** scripts.


## Simulation

The [simulation](simulation/) folder contains the following files/folder:
- [megabot](simulation/megabot/) folder: URDF of the Megabot robot (model)
- [megabot.py](simulation/megabot.py): function related to the robot (loading the model and make solvers)
- [global_functions.py](simulation/global_functions.py): functions common to several scripts (robot initializations, solver, tasks, communication, visualization, etc.)
- [move_base_with_keyboard.py](simulation/move_base_with_keyboard.py): make the Megabot "dance" by just moving its base
- [one_leg_reaching_target.py](simulation/one_leg_reaching_target.py): visualize where one leg can go on the ground (x, y range)
- [one_leg_range_on_ground.py](simulation/one_leg_range_on_ground.py): visualize the range of action of one leg on the ground with the base at initial height (urdf height)
- [one_leg_creeping_gait.py](simulation/one_leg_creeping_gait.py): see the creeping gait movement for just one leg in loop
- [creeping_gait.py](simulation/creeping_gait.py): move the Megabot in one of the cardinal directions in the simulation, following the creeping gait

## Simulation and real Megabot

The [simulation_and_real_megabot](simulation_and_real_megabot/) folder contains the following files/folder:
- [megabot](simulation_and_real_megabot/megabot/) folder: URDF of the Megabot robot (model)
- [megabot.py](simulation_and_real_megabot/megabot.py): function related to the robot (loading the model and make solvers)
- [global_functions.py](simulation_and_real_megabot/global_functions.py): functions common to several scripts (robot initializations, solver, tasks, communication, visualization, etc.)
- [com.py](simulation_and_real_megabot/com.py): protocol to communicate with the microcontroller
- [com_data.py](simulation_and_real_megabot/com_data.py): library used by com.py
- [one_leg_creeping_gait.py](simulation_and_real_megabot/one_leg_creeping_gait.py): see the creeping gait movement for just one leg in loop (in simulation and real life)
- [creeping_gait.py](simulation_and_real_megabot/creeping_gait.py): move the Megabot in one of the cardinal directions in the simulation, following the creeping gait (in simulation and real life)
- [Verin22](simulation_and_real_megabot/Verin22/) folder : contains all the cylinder 2.2 characterisation data and the scrips for displaying it
- [Verin23](simulation_and_real_megabot/Verin23/) folder : contains all the cylinder 2.3 characterisation data and the scrips for displaying it


**Each script contains a more detailed description of its purpose and implementation at the beginning of the script.**

