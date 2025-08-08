# clr_dynamic_sim_demo

This package contains an application demonstration of the CLR robot picking up a CTB.
The demonstration can be run in the MuJoCo simulation or on hardware and is used to evaluate sim-to-real transfer of simulated behaviors.

## Sim Instructions
To run this demo in simulation, open three terminals in the clr_dynamic_sim_ws Docker container and run:

```bash
    ros2 launch clr_mujoco_config clr_mujoco.launch.py model_env:=true

    ros2 launch clr_moveit_config clr_moveit.launch.py launch_rviz:=false include_mockups_in_description:=true use_sim_time:=true

    ros2 launch clr_dynamic_sim_demo demo.launch.py rviz:=true
```
## Hardware Instructions
To run this demo on hardware, bring up the clr_dynamic_sim_ws Docker container on the controls computer, open two terminals, and run:

```bash
    ros2 launch clr_deploy clr_hw.launch.py include_mockups_in_description:=true

    ros2 launch chonkur_deploy ur_tools.launch.py
```
Then open two terminals in the clr_dynamic_sim_ws Docker container on the console computer and run:

```bash
    ros2 launch clr_moveit_config clr_moveit.launch.py launch_rviz:=false include_mockups_in_description:=true

    ros2 launch clr_dynamic_sim_demo demo.launch.py rviz:=true sim:=false
```
