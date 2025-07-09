# clr_dynamic_sim_demo

This package contains an application demonstration of the CLR robot picking up a CTB.
While the demonstration should run on hardware (not yet tested), it is primarily intended for use in CLR's MuJoCo simulation.

To run this demo, open three terminals in the clr_dynamic_sim_ws Docker container and run:

```bash
    ros2 launch clr_mujoco_config clr_mujoco.launch.py model_env:=true

    ros2 launch clr_moveit_config clr_moveit.launch.py use_sim_time:=true sim_env:=true

    ros2 launch clr_dynamic_sim_demo demo.launch.py
```
