# ROS 2 FR3 Redundancy Resolution Comparison
This package provides a comparison of two methods for redundancy resolution in the Franka Emika Panda robot using ROS 2: the Reduced Gradient (RG) and the Projected Gradient (PG). 
It includes the `2cde545dea97eb5fdbae342ddbec049c445f4224` commit of the `franka_ros2` repository, which contains the necessary files for the robot's description and control.
You should follow the documentation of the repository to install the necessary dependencies and set up the environment first. Then, you can add the `my_fr3_control` directory to your ROS 2 workspace inside the `src` directory.

## Launch

First, source the ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
```
Then, build the package:

```bash
colcon build
``` 

Finally, launch the package:

```bash
source install/setup.bash
ros2 launch my_fr3_control my_fr3_launch.py
``` 

## Problems
You may need to install kdl_parser_py from pip
```bash
pip install kdl_parser_py
```

## Files
- `src/my_fr3_control/launch/my_fr3_launch.py`: Launch file for the FR3 control package. Here you can set the parameters:
    - `isRG`: use the Reduced Gradient if true, otherwise use the Prjected Gradient.
    - `circular`: simulation on the circular path if True, otherwise on the linear path .
    - `acceleration`: compute the gradient step at acceleration level if True, at velocity level otherwise.
    - `orientation`: put also the orientation in the task  if True, otherwise only the position. 

- `src/my_fr3_control/my_fr3_control/worker_node.py`: Worker node that computes the Jacobian, the pose, and the gradient of the manipulability function. It also publishes the trajectory to the robot.
- `src/my_fr3_control/my_fr3_control/acceleration_controller_node.py`: Node that computes the gradient step at acceleration level.
- `src/my_fr3_control/my_fr3_control/velocity_controller_node.py`: Node that computes the gradient step at velocity level.

## Modifications
If you want to apply modifications to the code, you will need to rebuild the package:

```bash
colcon build --packages-select my_fr3_control
```

Then, source the setup file again and relaunch:

```bash 
source install/setup.bash
ros2 launch my_fr3_control my_fr3_launch.py
```