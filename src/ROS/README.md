first, install kdl_parser_py from pip
```bash
pip install kdl_parser_py
```

Then, put this dir into the `franka_ros2_ws/src` folder, then run the following commands from the workspace root:

```bash
colcon build --packages-select my_fr3_control
source install/setup.bash
ros2 launch my_fr3_control my_fr3_launch.py
```