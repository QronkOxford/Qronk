# Qronk Pybullet Usage

At the moment there are two ways of launching pybullet from this module:

```bash
ros2 run qronk_pybullet startSim
```

This is essentially the same as the old method of directly running the python script.

There is also a launch script

```bash
ros2 launch qronk_pybullet pybullet_launch.py
```

This is currently has the same effect as running the first command. The launch file could be updated to launch multiple nodes.
