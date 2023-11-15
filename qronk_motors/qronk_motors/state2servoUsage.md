# State 2 Servo Usage

A ros2 python script for controling some servos with the joint state publisher.

## Dependencies

Either run this on a real raspberry pi (remove simulation code) or using tkgpio which can be installed with:

```bash
pip3 install tkgpio
```

## Usage

Run using the launch command:
```bash
ros2 launch qronk_motors motors_launch.xml
```
This launches the servo control script, RViz and the joint state publisher GUI. The Joint state publisher can then be used to move the joint positions which are reflected in RViz and the servo simulation.