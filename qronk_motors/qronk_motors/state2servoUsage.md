# State 2 Servo Usage

A ros2 python script for controling servo angle with the joint state publisher.

## Dependencies
Run on a raspberry pi and run

```bash
pip3 install adafruit-circuitpython-pca9685
pip3 install adafruit-circuitpython-motor
```
to install the dependencies. Make sure that I2C is enabled in `raspi-config`.

## Usage

Run using the launch command:
```bash
ros2 launch qronk_motors motors_launch.xml
```
or
```bash
ros2 launch qronk_motors motors_lite_launch.xml
```
This launches the servo control script, RViz and the joint state publisher GUI. The Joint state publisher can then be used to move the joint positions which are reflected in RViz and the servo simulation.

motors_lite_launch.xml launches the control script and the joint state publisher only to increase performance on a lower power Raspberry Pi.