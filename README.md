# Qronk

A quadruped robot project by a group of Oxford undergraduates.

## Setup

The project is currently being developed based on [ROS2 Humble](https://docs.ros.org/en/humble/index.html). Follow the setup instructions below to use the visualization and simulation software.

### Download ROS2 Humble

Follow the [installation page](https://docs.ros.org/en/humble/Installation.html) on ROS2 Humble documentation, preferably using the Ubuntu 22.04 Debian packages.

### Build workspace

Create a source directory under your workspace (i.e. `~/qronk_ws/src/`), and clone this repository into the `src` folder.

```bash
mkdir ~/qronk_ws/src/
cd ~/qronk_ws/src/
git clone https://github.com/shinben0327/Qronk.git
```

On terminal, head to the `qronk_ws` directory and run the `colcon build` command in order to build the workspace.

```bash
cd ~/qronk_ws/
colcon build
```

### Source workspace

Source the workspace using the following command. (It may be helpful to add this to the `.bashrc` file to avoid the need to run this for every terminal session.)

```bash
source /opt/ros/humble/setup.bash
source ~/qronk_ws/install/setup.bash
```

## Usage

You can now use any of the launch files using the following command.

```bash
ros2 launch <package_name> <launch_file_name>
```
