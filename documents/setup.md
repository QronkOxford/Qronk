# Setup

Please contact [Jihwan Ben Shin](mailto:jihwan.shin@sjc.ox.ac.uk) or [Jin Rhee](mailto:jin.rhee@sjc.ox.ac.uk) if you have any questions.

## 1. Install Ubuntu and VS Code

Follow the guides for [Windows](ubuntu_windows.md) or [Mac](ubuntu_mac.md) to install Ubuntu virtual machine and Visual Studio Code for your computer.

## 2. Install ROS2 Humble

For our project, we will be using ROS2 Humble as our main framework to move our robot. Install ROS2 Humble following [Open Robotics' Documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) in **your Ubuntu virtual machine, not your local Windows/Mac machine**.

In the step "Install ROS 2 packages", you will be given 3 options: Desktop Install, ROS-Base install, and Development tools. Please choose the Desktop version with the following line of code for full access to our project.

```bash
sudo apt install ros-humble-desktop
```

### 2.1. Sourcing the setup script

To use ROS2, you will have to source the setup script using the following line of code each time you open a new terminal session.

```bash
source /opt/ros/humble/setup.bash
```

If you want to do this automatically, add the line of code above to `.bashrc` in your home directory. You can easily do so by using the following line of code.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 2.2. Install colcon

In the next step, you will have to use a tool called "colcon" to build packages in ROS2. Install colcon using the following line of code.

```bash
sudo apt install python3-colcon-common-extensions
```

You can also follow [Open Robotics' Documentation](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) to learn more about colcon.

### 2.3. Learning more about ROS2

You can learn more about ROS2 by following [ROS2 Humble tutorials](https://docs.ros.org/en/humble/Tutorials.html). You can also search for more resources online, but do note that some functionalities may vary for different versions of ROS2.

## 3. Download Qronk workspace

Create a source directory under your workspace (i.e. `~/qronk_ws/src/`), and clone this repository into the `src` folder.

```bash
mkdir -p ~/qronk_ws/src/
cd ~/qronk_ws/src/
git clone https://github.com/QronkOxford/Qronk.git .
```

On terminal, head to the `qronk_ws` directory and run the `colcon build` command in order to build the workspace.

```bash
cd ~/qronk_ws/
colcon build
```

### 3.1. Install dependencies

To run some of our files, you may need to install some additional dependencies. We have a [list of dependencies](dependencies.md). Please let us know if we are missing any dependencies.

## 4. Usage

### 4.1. Sourcing the workspace setup script

To use the package you have built, you need to source the workspace setup script in addition to the ROS2 setup script. Use the following line of code each time you open a new terminal session.

```bash
source ~/qronk_ws/install/setup.bash
```

If you want to do this automatically, add the line of code above to `.bashrc` in your home directory. You can easily do so by using the following line of code.

```bash
echo "source ~/qronk_ws/install/setup.bash" >> ~/.bashrc
```

### 4.2. Using the launch files

You can now use any of the launch files using the following command:

```bash
ros2 launch <package_name> <launch_file_name>
```

For example, to display our model on Rviz:

```bash
ros2 launch qronk_description display.launch.xml
```

## 5. Troubleshooting

We will add any common problems here with its answers.
