# Ubuntu Virtual Machine (Mac)

You will have to install Ubuntu on your system in order to run our code. Please follow this guide to install Ubuntu virtual machine on your Mac using Multipass.

Please contact [Jin Rhee](mailto:jin.rhee@sjc.ox.ac.uk) if you have any questions.

## 1. Ubuntu 22.04

### 1.1. Installation

Install Multipass following [this link](https://multipass.run/docs/installing-on-macos).

Once installed, you should be able to use multipass from your terminal. The following will find the available images:
```bash
% multipass find
Image                       Aliases           Version          Description
...
22.04                       jammy,lts         20231026         Ubuntu 22.04 LTS
...
```
Multipass by default offers the Ubuntu 22.04 image, which we need to run ROS2. Create a Multipass instance using the following:
```bash
% multipass launch 22.04
Starting preferable-duckling \
```
Multipass will automatically generate a random name for your instance if not named (ex. preferable-duckling). Once you have created your instance, you can check all instances by running:
```bash
% multipass list
Name                    State             IPv4             Image
...
preferable-duckling     Running           192.168.64.8     Ubuntu 22.04 LTS
...
```
Your instance should be running (multipass allows you to stop and start specific instances). You can now access the shell of your instance by running:
```bash
% multipass shell preferable-duckling
\Welcome to Ubuntu 22.04.3 LTS (GNU/Linux 5.15.0-87-generic aarch64)\
...
ubuntu@preferable-duckling:~$
```
You have now accessed your virtual machine's shell.

Once you have your Multipass instance created (make sure it's Ubuntu 22.04), you probably want to increase its disk memory, RAM, and CPU allocations. I recommend at least 4GB of RAM (memory), 20GB of disk memory (disk), and 2 CPUs. You can use the "set" command by following the instructions on [this link](https://multipass.run/docs/modify-an-instance#heading--set-the-cpu-ram-or-disk-of-an-instance).

### 1.2. Usage

Ubuntu 22.04 is a Linux system which uses a language named "bash" on the command line. We recommend that you learn some of the basic commands if you are not familiar. 

Follow [Lorenz Frank's tutorial](https://medium.com/geekculture/basic-bash-commands-c54933183c89) to learn how to navigate within the command line interface. 

It may be helpful to search for more elaborate guides online once you start to get comfortable.

## 2. Visual Studio Code

### 2.1. Installation

To edit our code, we recommend using Visual Studio Code. Please install it from the Ubuntu Software store and get familiar with the program.

### 2.2. Usage

To use Visual Studio Code in your virtual machine, head to your working directory and run the following line of code.

```bash
code .
```

We recommend that you do this in `~/qronk_ws/src` when editing our project as that is where we initialised our git repository.
