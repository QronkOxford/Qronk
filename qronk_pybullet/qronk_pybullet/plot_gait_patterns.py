import os
from math import cos, sin, pi
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d

# Load URDF model
URDF_DIR = os.getcwd() + "/../../src/qronk_description/urdf/"
xacro_model = URDF_DIR + "qronk.urdf.xacro"
#urdf_model = !xacro {xacro_model}

def get_joint_positions(
                        B = 0, # body to hip
                        L1 = 3.5,  # hip to upper-leg
                        L2 = 10,  # upper-leg to lower-leg
                        L3 = 10,  # lower-leg to foot
                        th1 = 0.5,  # hip servo
                        th2 = 0.2,  # upper-leg servo
                        th3 = -0.7,  # lower-leg servo
                        ):
    T01_translation = np.array([[1, 0, 0, B],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T01_rotation = np.array([[1, 0, 0, 0],
                    [0, cos(th1), -sin(th1), 0],
                    [0, sin(th1), cos(th1), 0],
                    [0, 0, 0, 1]])
    T01 = T01_translation @ T01_rotation

    T12_translation = np.array([[1, 0, 0, 0],
                    [0, 1, 0, L1],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
    T12_rotation = np.array([[cos(th2), 0, sin(th2), 0],
                    [0, 1, 0, 0],
                    [-sin(th2), 0, cos(th2), 0],
                    [0, 0, 0, 1]])
    T12 = T12_translation @ T12_rotation

    T23_translation = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, -L2],
                    [0, 0, 0, 1]])
    T23_rotation = np.array([[cos(th3), 0, sin(th3), 0],
                    [0, 1, 0, 0],
                    [-sin(th3), 0, cos(th3), 0],
                    [0, 0, 0, 1]])
    T23 = T23_translation @ T23_rotation

    T3e = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, -L3],
                    [0, 0, 0, 1]])

    T01 = T01
    T02 = T01 @ T12
    T03 = T01 @ T12 @ T23
    T0e = T01 @ T12 @ T23 @ T3e

    J1 = T01[0:3, 3]
    J2 = T02[0:3, 3]
    J3 = T03[0:3, 3]
    Je = T0e[0:3, 3]

    JointPos = [J1, J2, J3, Je]
    
    return JointPos

def plotLeg(JointPos): 
    """Plots the leg in 3D space, given the joint positions in coordinates."""

    # unpack the joint positions
    x, y, z = zip(*JointPos)
    
    # create a figure
    plt.figure(figsize=(20, 4))
    
    # plot x-z plane (side-view)
    plt.subplot(141)
    plt.plot(x, z, 'b-')
    plt.scatter(x, z, color='red')
    plt.title('Side-view')
    plt.xlabel('x')
    plt.ylabel('z')
    plt.axis('square')
    plt.axis([-20, 20, -20, 20])

    # plot y-z plane (front-view)
    plt.subplot(142)
    plt.plot(y, z, 'b-')
    plt.scatter(y, z, color='red')
    plt.title('Front-view')
    plt.xlabel('y')
    plt.ylabel('z')
    plt.axis('square')
    plt.axis([-20, 20, -20, 20])

    # plot x-y plane (bottom-view)
    plt.subplot(143)
    plt.plot(y, x, 'b-')
    plt.scatter(y, x, color='red')
    plt.title('Bottom-view')
    plt.xlabel('y')
    plt.ylabel('x')
    plt.axis('square')
    plt.axis([-20, 20, -20, 20])

    # plot 3D view
    plt.subplot(144, projection='3d')
    plt.plot(x, y, z, 'b-')
    plt.title('3D view')

    plt.show()


# Plot an example leg with arbitrary joint positions (for testing)
JointPos = get_joint_positions( B = 0, # body to hip
                                L1 = 3.5,  # hip to upper-leg
                                L2 = 10,  # upper-leg to lower-leg
                                L3 = 10,  # lower-leg to foot
                                th1 = 0.5,  # hip servo angle
                                th2 = 0.2,  # upper-leg servo angle
                                th3 = -0.7,  # lower-leg servo angle
                                )

def simulate_walk_cycle(step_height, step_length, walking_speed, B, L1, L2, L3, total_time):
    gait_cycle_time = 1 / walking_speed
    time_array = np.linspace(0, total_time, 500)
    foot_heights = []

    for t in time_array:
        gait_phase = (t % gait_cycle_time) / gait_cycle_time
        swing_phase_ratio = 0.3

        # Calculate the target angles based on the phase
        if 0 <= leg_phase < swing_phase_ratio:
            # Swing phase (lifting the foot and moving it forward)
            phase_ratio = leg_phase / swing_phase_ratio
            hip_angle = resting_hip_angle - (step_length * phase_ratio)  # Moving forward
            knee_angle = resting_knee_angle - (step_height * math.sin(math.pi * phase_ratio))  # Lifting up
        elif swing_phase_ratio <= leg_phase < 1:
            # Stance phase (foot is on the ground and dragging back)
            phase_ratio = (leg_phase - swing_phase_ratio) / stance_phase_ratio
           # hip_angle = resting_hip_angle + step_length * (1 - phase_ratio)  
            hip_angle = resting_hip_angle - (step_length * (1 - phase_ratio)) # Moving back to the starting position
            knee_angle = resting_knee_angle  # Keep the foot on the ground

        joint_positions = get_joint_positions(B, L1, L2, L3, th1, th2, th3)
        foot_position = joint_positions[-1]  # Get foot position
        foot_heights.append(foot_position[1])  # Append foot's y-coordinate (height)

    return time_array, foot_heights

# Example usage
time_array, foot_heights = simulate_walk_cycle(step_height=0.5, step_length=0.5, walking_speed=2, B=0, L1=3.5, L2=10, L3=10, total_time=5)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(time_array, foot_heights, label='Foot Height')
plt.xlabel('Time (s)')
plt.ylabel('Height (units)')
plt.title('Foot Height Over Time During Walk Cycle')
plt.legend()
plt.grid(True)
plt.show()



def plot_foot_position_over_time(step_height, walking_speed, 
                                 upper_leg_length=10, 
                                 lower_leg_length=10, 
                                 total_time=5):
    # Calculate normalized time in the gait cycle [0, 1)
    gait_cycle_time = 1 / walking_speed

    # Time array
    time_array = np.linspace(0, total_time, 1000)
    foot_height_array = []
    foot_horizontal_array = []

    # Loop through time array and calculate the position
    for t in time_array:
        gait_phase = (t % gait_cycle_time) / gait_cycle_time

        # Define swing and stance phases
        swing_phase_ratio = 0.3  # 30% of the gait cycle
        if 0 <= gait_phase < swing_phase_ratio:
            # Swing phase
            phase_ratio = gait_phase / swing_phase_ratio
            foot_height = step_height * math.sin(math.pi * phase_ratio)  # Lifting up
            foot_horizontal = -step_height * math.cos(math.pi * phase_ratio)  # Moving forward
        elif swing_phase_ratio <= gait_phase < 1:
            # Stance phase
            phase_ratio = (gait_phase - swing_phase_ratio) / (1 - swing_phase_ratio)
            foot_height = 0  # Foot on the ground
            foot_horizontal = -step_height * (1 - phase_ratio)  # Moving backward

        # Adjust for leg lengths
        total_leg_length = upper_leg_length + lower_leg_length
        foot_height += total_leg_length
        foot_horizontal_array.append(foot_horizontal)
        foot_height_array.append(foot_height)

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(foot_horizontal_array, foot_height_array, label='Foot Position')
    plt.xlabel('Horizontal Position (units)')
    plt.ylabel('Height (units)')
    plt.title('Foot Position Over Time During Walk Cycle')
    plt.legend()
    plt.grid(True)
    plt.show()

# Example usage
#plot_foot_position_over_time(step_height=0.5, walking_speed=2)

