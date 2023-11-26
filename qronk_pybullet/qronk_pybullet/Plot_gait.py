import numpy as np
import matplotlib.pyplot as plt
import math
import os
from math import cos, sin, pi
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d

def normalize_3d_arrays(arrays):
    """
    Normalize a list of 3D arrays by subtracting the starting value of each dimension.

    :param arrays: List of 3D arrays (lists or tuples) where each array is of the form [x, y, z]
    :return: List of normalized 3D arrays
    """
    normalized_arrays = []

    for array in arrays:
        # Extract the start values for x, y, and z
        start_x, start_y, start_z = arrays[0]

        # Normalize each dimension
        normalized_x = array[0] - start_x
        normalized_y = array[1] - start_y
        normalized_z = array[2] - start_z

        # Append the normalized array to the result list
        normalized_arrays.append([normalized_x, normalized_y, normalized_z])

    return normalized_arrays

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

def simulate_foot_position_over_cycle(step_height, 
                                      step_length, 
                                      walking_speed, 
                                      resting_hip_angle, 
                                      resting_knee_angle, 
                                      gait_cycle_time, 
                                      total_time,
                                      swing_phase_ratio=0.3,
                                      angle_lists=False,
                                      shoulder_angle_list=False,
                                      hip_angle_list=False,
                                      knee_angle_list=False,
                                      ):
    

    time_array = np.linspace(0, total_time, 1000)
    foot_positions = []
    joint_positions_list = []
    for t in time_array:
        gait_phase = (t % gait_cycle_time) / gait_cycle_time

        # Define swing and stance phases for the front_right leg
        swing_phase_ratio = 0.3  # 30% of the gait cycle
        leg_phase = gait_phase  # Assuming front_right leg is in phase
        stance_phase_ratio = 1 - swing_phase_ratio  # 70% of the gait cycle

        if angle_lists==False:
            
            """ 
            # Walk Backwards
            constant = 0.4 # Acts as a constant decrease on hip angle (higher constant means overall lower foot position)
            stance_constant = 0.3 # Mitigates the parabolic curve resulting from one angle rotating faster than the other during stance_phase.
            if 0 <= leg_phase < swing_phase_ratio:
                # Swing phase (lifting the foot and moving it forward)
                phase_ratio = leg_phase / (swing_phase_ratio)
                hip_angle = resting_hip_angle + step_length * phase_ratio - constant
                knee_angle = resting_knee_angle - (step_height * math.sin(math.pi * phase_ratio))
                shoulder_angle = 0
            else:
                # Stance phase (foot is on the ground and dragging back)
                phase_ratio = (leg_phase - swing_phase_ratio) / (1 - swing_phase_ratio)
                hip_angle = resting_hip_angle + step_length * (1 - phase_ratio) - constant
                shoulder_angle = 0
                if phase_ratio < 0.5:
                    knee_angle =  resting_knee_angle - (phase_ratio*stance_constant) 
                else:
                    knee_angle =  resting_knee_angle - ((1-phase_ratio)*stance_constant)

            """
            # Walk forwards
            constant = 0.1 # Acts as a constant decrease on hip angle (higher constant means overall lower foot position)
            stance_constant = 0.35 # Mitigates the curve resulting from one angle rotating faster than the other during stance_phase.
            if 0 <= leg_phase < swing_phase_ratio:
                # Swing phase (lifting the foot and moving it forward)
                phase_ratio = leg_phase / (swing_phase_ratio)
                hip_angle = resting_hip_angle - (step_length * phase_ratio - constant)
                knee_angle = resting_knee_angle - (step_height * math.sin(math.pi * phase_ratio))
                shoulder_angle = 0
            else:
                # Stance phase (foot is on the ground and dragging back)
                phase_ratio = (leg_phase - swing_phase_ratio) / (1 - swing_phase_ratio)
                hip_angle = resting_hip_angle - (step_length * (1 - phase_ratio) - constant)
                shoulder_angle = 0
                if phase_ratio < 0.5:
                    knee_angle =  resting_knee_angle - (phase_ratio*stance_constant)
                else:
                    knee_angle =  resting_knee_angle - ((1-phase_ratio)*stance_constant)
            
        else: 
            shoulder_angle = shoulder_angle_list[t],
            hip_angle = hip_angle_list[t]
            knee_angle = knee_angle_list[t]

        # Calculate the foot height based on joint angles
        # Assuming simple linear relationship for demonstration
        joint_positions = get_joint_positions(
                                                B = 0, # body to hip
                                                L1 = 3.5,  # hip to upper-leg
                                                L2 = 10,  # upper-leg to lower-leg
                                                L3 = 10,  # lower-leg to foot
                                                th1 = shoulder_angle,  # hip servo
                                                th2 = hip_angle,  # upper-leg servo
                                                th3 = knee_angle,  # lower-leg servo
                                                )
        print(len(joint_positions))
        foot_position = joint_positions[-1]  # Simplified calculation
        foot_positions.append(foot_position)
        joint_positions_list.append(joint_positions)

    return time_array, foot_positions, joint_positions_list

# Get foot positions 
swing_phase_ratio = 0.3
gait_cycle_time = 1 / 2  # Example gait cycle time for walking_speed=2
time_array, foot_positions, joint_positions_list = simulate_foot_position_over_cycle(
    step_height=0.5, 
    step_length=0.5, 
    walking_speed=2, 
    resting_hip_angle=0.5, 
    resting_knee_angle=-0.7, 
    gait_cycle_time=gait_cycle_time, 
    total_time=gait_cycle_time,  # Plotting for one cycle
    swing_phase_ratio=swing_phase_ratio,
)


def plot_gait(foot_positions, time_array, swing_phase_ratio, joint_positions_list):

    foot_positions = normalize_3d_arrays(foot_positions)
    
    x_positions = [pos[0] for pos in foot_positions]
    y_positions = [pos[1] for pos in foot_positions]
    z_positions = [pos[2] for pos in foot_positions]

    # Plotting gait patter overview 
    fig = plt.figure(figsize=(15, 6))
    # Add the first 2D subplot on the left (3 row, 2 columns, position 1)
    ax0 = fig.add_subplot(3, 2, 1)
    ax0.plot(time_array, x_positions, label='X', color='C0')
    ax0.axvline(x=(max(time_array)*swing_phase_ratio), ls="--", color='k', label='End of Swing Phase')

    ax0.set_xlabel('Time (s)')
    ax0.set_ylabel('Centimeters')
    #ax.title('Foot Positions Over One Gait Cycle')
    ax0.legend(labels=['X - Forward motion', 'End of Swing Phase'])
    ax0.grid(True)

    # Add the first 2D subplot on the left (3 row, 2 columns, position 3)
    ax2 = fig.add_subplot(3, 2, 3)
    ax2.plot(time_array, y_positions, label='Y', color='C1')
    ax2.axvline(x=(max(time_array)*swing_phase_ratio), ls="--", color='k', label='End of Swing Phase')

    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Centimeters')
    #ax.title('Foot Positions Over One Gait Cycle')
    ax2.legend(labels=['Y - Side motion','End of Swing Phase'])
    ax2.grid(True)

    # Add the first 2D subplot on the left (3 row, 2 columns, position 5)
    ax3 = fig.add_subplot(3, 2, 5)
    ax3.plot(time_array, z_positions, label='Z', color='C2')
    ax3.axvline(x=(max(time_array)*swing_phase_ratio), ls="--", color='k', label='End of Swing Phase')

    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Centimeters')
    #ax.title('Foot Positions Over One Gait Cycle')
    ax3.legend(labels=['Z - Height', 'End of Swing Phase'])
    ax3.grid(True)

    # Hide spines
    ax0.spines[['right', 'top']].set_visible(False)
    ax2.spines[['right', 'top']].set_visible(False)
    ax3.spines[['right', 'top']].set_visible(False)

    # Find the overall min and max for uniform scaling
    min_pos = min(min(x_positions), min(y_positions), min(z_positions))
    max_pos = max(max(x_positions), max(y_positions), max(z_positions))

    # Set equal axis 
   # ax0.set_ylim(min_pos-2, max_pos+2)
   # ax2.set_ylim(min_pos-2, max_pos+2)
   # ax3.set_ylim(min_pos-2, max_pos+2)

    joint_data = np.array(joint_positions_list)


    # Creating the figure for the animation
    ax = fig.add_subplot(1, 2, 2, projection='3d')

    # Initialize the line and scatter objects (starting at the first time step)
    line, = ax.plot(joint_data[0, :, 0], joint_data[0, :, 1], joint_data[0, :, 2], color='blue')

    def update(num, joint_data, line):
        # Updates lines in animation
        current_data = joint_data[num].T
        line.set_data(current_data[:2, :])
        line.set_3d_properties(current_data[2, :])

        return line,

    x = joint_data[:, :, 0]
    y = joint_data[:, :, 1]
    z = joint_data[:, :, 2]

    # Setting the axes labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Gait Animation')

    # Extracting the ranges for each axis
    x_min, x_max = np.min(joint_data[:,:,0]), np.max(joint_data[:,:,0])
    y_min, y_max = np.min(joint_data[:,:,1]), np.max(joint_data[:,:,1])
    z_min, z_max = np.min(joint_data[:,:,2]), np.max(joint_data[:,:,2])

    # Calculating the maximum range
    max_range = max(x_max - x_min, y_max - y_min, z_max - z_min) / 2.0

    # Calculating the mid points for each axis
    mid_x = (x_max + x_min) * 0.5
    mid_y = (y_max + y_min) * 0.5
    mid_z = (z_max + z_min) * 0.5

    # Setting the same scale for all axes
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    # Plot Z height of starting position (presumably the ground)
    z_height = z[0][-1]  # This gets the first Z value

    # Define the range for x and y axis where the plane will be shown
    x_range = np.linspace(np.min(x), np.max(x), 10)
    y_range = np.linspace(np.min(y)-5, np.max(y)+5, 10)

    # Create a meshgrid for the x and y axis
    x, y = np.meshgrid(x_range, y_range)

    # Create a plane at the specific Z height
    ax.plot_surface(x, y, np.full_like(x, z_height), alpha=0.5, color='brown')  # Adjust alpha for transparency

    # Setting the view angle
    #ax.view_init(elev=0, azim=0)  # Focus on the X and Z axes
    #ax.view_init(elev=90, azim=-90)  # Top-down view focusing on the X and Y axes
    ax.view_init(elev=0, azim=90)  # Side view focusing on the X and Z axes

    # Creating the Animation object
    ani = animation.FuncAnimation(fig, update, frames=len(joint_data), 
                                fargs=(joint_data, line), 
                                interval=5, blit=True)
    
    # remove color of axis planes 
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False

    # To display the animation
    plt.tight_layout()
    plt.show()





plot_gait(foot_positions, time_array, swing_phase_ratio, joint_positions_list)