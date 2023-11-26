import numpy as np
import matplotlib.pyplot as plt
import math
import os
from math import cos, sin, pi
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits import mplot3d


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
                                      swing_phase_ratio=0.3):
    time_array = np.linspace(0, total_time, 1000)
    foot_heights = []
    joint_positions_list = []
    for t in time_array:
        gait_phase = (t % gait_cycle_time) / gait_cycle_time

        # Define swing and stance phases for the front_right leg
        swing_phase_ratio = 0.3  # 30% of the gait cycle
        leg_phase = gait_phase  # Assuming front_right leg is in phase
        stance_phase_ratio = 1 - swing_phase_ratio  # 70% of the gait cycle
        """ 
        # Calculate the joint angles based on the phase
        if 0 <= leg_phase < swing_phase_ratio:
            # Swing phase (lifting the foot and moving it forward)
            phase_ratio = leg_phase / swing_phase_ratio
            hip_angle = resting_hip_angle + step_length * phase_ratio
            knee_angle = resting_knee_angle + step_height * math.sin(math.pi * phase_ratio)
        else:
            # Stance phase (foot is on the ground and dragging back)
            #phase_ratio = (leg_phase - swing_phase_ratio) / (1 - swing_phase_ratio)
            hip_angle = resting_hip_angle + step_length * (1 - phase_ratio)
            knee_angle = resting_knee_angle """

        if 0 <= leg_phase < swing_phase_ratio:
            # Swing phase (lifting the foot and moving it forward)
            phase_ratio = leg_phase / swing_phase_ratio
            hip_angle = resting_hip_angle - (step_length * phase_ratio)  # Moving forward
            knee_angle = resting_knee_angle - (step_height * math.sin(math.pi * phase_ratio))  # Lifting up

        elif swing_phase_ratio <= leg_phase < 1:
            # Stance phase (foot is on the ground and dragging back)
            phase_ratio = (leg_phase - swing_phase_ratio) / stance_phase_ratio
            hip_angle = resting_hip_angle - (step_length * (1 - phase_ratio)) # Moving back to the starting position
            knee_angle = resting_knee_angle  # Keep the foot on the ground 

        # Calculate the foot height based on joint angles
        # Assuming simple linear relationship for demonstration
        joint_positions = get_joint_positions(
                                                B = 0, # body to hip
                                                L1 = 3.5,  # hip to upper-leg
                                                L2 = 10,  # upper-leg to lower-leg
                                                L3 = 10,  # lower-leg to foot
                                                th1 = 0,  # hip servo
                                                th2 = hip_angle,  # upper-leg servo
                                                th3 = knee_angle,  # lower-leg servo
                                                )
        foot_height = joint_positions[-1]  # Simplified calculation
        foot_heights.append(foot_height)
        joint_positions_list.append(joint_positions)

    return time_array, foot_heights, joint_positions_list

# Get foot positions 
swing_phase_ratio = 0.3
gait_cycle_time = 1 / 2  # Example gait cycle time for walking_speed=2
time_array, foot_positions, joint_positions_list = simulate_foot_position_over_cycle(
    step_height=0.5, 
    step_length=0.5, 
    walking_speed=2, 
    resting_hip_angle=-0.5, 
    resting_knee_angle=0.7, 
    gait_cycle_time=gait_cycle_time, 
    total_time=gait_cycle_time,  # Plotting for one cycle
    swing_phase_ratio=swing_phase_ratio,
)



# Plotting gait patter overview 
fig, ax = plt.subplots(1,1, figsize=(10, 6))
ax.plot(time_array, foot_positions, label='foot position xyz')
ax.axvline(x=(max(time_array)*swing_phase_ratio), ls="--", color='k')
#x = [0, max((time_array)*swing_phase_ratio)]

#plt.fill_between(x, y1=[0,10], color='grey', alpha='0.5')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Centimeters')
#ax.title('Foot Positions Over One Gait Cycle')
ax.legend(labels=['X - Forward motion', 'Y - Side motion', 'Z - Height'])
ax.grid(True)
plt.show()

""" 
joint_data = np.array(joint_positions_list)

# Update the animation function
def update(num, joint_data, line):
    # Reshape the joint_data for the current frame
    current_data = joint_data[num].T  # Transpose to get shape (xyz, num_joints)
    line.set_data(current_data[:2, :])  # Set x and y data
    line.set_3d_properties(current_data[2, :])  # Set z data
    return line,

# Creating the figure for the animation
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Initialize the line object (starting at the first time step)
line, = ax.plot(joint_data[0, :, 0], joint_data[0, :, 1], joint_data[0, :, 2])

# Setting the axes properties based on the range of joint_data
ax.set_xlim3d([np.min(joint_data[:,:,0]), np.max(joint_data[:,:,0])])
ax.set_xlabel('X')

ax.set_ylim3d([np.min(joint_data[:,:,1]), np.max(joint_data[:,:,1])])
ax.set_ylabel('Y')

ax.set_zlim3d([np.min(joint_data[:,:,2]), np.max(joint_data[:,:,2])])
ax.set_zlabel('Z')

ax.set_title('3D Test')

# Creating the Animation object
ani = animation.FuncAnimation(fig, update, frames=len(joint_data), fargs=(joint_data, line), interval=50)
 """
