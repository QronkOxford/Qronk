import pybullet as p
import time
import pybullet_data
import os
import math
import numpy as np 
from ament_index_python import get_package_share_directory

# TODO
# 0. invert front to back to align with udrf file - Done!
# 1. fix backwards gait sliding
# 2. connect to ros 
# 3. calculate centre of gravity to enable balancing

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

# Load the ground plane
planeId = p.loadURDF("plane.urdf")

# Set the initial position and orientation for the quadruped
startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
#p.setAdditionalSearchPath(os.environ['HOME'] + "/qronk_ws/src/qronk_pybullet/")
pathURDF=get_package_share_directory('qronk_pybullet')
p.setAdditionalSearchPath(pathURDF)
qronkId = p.loadURDF("qronk.urdf", startPos, startOrientation)


# Get the number of joints in the quadruped
num_joints = p.getNumJoints(qronkId)

# Initialize a dictionary to hold joint indices
joint_indices = {}

# Loop over all joints to get their names and indices
for joint_index in range(num_joints):
    joint_info = p.getJointInfo(qronkId, joint_index)
    joint_name = joint_info[1].decode('utf-8')
    joint_indices[joint_name] = joint_index

# Define the joint names for each leg
legs = {
    'front_right': [
        'chassis_shoulder_FR_joint',
        'shoulder_upper_leg_FR_joint',
        'upper_leg_lower_leg_FR_joint',
        'lower_leg_foot_FR_joint'
    ],
    'front_left': [
        'chassis_shoulder_FL_joint',
        'shoulder_upper_leg_FL_joint',
        'upper_leg_lower_leg_FL_joint',
        'lower_leg_foot_FL_joint'
    ],
    'back_right': [
        'chassis_shoulder_BR_joint',
        'shoulder_upper_leg_BR_joint',
        'upper_leg_lower_leg_BR_joint',
        'lower_leg_foot_BR_joint'
    ],
    'back_left': [
        'chassis_shoulder_BL_joint',
        'shoulder_upper_leg_BL_joint',
        'upper_leg_lower_leg_BL_joint',
        'lower_leg_foot_BL_joint'
    ]
}


# Parameters for the camera
camera_distance = 1
camera_yaw = -50
camera_pitch = -35
camera_target_position = [0, 0, 0]  # This will be updated to the robot's position


# Walking parameters (negative angles are forwards)
step_height = 0.5
step_length = 0.5
walking_speed = 2  # A lower speed to give more control over the gait
resting_hip_angle = 0.5  # This is a slight angle for the 'resting' position of the hip joint
resting_knee_angle = -0.7  # This is a slight angle for the 'resting' position of the knee joint


# Constants for key bindings
KEY_W = 65297  # Up arrow key
KEY_S = 65298  # Down arrow key
KEY_LEFT = 65295  # Left arrow key
KEY_RIGHT = 65296  # Right arrow key

def walk_forward(step_height=step_height, 
                 step_length=step_length,
                 walking_speed=walking_speed,
                 resting_hip_angle=resting_hip_angle,
                 resting_knee_angle=resting_knee_angle):

    # Get the current simulation time
    t = time.time()

    # Calculate normalized time in the gait cycle [0, 1)
    gait_cycle_time = 1 / walking_speed
    gait_phase = (t % gait_cycle_time) / gait_cycle_time

    # Set the desired fixed angle for the chassis to shoulder joint
    fixed_chassis_shoulder_angle = 0  # Adjust this value to the desired fixed angle

    # Loop through each leg and set the position for each joint
    for leg_name, joint_names in legs.items():
        if leg_name in ['front_right', 'back_left']:
            # These legs will be in phase
            leg_phase = gait_phase
        elif leg_name in ['front_left', 'back_right']:
            # These legs will be out of phase
            leg_phase = (gait_phase + 0.5) % 1
        
        # Set the fixed angle for chassis to shoulder joint
        for joint_name in joint_names:
            joint_index = joint_indices[joint_name]
            if 'chassis_shoulder' in joint_name:
                # Set the joint to the fixed angle using position control
                p.setJointMotorControl2(bodyUniqueId=qronkId,
                                        jointIndex=joint_index,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=fixed_chassis_shoulder_angle)
                continue  # Skip the rest of the loop for this joint

        # Define swing and stance phases
        swing_phase_ratio = 0.3  # 30% of the gait cycle
        stance_phase_ratio = 1 - swing_phase_ratio  # 70% of the gait cycle
        
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

        # Ankle angle could be set to keep the foot parallel to the ground
        ankle_angle = -knee_angle / 2  # Adjust this to maintain foot parallelism

        # Apply the calculated angles to each joint in the current leg
        for joint_name in joint_names:
            joint_index = joint_indices[joint_name]
            if 'shoulder_upper_leg' in joint_name:
                target_angle = hip_angle
            elif 'upper_leg_lower' in joint_name:
                target_angle = knee_angle
            elif 'lower_leg_foot' in joint_name:
                target_angle = ankle_angle
            else:
                continue

            # Set the joint to the target angle using position control
            p.setJointMotorControl2(bodyUniqueId=qronkId,
                                    jointIndex=joint_index,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target_angle)
            
        

def walk_backward(step_height=0.5, 
                  step_length=0.5,
                  walking_speed=2,
                  resting_hip_angle = 0.5,
                  resting_knee_angle= -0.7):
    # Get the current simulation time
    t = time.time()

    # Calculate normalized time in the gait cycle [0, 1)
    gait_cycle_time = 1 / walking_speed
    gait_phase = (t % gait_cycle_time) / gait_cycle_time

    # Set the desired fixed angle for the chassis to shoulder joint
    fixed_chassis_shoulder_angle = 0  # Adjust this value to the desired fixed angle

    # Loop through each leg and set the position for each joint
    for leg_name, joint_names in legs.items():
        if leg_name in ['front_right', 'back_left']:
            # These legs will be in phase
            leg_phase = gait_phase
        elif leg_name in ['front_left', 'back_right']:
            # These legs will be out of phase
            leg_phase = (gait_phase + 0.5) % 1
        
        # Set the fixed angle for chassis to shoulder joint
        for joint_name in joint_names:
            joint_index = joint_indices[joint_name]
            if 'chassis_shoulder' in joint_name:
                # Set the joint to the fixed angle using position control
                p.setJointMotorControl2(bodyUniqueId=qronkId,
                                        jointIndex=joint_index,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=fixed_chassis_shoulder_angle)
                continue  # Skip the rest of the loop for this joint

        # Define swing and stance phases
        swing_phase_ratio = 0.2  # 30% of the gait cycle
        stance_phase_ratio = 1 - swing_phase_ratio  # 70% of the gait cycle

        """         
        # Calculate the target angles based on the phase
        if 0 <= leg_phase < swing_phase_ratio:
            # Swing phase (lifting the foot and moving it backward)
            phase_ratio = leg_phase / swing_phase_ratio
            hip_angle = resting_hip_angle + (step_length * phase_ratio)  # Moving backward
            knee_angle = resting_knee_angle + (step_height * math.sin(math.pi * phase_ratio))  # Lifting up
        elif swing_phase_ratio <= leg_phase < 1:
            # Stance phase (foot is on the ground and dragging forward)
            phase_ratio = (leg_phase - swing_phase_ratio) / stance_phase_ratio
            hip_angle = resting_hip_angle + (step_length * (1 - phase_ratio))  # Moving forward to the starting position
            knee_angle = resting_knee_angle  # Keep the foot on the ground """
        
        """
        constant = 0.1
        if 0 <= leg_phase < swing_phase_ratio:
            # Swing phase (lifting the foot and moving it forward)
            phase_ratio = leg_phase / (swing_phase_ratio)
            hip_angle = resting_hip_angle + step_length * phase_ratio - constant
            knee_angle = resting_knee_angle + step_height * math.sin(math.pi * phase_ratio)
        else:
            # Stance phase (foot is on the ground and dragging back)
            phase_ratio = (leg_phase - swing_phase_ratio) / (1 - swing_phase_ratio)
            hip_angle = resting_hip_angle + step_length * (1 - phase_ratio) - constant
            if phase_ratio < 0.5:
                knee_angle =  resting_knee_angle + (phase_ratio*0.3)
            else:
                knee_angle =  resting_knee_angle + ((1-phase_ratio)*0.3)

        """
        constant = 0.35 # Acts as a constant decrease on hip angle (higher constant means overall lower foot position)
        stance_constant = 0.1 # Mitigates the curve resulting from one angle rotating faster than the other during stance_phase.

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

        # Ankle angle could be set to keep the foot parallel to the ground
        ankle_angle = -knee_angle / 2  # Adjust this to maintain foot parallelism

        # Apply the calculated angles to each joint in the current leg
        for joint_name in joint_names:
            joint_index = joint_indices[joint_name]
            if 'shoulder_upper_leg' in joint_name:
                target_angle = hip_angle
            elif 'upper_leg_lower' in joint_name:
                target_angle = knee_angle
            elif 'lower_leg_foot' in joint_name:
                target_angle = ankle_angle
            else:
                continue

            # Set the joint to the target angle using position control
            p.setJointMotorControl2(bodyUniqueId=qronkId,
                                    jointIndex=joint_index,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target_angle)


def turn(side_to_turn, 
         step_length=0.5, 
         turning_speed=2, 
         resting_hip_angle=0.5, 
         resting_knee_angle=-0.7,
         step_height=0.5, 
         fixed_chassis_shoulder_angle=0,
         ):
    
    # Get the current simulation time
    t = time.time()

    # Calculate normalized time in the gait cycle [0, 1)
    gait_cycle_time = 1 / turning_speed
    gait_phase = (t % gait_cycle_time) / gait_cycle_time

    # Set turn side step scalar 
    if side_to_turn == 'right':
        step_scalar = 0
    if side_to_turn == 'left':
        step_scalar = 1

    # Loop through each leg and set the position for each joint
    for leg_name, joint_names in legs.items():
        if leg_name in ['front_right', 'back_right']:
            # Shorter stride for turn side 
            current_step_length = step_length * step_scalar
        else:
            current_step_length = step_length * (1-step_scalar)

        if leg_name in ['front_right', 'back_left']:
            # These legs will be in phase
            leg_phase = gait_phase
        elif leg_name in ['front_left', 'back_right']:
            # These legs will be out of phase
            leg_phase = (gait_phase + 0.5) % 1

        # Set the fixed angle for chassis to shoulder joint
        for joint_name in joint_names:
            joint_index = joint_indices[joint_name]
            if 'chassis_shoulder' in joint_name:
                # Set the joint to the fixed angle using position control
                p.setJointMotorControl2(bodyUniqueId=qronkId,
                                        jointIndex=joint_index,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=fixed_chassis_shoulder_angle)
                continue  # Skip the rest of the loop for this joint

        # Define swing and stance phases
        swing_phase_ratio = 0.3  # 30% of the gait cycle
        stance_phase_ratio = 1 - swing_phase_ratio  # 70% of the gait cycle

        # Calculate the target angles based on the phase
        if 0 <= leg_phase < swing_phase_ratio:
            # Swing phase (lifting the foot and moving it forward)
            phase_ratio = leg_phase / swing_phase_ratio
            hip_angle = resting_hip_angle - (current_step_length * phase_ratio)  # Moving forward
            knee_angle = resting_knee_angle - (step_height * math.sin(math.pi * phase_ratio))  # Lifting up
        elif swing_phase_ratio <= leg_phase < 1:
            # Stance phase (foot is on the ground and dragging back)
            phase_ratio = (leg_phase - swing_phase_ratio) / stance_phase_ratio
            hip_angle = resting_hip_angle - (current_step_length * (1 - phase_ratio))  # Moving back to the starting position
            knee_angle = resting_knee_angle  # Keep the foot on the ground

        # Ankle angle could be set to keep the foot parallel to the ground
        ankle_angle = -knee_angle / 2  # Adjust this to maintain foot parallelism

        # Apply the calculated angles to each joint in the current leg
        for joint_name in joint_names:
            joint_index = joint_indices[joint_name]
            if 'shoulder_upper_leg' in joint_name:
                target_angle = hip_angle
            elif 'upper_leg_lower_leg' in joint_name:
                target_angle = knee_angle
            elif 'lower_leg_foot' in joint_name:
                target_angle = ankle_angle
            else:
                continue

            # Set the joint to the target angle using position control
            p.setJointMotorControl2(bodyUniqueId=qronkId,
                                    jointIndex=joint_index,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=target_angle)

# Main simulation loop
first_loop = True
while True: # Loop indefinitely 

    p.stepSimulation()

    if first_loop == True:
        time.sleep(1) # Wait for qronk to stabilize
        first_loop=False 

    # Handle keyboard events
    keys = p.getKeyboardEvents()
    if KEY_W in keys and keys[KEY_W] & p.KEY_IS_DOWN:
        walk_forward()
        
    if KEY_S in keys and keys[KEY_S] & p.KEY_IS_DOWN:
        walk_backward()

    if KEY_RIGHT in keys and keys[KEY_RIGHT] & p.KEY_IS_DOWN:
        turn(side_to_turn='right')

    if KEY_LEFT in keys and keys[KEY_LEFT] & p.KEY_IS_DOWN:
        turn(side_to_turn='left')


    # Update the camera position
    qronk_position, qronk_orientation = p.getBasePositionAndOrientation(qronkId)
    camera_target_position = qronk_position

    p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                 cameraYaw=camera_yaw,
                                 cameraPitch=camera_pitch,
                                 cameraTargetPosition=camera_target_position)
    
    # Sleep to simulate real-time. This slows down the simulation to match the real world.
    time.sleep(1./240.)


