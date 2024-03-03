""" 
gait_sequences.py: generates the gait sequences for the robot. 
"""

from numpy import pi, cos, sin
from qronk_control.kinematics import inverseKinematics


def test_leg(frames=10, reach=5, origin=(0, 3.5, -12)):
    """
    Moves the FL leg in a straight line in each directions to test the leg's movement.

    Args:
        frames (int): The number of frames to move the leg.
        reach (int): The distance reached by the leg. 
        origin (tuple): The starting position of the leg.
    """    

    # Unpack the origin 
    x, y, z = origin

    # Moves the leg + and - in x, y, and z directions
    coords = []
    for i in range(frames):
        x += reach/frames
        coords.append((x, y, z))
    for i in range(frames):
        x -= reach/frames
        coords.append((x, y, z))
    for i in range(frames):
        y += reach/frames
        coords.append((x, y, z))
    for i in range(frames):
        y -= reach/frames
        coords.append((x, y, z))
    for i in range(frames):
        z += reach/frames
        coords.append((x, y, z))
    for i in range(frames):
        z -= reach/frames
        coords.append((x, y, z))
    
    # Converts the coordinates to joint angles
    angles = []
    for coord in coords:
        invKin = inverseKinematics("right", coord)
        angle = [invKin[0], invKin[1], invKin[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        angles.append(angle)
        
    return angles


def semicircle_crawl(period=100, swing_phase_ratio=0.25, radius=2.5):

    # Moves forward in a semicircle trajectory to represent swing
    def swing(frames):
        swing_coords = []
        for t in range(frames):
            x = radius - radius * cos(pi * t / frames)
            y = 0
            z = radius * sin(pi * t / frames)
            swing_coords.append((x, y, z))
        return swing_coords

    # Moves backwards in a straight line to represent stance
    def stance(frames):
        stance_coords = []
        for t in range(frames):
            x = radius + radius - 2 * radius * t / frames
            y = 0
            z = 0
            stance_coords.append((x, y, z))
        return stance_coords
    
    coords = []

    FL_coords = []
    BL_coords = []
    FR_coords = []
    BR_coords = []

    # * Arbitrarily decided starting values
    FL_origin = (0, 3.5, -15)
    BL_origin = (0, 3.5, -15)
    FR_origin = (0, -3.5, -15)
    BR_origin = (0, -3.5, -15)

    # * 1. body slightly to the back so the centre of gravity is behind
    


    # * 2. (FL - BR - FR - BL) repeats as a cycle

    return coords


def stand():
    pass


def sit():
    pass


def idle():
    """
    Returns a single idle position of the robot for it to stay in.
    """
    JeLeft = [0, 3.5, -12]
    JeRight = [0, -3.5, -12]

    invKinFL = inverseKinematics("left", JeLeft)
    invKinBL = inverseKinematics("left", JeLeft)
    invKinFR = inverseKinematics("right", JeRight)
    invKinBR = inverseKinematics("right", JeRight)

    angle = [invKinFR[0], invKinFR[1], invKinFR[2],
             invKinFL[0], invKinFL[1], invKinFL[2],
             invKinBR[0], invKinBR[1], invKinBR[2],
             invKinBL[0], invKinBL[1], invKinBL[2]
             ]
    return angle


def angle_derivative(angles, dt):
    """
    Returns the derivative of the angles at each frame.
    """
    
    velocities = []
    for i in range(len(angles)-1):
        derivative = [(angles[i+1][j] - angles[i][j])/dt for j in range(len(angles[i]))] * (2*pi/60) # Angular vel [rpm]
        velocities.append(derivative)
    return velocities

print(test_leg()[3])