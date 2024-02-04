"""
kinematics.py: contains the forward and inverse kinematics functions of the robot.
"""

import numpy as np
from numpy import cos, sin, sqrt, arctan2, arccos

# TODO: Currently, the lengths are approximate values and need to be updated with the actual lengths of the robot.
# Define lengths of the links in the leg (cm)
B = 0  # body to hip
L1 = 3.5  # hip to upper-leg
L2 = 10  # upper-leg to lower-leg
L3 = 10  # lower-leg to foot


def forwardKinematics(side, angles):
    """
    Calculates the forward kinematics of the robot.
    Input: side (str "left" or "right"), angles (list of joint angles in radians in the form [th1, th2, th3])
    Output: JointPos (list of joint positions in the form [J1, J2, J3, Je])

    For more information on the derivation of the forward kinematics, follow the link below.
    https://www.rosroboticslearning.com/forward-kinematics
    """

    if side not in ["left", "right"]:
        raise ValueError("Invalid side. Must be 'right' or 'left'.")

    th1, th2, th3 = angles

    # Body to hip
    T01_translation = np.array([[1, 0, 0, B], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    T01_rotation = np.array(
        [
            [1, 0, 0, 0],
            [0, cos(th1), -sin(th1), 0],
            [0, sin(th1), cos(th1), 0],
            [0, 0, 0, 1],
        ]
    )
    T01 = T01_translation @ T01_rotation

    # Hip to upper-leg
    T12_translation = np.array(
        [
            [1, 0, 0, 0],
            [0, 1, 0, (L1 if side == "left" else -L1)],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ]
    )
    T12_rotation = np.array(
        [
            [cos(th2), 0, sin(th2), 0],
            [0, 1, 0, 0],
            [-sin(th2), 0, cos(th2), 0],
            [0, 0, 0, 1],
        ]
    )
    T12 = T12_translation @ T12_rotation

    # Upper-leg to lower-leg
    T23_translation = np.array(
        [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -L2], [0, 0, 0, 1]]
    )
    T23_rotation = np.array(
        [
            [cos(th3), 0, sin(th3), 0],
            [0, 1, 0, 0],
            [-sin(th3), 0, cos(th3), 0],
            [0, 0, 0, 1],
        ]
    )
    T23 = T23_translation @ T23_rotation

    # Lower-leg to foot
    T3e = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, -L3], [0, 0, 0, 1]])

    # Combine matrices to get end-effector pose
    T01 = T01
    T02 = T01 @ T12
    T03 = T01 @ T12 @ T23
    T0e = T01 @ T12 @ T23 @ T3e

    # Get joint positions
    J1 = T01[0:3, 3]
    J2 = T02[0:3, 3]
    J3 = T03[0:3, 3]
    Je = T0e[0:3, 3]

    JointPos = [J1, J2, J3, Je]
    return JointPos


def inverseKinematics(side, Je):
    """
    Calculates the inverse kinematics of the robot.
    Input: side (str "left" or "right"), Je (end effector position in the form [x, y, z])
    Output: JointAngles (list of joint angles in radians in the form [th1, th2, th3])

    For more information on the derivation of the inverse kinematics, see /notebooks/kinematics.ipynb
    """

    if side not in ["left", "right"]:
        raise ValueError("Invalid side. Must be 'right' or 'left'.")

    # Extract the x, y, z coordinates of the end effector position
    x, y, z = Je
    y = -y if side == "right" else y

    # Find th1 by considering the front view of the leg
    d1yz = sqrt(y**2 + z**2)
    a1 = abs(arccos(L1 / d1yz))
    a2 = arctan2(z, y)
    th1 = a1 + a2
    th1 = -th1 if side == "right" else th1

    # Find the required lengths from the 3D view
    d1xyz = sqrt(x**2 + y**2 + z**2)
    d2 = sqrt(d1xyz**2 - L1**2)
    x_A, y_A, z_A = 0, L1 * cos(th1), L1 * sin(th1)

    # Find the required angle by considering the side view of the leg
    a3 = arctan2((x_A - x), (z_A - z))

    # Find th2 th3 by considering the plane of lower/upper leg
    a4 = (d2**2 + L2**2 - L3**2) / (2 * d2 * L2)
    th2 = a3 + a4
    th3 = -2 * a4

    JointAngles = [th1, th2, th3]
    return JointAngles
