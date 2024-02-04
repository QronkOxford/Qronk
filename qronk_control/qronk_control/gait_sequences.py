""" 
gait_sequences.py: generates the gait sequences for the robot. 
"""

from numpy import pi, cos, sin

# * We will create a class for each type of gait sequence
class SemicircleCrawl:

    def __init__(self, period=100, swing_phase_ratio=0.25, radius=2.5):
        """
        Args:
            period (int, optional): Period of the gait sequence. Defaults to 100.
            swing_phase_ratio (float, optional): Ratio of the period that the swing phase occupies. Defaults to 0.25.
            radius (float, optional): Radius of the semicircle. Defaults to 2.5.
        """

        self.period = period
        self.swing_phase_ratio = swing_phase_ratio
        self.radius = radius


    def trajectory(self):
        """
        Generates a semicircle trajectory starting at (0,0,0).

        Returns:
            list: List of coordinates for the trajectory. 
        """

        swing_time = self.period * self.swing_phase_ratio
        ground_time = self.period * (1 - self.swing_phase_ratio)
        coords = []

        for t in range(self.period):
            if t < swing_time:
                x = self.radius - self.radius * cos(pi * t / swing_time)
                y = 0
                z = self.radius * sin(pi * t / swing_time)
            else:
                x = self.radius + self.radius - 2 * self.radius * (t - swing_time) / ground_time
                y = 0
                z = 0
            coords.append((x, y, z))

        return coords

    # * Create a gait method that returns the list of foot coordinates for each foot.
    # * The joint angles will also have to be found by importing the kinematics module. 
    # * Consider where the origin (the location of the foot when the sequence starts) should be. 
    # * Also consider whether we should also find the angular velocity here. 
    def gait(self):
        pass