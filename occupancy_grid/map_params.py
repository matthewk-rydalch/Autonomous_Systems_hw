import numpy as np

class MapParams:
    def __init__(self):
        self.alpha = 1
        self.beta = 5
        self.z_max = 150

        self.xlim = 100
        self.ylim = 100
        self.hit = .65
        self.nohit = .35

        self.map0 = 0.5

        #he plots the true world, and then fills the map in another color.
        #He also animates the movement of the robot
        #There are a few cells where the robot jumps
