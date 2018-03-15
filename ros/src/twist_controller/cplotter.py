"""Plotter specifically designed for the car, built on top of Plotter.
"""
from plotter import Plotter
import random
import time
import math

class Cplotter:
    """Plotter for Car.
    """
    def __init__(self):
        # Create a plotter
        self.plotter = Plotter()
        # For now just 1 subplot, may add more later
        self.plotter.add_subplot(111, "speed", "sample", "vel", "Speed")
        # Lists of values used for plotting
        self.pxl = [0]
        self.pdsl = [0]
        self.psl = [0]

        # Add the lines to the plots
        self.plotter.add_line("speed", "des_speed", self.pxl, self.pdsl, "r-", label="des")
        self.plotter.add_line("speed", "act_speed", self.pxl, self.psl, "g--", label="act")

        # Add legends
        self.plotter.add_legends()
        self.plotter.draw()
        # Set a max number of points, and batch size of points to drop
        self.maxplotv = 5000
        self.pltvdrop = 25

    def clear_pl(self):
        """Clear the plot data"""
        self.pxl = [0]
        self.pdsl = [0]
        self.psl = [0]

    def add_plpoint(self, pds, ps):
        """Add a set of plotter data points
        pds: desired speed
        ps: actual speed
        """
        self.pxl.append(self.pxl[-1]+1)
        self.pdsl.append(pds)
        self.psl.append(ps)
        if len(self.pxl) > self.maxplotv:
            self.pxl = self.pxl[self.pltvdrop:]
            self.pdsl = self.pdsl[self.pltvdrop:]
            self.psl = self.psl[self.pltvdrop:]

    def update(self):
        # Update the plots with the current data
        self.plotter.set_line("speed", "des_speed", self.pxl, self.pdsl)
        self.plotter.set_line("speed", "act_speed", self.pxl, self.psl)
        self.plotter.draw()

if __name__ == '__main__':
    def test():
        """Simple test code"""
        cplotter = Cplotter()
        dspeed = random.uniform(0.0, 100.0)
        speed = random.uniform(0.0, 100.0)
        for i in range(0,25):
            cplotter.add_plpoint(dspeed, speed)
            cplotter.update()
        time.sleep(10)
    
    test()
