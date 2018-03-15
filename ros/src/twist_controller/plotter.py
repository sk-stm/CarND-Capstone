"""A class to show plots of measures.  Based on matplotlib/pyplot.
"""
import matplotlib.pyplot as plt
import time

class Plotter:
    """Class to show a window with multiple plots (based on matplotlib)

    Attributes:
        fig:      Reference to figure
        subplots: Dictionary of subplots, indexed by name
        lines:    Dictionary of lines, indexed by subplot_name, line_name
    """
    def __init__(self):
        """Initiatize the figure and attributes.
        """
        plt.ion()
        self.fig = plt.figure()
        self.subplots = {}
        self.lines = {}
        plt.show()

    def add_subplot(self, pos, sp_name, xlabel='', ylabel='', sp_title=''):
        """Add a subplot.
        """
        self.subplots[sp_name] = self.fig.add_subplot(pos)
        self.subplots[sp_name].set_title(sp_title)
        self.subplots[sp_name].set_xlabel(xlabel)
        self.subplots[sp_name].set_ylabel(ylabel)
        self.subplots[sp_name].grid()
        #self.fig.tight_layout()

    def add_line(self, sp_name, l_name, xlst, ylst, symb, label=''):
        """Add a line to a subplot. After all the lines have been
        added, call the draw() method of this class.
        """
        splot = self.subplots[sp_name]
        line, = splot.plot(xlst, ylst, symb, label=label)
        self.lines[sp_name, l_name] = line

    def add_legends(self):
        """Add legends to a subplot
        """
        for sp_name in self.subplots:
            splot = self.subplots[sp_name]
            splot.legend(loc="upper left")
            
    def set_line(self, sp_name, l_name, xlst, ylst):
        """Set an existing line's data to a new set of values.
        After setting all the lines to new data, call the draw()
        method of this class.
        """
        splot = self.subplots[sp_name]
        line = self.lines[sp_name, l_name]
        line.set_xdata(xlst)
        line.set_ydata(ylst)
        splot.relim()
        splot.autoscale_view()

    def rem_line(self, sp_name, l_name):
        """Remove a line from a subplot. Call draw() method after removing
        the lines that you want to remove.
        """
        splot = self.subplots[sp_name]
        line = self.lines[sp_name, l_name]
        line.remove()
        splot.relim()
        splot.autoscale_view()

    def draw(self):
        """wrapper to include plt.pause() to allow chart to update
        """
        plt.draw()
        plt.pause(0.000001)

def test():
    """Simple test code"""
    plotter = Plotter()
    plotter.add_subplot(211, 'sp1', 'x val', 'y val', 'subplot 1')
    plotter.add_subplot(212, 'sp2', 'x', 'y', 'subplot 2')
    x = range(50)
    y1 = [i*2 for i in x]
    y2 = [i*i for i in x]
    y3 = [i*4 for i in x]
    y4 = [i*i/3 +2*i for i in x]
    plotter.add_line('sp1', 'l1', x, y1, 'r-', 'i*2')
    plotter.add_line('sp1', 'l2', x, y2, 'g--', 'i*i')
    plotter.add_line('sp2', 'l3', x, y3, 'r^--', 'i*4')
    plotter.add_line('sp2', 'l4', x, y4, 'go-', 'i*i/3 +2*i')
    plotter.add_legends()
    plotter.draw()
    for i in range(50, 60):
        if len(x) > 50:
            x.pop(0)
            y1.pop(0)
            y2.pop(0)
            y3.pop(0)
            y4.pop(0)

        x.append(i)
        y1.append(i*2)
        y2.append(i*i)
        y3.append(i*4)
        y4.append(i*i/3 + 2*i)
        plotter.set_line('sp1', 'l1', x, y1)
        plotter.set_line('sp1', 'l2', x, y2)
        plotter.set_line('sp2', 'l3', x, y3)
        plotter.set_line('sp2', 'l4', x, y4)
        plotter.draw()

    plotter.rem_line('sp1', 'l1')
    plotter.draw()
    time.sleep(10)

if __name__ == '__main__':
    test()
