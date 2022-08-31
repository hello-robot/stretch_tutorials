#!/usr/bin/env python3

# Import needed python packages
import numpy as np
import os
import csv
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import animation

class Plotter():
    """
    Class the plots stored data from the effort sensing node.
    """
    def __init__(self,animate=False):
        """
        Function that stores the dataframe.
        :param self: The self reference.
        :param animate: Boolean to determine animation plotting.
        """
        # Create path to save effort and position values
        dir_path = '/home/hello-robot/catkin_ws/src/stretch_tutorials/stored_data'

        ####################### Copy the file name here! #######################
        file_name = '2022-06-30_11:26:20-AM'

        # Complete name of directory to pull from
        self.completeName = os.path.join(dir_path, file_name)

        # Store dataframe
        self.data = pd.read_csv(self.completeName)

        # Create empty list for animation
        self.y_anim = []

        # Set self.animate to boolean argument
        self.animate = animate

    def plot_data(self):
        """
        Function that plots dataframe
        :param self: The self reference.
        """
        # Utililze a forloop to print each joint's effort
        for joint in self.data.columns:

            # Create figure, labels, and title
            fig = plt.figure()
            plt.title(joint + ' Effort Sensing')
            plt.ylabel('Effort')
            plt.xlabel('Data Points')

            # Conditional statement for animation plotting
            if self.animate:
                self.effort = self.data[joint]
                frames = len(self.effort)-1
                anim = animation.FuncAnimation(fig=fig,func=self.plot_animate, repeat=False,blit=False,frames=frames, interval =75)
                plt.show()

                ## If you want to save a video, make sure to comment out plt.show(),
                ## right before this comment.
                # save_path = str(self.completeName + '.mp4')
                # anim.save(save_path, writer=animation.FFMpegWriter(fps=10))

                # Reset y_anim for the next joint effort animation
                del self.y_anim[:]

            # Conditional statement for regular plotting (No animation)
            else:
                self.data[joint].plot(kind='line')
                # save_path = str(self.completeName + '.png')
                # plt.savefig(save_path, bbox_inches='tight')
                plt.show()

    def plot_animate(self,i):
        """
        Function that plots every increment of the dataframe.
        :param self: The self reference.
        :param i: index value.
        """
        # Append self.effort values for given joint
        self.y_anim.append(self.effort.values[i])
        plt.plot(self.y_anim, color='blue')

if __name__ == '__main__':
    # Instanstiate a `Plotter()` object and execute the `plot_data()` method
    viz = Plotter(animate=True)
    viz.plot_data()
