import numpy as np
from numpy import *
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d
import csv

with open("jointreaction_Un-named analysis._ReactionLoads.sto") as jrfile:

    reader = csv.reader(jrfile, delimiter=',', quotechar='|')
    i=0
    forces = []
    for row in reader:
        if i == 15:
            forces.append(row)
        i += 1

    fig = plt.figure(figsize=(15,15))
    ax = fig.add_subplot(111, projection='3d')

    print(float(forces[0][1]) ,float(forces[0][2]),float(forces[0][3]))
    ax.quiver(0.0, 0.0 ,0.0, float(forces[0][1]) ,float(forces[0][2]),float(forces[0][3]), color = 'red', alpha = .8, lw = 3)

    plt.title('force')

    plt.draw()
    plt.show()