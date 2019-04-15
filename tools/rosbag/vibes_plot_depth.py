#!/bin/python2.7
# from copy import copy, deepcopy

import sys
from load_data import *

from pyibex import *
from vibes import vibes
import numpy as np
import math

if(len(sys.argv)<2):
	sys.exit(0)

load_bag(sys.argv[1])

print("Data has been loaded")
file_directory = "/home/lemezoth/workspaceQT/tikz-adapter/tikz/figs/svg/"

offset = 0.
x_min = -1 # for display
x_max = 8200
y_min = -0.2
y_max = 2.

def newFigure(name):
	vibes.newFigure(name)
	vibes.setFigureProperties({'x':0, 'y':0, 'width':1024, 'height':500, 'viewbox':'equal'})
	vibes.axisLimits(x_min-offset, x_max+offset, y_min-offset, y_max+offset)
	vibes.drawBox(x_min-offset, x_max+offset, y_min-offset, y_max+offset, "white[white]")

data = np.transpose(np.vstack([time_fusion_depth, fusion_depth]))
data_ref = np.transpose(np.vstack([time_mission, mission_depth]))

data_piston = np.transpose(np.vstack([time_piston_state, piston_state_position]))

vibes.beginDrawing()
newFigure("Depth regulation")
vibes.drawLine(data.tolist(), "black")
vibes.drawLine(data_ref.tolist(), "red")
vibes.saveImage(file_directory + 'float_regulation.svg')


x_min = -1 # for display
x_max = 8200
y_min = 1600
y_max = 1950
newFigure("Piston state")
vibes.drawLine(data_piston.tolist(), "black")
vibes.saveImage(file_directory + 'float_regulation_piston.svg')

vibes.endDrawing()


