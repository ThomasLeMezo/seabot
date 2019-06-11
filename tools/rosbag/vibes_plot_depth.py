#!/bin/python2.7
# from copy import copy, deepcopy

import sys
from load_data import *

from pyibex import *
from vibes import vibes
import numpy as np
from math import *

if(len(sys.argv)<2):
	sys.exit(0)

load_bag(sys.argv[1])
g = 9.81
m = 9.045
rho = 1025.0
d_flange = 0.24
d_piston = 0.05
Cf = pi*(d_flange/2.0)**2
screw_thread = 1.75e-3
tick_per_turn = 48
piston_full_volume = 1.718e-4 # m3
tick_to_volume = (screw_thread/tick_per_turn)*((d_piston/2.0)**2)*np.pi


print("Data has been loaded")
file_directory = "/home/lemezoth/workspaceQT/tikz-adapter/tikz/figs/svg/"

offset = 0.
x_min = -1. # for display
x_max = 4368
y_min = -0.2
y_max = 20.

def newFigure(name):
	vibes.newFigure(name)
	vibes.setFigureProperties({'x':0, 'y':0, 'width':1024, 'height':500, 'viewbox':'equal'})
	vibes.axisLimits(x_min-offset, x_max+offset, y_min-offset, y_max+offset)
	vibes.drawBox(x_min-offset, x_max+offset, y_min-offset, y_max+offset, "white[white]")

data = np.transpose(np.vstack([time_fusion_depth, fusion_depth]))
data_ref = np.transpose(np.vstack([time_mission, mission_depth]))

data_piston = np.transpose(np.vstack([time_piston_state, np.array(piston_state_position)*tick_to_volume*1e6]))

vibes.beginDrawing()
newFigure("Depth regulation")
vibes.drawLine(data.tolist(), "black")
vibes.drawLine(data_ref.tolist(), "red")
vibes.axisAuto()
vibes.drawLine([[0,0.3],[4360,0.3]])
vibes.saveImage(file_directory + 'float_regulation.svg')

y_min = -1.*tick_to_volume*1e6
y_max = 2400.*tick_to_volume*1e6
newFigure("Piston state")
vibes.drawLine(data_piston.tolist(), "black")
vibes.axisAuto()
vibes.saveImage(file_directory + 'float_regulation_piston.svg')


###
ti=280*5
tf=3500*5
data_chi = np.transpose(np.vstack([time_kalman[ti:tf], np.array(kalman_chi[ti:tf])*1e6]))
data_offset = np.transpose(np.vstack([time_kalman[ti:tf], np.array(kalman_offset[ti:tf])*1e6]))
# data_chi_cov = np.transpose(np.vstack([time_kalman[ti:tf], np.array(kalman_cov_chi[ti:tf])]))
# data_offset_cov = np.transpose(np.vstack([time_kalman[ti:tf], np.array(kalman_cov_offset[ti:tf])]))

#t = 280 -> 3640
x_min = ti/5.-1. # for display
x_max = tf/5.+1

y_min = 240.*tick_to_volume*1e6
y_max = 400.*tick_to_volume*1e6
newFigure("Offset")
vibes.drawLine(data_offset.tolist(), "black")
# vibes.axisAuto()
vibes.saveImage(file_directory + 'float_offset.svg')

y_min = -60.*tick_to_volume*1e6
y_max = 70.*tick_to_volume*1e6
newFigure("Chi")
vibes.drawLine(data_chi.tolist(), "black")
# vibes.axisAuto()
vibes.saveImage(file_directory + 'float_chi.svg')

# newFigure("Offset_Cov")
# y_min = -60.*tick_to_volume*1e6
# y_max = 70.*tick_to_volume*1e6
# vibes.drawLine(data_chi_cov.tolist(), "black")
# vibes.axisAuto()
# vibes.saveImage(file_directory + 'float_offset_cov.svg')


vibes.endDrawing()


