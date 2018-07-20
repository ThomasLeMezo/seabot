#!/bin/python3
from copy import copy, deepcopy
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph.console
from pyqtgraph.dockarea import *
import datetime
from PyQt4.QtCore import QTime, QTimer

from scipy import signal, interpolate
import numpy as np
import matplotlib.pyplot as plt

import sys
from load_data import *

if(len(sys.argv)<2):
	sys.exit(0)

load_bag(sys.argv[1])

tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*np.pi

 #####################################################
 ### PLOT

app = QtGui.QApplication([])
win = QtGui.QMainWindow()
area = DockArea()
win.setCentralWidget(area)
win.showMaximized()	
win.setWindowTitle("Seabot log - " + sys.argv[1])

#################### Sensor Internal ####################

P = np.array(sensor_internal_pressure)*1e2
Ti = np.array(sensor_internal_temperature) + 273.15

f_dV = interpolate.interp1d(time_piston_state, piston_state_position, bounds_error=False)
f_Te = interpolate.interp1d(time_sensor_external, sensor_external_temperature, bounds_error=False)
dV = np.array(f_dV(time_sensor_internal))*tick_to_volume
Te = np.array(f_Te(time_sensor_internal)) + 273.15

P1 = np.mean(P[0:10])
T1 = np.mean(Ti[0:10])
T = Ti

# P[5000:7000] = 850.0e2

dock_pression_temp = Dock("PV=nRT")
area.addDock(dock_pression_temp)

pg_internal_pressure = pg.PlotWidget()
pg_internal_pressure.addLegend()
pg_internal_pressure.plot(time_sensor_internal, (dV*P/T)/(P/T - P1/T1), pen=(255,0,0), name="pressure")
pg_internal_pressure.setLabel('left', "Pressure/Temp")
dock_pression_temp.addWidget(pg_internal_pressure)

pg_internal_pressure2 = pg.PlotWidget()
pg_internal_pressure2.addLegend()
pg_internal_pressure2.plot(time_sensor_internal, (P/T - P1/T1), pen=(255,0,0), name="pressure")
pg_internal_pressure2.setLabel('left', "Pressure/Temp diff")
dock_pression_temp.addWidget(pg_internal_pressure2)

# pg_fusion_depth = pg.PlotWidget()
# pg_fusion_depth.addLegend()
# pg_fusion_depth.plot(time_fusion_depth, fusion_depth, pen=(255,0,0), name="depth")
# pg_fusion_depth.plot(time_regulation_depth_set_point, regulation_depth_set_point, pen=(0,255,0), name="set point")
# pg_fusion_depth.setLabel('left', "Depth", units="m")
# dock_pression_temp.addWidget(pg_fusion_depth)

pg_piston_position = pg.PlotWidget()
pg_piston_position.addLegend()
pg_piston_position.plot(time_piston_state, piston_state_position, pen=(255,0,0), name="piston_position")
pg_piston_position.setLabel('left', "piston_position", units="")
dock_pression_temp.addWidget(pg_piston_position)

pg_internal_pressure2.setXLink(pg_internal_pressure)
pg_piston_position.setXLink(pg_internal_pressure)

###################################################

# dock_pressure = Dock("Pressure")
# area.addDock(dock_pressure, 'above', dock_pression_temp)

# pg_internal_pressure = pg.PlotWidget()
# pg_internal_pressure.addLegend()
# pg_internal_pressure.plot(time_sensor_internal, sensor_internal_pressure, pen=(255,0,0), name="pressure")
# pg_internal_pressure.setLabel('left', "Pressure")
# dock_pressure.addWidget(pg_internal_pressure)

win.show()
	
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

