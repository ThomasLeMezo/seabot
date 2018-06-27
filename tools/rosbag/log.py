#!/bin/python3
from copy import copy, deepcopy
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph.console
from pyqtgraph.dockarea import *


from scipy import signal
import numpy as np
import matplotlib.pyplot as plt

from load_data import *

load_bag('bag/2018-06-27-18-28-07.bag')

 #####################################################
 ### PLOT

app = QtGui.QApplication([])
win = QtGui.QMainWindow()
area = DockArea()
win.setCentralWidget(area)
win.showMaximized()	
win.setWindowTitle('Seabot log')

dock_battery = Dock("Battery")
dock_piston = Dock("Piston")
dock_depth = Dock("Depth")
dock_internal_sensor = Dock("Internal Sensor")
dock_external_sensor = Dock("External Sensor")
dock_regulation1 = Dock("Regulation 1")
dock_regulation2 = Dock("Regulation 2")
dock_regulation3 = Dock("Regulation 3")

area.addDock(dock_battery)
area.addDock(dock_piston, 'above', dock_battery)
area.addDock(dock_depth, 'above', dock_battery)
area.addDock(dock_internal_sensor, 'above', dock_battery)
area.addDock(dock_external_sensor, 'above', dock_battery)
area.addDock(dock_regulation1, 'above', dock_battery)
area.addDock(dock_regulation2, 'above', dock_battery)
area.addDock(dock_regulation3, 'above', dock_battery)

#################### Regulation 1 ####################
pg_regulation_u = pg.PlotWidget()
pg_regulation_u.plot(time_regulation_debug, regulation_debug_u, pen=(255,0,0))
pg_regulation_u.setLabel('left', "u")
dock_regulation1.addWidget(pg_regulation_u)

pg_regulation_set_point = pg.PlotWidget()
pg_regulation_set_point.plot(time_regulation_debug, regulation_debug_piston_set_point, pen=(255,0,0))
pg_regulation_set_point.plot(time_piston_state, np.array(piston_state_position)-regulation_debug_piston_set_point_offset[0], pen=(0,0,255))
pg_regulation_set_point.setLabel('left', "set point")
dock_regulation1.addWidget(pg_regulation_set_point)

pg_regulation_depth = pg.PlotWidget()
pg_regulation_depth.plot(time_fusion_depth, fusion_depth, pen=(255,0,0))
pg_regulation_depth.setLabel('left', "Depth")
dock_regulation1.addWidget(pg_regulation_depth)

pg_regulation_set_point.setXLink(pg_regulation_u)
pg_regulation_depth.setXLink(pg_regulation_u)

#################### Regulation 2 ####################

pg_regulation_depth_error = pg.PlotWidget()
pg_regulation_depth_error.plot(time_regulation_debug, regulation_debug_depth_error, pen=(255,0,0))
pg_regulation_depth_error.setLabel('left', "Depth error")
dock_regulation2.addWidget(pg_regulation_depth_error)

pg_regulation_depth1 = pg.PlotWidget()
pg_regulation_depth1.plot(time_fusion_depth, fusion_depth, pen=(255,0,0))
pg_regulation_depth1.setLabel('left', "Depth")
dock_regulation2.addWidget(pg_regulation_depth1)

pg_regulation_depth1.setXLink(pg_regulation_depth_error)

#################### Regulation 3 ####################

pg_regulation_depth_error1 = pg.PlotWidget()
pg_regulation_depth_error1.plot(time_regulation_debug, regulation_debug_depth_error, pen=(255,0,0))
pg_regulation_depth_error1.setLabel('left', "Depth error factor")
dock_regulation3.addWidget(pg_regulation_depth_error1)

pg_regulation_velocity = pg.PlotWidget()
pg_regulation_velocity.plot(time_regulation_debug, regulation_debug_velocity, pen=(255,0,0))
pg_regulation_velocity.setLabel('left', "Velocity factor")
dock_regulation3.addWidget(pg_regulation_velocity)

pg_regulation_acceleration = pg.PlotWidget()
pg_regulation_acceleration.plot(time_regulation_debug, regulation_debug_acceleration, pen=(255,0,0))
pg_regulation_acceleration.setLabel('left', "Acceleration factor")
dock_regulation3.addWidget(pg_regulation_acceleration)

pg_regulation_velocity.setXLink(pg_regulation_depth_error1)
pg_regulation_acceleration.setXLink(pg_regulation_depth_error1)

#################### Fusion ####################

# plt.subplot(311)
# plt.ylabel("sensor_external_pressure")
# plt.plot(time_sensor_external,sensor_external_pressure)

# plt.subplot(312)
# plt.ylabel("fusion_depth")
# plt.plot(time_fusion_depth,fusion_depth)

# plt.subplot(313)
# plt.ylabel("fusion_velocity")
# plt.plot(time_fusion_depth,fusion_velocity)

#################### Battery ####################

pg_battery = pg.PlotWidget(title="Battery")
pg_battery.plot(time_battery, battery1, pen=(255,0,0), name="Battery 1")
pg_battery.plot(time_battery, battery2, pen=(0,255,0), name="Battery 2")
pg_battery.plot(time_battery, battery3, pen=(0,0,255), name="Battery 3")
pg_battery.plot(time_battery, battery4, pen=(255,0,255), name="Battery 4")
dock_battery.addWidget(pg_battery)

#################### Sensor Internal ####################

pg_internal_pressure = pg.PlotWidget()
pg_internal_pressure.plot(time_sensor_internal, sensor_internal_pressure, pen=(255,0,0))
pg_internal_pressure.setLabel('left', "Pressure")
dock_internal_sensor.addWidget(pg_internal_pressure)

pg_internal_temperature = pg.PlotWidget()
pg_internal_temperature.plot(time_sensor_internal, sensor_internal_temperature, pen=(255,0,0))
pg_internal_temperature.setLabel('left', "Temperature")
dock_internal_sensor.addWidget(pg_internal_temperature)

pg_internal_humidity = pg.PlotWidget()
pg_internal_humidity.plot(time_sensor_internal, sensor_internal_humidity, pen=(255,0,0))
pg_internal_humidity.setLabel('left', "Humidity")
dock_internal_sensor.addWidget(pg_internal_humidity)

pg_internal_temperature.setXLink(pg_internal_pressure)
pg_internal_humidity.setXLink(pg_internal_pressure)

#################### Sensor External ####################

pg_external_pressure = pg.PlotWidget()
pg_external_pressure.plot(time_sensor_external, sensor_external_pressure, pen=(255,0,0), symbol='+')
pg_external_pressure.setLabel('left', "Pressure")
dock_external_sensor.addWidget(pg_external_pressure)

pg_external_temperature = pg.PlotWidget()
pg_external_temperature.plot(time_sensor_external, sensor_external_temperature, pen=(255,0,0), symbol='+')
pg_external_temperature.setLabel('left', "Temperature")
dock_external_sensor.addWidget(pg_external_temperature)

pg_external_temperature.setXLink(pg_external_pressure)

#################### Depth ####################

pg_fusion_depth = pg.PlotWidget()
pg_fusion_depth.plot(time_fusion_depth, fusion_depth, pen=(255,0,0))
pg_fusion_depth	.setLabel('left', "Depth", units="m")
dock_depth.addWidget(pg_fusion_depth)

pg_piston_state_position2 = pg.PlotWidget()
pg_piston_state_position2.plot(time_piston_state, piston_state_position, pen=(255,0,0))
pg_piston_state_position2.setLabel('left', "Piston state position")
dock_depth.addWidget(pg_piston_state_position2)

pg_piston_state_position2.setXLink(pg_fusion_depth)

#################### Piston ####################

pg_piston_state_position = pg.PlotWidget()
pg_piston_state_position.plot(time_piston_state, piston_state_position, pen=(255,0,0))
pg_piston_state_position.plot(time_piston_state, piston_state_position_set_point, pen=(0,0,255))
pg_piston_state_position.setLabel('left', "Piston state position and set point")
dock_piston.addWidget(pg_piston_state_position)

pg_piston_switch = pg.PlotWidget()
pg_piston_switch.plot(time_piston_state, np.array(piston_state_switch_out).astype(int), pen=(255,0,0))
pg_piston_switch.plot(time_piston_state, np.array(piston_state_switch_in).astype(int), pen=(0,0,255))
pg_piston_switch.setLabel('left', "Switch")
dock_piston.addWidget(pg_piston_switch)

pg_piston_switch.setXLink(pg_piston_state_position)

#################### Pose (GPS) ####################

# w0 = pg.PlotWidget(title="Pose (Lambert)")
# w0.plot(fusion_pose_x, fusion_pose_y, pen=(255,0,0), name="Red curve")
# d0.addWidget(w0)

###################################################

win.show()
	
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

