#!/bin/python2.7
# from copy import copy, deepcopy

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

print("Data has been loaded")

class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super(TimeAxisItem, self).__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        return [int2dt(value).strftime("%Hh%Mmn%Ss") for value in values]

def int2dt(ts):
    # if not ts:
    #     return datetime.datetime.utcfromtimestamp(ts) # workaround fromtimestamp bug (1)
    return(datetime.datetime.fromtimestamp(ts-3600.0))

#####################################################
### PLOT

app = QtGui.QApplication([])
win = QtGui.QMainWindow()
win.showMaximized() 
win.setWindowTitle("Seabot log - " + sys.argv[1])

tab = QtGui.QTabWidget()
win.setCentralWidget(tab)

area = DockArea()
area_safety = DockArea()
area_data = DockArea()
area_piston = DockArea()
area_iridium = DockArea()
area_position = DockArea()

tab.addTab(area_safety, "Safety")
tab.addTab(area_data, "Data")
tab.addTab(area_piston, "Piston")
tab.addTab(area_iridium, "Iridium")
tab.addTab(area_position, "Position")


#################### Safety ####################

#### Battery ####
if(len(time_battery)>0):
    dock_battery = Dock("Battery")
    area_safety.addDock(dock_battery)

    pg_battery = pg.PlotWidget(title="Battery")
    pg_battery.addLegend()

    if(len(time_fusion_battery)>0):
        pg_battery.plot(time_fusion_battery, fusion_battery1, pen=(255,0,0), name="Battery 1")
        pg_battery.plot(time_fusion_battery, fusion_battery2, pen=(0,255,0), name="Battery 2")
        pg_battery.plot(time_fusion_battery, fusion_battery3, pen=(0,0,255), name="Battery 3")
        pg_battery.plot(time_fusion_battery, fusion_battery4, pen=(255,0,255), name="Battery 4")
        pg_battery.setLabel('left', "Tension (Fusion)", units="V")
    else:
        pg_battery.plot(time_battery, battery1, pen=(255,0,0), name="Battery 1")
        pg_battery.plot(time_battery, battery2, pen=(0,255,0), name="Battery 2")
        pg_battery.plot(time_battery, battery3, pen=(0,0,255), name="Battery 3")
        pg_battery.plot(time_battery, battery4, pen=(255,0,255), name="Battery 4")
        pg_battery.setLabel('left', "Tension (sensor)", units="V")
    dock_battery.addWidget(pg_battery)

#### Safety #### 
if(len(time_safety)>0):
    dock_safety = Dock("Safety", 'above', dock_battery)
    area_safety.addDock(dock_safety)
    pg_safety = pg.PlotWidget()
    pg_safety.addLegend()
    pg_safety.plot(time_safety, safety_published_frequency, pen=(255,0,0), name="published_frequency")
    pg_safety.plot(time_safety, safety_depth_limit, pen=(0,255,0), name="depth_limit")
    pg_safety.plot(time_safety, safety_batteries_limit, pen=(0,0,255), name="batteries_limit")
    pg_safety.plot(time_safety, safety_depressurization, pen=(255,255,0), name="depressurization")
    dock_safety.addWidget(pg_safety)

    if(len(time_safety_debug)>0):
        pg_safety_debug_flash = pg.PlotWidget()
        pg_safety_debug_flash.addLegend()
        pg_safety_debug_flash.plot(time_safety_debug, safety_debug_flash, pen=(255,0,0), name="flash")
        dock_safety.addWidget(pg_safety_debug_flash)
        pg_safety_debug_flash.setXLink(pg_safety)

#### Safety Debug ####

if(len(time_safety_debug)>0):
    dock_safety_debug = Dock("Safety Debug", 'above')
    area_safety.addDock(dock_safety_debug)

    pg_safety_debug_ratio = pg.PlotWidget()
    pg_safety_debug_ratio.addLegend()
    pg_safety_debug_ratio.plot(time_safety_debug, safety_debug_ratio_p_t, pen=(255,0,0), name="ratio_p_t")
    dock_safety_debug.addWidget(pg_safety_debug_ratio)

    pg_safety_debug_ratio_delta = pg.PlotWidget()
    pg_safety_debug_ratio_delta.addLegend()
    pg_safety_debug_ratio_delta.plot(time_safety_debug, safety_debug_ratio_delta, pen=(255,0,0), name="delta ratio_p_t")
    dock_safety_debug.addWidget(pg_safety_debug_ratio_delta)

    pg_safety_debug_volume = pg.PlotWidget()
    pg_safety_debug_volume.addLegend()
    pg_safety_debug_volume.plot(time_safety_debug, safety_debug_volume, pen=(255,0,0), name="volume")
    dock_safety_debug.addWidget(pg_safety_debug_volume)

    pg_safety_debug_volume_delta = pg.PlotWidget()
    pg_safety_debug_volume_delta.addLegend()
    pg_safety_debug_volume_delta.plot(time_safety_debug, safety_debug_volume_delta, pen=(255,0,0), name="delta volume")
    dock_safety_debug.addWidget(pg_safety_debug_volume_delta)

    pg_safety_debug_ratio_delta.setXLink(pg_safety_debug_ratio)
    pg_safety_debug_volume.setXLink(pg_safety_debug_ratio)
    pg_safety_debug_volume_delta.setXLink(pg_safety_debug_ratio)

#################### Regulation 1 ####################

if(len(time_regulation_debug)>0):
    dock_regulation1 = Dock("Regulation 1")
    area.addDock(dock_regulation1)

    pg_regulation_depth = pg.PlotWidget()
    pg_regulation_depth.addLegend()
    pg_regulation_depth.plot(time_fusion_depth, fusion_depth, pen=(255,0,0), name="depth")
    pg_regulation_depth.setLabel('left', "Depth", units="m")
    dock_regulation1.addWidget(pg_regulation_depth)

    pg_regulation_u = pg.PlotWidget()
    pg_regulation_u.addLegend()
    pg_regulation_u.plot(time_regulation_debug, regulation_u, pen=(255,0,0), name="u")
    pg_regulation_u.setLabel('left', "u")
    dock_regulation1.addWidget(pg_regulation_u)

    pg_regulation_set_point = pg.PlotWidget()
    pg_regulation_set_point.addLegend()
    pg_regulation_set_point.plot(time_regulation_debug, regulation_piston_set_point, pen=(255,0,0), name="set_point")
    pg_regulation_set_point.setLabel('left', "set_point")
    dock_regulation1.addWidget(pg_regulation_set_point)

    pg_regulation_u.setXLink(pg_regulation_depth)
    pg_regulation_set_point.setXLink(pg_regulation_depth)

#################### Sensor Internal ####################
if(len(time_sensor_internal)>0):
    dock_internal_sensor = Dock("Internal Sensor")
    area.addDock(dock_internal_sensor)

    if(len(time_fusion_sensor_internal)>0):
        pg_internal_pressure = pg.PlotWidget()
        pg_internal_pressure.addLegend()
        pg_internal_pressure.plot(time_fusion_sensor_internal, sensor_fusion_internal_pressure, pen=(255,0,0), name="pressure")
        pg_internal_pressure.setLabel('left', "Pressure [fusion]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_pressure)
    else:
        pg_internal_pressure = pg.PlotWidget()
        pg_internal_pressure.addLegend()
        pg_internal_pressure.plot(time_sensor_internal, sensor_internal_pressure, pen=(255,0,0), name="pressure")
        pg_internal_pressure.setLabel('left', "Pressure [sensor]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_pressure)

    if(len(time_fusion_sensor_internal)>0):
        pg_internal_temperature = pg.PlotWidget()
        pg_internal_temperature.addLegend()
        pg_internal_temperature.plot(time_fusion_sensor_internal, sensor_fusion_internal_temperature, pen=(255,0,0), name="temperature")
        pg_internal_temperature.setLabel('left', "Temperature [fusion]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_temperature)
    else:
        pg_internal_temperature = pg.PlotWidget()
        pg_internal_temperature.addLegend()
        pg_internal_temperature.plot(time_sensor_internal, sensor_internal_temperature, pen=(255,0,0), name="temperature")
        pg_internal_temperature.setLabel('left', "Temperature [sensor]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_temperature)

    pg_internal_humidity = pg.PlotWidget()
    pg_internal_humidity.addLegend()
    pg_internal_humidity.plot(time_sensor_internal, sensor_internal_humidity, pen=(255,0,0), name="humidity")
    pg_internal_humidity.setLabel('left', "Humidity")
    dock_internal_sensor.addWidget(pg_internal_humidity)

    pg_internal_temperature.setXLink(pg_internal_pressure)
    pg_internal_humidity.setXLink(pg_internal_pressure)

#################### Sensor External ####################
if(len(time_sensor_external)>0):
    dock_external_sensor = Dock("External Sensor")
    area.addDock(dock_external_sensor)
    pg_external_pressure = pg.PlotWidget()
    pg_external_pressure.addLegend()
    pg_external_pressure.plot(time_sensor_external, sensor_external_pressure, pen=(255,0,0), name="pressure")
    pg_external_pressure.setLabel('left', "Pressure", units="bar")
    dock_external_sensor.addWidget(pg_external_pressure)

    pg_external_temperature = pg.PlotWidget()
    pg_external_temperature.addLegend()
    pg_external_temperature.plot(time_sensor_external, sensor_external_temperature, pen=(255,0,0), name="temperature")
    pg_external_temperature.setLabel('left', "Temperature", units="C")
    dock_external_sensor.addWidget(pg_external_temperature)

    pg_external_temperature.setXLink(pg_external_pressure)

#################### Depth ####################
if(len(time_fusion_depth)>0):
    dock_depth = Dock("Depth")
    area.addDock(dock_depth)
    pg_fusion_depth = pg.PlotWidget()
    pg_fusion_depth.addLegend()
    pg_fusion_depth.plot(time_fusion_depth, fusion_depth, pen=(255,0,0), name="depth")
    pg_fusion_depth.plot(time_regulation_depth_set_point, regulation_depth_set_point, pen=(0,255,0), name="set point")
    pg_fusion_depth.setLabel('left', "Depth", units="m")
    dock_depth.addWidget(pg_fusion_depth)

    pg_piston_state_position2 = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')})
    pg_piston_state_position2.addLegend()
    pg_piston_state_position2.plot(time_piston_state, piston_state_position, pen=(255,0,0), name="position")
    pg_piston_state_position2.plot(time_piston_state, piston_state_position_set_point, pen=(0,0,255), name="set point")
    pg_piston_state_position2.setLabel('left', "Piston state position")
    # pg_piston_state_position2.setLabel('bottom', "Time", units="s")
    dock_depth.addWidget(pg_piston_state_position2)

    pg_piston_state_position2.setXLink(pg_fusion_depth)

#################### Temperature ####################
if(len(time_sensor_temperature)>0):
    dock_temperature = Dock("Temperature")
    area.addDock(dock_temperature)
    pg_sensor_temperature = pg.PlotWidget()
    pg_sensor_temperature.addLegend()
    pg_sensor_temperature.plot(time_sensor_temperature, sensor_temperature, pen=(255,0,0), name="temperature")
    pg_sensor_temperature.setLabel('left', "Temperature", units="C")
    dock_temperature.addWidget(pg_sensor_temperature)

#################### Piston ####################
if(len(time_piston_state)>0):
    dock_piston = Dock("Piston")
    area.addDock(dock_piston)
    pg_piston_state_position = pg.PlotWidget()
    pg_piston_state_position.addLegend()
    pg_piston_state_position.plot(time_piston_state, piston_state_position, pen=(255,0,0), name="position")
    pg_piston_state_position.plot(time_piston_state, piston_state_position_set_point, pen=(0,0,255), name="set point")
    pg_piston_state_position.setLabel('left', "Piston state position and set point")
    dock_piston.addWidget(pg_piston_state_position)

    pg_piston_switch = pg.PlotWidget()
    pg_piston_switch.addLegend()
    pg_piston_switch.plot(time_piston_state, np.array(piston_state_switch_out).astype(int), pen=(255,0,0), name="out")
    pg_piston_switch.plot(time_piston_state, np.array(piston_state_switch_in).astype(int), pen=(0,0,255), name="in")
    pg_piston_switch.setLabel('left', "Switch")
    dock_piston.addWidget(pg_piston_switch)

    pg_piston_state = pg.PlotWidget()
    pg_piston_state.addLegend()
    pg_piston_state.plot(time_piston_state, piston_state_state, pen=(0,255,0), name="state")
    pg_piston_state.setLabel('left', "State")
    dock_piston.addWidget(pg_piston_state)    

    pg_piston_switch.setXLink(pg_piston_state_position)
    pg_piston_state.setXLink(pg_piston_state_position)

#################### Piston2 ####################
if(len(time_piston_state)>0):
    dock_piston2 = Dock("Piston2")
    area.addDock(dock_piston2)
    pg_piston_state_position2 = pg.PlotWidget()
    pg_piston_state_position2.addLegend()
    pg_piston_state_position2.plot(time_piston_state, piston_state_position, pen=(255,0,0), name="position pic")
    pg_piston_state_position2.plot(time_piston_state, piston_state_position_set_point, pen=(0,0,255), name="set point pic")
    pg_piston_state_position2.plot(time_piston_position, piston_position, pen=(0,255,0), name="set point pi")
    pg_piston_state_position2.setLabel('left', "Piston state position and set point")
    dock_piston2.addWidget(pg_piston_state_position2)

    pg_piston_speed = pg.PlotWidget()
    pg_piston_speed.addLegend()
    pg_piston_speed.plot(time_piston_state, np.array(piston_state_motor_speed).astype(int), pen=(255,0,0), name="speed")
    pg_piston_speed.setLabel('left', "Speed")
    dock_piston2.addWidget(pg_piston_speed)

    pg_piston_velocity = pg.PlotWidget()
    pg_piston_velocity.addLegend()
    pg_piston_velocity.plot(time_piston_velocity, piston_velocity, pen=(255,0,0), name="velocity")
    pg_piston_velocity.setLabel('left', "Velocity")
    dock_piston2.addWidget(pg_piston_velocity)

    pg_piston_speed.setXLink(pg_piston_state_position2)
    pg_piston_velocity.setXLink(pg_piston_state_position2)

#################### Fusion ####################
if(len(time_fusion_depth)>0):
    dock_fusion = Dock("Fusion")
    area.addDock(dock_fusion)
    pg_fusion_depth1 = pg.PlotWidget()
    pg_fusion_depth1.addLegend()
    pg_fusion_depth1.plot(time_fusion_depth, fusion_depth, pen=(255,0,0), name="depth")
    pg_fusion_depth1.setLabel('left', "Depth", units="m")
    dock_fusion.addWidget(pg_fusion_depth1)

    pg_fusion_velocity = pg.PlotWidget()
    pg_fusion_velocity.addLegend()
    pg_fusion_velocity.plot(time_fusion_depth, fusion_velocity, pen=(255,0,0), name="velocity")
    pg_fusion_velocity.setLabel('left', "Velocity", units="m/s")
    dock_fusion.addWidget(pg_fusion_velocity)

    pg_fusion_velocity.setXLink(pg_fusion_depth1)

#################### GPS Status ####################
if(len(time_fix)>0):
    dock_gps = Dock("GPS Signal")
    area.addDock(dock_gps)
    pg_gps = pg.PlotWidget()
    pg_gps.addLegend()
    pg_gps.plot(time_fix, fix_status, pen=(255,0,0), name="status")
    pg_gps.setLabel('left', "Fix status")
    dock_gps.addWidget(pg_gps)

    pg_fusion_depth2 = pg.PlotWidget()
    pg_fusion_depth2.addLegend()
    pg_fusion_depth2.plot(time_fusion_depth, fusion_depth, pen=(255,0,0), name="depth")
    pg_fusion_depth2.setLabel('left', "Depth", units="m")
    dock_gps.addWidget(pg_fusion_depth2)

    pg_gps.setXLink(pg_fusion_depth2)

#################### GPS Pose ####################
if(len(fusion_pose_east)>0):
    dock_gps2 = Dock("GPS Pose")
    area.addDock(dock_gps2)
    pg_gps2 = pg.PlotWidget()
    pg_gps2.addLegend()
    Y = np.array(fusion_pose_north)
    X = np.array(fusion_pose_east)
    X = X[~np.isnan(X)]
    Y = Y[~np.isnan(Y)]
    X -= np.mean(X)
    Y -= np.mean(Y)
    pg_gps2.plot(X, Y, pen=(255,0,0), name="pose (centered on mean)")
    pg_gps2.setLabel('left', "Y", units="m")
    pg_gps2.setLabel('bottom', "X", units="m")
    dock_gps2.addWidget(pg_gps2)

#################### Mag ####################
if(len(time_mag)>0):
    dock_mag = Dock("Mag")
    area.addDock(dock_mag)
    pg_mag1 = pg.PlotWidget()
    pg_mag1.addLegend()
    pg_mag1.plot(time_mag, mag_x, pen=(255,0,0), name="mag x")
    pg_mag1.plot(time_mag, mag_y, pen=(0,255,0), name="mag y")
    pg_mag1.plot(time_mag, mag_z, pen=(0,0,255), name="mag z")
    dock_mag.addWidget(pg_mag1)

    pg_mag2 = pg.PlotWidget()
    pg_mag2.addLegend()
    mag_N = np.sqrt(np.power(np.array(mag_x), 2)+np.power(np.array(mag_y), 2)+np.power(np.array(mag_z), 2))
    pg_mag2.plot(time_mag, mag_N, pen=(255,0,0), name="mag N")
    dock_mag.addWidget(pg_mag2)

    pg_mag2.setXLink(pg_mag1)

#################### Euler ####################
if(len(time_euler)>0):
    dock_euler = Dock("Euler")
    area.addDock(dock_euler)
    pg_euler1 = pg.PlotWidget()
    pg_euler1.addLegend()
    # pg_euler1.plot(time_euler, euler_x, pen=(255,0,0), name="euler x")
    # pg_euler1.plot(time_euler, euler_y, pen=(0,255,0), name="euler y")
    pg_euler1.plot(time_euler, np.array(euler_z)*180.0/np.pi, pen=(0,0,255), name="Heading")
    dock_euler.addWidget(pg_euler1)

    # pg_euler_depth = pg.PlotWidget()
    # pg_euler_depth.addLegend()
    # pg_euler_depth.plot(time_fusion_depth, fusion_depth, pen=(255,0,0), name="depth")
    # pg_euler_depth.setLabel('left', "Depth", units="m")
    # dock_euler.addWidget(pg_euler_depth)

#################### Kalman ####################
if(len(time_kalman)>0):
    dock_kalman = Dock("Kalman")
    area.addDock(dock_kalman)

    pg_kalman_velocity = pg.PlotWidget()
    pg_kalman_velocity.addLegend()
    pg_kalman_velocity.plot(time_kalman, kalman_velocity, pen=(255,0,0), name="velocity")
    dock_kalman.addWidget(pg_kalman_velocity)

    pg_kalman_depth = pg.PlotWidget()
    pg_kalman_depth.addLegend()
    pg_kalman_depth.plot(time_kalman, kalman_depth, pen=(255,0,0), name="depth")
    dock_kalman.addWidget(pg_kalman_depth)

    pg_kalman_volume = pg.PlotWidget()
    pg_kalman_volume.addLegend()
    pg_kalman_volume.plot(time_kalman, kalman_volume, pen=(255,0,0), name="volume")
    dock_kalman.addWidget(pg_kalman_volume)

    pg_kalman_offset = pg.PlotWidget()
    pg_kalman_offset.addLegend()
    pg_kalman_offset.plot(time_kalman, kalman_offset, pen=(255,0,0), name="offset")
    dock_kalman.addWidget(pg_kalman_offset)

    # pg_kalman_error_velocity = pg.PlotWidget()
    # pg_kalman_error_velocity.addLegend()
    # pg_kalman_error_velocity.plot(time_kalman, kalman_error_velocity, pen=(255,0,0), name="error velocity")
    # dock_kalman.addWidget(pg_kalman_error_velocity)

    pg_kalman_depth.setXLink(pg_kalman_velocity)
    pg_kalman_volume.setXLink(pg_kalman_velocity)
    pg_kalman_offset.setXLink(pg_kalman_velocity)
    # pg_kalman_error_velocity.setXLink(pg_kalman_velocity)


#################### Temperature / Depth ####################
if(len(time_sensor_temperature)>0 and len(time_fusion_depth)>0):
    dock_temp = Dock("T/depth")
    area.addDock(dock_temp)
    if(len(fusion_depth)>0):
        pg_temp = pg.PlotWidget()
        pg_temp.addLegend()
        
        f_temp = interpolate.interp1d(time_sensor_temperature, sensor_temperature, bounds_error=False)
        f_depth = interpolate.interp1d(time_fusion_depth, fusion_depth, bounds_error=False)

        time_interp = np.linspace(time_fusion_depth[0], time_fusion_depth[-1], 50000)
        temperature_interp = f_temp(time_interp)
        depth_interp = f_depth(time_interp)

        pg_temp.plot(temperature_interp, depth_interp, pen=(255,0,0), name="Temperature")
        pg_temp.setLabel('left', "Depth")
        pg_temp.setLabel('bottom', "Temperature")
        
        pg_temp.getViewBox().invertY(True)
        dock_temp.addWidget(pg_temp)

###################################################

win.show()
	
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

