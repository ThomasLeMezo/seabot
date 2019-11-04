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

from numpy import linalg as LA

import sys
from load_data import *

import math

class TimeAxisItem(pg.AxisItem):
    def __init__(self, *args, **kwargs):
        super(TimeAxisItem, self).__init__(*args, **kwargs)

    def tickStrings(self, values, scale, spacing):
        return [int2dt(value).strftime("%Hh%Mmn%Ss") for value in values]

def int2dt(ts):
    # if not ts:
    #     return datetime.datetime.utcfromtimestamp(ts) # workaround fromtimestamp bug (1)
    return(datetime.datetime.fromtimestamp(ts-3600.0))


def save_gpx():
    import gpxpy
    import gpxpy.gpx

    gpx = gpxpy.gpx.GPX()
    last_fix_time = 0.

    gpx_track = gpxpy.gpx.GPXTrack()
    gpx_segment = gpxpy.gpx.GPXTrackSegment()

    for i in range(len(fixData.latitude)):
        if(abs(last_fix_time-fixData.time[i])>30.):
            if(fixData.status[i]==3):
                last_fix_time = fixData.time[i]

                gpx_segment.points.append(gpxpy.gpx.GPXTrackPoint(latitude=fixData.latitude[i], 
                    longitude=fixData.longitude[i],
                    elevation=fixData.altitude[i],
                    time=datetime.datetime.fromtimestamp(fixData.time[i]+startTime),
                    horizontal_dilution=fixData.hdop[i],
                    vertical_dilution=fixData.hdop[i]
                    ))
    gpx_track.segments.append(gpx_segment)
    gpx.tracks.append(gpx_track)

    file = open(filename+".gpx","w") 
    file.write(gpx.to_xml()) 
    file.close()


#####################################################
### Init

if ('filename' in locals()):
    print("filename = ", filename)
else:
    if(len(sys.argv)<2):
        sys.exit(0)
    filename = sys.argv[1]

pistonStateData = PistonStateData()
pistonSetPointData = PistonSetPointData()
imuData = ImuData()
magData = MagData()
eulerData = EulerData()
pistonVelocityData = PistonVelocityData()
pistonDistanceData = PistonDistanceData()
pistonSpeedData = PistonSpeedData()
batteryData = BatteryData()
sensorExtData = SensorExtData()
sensorIntData = SensorIntData()
engineData = EngineData()
engineCmdData = EngineCmdData()
fixData = FixData()
temperatureData = TemperatureData()
batteryFusionData = BatteryFusionData()
sensorIntFusionData = SensorIntFusionData()
depthFusionData = DepthFusionData()
poseFusionData = PoseFusionData()
kalmanData = KalmanData()
regulationData = RegulationData()
regulationHeadingData = RegulationHeadingData()
regulationHeadingSetPointData = RegulationHeadingSetPointData()
missionData = MissionData()
safetyData = SafetyData()
safetyDebugData = SafetyDebugData()
iridiumStatusData = IridiumStatusData()
iridiumSessionData = IridiumSessionData()
regulationWaypointData = RegulationWaypointData()
rosoutData = RosoutData()
rosoutAggData = RosoutAggData()

load_bag(filename, rosoutData, rosoutAggData, pistonStateData, pistonSetPointData, imuData, magData, eulerData, pistonVelocityData, pistonDistanceData, pistonSpeedData, batteryData, sensorExtData, sensorIntData, engineData, engineCmdData, fixData, temperatureData, batteryFusionData, sensorIntFusionData, depthFusionData, poseFusionData, kalmanData, regulationData, regulationHeadingData, regulationHeadingSetPointData, missionData, safetyData, safetyDebugData, iridiumStatusData, iridiumSessionData, regulationWaypointData)

print("Data has been loaded")

tick_to_volume = (1.75e-3/48.0)*((0.05/2.0)**2)*np.pi

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
area_regulation = DockArea()
area_iridium = DockArea()
area_position = DockArea()
area_waypoints = DockArea()

tab.addTab(area_safety, "Safety")
tab.addTab(area_data, "Data")
tab.addTab(area_piston, "Piston")
tab.addTab(area_regulation, "Regulation")
tab.addTab(area_iridium, "Iridium")
tab.addTab(area_position, "GNSS")
tab.addTab(area_waypoints, "Waypoints")

#################### Standard plot ####################

def set_plot_options(p):
    p.addLegend()
    # p.setAutoVisible(y=True)

def plot_depth(dock):
    pg_depth = pg.PlotWidget()
    set_plot_options(pg_depth)
    pg_depth.plot(depthFusionData.time, depthFusionData.depth[:-1], pen=(255,0,0), name="depth", stepMode=True)
    if(np.size(missionData.time)>0):
        pg_depth.plot(missionData.time, missionData.depth[:-1], pen=(0,255,0), name="set point", stepMode=True)
    pg_depth.setLabel('left', "Depth", units="m")
    pg_depth.showGrid(y=True)
    dock.addWidget(pg_depth)
    return pg_depth

def plot_piston_position(dock):
    pg_position = pg.PlotWidget()
    set_plot_options(pg_position)
    pg_position.plot(pistonStateData.time, pistonStateData.position[:-1],pen=(255,0,0), name="position", stepMode=True)
    pg_position.plot(pistonStateData.time, pistonStateData.position_set_point[:-1],pen=(0,0,255), name="set point (pic)", stepMode=True)
    pg_position.setLabel('left', "Piston state position and set point")
    pg_position.showGrid(y=True)
    dock.addWidget(pg_position)
    return pg_position

def plot_regulation_state(dock):
    pg_regulation_state = pg.PlotWidget()
    set_plot_options(pg_regulation_state)
    pg_regulation_state.plot(np.array(regulationData.time), np.array(regulationData.mode[:-1]), pen=(255,0,0), name="mode",stepMode=True)
    pg_regulation_state.setLabel('left', "mode")
    dock.addWidget(pg_regulation_state)

    tab = np.array(regulationData.mode)
    for i in np.where(tab[:-1] != tab[1:])[0]:
        text_write_plot(pg_regulation_state, regulationData.time[i+1], tab[i+1], regulation_state[tab[i+1]])
    return pg_regulation_state

def plot_regulation_waypoint_heading(dock):
    pg_heading = pg.PlotWidget()
    set_plot_options(pg_heading)
    pg_heading.plot(regulationWaypointData.time, 2.*np.arctan(np.tan(regulationWaypointData.yaw_set_point[:-1]/2.)), pen=(255,0,0), name="set_point", stepMode=True)
    pg_heading.plot(eulerData.time, eulerData.z[:-1], pen=(0,255,0), name="yaw", stepMode=True)
    pg_heading.setLabel('left', "heading")
    dock.addWidget(pg_heading)

    return pg_heading

def text_write_reset(p, t):
    text = pg.TextItem(html='<div style="text-align: left"><span style="color: #FFF;">RESET</span></div>', anchor=(-0.3,1.3),border='w', fill=(0, 0, 255, 100))
    p.addItem(text)
    text.setPos(t, 0)
    arrow = pg.ArrowItem(pos=(t, 0), angle=-45)
    p.addItem(arrow)

regulation_state = {
    0: "Surface",
    1: "Sink",
    2: "Regulation",
    3: "Stationary",
    4: "Emergency",
    5: "Piston issue"
}

safety_warning = {
    0: "published_frequency",
    1: "depth_limit",
    2: "batteries_limit",
    3: "depressurization",
    4: "seafloor"
}

# def text_write_plot(p, t, mode):
#     text = pg.TextItem(html='<div style="text-align: left"><span style="color: #FFF;">'+regulation_state[mode]+'</span></div>', anchor=(-0.3,1.3),border='w', fill=(0, 0, 255, 100))
#     p.addItem(text)
#     text.setPos(t, mode)
#     arrow = pg.ArrowItem(pos=(t, mode), angle=-45)
#     p.addItem(arrow)

def text_write_plot(p, t, x, msg):
    text = pg.TextItem(html='<div style="text-align: left"><span style="color: #FFF;">'+msg+'</span></div>', anchor=(-0.3,1.3),border='w', fill=(0, 0, 255, 100))
    p.addItem(text)
    text.setPos(t, x)
    arrow = pg.ArrowItem(pos=(t, x), angle=-45)
    p.addItem(arrow)

def text_write_safety_msg(p, t, msg, data):
    tab = np.array(data)
    for i in np.where(tab[:-1] != tab[1:])[0]:
        if(tab[i+1]==1):
            text_write_plot(p, t[i+1], tab[i+1], msg)

#################### Safety ####################

#### Battery ####
if(len(batteryData.time)>0):
    dock_battery = Dock("Battery")
    area_safety.addDock(dock_battery)

    pg_battery = pg.PlotWidget(title="Battery")
    set_plot_options(pg_battery)

    if(len(batteryFusionData.time)>0):
        pg_battery.plot(batteryFusionData.time, batteryFusionData.b1[:-1], pen=(255,0,0), name="Battery 1", stepMode=True)
        pg_battery.plot(batteryFusionData.time, batteryFusionData.b2[:-1], pen=(0,255,0), name="Battery 2", stepMode=True)
        pg_battery.plot(batteryFusionData.time, batteryFusionData.b3[:-1], pen=(0,0,255), name="Battery 3", stepMode=True)
        pg_battery.plot(batteryFusionData.time, batteryFusionData.b4[:-1], pen=(255,0,255), name="Battery 4", stepMode=True)
        pg_battery.setLabel('left', "Tension (Fusion)", units="V")
    else:
        pg_battery.plot(batteryData.time, batteryData.b1[:-1], pen=(255,0,0), name="Battery 1", stepMode=True)
        pg_battery.plot(batteryData.time, batteryData.b2[:-1], pen=(0,255,0), name="Battery 2", stepMode=True)
        pg_battery.plot(batteryData.time, batteryData.b3[:-1], pen=(0,0,255), name="Battery 3", stepMode=True)
        pg_battery.plot(batteryData.time, batteryData.b4[:-1], pen=(255,0,255), name="Battery 4", stepMode=True)
        pg_battery.setLabel('left', "Tension (sensor)", units="V")
    dock_battery.addWidget(pg_battery)

#### Safety Debug ####
if(len(safetyDebugData.time)>0):
    dock_safety_debug = Dock("Safety Debug")
    area_safety.addDock(dock_safety_debug, 'above', dock_battery)

    pg_safety_debug_ratio = pg.PlotWidget()
    set_plot_options(pg_safety_debug_ratio)
    pg_safety_debug_ratio.plot(safetyDebugData.time, safetyDebugData.ratio_p_t[:-1], pen=(255,0,0), name="ratio_p_t", stepMode=True)
    dock_safety_debug.addWidget(pg_safety_debug_ratio)

    pg_safety_debug_ratio_delta = pg.PlotWidget()
    set_plot_options(pg_safety_debug_ratio_delta)
    pg_safety_debug_ratio_delta.plot(safetyDebugData.time, safetyDebugData.ratio_delta[:-1], pen=(255,0,0), name="delta ratio_p_t", stepMode=True)
    dock_safety_debug.addWidget(pg_safety_debug_ratio_delta)

    pg_safety_debug_volume = pg.PlotWidget()
    set_plot_options(pg_safety_debug_volume)
    pg_safety_debug_volume.plot(safetyDebugData.time, safetyDebugData.volume[:-1], pen=(255,0,0), name="volume", stepMode=True)
    dock_safety_debug.addWidget(pg_safety_debug_volume)

    pg_safety_debug_volume_delta = pg.PlotWidget()
    set_plot_options(pg_safety_debug_volume_delta)
    pg_safety_debug_volume_delta.plot(safetyDebugData.time, safetyDebugData.volume_delta[:-1], pen=(255,0,0), name="delta volume", stepMode=True)
    dock_safety_debug.addWidget(pg_safety_debug_volume_delta)

    pg_safety_debug_ratio_delta.setXLink(pg_safety_debug_ratio)
    pg_safety_debug_volume.setXLink(pg_safety_debug_ratio)
    pg_safety_debug_volume_delta.setXLink(pg_safety_debug_ratio)

#### Safety #### 
if(len(safetyData.time)>0):
    dock_safety = Dock("Safety")
    area_safety.addDock(dock_safety, 'above', dock_battery)
    pg_safety = pg.PlotWidget()
    set_plot_options(pg_safety)
    pg_safety.plot(safetyData.time, safetyData.published_frequency[:-1], pen=(255,0,0), name="published_frequency", stepMode=True)
    pg_safety.plot(safetyData.time, safetyData.depth_limit[:-1], pen=(0,255,0), name="depth_limit", stepMode=True)
    pg_safety.plot(safetyData.time, safetyData.batteries_limit[:-1], pen=(0,0,255), name="batteries_limit", stepMode=True)
    pg_safety.plot(safetyData.time, safetyData.depressurization[:-1], pen=(255,255,0), name="depressurization", stepMode=True)
    pg_safety.plot(safetyData.time, safetyData.seafloor[:-1], pen=(255,255,150), name="seafloor", stepMode=True)
    text_write_safety_msg(pg_safety, safetyData.time, "published_frequency", safetyData.published_frequency)
    text_write_safety_msg(pg_safety, safetyData.time, "depth_limit", safetyData.depth_limit)
    text_write_safety_msg(pg_safety, safetyData.time, "batteries_limit", safetyData.batteries_limit)
    text_write_safety_msg(pg_safety, safetyData.time, "depressurization", safetyData.depressurization)
    text_write_safety_msg(pg_safety, safetyData.time, "seafloor", safetyData.seafloor)
    dock_safety.addWidget(pg_safety)

    if(len(safetyDebugData.time)>0):
        pg_safety_debug_flash = pg.PlotWidget()
        set_plot_options(pg_safety_debug_flash)
        pg_safety_debug_flash.plot(safetyDebugData.time, safetyDebugData.flash[:-1], pen=(255,0,0), name="flash", stepMode=True)
        dock_safety.addWidget(pg_safety_debug_flash)
        pg_safety_debug_flash.setXLink(pg_safety)

#################### Data ####################

#### Sensor Internal ####
if(len(sensorIntData.time)>0):
    dock_internal_sensor = Dock("Internal Sensor")
    area_data.addDock(dock_internal_sensor)

    if(len(sensorIntFusionData.time)>0):
        pg_internal_pressure = pg.PlotWidget()
        set_plot_options(pg_internal_pressure)
        pg_internal_pressure.plot(sensorIntFusionData.time, sensorIntFusionData.pressure[:-1], pen=(255,0,0), name="pressure", stepMode=True)
        pg_internal_pressure.setLabel('left', "Pressure [fusion]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_pressure)
    else:
        pg_internal_pressure = pg.PlotWidget()
        set_plot_options(pg_internal_pressure)
        pg_internal_pressure.plot(sensorIntData.time, sensorIntFusionData.pressure[:-1], pen=(255,0,0), name="pressure", stepMode=True)
        pg_internal_pressure.setLabel('left', "Pressure [sensor]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_pressure)

    if(len(sensorIntFusionData.time)>0):
        pg_internal_temperature = pg.PlotWidget()
        set_plot_options(pg_internal_temperature)
        pg_internal_temperature.plot(sensorIntFusionData.time, sensorIntFusionData.temperature[:-1], pen=(255,0,0), name="temperature", stepMode=True)
        pg_internal_temperature.setLabel('left', "Temperature [fusion]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_temperature)
    else:
        pg_internal_temperature = pg.PlotWidget()
        set_plot_options(pg_internal_temperature)
        pg_internal_temperature.plot(sensorIntData.time, sensorIntFusionData.temperature[:-1], pen=(255,0,0), name="temperature", stepMode=True)
        pg_internal_temperature.setLabel('left', "Temperature [sensor]", "mBar")
        dock_internal_sensor.addWidget(pg_internal_temperature)

    pg_internal_humidity = pg.PlotWidget()
    set_plot_options(pg_internal_humidity)
    pg_internal_humidity.plot(sensorIntFusionData.time, sensorIntFusionData.humidity[:-1], pen=(255,0,0), name="humidity", stepMode=True)
    pg_internal_humidity.setLabel('left', "Humidity")
    dock_internal_sensor.addWidget(pg_internal_humidity)

    pg_internal_temperature.setXLink(pg_internal_pressure)
    pg_internal_humidity.setXLink(pg_internal_pressure)

#### Temperature / Depth ####
if(len(temperatureData.time)>0 and len(depthFusionData.time)>0):
    dock_temp = Dock("T/depth")
    area_data.addDock(dock_temp, 'above', dock_internal_sensor)
    if(len(depthFusionData.depth)>0):
        pg_temp_depth = pg.PlotWidget()
        set_plot_options(pg_temp_depth)
        
        f_temp = interpolate.interp1d(temperatureData.time, temperatureData.temperature, bounds_error=False)
        f_depth = interpolate.interp1d(depthFusionData.time, depthFusionData.depth, bounds_error=False)

        time_interp = np.linspace(depthFusionData.time[0], depthFusionData.time[-1], 50000)
        temperature_interp = f_temp(time_interp)
        depth_interp = f_depth(time_interp)

        mask_temp = (temperature_interp>0)
        temp_mask = temperature_interp[mask_temp]
        depth_mask = depth_interp[mask_temp]
        time_mask = time_interp[mask_temp]

        plot_t_d = pg_temp_depth.plot(temp_mask, depth_mask[:-1], pen=(255,0,0), name="T/D", stepMode=True)
        pg_temp_depth.setDownsampling(mode='peak') # ToCheck
        pg_temp_depth.setLabel('left', "Depth")
        pg_temp_depth.setLabel('bottom', "Temperature")
        
        pg_temp_depth.getViewBox().invertY(True)
        dock_temp.addWidget(pg_temp_depth)

        pg_temp= pg.PlotWidget()
        set_plot_options(pg_temp)
        pg_temp.plot(time_mask, depth_mask[:-1], pen=(255,0,0), name="Depth", stepMode=True)
        dock_temp.addWidget(pg_temp)

        lr_TP = pg.LinearRegionItem([0, time_mask[-1]], bounds=[0,time_mask[-1]], movable=True)
        pg_temp.addItem(lr_TP)
        lr_TP_bounds = lr_TP.getRegion()

        def update_TP():
            global plot_t_d, temp_mask, depth_mask, lr_TP, lr_TP_bounds
            t_bounds = lr_TP.getRegion()
            if(t_bounds != lr_TP_bounds):
                lr_TP_bounds = t_bounds
                ub = np.where(time_mask <= np.max((1,t_bounds[1])))[0][-1]
                lb = np.where(time_mask >= np.min((time_mask[-1],t_bounds[0])))[0][0]

                ub = np.min((ub, np.size(time_mask)))
                lb = np.max((lb,0))
                plot_t_d.setData(temp_mask[lb:ub], depth_mask[lb:ub][:-1])

        timer = pg.QtCore.QTimer()
        timer.timeout.connect(update_TP)
        timer.start(50)

#### Sensor External ####
if(len(sensorExtData.time)>0):
    dock_external_sensor = Dock("External Sensor")
    area_data.addDock(dock_external_sensor, 'above', dock_internal_sensor)
    pg_external_pressure = pg.PlotWidget()
    set_plot_options(pg_external_pressure)
    pg_external_pressure.plot(sensorExtData.time, sensorExtData.pressure[:-1], pen=(255,0,0), name="pressure", stepMode=True)
    pg_external_pressure.setLabel('left', "Pressure", units="bar")
    dock_external_sensor.addWidget(pg_external_pressure)

    pg_external_temperature = pg.PlotWidget()
    set_plot_options(pg_external_temperature)
    pg_external_temperature.plot(sensorExtData.time, sensorExtData.temperature[:-1], pen=(255,0,0), name="temperature", stepMode=True)
    pg_external_temperature.setLabel('left', "Temperature", units="C")
    dock_external_sensor.addWidget(pg_external_temperature)

    pg_external_temperature.setXLink(pg_external_pressure)

#### Temperature ####
if(len(temperatureData.time)>0):
    dock_temperature = Dock("Temperature")
    area_data.addDock(dock_temperature, 'above', dock_internal_sensor)
    pg_sensor_temperature = pg.PlotWidget()
    set_plot_options(pg_sensor_temperature)
    pg_sensor_temperature.plot(temperatureData.time, temperatureData.temperature[:-1], pen=(255,0,0), name="temperature", stepMode=True)
    pg_sensor_temperature.setLabel('left', "Temperature", units="C")
    dock_temperature.addWidget(pg_sensor_temperature)

#### Fusion ####
if(len(depthFusionData.time)>0):
    dock_fusion = Dock("Fusion")
    area_data.addDock(dock_fusion, 'above', dock_internal_sensor)
    pg_depth = plot_depth(dock_fusion)

    pg_fusion_velocity = pg.PlotWidget()
    set_plot_options(pg_fusion_velocity)
    pg_fusion_velocity.plot(depthFusionData.time, depthFusionData.velocity[:-1], pen=(255,0,0), name="velocity", stepMode=True)
    pg_fusion_velocity.setLabel('left', "Velocity", units="m/s")
    dock_fusion.addWidget(pg_fusion_velocity)

    pg_fusion_velocity.setXLink(pg_depth)

#### Mag ####
if(len(magData.time)>0):
    dock_mag = Dock("Mag")
    area_data.addDock(dock_mag, 'above', dock_internal_sensor)
    pg_mag1 = pg.PlotWidget()
    set_plot_options(pg_mag1)
    pg_mag1.plot(magData.time, magData.x[:-1], pen=(255,0,0), name="mag x", stepMode=True)
    pg_mag1.plot(magData.time, magData.y[:-1], pen=(0,255,0), name="mag y", stepMode=True)
    pg_mag1.plot(magData.time, magData.z[:-1], pen=(0,0,255), name="mag z", stepMode=True)
    dock_mag.addWidget(pg_mag1)

    pg_mag2 = pg.PlotWidget()
    set_plot_options(pg_mag2)
    magData.N = np.sqrt(np.power(np.array(magData.x), 2)+np.power(np.array(magData.y), 2)+np.power(np.array(magData.z), 2))
    pg_mag2.plot(magData.time, magData.N[:-1], pen=(255,0,0), name="mag N", stepMode=True)
    dock_mag.addWidget(pg_mag2)

    pg_mag2.setXLink(pg_mag1)

#### Imu Acc ####
if(len(imuData.time)>0):
    dock_imu_acc = Dock("Acc")
    area_data.addDock(dock_imu_acc, 'above', dock_internal_sensor)
    pg_acc = pg.PlotWidget()
    set_plot_options(pg_acc)
    pg_acc.plot(imuData.time, imuData.acc_x[:-1], pen=(255,0,0), name="acc x", stepMode=True)
    pg_acc.plot(imuData.time, imuData.acc_y[:-1], pen=(0,255,0), name="acc y", stepMode=True)
    pg_acc.plot(imuData.time, imuData.acc_z[:-1], pen=(0,0,255), name="acc z", stepMode=True)
    dock_imu_acc.addWidget(pg_acc)

    pg_acc_n = pg.PlotWidget()
    set_plot_options(pg_acc_n)
    acc_n = LA.norm(np.array([imuData.acc_x, imuData.acc_y, imuData.acc_z]), axis=0)
    pg_acc_n.plot(imuData.time, acc_n[:-1], pen=(255,0,0), name="norm", stepMode=True)
    dock_imu_acc.addWidget(pg_acc_n)

    pg_acc_n.setXLink(pg_acc)

#### Imu Gyro ####
if(len(imuData.time)>0):
    dock_imu_gyro = Dock("Gyro")
    area_data.addDock(dock_imu_gyro, 'above', dock_internal_sensor)
    pg_gyro = pg.PlotWidget()
    set_plot_options(pg_gyro)
    pg_gyro.plot(imuData.time, imuData.gyro_x[:-1], pen=(255,0,0), name="gyro x", stepMode=True)
    pg_gyro.plot(imuData.time, imuData.gyro_y[:-1], pen=(0,255,0), name="gyro y", stepMode=True)
    pg_gyro.plot(imuData.time, imuData.gyro_z[:-1], pen=(0,0,255), name="gyro z", stepMode=True)
    dock_imu_gyro.addWidget(pg_gyro)

#### Euler ####
if(len(eulerData.time)>0):
    dock_euler = Dock("Euler")
    area_data.addDock(dock_euler, 'above', dock_internal_sensor)
    pg_euler = pg.PlotWidget()
    set_plot_options(pg_euler)
    pg_euler.plot(eulerData.time, eulerData.x[:-1], pen=(255,0,0), name="euler x", stepMode=True)
    pg_euler.plot(eulerData.time, eulerData.y[:-1], pen=(0,255,0), name="euler y", stepMode=True)
    pg_euler.plot(eulerData.time, eulerData.z[:-1], pen=(0,255,0), name="euler z", stepMode=True)
    dock_euler.addWidget(pg_euler)

#################### Piston ####################

#### Distance ####
if(len(pistonDistanceData.time)>0):
    dock_piston_distance = Dock("Distance")
    area_piston.addDock(dock_piston_distance)

    pg_depth = plot_depth(dock_piston_distance)

    pg_piston_distance = pg.PlotWidget(axisItems={'bottom': TimeAxisItem(orientation='bottom')})
    set_plot_options(pg_piston_distance)
    pg_piston_distance.plot(pistonDistanceData.time, pistonDistanceData.distance[:-1], pen=(0,0,255), name="distance", stepMode=True)
    pg_piston_distance.setLabel('left', "Piston distance travelled")
    dock_piston_distance.addWidget(pg_piston_distance)

    pg_piston_distance.setXLink(pg_depth)

#### Depth ####
if(len(depthFusionData.time)>0):
    dock_depth = Dock("Position")
    area_piston.addDock(dock_depth, 'above', dock_piston_distance)
    pg_depth = plot_depth(dock_depth)
    pg_piston = plot_piston_position(dock_depth)

    pg_piston.setXLink(pg_depth)

#### Piston ####
if(np.size(pistonStateData.time)>0):
    dock_piston = Dock("State")
    area_piston.addDock(dock_piston, 'above', dock_piston_distance)
    pg_piston = plot_piston_position(dock_piston)

    pg_piston_switch = pg.PlotWidget()
    set_plot_options(pg_piston_switch)
    pg_piston_switch.plot(pistonStateData.time, pistonStateData.switch_out[:-1].astype(int), pen=(255,0,0), name="out", stepMode=True)
    pg_piston_switch.plot(pistonStateData.time, pistonStateData.switch_in[:-1].astype(int), pen=(0,0,255), name="in", stepMode=True)
    pg_piston_switch.setLabel('left', "Switch")
    dock_piston.addWidget(pg_piston_switch)

    pg_piston_state = pg.PlotWidget()
    set_plot_options(pg_piston_state)
    pg_piston_state.plot(pistonStateData.time, pistonStateData.state[:-1], pen=(0,255,0), name="state", stepMode=True)
    pg_piston_state.setLabel('left', "State")

    tab = pistonStateData.state
    for i in np.where(tab[:-1] != tab[1:])[0]:
        if(tab[i]==1):
            text_write_reset(pg_piston_state, pistonStateData.time[i])

    dock_piston.addWidget(pg_piston_state)

    pg_piston_switch.setXLink(pg_piston)
    pg_piston_state.setXLink(pg_piston)

#### Piston Velocity ####
if(np.size(pistonStateData.time)>0):
    dock_piston2 = Dock("Velocity")
    area_piston.addDock(dock_piston2, 'above', dock_piston_distance)
    pg_piston2 = plot_piston_position(dock_piston)
    if(np.size(pistonSetPointData.time)>0):
        pg_piston2.plot(pistonSetPointData.time, pistonSetPointData.position[:-1], pen=(0,255,0), name="set point pi", stepMode=True)
        pg_piston2.setLabel('left', "Piston state position and set point")
    dock_piston2.addWidget(pg_piston2)

    pg_piston_speed = pg.PlotWidget()
    set_plot_options(pg_piston_speed)
    pg_piston_speed.plot(pistonStateData.time, pistonStateData.motor_speed[:-1], pen=(255,0,0), name="speed", stepMode=True)
    
    if(len(pistonSpeedData.speed_in)>1):
        pg_piston_speed.plot(pistonSpeedData.time, 50-np.array(pistonSpeedData.speed_in)[:-1], pen=(0,255,0), name="speed_max_in", stepMode=True)
        pg_piston_speed.plot(pistonSpeedData.time, 50+np.array(pistonSpeedData.speed_out)[:-1], pen=(0,255,0), name="speed_max_out", stepMode=True)
    pg_piston_speed.setLabel('left', "Speed")
    dock_piston2.addWidget(pg_piston_speed)

    pg_piston_velocity = pg.PlotWidget()
    set_plot_options(pg_piston_velocity)
    pg_piston_velocity.plot(pistonVelocityData.time, pistonVelocityData.velocity[:-1], pen=(0,0,255), name="velocity", stepMode=True)
    pg_piston_velocity.setLabel('left', "Velocity")
    dock_piston2.addWidget(pg_piston_velocity)

    pg_piston_speed.setXLink(pg_piston2)
    pg_piston_velocity.setXLink(pg_piston2)

#################### Regulation ####################

#### Regulation ####
if(len(regulationData.time)>0):
    dock_regulation = Dock("Regulation1")
    area_regulation.addDock(dock_regulation)

    pg_depth = plot_depth(dock_regulation)

    pg_regulation_velocity = pg.PlotWidget()
    set_plot_options(pg_regulation_velocity)
    pg_regulation_velocity.plot(kalmanData.time, kalmanData.velocity[:-1], pen=(255,0,0), name="velocity", stepMode=True)
    pg_regulation_velocity.plot(missionData.time, missionData.limit_velocity[:-1], pen=(0,255,0), name="target_velocity_max", stepMode=True)
    pg_regulation_velocity.plot(missionData.time, -np.array(missionData.limit_velocity[:-1]), pen=(0,255,0), name="target_velocity_min", stepMode=True)
    dock_regulation.addWidget(pg_regulation_velocity)

    pg_regulation_u = pg.PlotWidget()
    set_plot_options(pg_regulation_u)
    pg_regulation_u.plot(regulationData.time, regulationData.u[:-1], pen=(255,0,0), name="u", stepMode=True)
    pg_regulation_u.setLabel('left', "u")
    dock_regulation.addWidget(pg_regulation_u)

    pg_regulation_velocity.setXLink(pg_depth)
    pg_regulation_u.setXLink(pg_depth)

#### Regulation debug ####
if(len(regulationData.time)>0):
    dock_command = Dock("Command")
    area_regulation.addDock(dock_command, 'above', dock_regulation)

    pg_depth = plot_depth(dock_command)
    pg_regulation_state = plot_regulation_state(dock_command)

    pg_regulation_set_point = pg.PlotWidget()
    set_plot_options(pg_regulation_set_point)
    pg_regulation_set_point.plot(regulationData.time, regulationData.set_point[:-1], pen=(0,0,255), name="set_point", stepMode=True)
    pg_regulation_set_point.plot(pistonStateData.time, pistonStateData.position[:-1],pen=(255,0,0), name="position", stepMode=True)
    pg_regulation_set_point.setLabel('left', "set_point")
    dock_command.addWidget(pg_regulation_set_point)

    pg_regulation_state.setXLink(pg_depth)
    pg_regulation_set_point.setXLink(pg_depth)

#### Regulation2 ####
if(len(regulationData.time)>0):
    dock_regulation2 = Dock("Regulation2")
    area_regulation.addDock(dock_regulation2, 'above', dock_regulation)

    pg_depth = plot_depth(dock_regulation2)
    pg_regulation_state = plot_regulation_state(dock_regulation2)

    pg_regulation_u = pg.PlotWidget()
    set_plot_options(pg_regulation_u)
    pg_regulation_u.plot(regulationData.time, regulationData.u[:-1], pen=(255,0,0), name="u", stepMode=True)
    pg_regulation_u.setLabel('left', "u")
    dock_regulation2.addWidget(pg_regulation_u)

    pg_regulation_state.setXLink(pg_depth)
    pg_regulation_u.setXLink(pg_depth)

#### Regulation debug 2 ####
if(len(regulationData.time)>0):
    dock_regulation2 = Dock("Regulation (Detailed)")
    area_regulation.addDock(dock_regulation2, 'below', dock_regulation)

    pg_regulation_depth2 = pg.PlotWidget()
    set_plot_options(pg_regulation_depth2)
    pg_regulation_depth2.plot(depthFusionData.time, depthFusionData.depth[:-1], pen=(255,0,0), name="depth", stepMode=True)
    pg_regulation_depth2.plot(missionData.time, missionData.depth[:-1], pen=(0,255,0), name="depth", stepMode=True)
    pg_regulation_depth2.setLabel('left', "depth")
    dock_regulation2.addWidget(pg_regulation_depth2)

    pg_regulation_y = pg.PlotWidget()
    set_plot_options(pg_regulation_y)
    pg_regulation_y.plot(regulationData.time, regulationData.y[:-1], pen=(255,0,0), name="y", stepMode=True)
    pg_regulation_y.setLabel('left', "y")
    dock_regulation2.addWidget(pg_regulation_y)

    pg_regulation_dy = pg.PlotWidget()
    set_plot_options(pg_regulation_dy)
    pg_regulation_dy.plot(regulationData.time, regulationData.dy[:-1], pen=(255,0,0), name="dy", stepMode=True)
    pg_regulation_dy.setLabel('left', "dy")
    dock_regulation2.addWidget(pg_regulation_dy)

    pg_regulation_u = pg.PlotWidget()
    set_plot_options(pg_regulation_u)
    pg_regulation_u.plot(kalmanData.time, np.array(kalmanData.offset[:-1])-np.array(kalmanData.depth[:-1])*7.158e-07, pen=(255,0,0), name="volume equilibrium", stepMode=True)
    pg_regulation_u.setLabel('left', "volume equilibrium")
    dock_regulation2.addWidget(pg_regulation_u)

    pg_regulation_u.setXLink(pg_regulation_depth2)
    pg_regulation_dy.setXLink(pg_regulation_depth2)
    pg_regulation_y.setXLink(pg_regulation_depth2)

#### Regulation Mission ####
if(len(regulationData.time)>0):
    dock_regulation_mission = Dock("Mission")
    area_regulation.addDock(dock_regulation_mission, 'above', dock_regulation)

    pg_depth = plot_depth(dock_regulation_mission)
    pg_regulation_state = plot_regulation_state(dock_regulation_mission)

    pg_regulation_mission = pg.PlotWidget()
    set_plot_options(pg_regulation_mission)
    pg_regulation_mission.plot(missionData.time, missionData.limit_velocity[:-1], pen=(255,0,0), name="limit_velocity", stepMode=True)
    pg_regulation_mission.plot(missionData.time, missionData.approach_velocity[:-1], pen=(0,255,0), name="mission_approach_velocity", stepMode=True)
    dock_regulation_mission.addWidget(pg_regulation_mission)

    pg_regulation_state.setXLink(pg_depth)
    pg_regulation_mission.setXLink(pg_depth)

#### Regulation Heading ####

if(len(regulationHeadingData.time)>0):
    dock_regulation_heading = Dock("Regulation Heading")
    area_regulation.addDock(dock_regulation_heading, 'below', dock_regulation)

    pg_euler_yaw = pg.PlotWidget()
    set_plot_options(pg_euler_yaw)
    pg_euler_yaw.plot(eulerData.time, np.array(eulerData.z[:-1])*180./np.pi, pen=(255,0,0), name="Yaw", stepMode=True)
    pg_euler_yaw.plot(regulationHeadingData.time, np.array(regulationHeadingData.set_point[:-1])*180./np.pi, pen=(0,255,0), name="Set point", stepMode=True)
    dock_regulation_heading.addWidget(pg_euler_yaw)

    pg_regulation_heading_error = pg.PlotWidget()
    set_plot_options(pg_regulation_heading_error)
    pg_regulation_heading_error.plot(regulationHeadingData.time, regulationHeadingData.error[:-1], pen=(255,0,0), name="error", stepMode=True)
    dock_regulation_heading.addWidget(pg_regulation_heading_error)

    pg_regulation_heading_var = pg.PlotWidget()
    set_plot_options(pg_regulation_heading_var)
    pg_regulation_heading_var.plot(regulationHeadingData.time, regulationHeadingData.p_var[:-1], pen=(255,0,0), name="P", stepMode=True)
    pg_regulation_heading_var.plot(regulationHeadingData.time, regulationHeadingData.d_var[:-1], pen=(0,255,0), name="D", stepMode=True)
    dock_regulation_heading.addWidget(pg_regulation_heading_var)

    pg_regulation_heading_command = pg.PlotWidget()
    set_plot_options(pg_regulation_heading_command)
    pg_regulation_heading_command.plot(regulationHeadingData.time, regulationHeadingData.command[:-1], pen=(255,0,0), name="command", stepMode=True)
    pg_regulation_heading_command.plot(regulationHeadingData.time, regulationHeadingData.command_limit[:-1], pen=(0,255,0), name="command_limit", stepMode=True)
    dock_regulation_heading.addWidget(pg_regulation_heading_command)
    
    pg_regulation_heading_var.setXLink(pg_regulation_heading_error)
    pg_regulation_heading_command.setXLink(pg_regulation_heading_error)
    if(len(eulerData.time)>0):
        pg_euler_yaw.setXLink(pg_regulation_heading_error)

#### Kalman ####
if(len(kalmanData.time)>0 and len(regulationData.time)>0):
    dock_kalman = Dock("Kalman")
    area_regulation.addDock(dock_kalman, 'below', dock_regulation)

    pg_kalman_velocity = pg.PlotWidget()
    set_plot_options(pg_kalman_velocity)
    pg_kalman_velocity.plot(kalmanData.time, kalmanData.velocity[:-1], pen=(255,0,0), name="velocity (x1)", stepMode=True)
    dock_kalman.addWidget(pg_kalman_velocity)

    pg_kalman_depth = pg.PlotWidget()
    set_plot_options(pg_kalman_depth)
    pg_kalman_depth.plot(kalmanData.time, kalmanData.depth[:-1], pen=(255,0,0), name="depth (x2)", stepMode=True)
    dock_kalman.addWidget(pg_kalman_depth)

    pg_kalman_offset = pg.PlotWidget()
    set_plot_options(pg_kalman_offset)
    pg_kalman_offset.plot(kalmanData.time, np.array(kalmanData.offset[:-1])/tick_to_volume, pen=(255,0,0), name="offset (x4) [ticks]", stepMode=True)
    dock_kalman.addWidget(pg_kalman_offset)

    pg_kalman_chi = pg.PlotWidget()
    set_plot_options(pg_kalman_chi)
    pg_kalman_chi.plot(kalmanData.time, np.array(kalmanData.chi[:-1])/tick_to_volume, pen=(255,0,0), name="chi (x5) [ticks]", stepMode=True)
    dock_kalman.addWidget(pg_kalman_chi)

    pg_kalman_depth.setXLink(pg_kalman_velocity)
    pg_kalman_offset.setXLink(pg_kalman_velocity)
    pg_kalman_chi.setXLink(pg_kalman_velocity)

if(len(kalmanData.time)>0 and len(regulationData.time)>0):
    dock_kalman_cov = Dock("Kalman Cov")
    area_regulation.addDock(dock_kalman_cov, 'below', dock_kalman)

    pg_kalman_cov_depth = pg.PlotWidget()
    set_plot_options(pg_kalman_cov_depth)
    pg_kalman_cov_depth.plot(kalmanData.time,kalmanData.cov_depth[:-1], pen=(255,0,0), name="cov depth", stepMode=True)
    # pg_kalman_cov_depth.setAutoVisible(y=True)
    dock_kalman_cov.addWidget(pg_kalman_cov_depth)

    pg_kalman_cov_velocity = pg.PlotWidget()
    set_plot_options(pg_kalman_cov_velocity)
    pg_kalman_cov_velocity.plot(kalmanData.time,kalmanData.cov_velocity[:-1], pen=(255,0,0), name="cov velocity", stepMode=True)
    # pg_kalman_cov_velocity.setAutoVisible(y=True)
    dock_kalman_cov.addWidget(pg_kalman_cov_velocity)

    pg_kalman_cov_offset = pg.PlotWidget()
    set_plot_options(pg_kalman_cov_offset)
    pg_kalman_cov_offset.plot(kalmanData.time,kalmanData.cov_offset[:-1], pen=(255,0,0), name="cov offset", stepMode=True)
    # pg_kalman_cov_offset.setAutoVisible(y=True)
    dock_kalman_cov.addWidget(pg_kalman_cov_offset)

    pg_kalman_cov_chi = pg.PlotWidget()
    set_plot_options(pg_kalman_cov_chi)
    pg_kalman_cov_chi.plot(kalmanData.time,kalmanData.cov_chi[:-1], pen=(255,0,0), name="cov chi", stepMode=True)
    # pg_kalman_cov_chi.setAutoVisible(y=True)
    dock_kalman_cov.addWidget(pg_kalman_cov_chi)

    pg_kalman_cov_velocity.setXLink(pg_kalman_cov_depth)
    pg_kalman_cov_offset.setXLink(pg_kalman_cov_depth)
    pg_kalman_cov_chi.setXLink(pg_kalman_cov_depth)

#################### Position ####################

#### GPS Status ####
if(len(fixData.time)>0):
    dock_gps = Dock("GPS Signal")
    area_position.addDock(dock_gps)
    pg_gps = pg.PlotWidget()
    set_plot_options(pg_gps)
    pg_gps.plot(fixData.time, fixData.status[:-1], pen=(255,0,0), name="status", stepMode=True)
    pg_gps.setLabel('left', "Fix status")
    dock_gps.addWidget(pg_gps)

    pg_depth = plot_depth(dock_gps)

    pg_gps.setXLink(pg_depth)

#### GPS Pose ####
if(len(poseFusionData.east)>0 and len(fixData.time)>0):
    dock_gps2 = Dock("GPS Pose")
    area_position.addDock(dock_gps2, 'above', dock_gps)
    pg_gps2 = pg.PlotWidget()
    set_plot_options(pg_gps2)
    Y = np.array(poseFusionData.north)
    X = np.array(poseFusionData.east)
    X = X[~np.isnan(X)]
    Y = Y[~np.isnan(Y)]
    print("mean_X = ", np.mean(X))
    print("mean_Y = ", np.mean(Y))
    X -= np.mean(X)
    Y -= np.mean(Y)
    pg_gps2.plot(X, Y, pen=(255,0,0), name="pose (centered on mean)", symbol='o')
    pg_gps2.setLabel('left', "Y", units="m")
    pg_gps2.setLabel('bottom', "X", units="m")
    dock_gps2.addWidget(pg_gps2)

    saveBtn = QtGui.QPushButton('Export GPX')
    dock_gps2.addWidget(saveBtn, row=1, col=0)
    saveBtn.clicked.connect(save_gpx)

####  Heading #### 
if(len(eulerData.time)>0 and len(fixData.time)>0):
    dock_euler = Dock("Heading")
    area_position.addDock(dock_euler, 'above', dock_gps)
    pg_euler1 = pg.PlotWidget()
    set_plot_options(pg_euler1)
    pg_euler1.plot(eulerData.time, np.array(eulerData.z[:-1])*180.0/np.pi, pen=(0,0,255), name="Heading", stepMode=True)
    dock_euler.addWidget(pg_euler1)

####  Speed & Heading GPS #### 
if(len(fixData.time)>0):
    dock_gps_speed_track = Dock("GPS Speed/Track")
    area_position.addDock(dock_gps_speed_track, 'above', dock_gps)
    pg_gps_speed = pg.PlotWidget()
    set_plot_options(pg_gps_speed)
    pg_gps_speed.plot(fixData.time, fixData.speed[:-1], pen=(0,0,255), name="Speed", stepMode=True)
    dock_gps_speed_track.addWidget(pg_gps_speed)

    pg_gps_track = pg.PlotWidget()
    set_plot_options(pg_gps_track)
    pg_gps_track.plot(fixData.time, fixData.track[:-1], pen=(0,0,255), name="Track", stepMode=True)
    dock_gps_speed_track.addWidget(pg_gps_track)

    pg_gps_track.setXLink(pg_gps_speed)

####  Error GPS #### 
if(len(fixData.time)>0):
    dock_gps_error = Dock("GPS Error")
    area_position.addDock(dock_gps_error, 'above', dock_gps)
    pg_gps_horz = pg.PlotWidget()
    set_plot_options(pg_gps_horz)
    pg_gps_horz.plot(fixData.time, fixData.err_horz[:-1], pen=(0,0,255), name="Horizontal Error", stepMode=True)
    dock_gps_error.addWidget(pg_gps_horz)

    pg_gps_vert = pg.PlotWidget()
    set_plot_options(pg_gps_vert)
    pg_gps_vert.plot(fixData.time, fixData.err_vert[:-1], pen=(0,0,255), name="Vertical Error", stepMode=True)
    dock_gps_error.addWidget(pg_gps_vert)

    pg_gps_vert.setXLink(pg_gps_horz)

####  Dop GPS #### 
if(len(fixData.time)>0):
    dock_gps_dop = Dock("GPS DOP")
    area_position.addDock(dock_gps_dop, 'above', dock_gps)
    pg_gps_hdop = pg.PlotWidget()
    set_plot_options(pg_gps_hdop)
    pg_gps_hdop.plot(fixData.time, fixData.hdop[:-1], pen=(0,0,255), name="hdop", stepMode=True)
    dock_gps_dop.addWidget(pg_gps_hdop)

    pg_gps_vdop = pg.PlotWidget()
    set_plot_options(pg_gps_vdop)
    pg_gps_vdop.plot(fixData.time, fixData.vdop[:-1], pen=(0,0,255), name="vdop", stepMode=True)
    dock_gps_dop.addWidget(pg_gps_vdop)

    pg_gps_vdop.setXLink(pg_gps_hdop)

####  Dop GPS #### 
if(len(fixData.time)>0):
    dock_gps_alt = Dock("GPS Alti")
    area_position.addDock(dock_gps_alt, 'above', dock_gps)
    pg_gps_alti = pg.PlotWidget()
    set_plot_options(pg_gps_alti)
    pg_gps_alti.plot(fixData.time, fixData.altitude[:-1], pen=(0,0,255), name="altitude", stepMode=True)
    dock_gps_alt.addWidget(pg_gps_alti)

#################### Iridium ####################

if(len(iridiumStatusData.time)>0):
    dock_iridium_status = Dock("Iridium status")
    area_iridium.addDock(dock_iridium_status)

    pg_iridium_status = pg.PlotWidget()
    set_plot_options(pg_iridium_status)
    pg_iridium_status.plot(iridiumStatusData.time, iridiumStatusData.service[:-1], pen=(255,0,0), name="status", stepMode=True)
    pg_iridium_status.setLabel('left', "status")
    # pg_iridium_status.plot(iridiumStatusData.time, iridiumStatusData.antenna[:-1], pen=(0,255,0), name="antenna", stepMode=True)
    dock_iridium_status.addWidget(pg_iridium_status)

    pg_iridium_signal = pg.PlotWidget()
    set_plot_options(pg_iridium_signal)
    pg_iridium_signal.plot(iridiumStatusData.time, iridiumStatusData.signal_strength[:-1], pen=(255,0,0), name="signal", stepMode=True)
    pg_iridium_signal.setLabel('left', "signal")
    dock_iridium_status.addWidget(pg_iridium_signal)

    pg_depth = plot_depth(dock_iridium_status)

    pg_iridium_status.setXLink(pg_depth)
    pg_iridium_signal.setXLink(pg_depth)

if(len(iridiumSessionData.time)>0):
    dock_iridium_session = Dock("Iridium session")
    area_iridium.addDock(dock_iridium_session, 'above', dock_iridium_status)

    pg_iridium_session_result = pg.PlotWidget()
    set_plot_options(pg_iridium_session_result)
    pg_iridium_session_result.plot(iridiumSessionData.time, iridiumSessionData.mo, pen=(255,0,0), name="mo", symbol='o')
    pg_iridium_session_result.plot(iridiumSessionData.time, iridiumSessionData.mt, pen=(0,255,0), name="mt", symbol='o')
    pg_iridium_session_result.setLabel('left', "mo")
    dock_iridium_session.addWidget(pg_iridium_session_result)

    pg_depth = plot_depth(dock_iridium_session)

    pg_iridium_session_result.setXLink(pg_depth)

    # # /iridium/session
    # time_iridium_session = []
    # iridium_session_mo = []
    # iridium_session_momsn = []
    # iridium_session_mt = []
    # iridium_session_mtmsn = []
    # iridium_session_waiting = []

#################### Waypoints Regulation ####################

if(len(engineData.time)>0):
    dock_thrusters = Dock("Thrusters")
    area_waypoints.addDock(dock_thrusters)

    pg_thruster_left = pg.PlotWidget()
    set_plot_options(pg_thruster_left)
    pg_thruster_left.plot(engineData.time, (engineData.left[:-1]-150)/400., pen=(0,0,255), name="left", stepMode=True)
    pg_thruster_left.setLabel('left', "left")
    dock_thrusters.addWidget(pg_thruster_left)

    pg_thruster_right = pg.PlotWidget()
    set_plot_options(pg_thruster_right)
    pg_thruster_right.plot(engineData.time, (engineData.right[:-1]-150)/400., pen=(0,0,255), name="right", stepMode=True)
    pg_thruster_right.setLabel('left', "right")
    dock_thrusters.addWidget(pg_thruster_right)

    pg_thruster_right.setXLink(pg_thruster_left)

# if(len(engineData.time)>0):
#     dock_regulation_output = Dock("Regulation Heading")
#     area_waypoints.addDock(dock_regulation_output, 'above', dock_thrusters)

#     pg_heading = plot_regulation_waypoint_heading(dock_regulation_output)

#     pg_thruster_angular = pg.PlotWidget()
#     set_plot_options(pg_thruster_angular)
#     pg_thruster_angular.plot(engineCmdData.time, engineCmdData.angular[:-1], pen=(255,0,0), name="angular", stepMode=True)
#     pg_thruster_angular.setLabel('left', "angular")
#     dock_regulation_output.addWidget(pg_thruster_angular)

#     pg_thruster_angular.setXLink(pg_heading)

if(np.size(engineData.time)>0 and np.size(regulationWaypointData.time)>0):
    dock_heading_error = Dock("Heading error")
    area_waypoints.addDock(dock_heading_error, 'above', dock_thrusters)

    pg_heading = plot_regulation_waypoint_heading(dock_heading_error)

    pg_heading_error = pg.PlotWidget()
    set_plot_options(pg_heading_error)
    pg_heading_error.plot(regulationWaypointData.time, regulationWaypointData.yaw_error[:-1], pen=(255,0,0), name="error", stepMode=True)
    dock_heading_error.addWidget(pg_heading_error)

    pg_heading_cmd = pg.PlotWidget()
    set_plot_options(pg_heading_cmd)
    pg_heading_cmd.plot(regulationWaypointData.time, regulationWaypointData.angular[:-1], pen=(255,0,0), name="angular", stepMode=True)
    pg_heading_cmd.plot(regulationWaypointData.time, regulationWaypointData.angular_limit[:-1], pen=(0,255,0), name="angular saturated", stepMode=True)
    dock_heading_error.addWidget(pg_heading_cmd)

    pg_heading_cmd.setXLink(pg_heading_error)
    pg_heading_error.setXLink(pg_heading)

# if(np.size(engineData.time)>0 and np.size(regulationWaypointData.time)>0):
#     dock_heading_regulation = Dock("Heading regulation")
#     area_waypoints.addDock(dock_heading_regulation, 'above', dock_thrusters)

#     pg_heading = plot_regulation_waypoint_heading(dock_heading_regulation)

#     pg_heading_error = pg.PlotWidget()
#     set_plot_options(pg_heading_error)
#     pg_heading_error.plot(regulationWaypointData.time, regulationWaypointData.yaw_error[:-1], pen=(255,0,0), name="error", stepMode=True)
#     dock_heading_regulation.addWidget(pg_heading_error)

#     pg_heading_cmd = pg.PlotWidget()
#     set_plot_options(pg_heading_cmd)
#     pg_heading_cmd.plot(regulationWaypointData.time, regulationWaypointData.angular[:-1], pen=(255,0,0), name="angular", stepMode=True)
#     pg_heading_cmd.plot(regulationWaypointData.time, regulationWaypointData.angular_limit[:-1], pen=(0,255,0), name="angular saturated", stepMode=True)
#     dock_heading_regulation.addWidget(pg_heading_cmd)

#     pg_heading_cmd.setXLink(pg_heading_error)
#     pg_heading_error.setXLink(pg_heading)

def get_circle(center, radius):
    t = np.linspace(-np.pi, np.pi, num=100)
    X = radius*np.cos(t)+center[0]
    Y = radius*np.sin(t)+center[1]
    return (X,Y)

#### Mission Path ####
if(np.size(poseFusionData.east)>0 and np.size(fixData.time)>0 and np.size(missionData.time)>0):
    dock_mission = Dock("Mission path")
    area_waypoints.addDock(dock_mission, 'above', dock_heading_error)
    pg_gps2 = pg.PlotWidget()
    set_plot_options(pg_gps2)
    Y = poseFusionData.north
    X = poseFusionData.east
    X_mission = missionData.east[missionData.east>0]
    Y_mission = missionData.north[missionData.north>0]
    plot_XY_mission = pg_gps2.plot(X, Y, pen=(255,0,0), name="pose (centered on mean)", symbol='o')
    pg_gps2.plot(X_mission, Y_mission, pen=(0,255,0), name="mission", symbol='x')
    pg_gps2.setLabel('left', "Y", units="m")
    pg_gps2.setLabel('bottom', "X", units="m")

    f_mission_set_point = interpolate.interp1d(missionData.time, (missionData.east, missionData.north), bounds_error=False, kind="zero")
    X_mission_interp, Y_mission_interp = f_mission_set_point(poseFusionData.time)

    X_Circ_inside, Y_Circ_inside = get_circle((X_mission_interp[-1],Y_mission_interp[-1]),4.)
    X_Circ_outside, Y_Circ_outside = get_circle((X_mission_interp[-1],Y_mission_interp[-1]),2.)
    plot_circle_outside = pg_gps2.plot(X_Circ_inside, Y_Circ_inside, pen=(0,0,255))
    plot_circle_inside = pg_gps2.plot(X_Circ_outside, Y_Circ_outside, pen=(0,255,255))

    dock_mission.addWidget(pg_gps2)

    f_pose_gnss = interpolate.interp1d(regulationWaypointData.time, regulationWaypointData.distance_error, bounds_error=False, kind="zero")
    distance_error_interp = f_pose_gnss(poseFusionData.time)

    pg_distance_error = pg.PlotWidget()
    set_plot_options(pg_distance_error)
    pg_distance_error.plot(poseFusionData.time, distance_error_interp[:-1], pen=(255,0,0), name="distance_error", stepMode=True)
    dock_mission.addWidget(pg_distance_error)

    lr_mission = pg.LinearRegionItem([0, poseFusionData.time[-1]], bounds=[0,poseFusionData.time[-1]], movable=True)
    pg_distance_error.addItem(lr_mission)
    lr_bounds_mission = lr_mission.getRegion()

    def update_plot_mission():
        global plot_XY_mission, poseFusionData, lr_mission, poseFusionData, lr_bounds_mission
        global plot_circle_outside, plot_circle_inside, X_mission_interp, Y_mission_interp
        t_bounds = lr_mission.getRegion()
        if(t_bounds != lr_bounds_mission):
            lr_bounds_mission = t_bounds
            ub = np.where(poseFusionData.time <= np.max((1,t_bounds[1])))[0][-1]
            lb = np.where(poseFusionData.time >= np.min((poseFusionData.time[-1],t_bounds[0])))[0][0]

            ub = np.min((ub, np.size(poseFusionData.time)))
            lb = np.max((lb,0))
            X = poseFusionData.east[lb:ub]
            Y = poseFusionData.north[lb:ub]
            X = X[~np.isnan(X)]
            Y = Y[~np.isnan(Y)]
            plot_XY_mission.setData(X,Y)

            X_Circ_inside, Y_Circ_inside = get_circle((X_mission_interp[ub],Y_mission_interp[ub]),4.)
            X_Circ_outside, Y_Circ_outside = get_circle((X_mission_interp[ub],Y_mission_interp[ub]),2.)

            plot_circle_outside.setData(X_Circ_inside, Y_Circ_inside)
            plot_circle_inside.setData(X_Circ_outside, Y_Circ_outside)

    timer_mission = pg.QtCore.QTimer()
    timer_mission.timeout.connect(update_plot_mission)
    timer_mission.start(50)

#### Distance error ####
if(np.size(engineData.time)>0 and np.size(regulationWaypointData.time)>0):
    dock_distance = Dock("Distance")
    area_waypoints.addDock(dock_distance, 'above', dock_heading_error)

    pg_distance_error = pg.PlotWidget()
    set_plot_options(pg_distance_error)
    pg_distance_error.plot(regulationWaypointData.time, regulationWaypointData.distance_error[:-1], pen=(255,0,0), name="distance error", stepMode=True)
    dock_distance.addWidget(pg_distance_error)

    pg_enable_regulation = pg.PlotWidget()
    set_plot_options(pg_enable_regulation)
    pg_enable_regulation.plot(regulationWaypointData.time, regulationWaypointData.enable_regulation[:-1], pen=(255,0,0), name="enable regulation", stepMode=True)
    pg_enable_regulation.plot(missionData.time, missionData.mission_enable[:-1], pen=(0,255,0), name="enable mission", stepMode=True)
    pg_enable_regulation.plot(regulationWaypointData.time, regulationWaypointData.valid_time[:-1], pen=(0,0,255), name="valid time", stepMode=True)
    dock_distance.addWidget(pg_enable_regulation)

    pg_hysteresis = pg.PlotWidget()
    set_plot_options(pg_hysteresis)
    pg_hysteresis.plot(regulationWaypointData.time, regulationWaypointData.hysteresis_inside[:-1], pen=(255,0,0), name="hysteresis inside", stepMode=True)
    dock_distance.addWidget(pg_hysteresis)

    pg_hysteresis.setXLink(pg_enable_regulation)
    pg_distance_error.setXLink(pg_hysteresis)

# pg_thruster_linear = pg.PlotWidget()
# set_plot_options(pg_thruster_linear)
# pg_thruster_linear.plot(engineCmdData.time, engineCmdData.linear[:-1], pen=(255,0,0), name="linear", stepMode=True)
# pg_thruster_linear.setLabel('left', "linear")
# dock_regulation_output.addWidget(pg_thruster_linear)
    
# regulationWaypointData.yaw_set_point
# regulationWaypointData.yaw_error
# regulationWaypointData.distance_error
# regulationWaypointData.enable_regulation
# regulationWaypointData.hysteresis_inside
# regulationWaypointData.angular
# regulationWaypointData.angular_limit

###################################################

win.show()
    
## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

