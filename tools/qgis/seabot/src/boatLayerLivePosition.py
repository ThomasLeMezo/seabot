#!/usr/bin/python3

from qgis.core import *
import qgis.utils
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import time
import oyaml as yaml
import os
from os.path import expanduser
import numpy as np
import time
import math
from gps import *
from pyproj import Proj, transform
import threading

gpsd = None
gpsd_latitude = 0.0
gpsd_longitude = 0.0
gpsd_track = 0.0

class GpsPoller(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
    global gpsd #bring it in scope
    gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
    self.current_value = None
    self.running = True #setting the thread running to true
 
  def run(self):
    global gpsd, gpsd_latitude, gpsd_longitude, gpsd_track
    while self.running:
        if(gpsd.waiting()):
            report = gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
            if report['class'] == 'TPV':
                gpsd_latitude = getattr(report,'lat',0.0)
                gpsd_longitude = getattr(report,'lon',0.0)
                gpsd_track = getattr(report,'track',0.0)
            time.sleep(0.2)

class BoatLayerLivePosition():

    east = 130241
    north =  6833446
    heading = 0.0
    gpsp_thread = GpsPoller()
    inProj = Proj(init='epsg:4326')
    outProj = Proj(init='epsg:2154')
    fields = QgsFields()
    layer_name = 'Boat'

    def __init__(self):
        self.fields.append(QgsField('Title', QVariant.String))
        self.fields.append(QgsField('gnss_heading', QVariant.Double))
        return

    def __del__(self):
        self.gpsp_thread.running = False
        self.gpsp_thread.join()

    def stop(self):
        self.gpsp_thread.running = False
        self.gpsp_thread.join()

    def start(self):
        self.gpsp_thread.start()

    def update(self):
        self.get_new_position()
        self.update_boat_position()
        return True

    def get_new_position(self):
        self.heading = gpsd_track
        # print(gpsd_latitude, gpsd_longitude)
        self.east, self.north = transform(self.inProj,self.outProj,gpsd_longitude,gpsd_latitude)

    def update_boat_position(self):
        ### ADD DATA TO LAYER ###
        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(self.layer_name)
        point = QgsPointXY(self.east, self.north)

        if(len(layer_list)==0):
            ### Add New layer Last Position
            layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', self.layer_name , "memory")

            pr = layer.dataProvider()

            pr.addAttributes(self.fields)
            layer.updateFields()

            # add the first point
            feature = QgsFeature()
            feature.setGeometry(QgsGeometry.fromPointXY(point))

            feature.setFields(self.fields)
            feature['Title'] = self.layer_name
            feature['gnss_heading'] = self.heading

            pr.addFeatures([feature])

            # Configure the marker.
            svg_marker = QgsSvgMarkerSymbolLayer("/usr/share/qgis/svg/arrows/NorthArrow_11.svg")
            svg_marker.setPreservedAspectRatio(True)
            svg_marker.setSize(5)
            # svg_marker.setAngle(0)
            svg_marker.setColor(Qt.green) # QColor(255,255,255)

            marker = QgsMarkerSymbol()
            marker.changeSymbolLayer(0, svg_marker)

            prop=QgsProperty()
            prop.setField("gnss_heading")
            marker.setDataDefinedAngle(prop) #QgsProperty () 

            renderer = QgsSingleSymbolRenderer(marker)
            layer.setRenderer(renderer)

            # add the layer to the canvas
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer)

        else:
            layer = layer_list[0]
            pr = layer.dataProvider()

            for feature in layer.getFeatures():
                pr.changeFeatures({feature.id():{0:self.layer_name,1:self.heading}}, {feature.id():QgsGeometry.fromPointXY(point)})
                layer.updateExtents()
                layer.triggerRepaint()
                break

        return True