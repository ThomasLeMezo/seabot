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
import datetime

gpsd = None
gpsd_latitude = 0.0
gpsd_longitude = 0.0
gpsd_track = 0.0
gpsd_received = True

class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.daemon=True
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
						gpsd_received = True
				# while wait
				time.sleep(0.2)

class LayerBoat():

	east = 130241
	north =  6833446
	heading = 0.0
	gpsd_thread = None

	inProj = Proj(init='epsg:4326')
	outProj = Proj(init='epsg:2154')
	fields = QgsFields()
	
	group_name = 'Boat'
	layer_track = 'Boat track'
	layer_pose = 'Boat pose'

	trace_max_points = 15
	enable_trace_vanish = True

	def __init__(self):
		self.fields.append(QgsField('Title', QVariant.String))
		self.fields.append(QgsField('gnss_heading', QVariant.Double))
		return

	def __del__(self):
		if self.gpsd_thread != None:
			self.gpsd_thread.running = False
			time.sleep(0.2)
			if(threading.active_count()!=0):
				self.gpsd_thread.join()

	def set_nb_points_max(self, trace_max_points=15, enable_trace_vanish=True):
		self.trace_max_points = trace_max_points
		self.enable_trace_vanish = enable_trace_vanish

	def stop(self):
		self.gpsd_thread.running = False
		time.sleep(0.2)
		if(threading.active_count()!=0):
			self.gpsd_thread.join()

	def start(self):
		self.gpsd_thread = GpsPoller()
		self.gpsd_thread.start()

	def update(self):
		if(gpsd_received):
			self.get_new_position()
			self.update_boat_pose()
			self.update_boat_trace()

	def get_new_position(self):
		self.heading = gpsd_track
		self.east, self.north = transform(self.inProj,self.outProj,gpsd_longitude,gpsd_latitude)

	def update_boat_trace(self):
		### ADD DATA TO LAYER ###
		root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
		if(root == None):
			root = QgsProject.instance().layerTreeRoot().insertGroup(0, self.group_name)

		## Find if layer already exist
		layer_list = QgsProject.instance().mapLayersByName(self.layer_track)
		point = QgsPoint(self.east, self.north)

		if(len(layer_list)==0):
			### Add New layer Last Position
			layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', self.layer_track , "memory")

			pr = layer.dataProvider()

			# add the first point
			feature = QgsFeature()

			feature.setGeometry(QgsGeometry.fromPolyline([point]))
			pr.addFeatures([feature])

			# Configure the marker.
			marker_line = QgsSimpleLineSymbolLayer(Qt.green)
			marker = QgsLineSymbol()
			marker.changeSymbolLayer(0, marker_line)

			renderer = QgsSingleSymbolRenderer(marker)
			layer.setRenderer(renderer)
			layer.updateExtents()
			QgsProject.instance().addMapLayer(layer, addToLegend=False)
			root.addLayer(layer)

		else:
			layer = layer_list[0]
			pr = layer.dataProvider()

			for feature in layer.getFeatures():
				geo = feature.geometry()
				if(geo.type()==QgsWkbTypes.LineGeometry):
					# Add new point
					geo.insertVertex(point, 0)

					# Remove last if necessary
					if(self.enable_trace_vanish):
						while(len(geo.asPolyline())>self.trace_max_points+2): # Keep always two points (to have a line geometry)
							geo.deleteVertex(len(geo.asPolyline())-1) # Remove vertex

					# Commit changes
					pr.changeGeometryValues({feature.id():geo})
					layer.triggerRepaint()

					break

	def update_boat_pose(self):
		### ADD DATA TO LAYER ###
		point = QgsPointXY(self.east, self.north)

		## Find if layer already exist
		root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
		if(root == None):
			root = QgsProject.instance().layerTreeRoot().insertGroup(0, self.group_name)

		layer_list = QgsProject.instance().mapLayersByName(self.layer_pose)
		if(len(layer_list)==0):
		# if(len(layer_list)==0):
			### Add New layer Last Position
			layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', self.layer_pose , "memory")

			pr = layer.dataProvider()

			pr.addAttributes(self.fields)
			layer.updateFields()

			# add the first point
			feature = QgsFeature()
			feature.setGeometry(QgsGeometry.fromPointXY(point))

			feature.setFields(self.fields)
			feature['Title'] = self.layer_pose
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
			QgsProject.instance().addMapLayer(layer, addToLegend=False) # Do not add to tree here, see next line
			root.addLayer(layer)

		else:
			layer = layer_list[0]

			# Data
			pr = layer.dataProvider()
			for feature in layer.getFeatures():
				pr.changeFeatures({feature.id():{0:self.layer_pose,1:self.heading}}, {feature.id():QgsGeometry.fromPointXY(point)})

				layer.updateExtents()
				layer.triggerRepaint()
				break

		return True