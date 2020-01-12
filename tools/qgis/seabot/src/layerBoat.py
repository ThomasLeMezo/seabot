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

from .database import *

class GpsPoller(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)
		self.daemon=True

		self.gpsd = gps(mode=WATCH_ENABLE) #starting the stream of info
		self.current_value = None
		self.running = True #setting the thread running to true

		self.gpsd_latitude = 0.0
		self.gpsd_longitude = 0.0
		self.gpsd_track = 0.0
		self.gpsd_received = True
	 
	def run(self):
		while self.running:
			if(self.gpsd.waiting()):
				report = self.gpsd.next() #this will continue to loop and grab EACH set of gpsd info to clear the buffer
				if report['class'] == 'TPV':
					self.gpsd_latitude = getattr(report,'lat',0.0)
					self.gpsd_longitude = getattr(report,'lon',0.0)
					self.gpsd_track = getattr(report,'track',0.0)
					self.gpsd_received = True
			# while wait
			time.sleep(0.2)

class LayerBoat():

	inProj = Proj(init='epsg:4326')
	outProj = Proj(init='epsg:2154')
	
	def __init__(self, iface):
		self.iface = iface
		self.trace_max_points = 15
		self.enable_trace_vanish = True
		self.enable_seabot = True

		self.east = 253247
		self.north =  8605639
		self.heading = 0.0
		self.gpsPoller = None

		self.fields = QgsFields()
		self.group_name = 'Boat'
		self.layer_track = 'Boat track'
		self.layer_pose = 'Boat pose'
		self.layer_seabot = 'Boat to seabot'
		self.fields.append(QgsField('Title', QVariant.String))
		self.fields.append(QgsField('gnss_heading', QVariant.Double))

		self.fields_seabot = QgsFields()
		self.fields_seabot.append(QgsField('seabot_info', QVariant.String))

		self.seabot_north = 0.0
		self.seabot_east = 0.0
		self.locked = False

		self.delete_layer_exist = True
		return

	def enable_lock_view(self, val=False):
		if val:
			self.locked = True
		else:
			self.locked = False
			self.iface.mapCanvas().setRotation(0)

	def remove_layer(self):
		if self.delete_layer_exist:
			root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
			if(root != None):
				root.removeAllChildren()
				QgsProject.instance().layerTreeRoot().removeChildNode(root)

	def __del__(self):
		if self.gpsPoller != None:
			self.gpsPoller.running = False
			time.sleep(0.2)
			if(threading.active_count()!=0 and self.gpsPoller!=None):
				self.gpsPoller.join()
		self.remove_layer()

	def set_nb_points_max(self, trace_max_points=15, enable_trace_vanish=True):
		self.trace_max_points = trace_max_points
		self.enable_trace_vanish = enable_trace_vanish

	def stop(self):
		self.gpsPoller.running = False
		time.sleep(0.2)
		if(threading.active_count()!=0):
			self.gpsPoller.join()

	def start(self):
		self.gpsPoller = GpsPoller()
		self.gpsPoller.start()

	def update(self):
		if(self.gpsPoller.gpsd_received):
			self.get_new_position()
			self.update_boat_pose()
			self.update_boat_trace()
			if(self.enable_seabot):
				self.update_boat_to_seabot()
			if(self.locked):
				self.lock_view()
			

	def get_new_position(self):
		self.heading = self.gpsPoller.gpsd_track
		self.east, self.north = transform(self.inProj,self.outProj,self.gpsPoller.gpsd_longitude,self.gpsPoller.gpsd_latitude)

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

	def update_boat_to_seabot(self):
		### ADD DATA TO LAYER ###
		root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
		if(root == None):
			root = QgsProject.instance().layerTreeRoot().insertGroup(0, self.group_name)

		## Find if layer already exist
		layer_list = QgsProject.instance().mapLayersByName(self.layer_seabot)
		point1 = QgsPoint(self.east, self.north)
		point2 = QgsPoint(self.seabot_east, self.seabot_north)

		if(len(layer_list)==0):
			### Add New layer Last Position
			layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', self.layer_seabot , "memory")

			pr = layer.dataProvider()


			pr.addAttributes(self.fields_seabot)
			layer.updateFields()

			# First feature
			feature = QgsFeature()
			feature.setGeometry(QgsGeometry.fromPolyline([point1, point2]))

			feature.setFields(self.fields_seabot)
			feature['seabot_info'] = ""

			# add the first point
			
			pr.addFeatures([feature])

			# Configure the marker.
			marker_line = QgsSimpleLineSymbolLayer(Qt.darkGreen)
			marker = QgsLineSymbol()
			marker.setWidth(2)
			marker.changeSymbolLayer(0, marker_line)

			renderer = QgsSingleSymbolRenderer(marker)
			layer.setRenderer(renderer)

			# Text
			# White background under the text
			bg_settings = QgsTextBackgroundSettings()
			bg_settings.setEnabled(True)
			bg_settings.setSizeType(QgsTextBackgroundSettings.SizeBuffer)
			bg_settings.setStrokeColor(Qt.white)
			bg_settings.setFillColor(Qt.white)
			bg_settings.setOpacity(0.7)

			text_format = QgsTextFormat()
			text_format.setFont(QFont("Ubuntu", 8))
			text_format.setSize(10)
			# text_format.setBuffer(buffer_settings)
			text_format.setBackground(bg_settings)

			layer_settings  = QgsPalLayerSettings()
			layer_settings.setFormat(text_format)
			layer_settings.fieldName = "(seabot_info)"
			layer_settings.isExpression = True
			layer_settings.placement = QgsPalLayerSettings.Horizontal
			# layer_settings.placementFlags = QgsPalLayerSettings.MapOrientation
			layer_settings.predefinedPositionOrder = QgsPalLayerSettings.TopMiddle
			# layer_settings.yOffset = -10
			layer_settings.enabled = True

			layer_label = QgsVectorLayerSimpleLabeling(layer_settings)
			layer.setLabelsEnabled(True)
			layer.setLabeling(layer_label)
			layer.triggerRepaint()

			layer.updateExtents()
			QgsProject.instance().addMapLayer(layer, addToLegend=False)
			root.addLayer(layer)

		else:
			layer = layer_list[0]
			pr = layer.dataProvider()

			for feature in layer.getFeatures():
				geo = QgsGeometry.fromPolyline([point1, point2])

				# Update feature
				pr.changeFeatures({feature.id():{0:self.get_text_seabot()}}, {feature.id():geo})
				# Commit changes
				layer.updateExtents()
				layer.triggerRepaint()

				break


	def rad_to_heading(self, angle):
		angle = -(angle - np.pi/2.)
		# [0, 2*pi]
		angle = 2*np.arctan(np.tan((angle+np.pi)/2.))+np.pi
		# Convert to def
		angle = angle*180.0/np.pi
		return angle

	def get_text_seabot(self):
		distance = math.sqrt(math.pow(self.seabot_east - self.east, 2)+math.pow(self.seabot_north - self.north, 2))
		
		heading_abs = self.rad_to_heading(math.atan2(self.seabot_north - self.north, self.seabot_east - self.east))

		heading_diff = heading_abs-self.heading
		
		return str(round(distance)) + "m\n" + str(round(heading_abs)) + "°"
		
	def lock_view(self):
		self.iface.mapCanvas().setRotation(self.heading)

	def set_enable_seabot(self, val):
		self.enable_seabot = val
		if(val == False):
			layer_list = QgsProject.instance().mapLayersByName(self.layer_seabot)
			if(len(layer_list)>0):
				QgsProject.instance().removeMapLayer(layer_list[0])

