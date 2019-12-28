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
from pyproj import Proj, transform
import threading

class MissionLayer():

	fields = QgsFields()
	layer_mission = 'Seabot - Mission Track'
	layer_set_point = 'Seabot - Mission Set Point'

	def __init__(self):
		self.fields.append(QgsField('Title', QVariant.String))
		self.fields.append(QgsField('wp_id', QVariant.Double))
		return

	def update_mission_layer(self, seabotMission):
		# Global mission

		## Find if layer already exist
		layer_list = QgsProject.instance().mapLayersByName(self.layer_mission)

		# Build list of points
		list_wp = []
		for wp in seabotMission.get_wp_list():
			list_wp.append(QgsPoint(wp.get_east(), wp.get_north()))

		if(len(layer_list)!=0):
			QgsProject.instance().removeMapLayer(layer_list[0])
		
		### Add New layer Last Position
		layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', self.layer_mission , "memory")

		pr = layer.dataProvider()

		# add the first point
		feature = QgsFeature()

		feature.setGeometry(QgsGeometry.fromPolyline(list_wp))
		pr.addFeatures([feature])

		# Configure the marker
		marker_line = QgsSimpleLineSymbolLayer()
		marker_point = QgsMarkerLineSymbolLayer()
		marker_point.setPlacement(QgsMarkerLineSymbolLayer.Vertex)

		marker = QgsLineSymbol()
		marker.changeSymbolLayer(0, marker_line)
		marker.appendSymbolLayer(marker_point)
		marker.setColor(Qt.blue)

		renderer = QgsSingleSymbolRenderer(marker)
		layer.setRenderer(renderer)
		layer.updateExtents()
		QgsProject.instance().addMapLayer(layer)

		return True

	def update_mission_set_point(self, seabotMission):
		if seabotMission.is_empty():
			return True
		### ADD DATA TO LAYER ###
		## Find if layer already exist
		layer_list = QgsProject.instance().mapLayersByName(self.layer_set_point)
		point = QgsPointXY(seabotMission.get_set_point_east(), seabotMission.get_set_point_north())
		# print(point)
		if(len(layer_list)==0):
			### Add New layer Last Position
			layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', self.layer_set_point , "memory")
			pr = layer.dataProvider()

			# add the first point
			feature = QgsFeature()
			feature.setGeometry(QgsGeometry.fromPointXY(point))
			pr.addFeatures([feature])

			# Configure the marker.
			simple_marker_large_circle = QgsSimpleMarkerSymbolLayer(size=8,color=Qt.darkBlue)
			simple_marker_small_circle = QgsSimpleMarkerSymbolLayer(color=Qt.red)
			marker = QgsMarkerSymbol()
			marker.setOpacity(0.5)
			marker.changeSymbolLayer(0, simple_marker_large_circle)
			marker.appendSymbolLayer(simple_marker_small_circle)

			renderer = QgsSingleSymbolRenderer(marker)
			layer.setRenderer(renderer)

			# add the layer to the canvas
			layer.updateExtents()
			QgsProject.instance().addMapLayer(layer)
		else:
			layer = layer_list[0]
			pr = layer.dataProvider()

			for feature in layer.getFeatures():
				pr.changeGeometryValues({feature.id():QgsGeometry.fromPointXY(point)})
				layer.triggerRepaint()
				break
		return True