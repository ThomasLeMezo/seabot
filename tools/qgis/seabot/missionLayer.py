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
	layer_name = 'Seabot - Mission Track'

	def __init__(self):
		self.fields.append(QgsField('Title', QVariant.String))
		self.fields.append(QgsField('wp_id', QVariant.Double))
		return

	def update_mission(self, seabotMission):
		# Global mission

		## Find if layer already exist
		layer_list = QgsProject.instance().mapLayersByName(self.layer_name)

		# Build list of points
		list_wp = []
		for wp in seabotMission.get_wp_list():
			list_wp.append(QgsPoint(wp.get_east(), wp.get_north()))

		if(len(layer_list)!=0):
			QgsProject.instance().removeMapLayer(layer_list[0])
		
		### Add New layer Last Position
		layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', self.layer_name , "memory")

		pr = layer.dataProvider()

		# add the first point
		feature = QgsFeature()

		feature.setGeometry(QgsGeometry.fromPolyline(list_wp))
		pr.addFeatures([feature])

		# Configure the marker.
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

	def update_current_wp(self, seabotMission):
		# Position of the current wp