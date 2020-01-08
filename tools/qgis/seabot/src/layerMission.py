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

class LayerMission():

	fields = QgsFields()
	group_name = 'Seabot Mission'
	layer_track = 'Seabot track'
	layer_pose = 'Seabot pose'
	surface = False

	def __init__(self):
		self.fields.append(QgsField('Title', QVariant.String))
		self.fields.append(QgsField('wp_id', QVariant.Double))
		return

	def update_mission_layer(self, seabotMission):
		# Global mission
		root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
		if(root == None):
			root = QgsProject.instance().layerTreeRoot().insertGroup(0, self.group_name)

		## Find if layer already exist
		layer_list = QgsProject.instance().mapLayersByName(self.layer_track)
		
		# Build list of points
		list_wp = []
		for wp in seabotMission.get_wp_list():
			list_wp.append(QgsPoint(wp.get_east(), wp.get_north()))

		if(len(layer_list)!=0):
			QgsProject.instance().removeMapLayer(layer_list[0])
		
		### Add New layer Last Position
		layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', self.layer_track , "memory")

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
		QgsProject.instance().addMapLayer(layer, addToLegend=False)
		root.addLayer(layer)

		return True

	def update_mission_set_point(self, seabotMission):
		if seabotMission.is_empty():
			return True

		point = QgsPointXY(seabotMission.get_set_point_east(), seabotMission.get_set_point_north())

		### ADD DATA TO LAYER ###
		## Find if layer already exist
		root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
		if(root == None):
			root = QgsProject.instance().layerTreeRoot().insertGroup(0, self.group_name)

		layer_list = QgsProject.instance().mapLayersByName(self.layer_pose)
		
		# print(point)
		if(len(layer_list)==0):
			### Add New layer Last Position
			layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', self.layer_pose , "memory")
			pr = layer.dataProvider()

			# add the first point
			feature = QgsFeature()
			feature.setGeometry(QgsGeometry.fromPointXY(point))
			pr.addFeatures([feature])

			# Configure the marker.
			simple_marker_large_circle = QgsSimpleMarkerSymbolLayer(size=8,color=self.color_symbol())
			simple_marker_small_circle = QgsSimpleMarkerSymbolLayer(color=Qt.red)
			marker = QgsMarkerSymbol()
			marker.setOpacity(0.5)
			marker.changeSymbolLayer(0, simple_marker_large_circle)
			marker.appendSymbolLayer(simple_marker_small_circle)

			renderer = QgsSingleSymbolRenderer(marker)
			layer.setRenderer(renderer)

			# add the layer to the canvas
			layer.updateExtents()
			QgsProject.instance().addMapLayer(layer, addToLegend=False)
			root.addLayer(layer)
		else:
			layer = layer_list[0]

			# Renderer
			if(seabotMission.is_surface()!=self.surface):
				self.surface = seabotMission.is_surface()
				singleSymbolRenderer = layer.renderer()
				markerSymbol = singleSymbolRenderer.symbol()
				simpleMarkerSymbolLayer = markerSymbol.symbolLayer(0)
				simpleMarkerSymbolLayer.setColor(self.color_symbol())
				# layer.updateExtents()

			# Data
			pr = layer.dataProvider()
			for feature in layer.getFeatures():
				pr.changeGeometryValues({feature.id():QgsGeometry.fromPointXY(point)})
				layer.triggerRepaint()
				break
		return True

	def color_symbol(self):
		if(self.surface):
			return Qt.darkBlue
		else:
			return Qt.gray