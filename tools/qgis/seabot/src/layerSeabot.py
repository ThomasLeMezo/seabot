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
from .database import *
import datetime

class LayerSeabot():

    def __init__(self, imei, name=None):
        self.fields = QgsFields()
        self.group_name = 'Seabot Live (' + str(imei) + ")"
        self.layer_track = 'Seabot track ('  + str(imei) + ")"
        self.layer_pose = 'Seabot pose ('  + str(imei) + ")"
        self.surface = False

        self.fields.append(QgsField('Title', QVariant.String))
        self.fields.append(QgsField('gnss_heading', QVariant.Double))
        self.fields.append(QgsField('sec_since_received', QVariant.Double))

        self.imei = imei
        if(name !=None):
            self.name = name
        else:
            self.name = ""

        self.db = DataBaseConnection()

    def update(self):
        self.update_track()
        self.update_pose()

    def __del__(self):
        root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
        if(root != None):
            root.removeAllChildren()
            QgsProject.instance().layerTreeRoot().removeChildNode(root)

    def update_track(self):
        # Global mission
        root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
        if(root == None):
            root = QgsProject.instance().layerTreeRoot().insertGroup(0, self.group_name)

        ## Find if layer already exist (and removed it)
        layer_list = QgsProject.instance().mapLayersByName(self.layer_track)
        if(len(layer_list)!=0):
            QgsProject.instance().removeMapLayer(layer_list[0])
        
        # Build list of points
        list_pose = self.db.get_pose(self.imei)
        if(len(list_pose)==0):
            return

        ### Add New layer Last Position
        layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', self.layer_track , "memory")

        pr = layer.dataProvider()

        # add the first point
        feature = QgsFeature()

        list_wp = []
        for wp in list_pose:
            list_wp.append(QgsPoint(wp[0], wp[1]))
        feature.setGeometry(QgsGeometry.fromPolyline(list_wp))
        pr.addFeatures([feature])

        # Configure the marker
        marker_line = QgsSimpleLineSymbolLayer()
        marker_point = QgsMarkerLineSymbolLayer()
        marker_point.setPlacement(QgsMarkerLineSymbolLayer.Vertex)

        marker = QgsLineSymbol()
        marker.changeSymbolLayer(0, marker_line)
        marker.appendSymbolLayer(marker_point)
        marker.setColor(Qt.red)

        renderer = QgsSingleSymbolRenderer(marker)
        layer.setRenderer(renderer)

        layer.updateExtents()
        QgsProject.instance().addMapLayer(layer, addToLegend=False)
        root.addLayer(layer)

        return True

    def update_pose(self):
        data = self.db.get_last_log_state(self.imei)[0]
        if(data==None):
            return
        point = QgsPointXY(float(data["east"]), float(data["north"]))

        ### ADD DATA TO LAYER ###
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
            feature['gnss_heading'] = data["gnss_heading"]
            feature['sec_since_received'] = -1

            pr.addFeatures([feature])

            # Configure the marker.
            svg_marker = QgsSvgMarkerSymbolLayer("/usr/share/qgis/svg/arrows/NorthArrow_11.svg")
            svg_marker.setPreservedAspectRatio(True)
            svg_marker.setSize(5)
            # svg_marker.setAngle(0)
            svg_marker.setColor(Qt.red) # QColor(255,255,255)

            marker = QgsMarkerSymbol()
            marker.changeSymbolLayer(0, svg_marker)

            prop=QgsProperty()
            prop.setField("gnss_heading")
            marker.setDataDefinedAngle(prop) #QgsProperty () 

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
            layer_settings.fieldName = "(sec_since_received)"
            layer_settings.isExpression = True
            layer_settings.placement = QgsPalLayerSettings.OverPoint
            layer_settings.predefinedPositionOrder = QgsPalLayerSettings.TopMiddle
            layer_settings.yOffset = -10
            layer_settings.enabled = True

            layer_label = QgsVectorLayerSimpleLabeling(layer_settings)
            layer.setLabelsEnabled(True)
            layer.setLabeling(layer_label)
            layer.triggerRepaint()

            # add the layer to the canvas
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer, addToLegend=False) # Do not add to tree here, see next line
            root.insertLayer(0, layer)

        else:
            layer = layer_list[0]

            # Data
            pr = layer.dataProvider()
            for feature in layer.getFeatures():
                delta_t =  datetime.timedelta(seconds=round(time.clock_gettime(time.CLOCK_REALTIME) - data["ts"]))
                # if(delta_t>60):
                #     delta_t /= 60.
                #     delta_t = str(round(delta_t)) + " min"
                # else:
                #     delta_t = str(delta_t) + " sec"
                delta_t = self.name + '\n' + str(delta_t)
                pr.changeFeatures({feature.id():{0:self.layer_pose,1:data["gnss_heading"], 2:delta_t}}, {feature.id():QgsGeometry.fromPointXY(point)})

                layer.updateExtents()
                layer.triggerRepaint()
                break

        return True