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

class LayerInfo():

    def __init__(self):
        self.fields = QgsFields()
        self.group_name = 'Info'
        self.layer_name = 'Seabot info'
        self.message_id = 0

        self.fields.append(QgsField('message_id', QVariant.Int))
        self.fields.append(QgsField('gnss_heading', QVariant.Double))

        self.db = DataBaseConnection(init_table=False)

    def update(self, message_id):
        self.message_id = message_id
        self.update_pose()

    def __del__(self):
        root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
        if(root != None):
            root.removeAllChildren()
            QgsProject.instance().layerTreeRoot().removeChildNode(root)

    def update_pose(self):
        data = self.db.get_log_state(self.message_id)
        if(data==None):
            return
        point = QgsPointXY(float(data["east"]), float(data["north"]))

        ### ADD DATA TO LAYER ###
        ## Find if layer already exist
        root = QgsProject.instance().layerTreeRoot().findGroup(self.group_name)
        if(root == None):
            root = QgsProject.instance().layerTreeRoot().insertGroup(0, self.group_name)

        layer_list = QgsProject.instance().mapLayersByName(self.layer_name)
        if(len(layer_list)==0):
        # if(len(layer_list)==0):
            ### Add New layer Last Position
            layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', self.layer_name , "memory")

            pr = layer.dataProvider()

            pr.addAttributes(self.fields)
            layer.updateFields()

            # add the first point
            feature = QgsFeature()
            feature.setGeometry(QgsGeometry.fromPointXY(point))

            feature.setFields(self.fields)
            feature['message_id'] = data["message_id"]
            feature['gnss_heading'] = data["gnss_heading"]

            pr.addFeatures([feature])

            # Configure the marker.
            # simple_marker_large_circle = QgsSimpleMarkerSymbolLayer(size=4,color=Qt.darkRed)
            # marker = QgsMarkerSymbol()
            # marker.setOpacity(0.5)
            # marker.changeSymbolLayer(0, simple_marker_large_circle)

            # Configure the marker.
            svg_marker = QgsSvgMarkerSymbolLayer("/usr/share/qgis/svg/arrows/NorthArrow_11.svg")
            svg_marker.setPreservedAspectRatio(True)
            svg_marker.setSize(5)
            # svg_marker.setAngle(0)
            svg_marker.setColor(Qt.darkRed) # QColor(255,255,255)

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
            root.insertLayer(0, layer)

        else:
            layer = layer_list[0]

            # Data
            pr = layer.dataProvider()
            for feature in layer.getFeatures():
                pr.changeFeatures({feature.id():{0:self.layer_name,1:data["gnss_heading"]}}, {feature.id():QgsGeometry.fromPointXY(point)})
                layer.triggerRepaint()
                break

        return True