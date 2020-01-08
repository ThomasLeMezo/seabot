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

class LayerSeabot():

    fields = QgsFields()
    group_name = 'Seabot Live ('
    layer_track = 'Seabot track'
    layer_pose = 'Seabot pose'
    surface = False

    def __init__(self, imei):
        self.group_name += str(imei) + ")"
        self.fields.append(QgsField('Title', QVariant.String))
        self.fields.append(QgsField('gnss_heading', QVariant.Double))
        return

    def update_seabot_layer(self, seabotMission):
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

    def update_seabot_pose(self, seabotMission):
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


    # def update_iridium_track(self):
    #         ### ADD DATA TO LAYER ###
    #         layer_name = 'Seabot - Iridium Track'

    #         ## Find if layer already exist
    #         layer_list = QgsProject.instance().mapLayersByName(layer_name)
    #         point = QgsPoint(self.iridium_map_file['east'], self.iridium_map_file['north'])

    #         if(len(layer_list)==0):
    #             ### Add New layer Last Position
    #             layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', layer_name , "memory")

    #             pr = layer.dataProvider()

    #             # add the first point
    #             feature = QgsFeature()

    #             feature.setGeometry(QgsGeometry.fromPolyline([point]))
    #             pr.addFeatures([feature])

    #             # Configure the marker.
    #             marker_line = QgsSimpleLineSymbolLayer()
    #             marker_point = QgsMarkerLineSymbolLayer()
    #             marker_point.setPlacement(QgsMarkerLineSymbolLayer.Vertex)

    #             marker = QgsLineSymbol()
    #             marker.changeSymbolLayer(0, marker_line)
    #             marker.appendSymbolLayer(marker_point)

    #             renderer = QgsSingleSymbolRenderer(marker)
    #             layer.setRenderer(renderer)
    #             layer.updateExtents()
    #             QgsProject.instance().addMapLayer(layer)

    #         else:
    #             layer = layer_list[0]
    #             pr = layer.dataProvider()

    #             for feature in layer.getFeatures():
    #                 geo = feature.geometry()
    #                 geo.insertVertex(point, 0)
    #                 pr.changeGeometryValues({feature.id():geo})
    #                 layer.triggerRepaint()
    #                 break

    #         return True

    #     def update_iridium_position(self):
    #         ### ADD DATA TO LAYER ###
    #         layer_name = 'Seabot - Last Iridium Position'

    #         ## Find if layer already exist
    #         layer_list = QgsProject.instance().mapLayersByName(layer_name)
    #         point = QgsPointXY(self.iridium_map_file['east'], self.iridium_map_file['north'])

    #         if(len(layer_list)==0):
    #             ### Add New layer Last Position
    #             layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', layer_name , "memory")

    #             pr = layer.dataProvider()

    #             if(self.first_load==True):
    #                 self.add_fields_from_yaml()
    #                 self.first_load = False

    #             pr.addAttributes(self.fields)
    #             layer.updateFields()

    #             # add the first point
    #             feature = QgsFeature()
    #             feature.setGeometry(QgsGeometry.fromPointXY(point))

    #             feature.setFields(self.fields)
    #             feature['Title'] = "Last Position Received"
    #             feature['min_since_received'] = -1
    #             self.update_feature(feature)

    #             pr.addFeatures([feature])

    #             # Configure the marker.
    #             svg_marker = QgsSvgMarkerSymbolLayer("/usr/share/qgis/svg/arrows/NorthArrow_11.svg")
    #             svg_marker.setPreservedAspectRatio(True)
    #             svg_marker.setSize(5)
    #             # svg_marker.setAngle(0)
    #             svg_marker.setColor(Qt.red) # QColor(255,255,255)

    #             marker = QgsMarkerSymbol()
    #             marker.changeSymbolLayer(0, svg_marker)

    #             prop=QgsProperty()
    #             prop.setField("gnss_heading")
    #             marker.setDataDefinedAngle(prop) #QgsProperty () 

    #             renderer = QgsSingleSymbolRenderer(marker)
    #             layer.setRenderer(renderer)

    #             # Text
    #             buffer_settings = QgsTextBufferSettings()
    #             buffer_settings.setEnabled(True)
    #             buffer_settings.setSize(6)
    #             buffer_settings.setColor(Qt.white)

    #             text_format = QgsTextFormat()
    #             text_format.setFont(QFont("Ubuntu", 8))
    #             text_format.setSize(8)
    #             text_format.setBuffer(buffer_settings)

    #             layer_settings  = QgsPalLayerSettings()
    #             layer_settings.setFormat(text_format)
    #             layer_settings.fieldName = "concat(to_string(min_since_received),' min')"
    #             layer_settings.isExpression = True
    #             layer_settings.placement = QgsPalLayerSettings.OverPoint
    #             layer_settings.predefinedPositionOrder = QgsPalLayerSettings.TopMiddle
    #             layer_settings.yOffset = -8
    #             layer_settings.enabled = True

    #             layer_label = QgsVectorLayerSimpleLabeling(layer_settings)
    #             layer.setLabelsEnabled(True)
    #             layer.setLabeling(layer_label)
    #             layer.triggerRepaint()

    #             # add the layer to the canvas
    #             layer.updateExtents()
    #             QgsProject.instance().addMapLayer(layer)

    #         else:
    #             layer = layer_list[0]
    #             pr = layer.dataProvider()

    #             for feature in layer.getFeatures():
    #                 pr.changeFeatures({feature.id():self.get_update_map_attribute()}, {feature.id():QgsGeometry.fromPointXY(point)})
    #                 layer.updateExtents()
    #                 layer.triggerRepaint()
    #                 break

    #         return True