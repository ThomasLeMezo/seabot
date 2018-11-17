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

class SeabotLayerLivePosition():

    first_load = True    
    fields = QgsFields()

    iridium_file = expanduser("~") + "/iridium/received/last_received.yaml"
    new_iridium_data = False
    iridium_file_ts = 0.
    iridium_map_file = {}

    forcast_track = np.empty(0)
    forcast_file_ts = 0.
    forcast_filename = expanduser("~") + "/iridium/seabot_forcast.txt"
    new_forcast_data = False
    forcast_start_time = 0.
    forcast_dt = 0.

    forcast_polygon_track = np.empty(0)
    forcast_polygon_file_ts = 0.
    forcast_polygon_filename = expanduser("~") + "/iridium/seabot_forcast_polygon.txt"
    new_forcast_polygon_data = False
    forcast_polygon_start_time = 0.
    forcast_polygon_dt = 0.
    last_offset_polygon = 0

    def __init__(self):
        self.fields.append(QgsField('Title', QVariant.String))
        self.fields.append(QgsField('min_since_received', QVariant.Double))
        return

    def add_fields_from_yaml(self):
        for key, value in self.iridium_map_file.items():
            if(isinstance(value, str)):
                self.fields.append(QgsField(key, QVariant.String))
            else:
                self.fields.append(QgsField(key, QVariant.Double))

    def get_update_map_attribute(self):
        map = {}
        for i in range(self.fields.size()):
            if self.fields.at(i).name() in self.iridium_map_file:
                map[i] = self.iridium_map_file[self.fields.at(i).name()]
            elif self.fields.at(i).name() == 'min_since_received':
                map[i] = round((time.clock_gettime(time.CLOCK_REALTIME) - self.iridium_map_file["ts"])/60.)
        return map

    def update_feature(self, feature):
        for i in range(feature.fields().size()):
            key = feature.fields().at(i).name()
            if key in self.iridium_map_file:
                feature[key] = self.iridium_map_file[key]
            # else:
            #     print("Error : ", key)

    def load_data(self):
        ### GET DATA ### 
        with open(self.iridium_file, 'r') as stream:
            try:
                self.iridium_map_file = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return False

            new_time = self.iridium_map_file['ts']
            if(self.iridium_file_ts != new_time):
                self.new_iridium_data = True
                self.iridium_file_ts = new_time

        self.load_forcast_track()
        self.load_forcast_polygon()

        return True

    def load_forcast_track(self):
        if(not os.path.isfile(self.forcast_filename)):
            return

        new_ts = os.stat(self.forcast_filename).st_mtime
        if(new_ts != self.forcast_file_ts):
            self.forcast_track = np.loadtxt(self.forcast_filename)
            self.forcast_file_ts = new_ts
            if(np.size(self.forcast_track)>0):
                self.new_forcast_data = True

                self.forcast_start_time = self.forcast_track[0][0]
                self.forcast_dt = self.forcast_track[1][0] - self.forcast_track[0][0]

    def load_forcast_polygon(self):
        if(not os.path.isfile(self.forcast_polygon_filename)):
            return

        new_ts = os.stat(self.forcast_polygon_filename).st_mtime
        if(new_ts != self.forcast_polygon_file_ts):
            self.forcast_polygon_file_ts = new_ts
            self.forcast_polygon_track = np.loadtxt(self.forcast_polygon_filename)
            if(np.size(self.forcast_polygon_track)>0):
                self.new_forcast_polygon_data = True

                self.forcast_polygon_start_time = self.forcast_polygon_track[0][0]
                self.forcast_polygon_dt = self.forcast_polygon_track[1][0] - self.forcast_polygon_track[0][0]

    def update(self):
        if(not self.load_data()):
            return False

        status = True

        if(self.new_iridium_data == True):
            status &= self.update_iridium_track()
            self.new_iridium_data = False

        status &= self.update_iridium_position()

        if(self.new_forcast_data == True):
            status &= self.update_forecast_track()
            self.new_forcast_data = False

        if(self.forcast_start_time != 0):
            self.update_forcast_position()

        if(self.forcast_polygon_start_time != 0):
            self.update_forcast_polygon()
        
        return status

    def update_forecast_track(self):
        if(np.size(self.forcast_track)==0):
            return

        layer_name = 'Seabot - Forcast Track'
        layer_list = QgsProject.instance().mapLayersByName(layer_name)

        feature = QgsFeature()
        point_list = []
        for i in range(np.shape(self.forcast_track)[0]):
            point_list.append(QgsPoint(self.forcast_track[i][1], self.forcast_track[i][2]))
        geo = QgsGeometry.fromPolyline(point_list)
        
        if(len(layer_list)==0):
            layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', layer_name , "memory")
            pr = layer.dataProvider()
            feature.setGeometry(geo)
            
            pr.addFeatures([feature])
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer)
        else:
            layer = layer_list[0]
            pr = layer.dataProvider()

            for f in layer.getFeatures():
                pr.changeGeometryValues({f.id():geo})
                layer.triggerRepaint()
                break

        return True

    def update_forcast_polygon(self):
        if(np.size(self.forcast_polygon_track)==0):
            return

        ts = time.clock_gettime(time.CLOCK_REALTIME)
        ### ADD DATA TO LAYER ###
        layer_name = 'Seabot - Forcast Polygon'

        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(layer_name)

        offset = int(min(max(round((ts - self.forcast_polygon_start_time)/self.forcast_polygon_dt), 0), np.shape(self.forcast_polygon_track)[0]-1))
        if(self.last_offset_polygon != offset):
            self.last_offset_polygon = offset

            feature = QgsFeature()
            point_list = []
            
            for i in range(int((np.shape(self.forcast_polygon_track)[1]-1)/2)):
                point_list.append(QgsPointXY(self.forcast_polygon_track[offset][2*i+1], self.forcast_polygon_track[offset][2*i+2]))
            geo = QgsGeometry.fromPolygonXY([point_list])

            if(len(layer_list)==0):
                ### Add New layer Last Position
                layer =  QgsVectorLayer('Polygon?crs=epsg:2154&index=yes', layer_name , "memory")

                pr = layer.dataProvider()
                feature.setGeometry(geo)
                
                pr.addFeatures([feature])
                layer.updateExtents()

                # Configure the marker.
                marker_polygon = QgsSimpleLineSymbolLayer()
                marker_polygon.setColor(Qt.blue)

                marker_point = QgsMarkerLineSymbolLayer()
                marker_point.setPlacement(QgsMarkerLineSymbolLayer.Vertex)
                marker_point.setColor(Qt.blue)

                marker = QgsFillSymbol()
                marker.changeSymbolLayer(0, marker_polygon)
                marker.appendSymbolLayer(marker_point)
                
                renderer = QgsSingleSymbolRenderer(marker)
                layer.setRenderer(renderer)
                layer.updateExtents()
                QgsProject.instance().addMapLayer(layer)

            else:
                layer = layer_list[0]
                pr = layer.dataProvider()

                for f in layer.getFeatures():
                    pr.changeGeometryValues({f.id():geo})
                    layer.triggerRepaint()
                    break
        return True


    def update_forcast_position(self):
        if(np.size(self.forcast_track)==0):
            return

        ts = time.clock_gettime(time.CLOCK_REALTIME)
        ### ADD DATA TO LAYER ###
        layer_name = 'Seabot - Forcast Position'

        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(layer_name)

        offset = int(min(max(round((ts - self.forcast_start_time)/self.forcast_dt), 0), np.shape(self.forcast_track)[0]-1))
        point = QgsPointXY(self.forcast_track[offset][1], self.forcast_track[offset][2])

        if(len(layer_list)==0):
            ### Add New layer Last Position
            layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', layer_name , "memory")

            pr = layer.dataProvider()

            # add the first point
            feature = QgsFeature()
            feature.setGeometry(QgsGeometry.fromPointXY(point))
            pr.addFeatures([feature])

            # Configure the marker.
            svg_marker = QgsSvgMarkerSymbolLayer("/usr/share/qgis/svg/crosses/Cross1.svg")
            svg_marker.setPreservedAspectRatio(True)
            svg_marker.setSize(5)
            svg_marker.setColor(Qt.green) # QColor(255,255,255)
            marker = QgsMarkerSymbol()
            marker.changeSymbolLayer(0, svg_marker)

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

    def update_iridium_track(self):
        ### ADD DATA TO LAYER ###
        layer_name = 'Seabot - Iridium Track'

        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(layer_name)
        point = QgsPoint(self.iridium_map_file['east'], self.iridium_map_file['north'])

        if(len(layer_list)==0):
            ### Add New layer Last Position
            layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', layer_name , "memory")

            pr = layer.dataProvider()

            # add the first point
            feature = QgsFeature()

            feature.setGeometry(QgsGeometry.fromPolyline([point]))
            pr.addFeatures([feature])

            # Configure the marker.
            marker_line = QgsSimpleLineSymbolLayer()
            marker_point = QgsMarkerLineSymbolLayer()
            marker_point.setPlacement(QgsMarkerLineSymbolLayer.Vertex)

            marker = QgsLineSymbol()
            marker.changeSymbolLayer(0, marker_line)
            marker.appendSymbolLayer(marker_point)

            renderer = QgsSingleSymbolRenderer(marker)
            layer.setRenderer(renderer)
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer)

        else:
            layer = layer_list[0]
            pr = layer.dataProvider()

            for feature in layer.getFeatures():
                geo = feature.geometry()
                geo.insertVertex(point, 0)
                pr.changeGeometryValues({feature.id():geo})
                layer.triggerRepaint()
                break

        return True

    def update_iridium_position(self):
        ### ADD DATA TO LAYER ###
        layer_name = 'Seabot - Last Iridium Position'

        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(layer_name)
        point = QgsPointXY(self.iridium_map_file['east'], self.iridium_map_file['north'])

        if(len(layer_list)==0):
            ### Add New layer Last Position
            layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', layer_name , "memory")

            pr = layer.dataProvider()

            if(self.first_load==True):
                self.add_fields_from_yaml()
                self.first_load = False

            pr.addAttributes(self.fields)
            layer.updateFields()

            # add the first point
            feature = QgsFeature()
            feature.setGeometry(QgsGeometry.fromPointXY(point))

            feature.setFields(self.fields)
            feature['Title'] = "Last Position Received"
            feature['min_since_received'] = -1
            self.update_feature(feature)

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
            buffer_settings = QgsTextBufferSettings()
            buffer_settings.setEnabled(True)
            buffer_settings.setSize(6)
            buffer_settings.setColor(Qt.white)

            text_format = QgsTextFormat()
            text_format.setFont(QFont("Ubuntu", 8))
            text_format.setSize(8)
            text_format.setBuffer(buffer_settings)

            layer_settings  = QgsPalLayerSettings()
            layer_settings.setFormat(text_format)
            layer_settings.fieldName = "concat(to_string(min_since_received),' min')"
            layer_settings.placement = QgsPalLayerSettings.OverPoint
            layer_settings.predefinedPositionOrder = QgsPalLayerSettings.TopMiddle
            layer_settings.yOffset = -8
            layer_settings.enabled = True

            layer_label = QgsVectorLayerSimpleLabeling(layer_settings)
            layer.setLabelsEnabled(True)
            layer.setLabeling(layer_label)
            layer.triggerRepaint()

            # add the layer to the canvas
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer)

        else:
            layer = layer_list[0]
            pr = layer.dataProvider()

            for feature in layer.getFeatures():
                pr.changeFeatures({feature.id():self.get_update_map_attribute()}, {feature.id():QgsGeometry.fromPointXY(point)})
                layer.updateExtents()
                layer.triggerRepaint()
                break

        return True
    
