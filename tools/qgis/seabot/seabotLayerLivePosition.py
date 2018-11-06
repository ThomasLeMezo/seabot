from qgis.core import *
import qgis.utils
from PyQt5.QtCore import *
import time
import oyaml as yaml
import os
from os.path import expanduser
import numpy as np
import time

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

    def __init__(self):
        self.fields.append(QgsField('Title', QVariant.String))
        return

    def add_fields_from_yaml(self):
        for key, value in self.iridium_map_file.items():
            self.fields.append(QgsField(key, QVariant.Double))

    def get_update_map_attribute(self):
        map = {}
        for i in range(self.fields.size()):
            if self.fields.at(i).name() in self.iridium_map_file:
                map[str(i)] = self.iridium_map_file[self.fields.at(i).name()]
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

        self.load_forcast_track()

        return True

    def load_forcast_track(self):
        if(not os.path.isfile(self.forcast_filename)):
            return

        new_ts = os.stat(self.forcast_filename).st_mtime
        if(new_ts != self.forcast_file_ts):
            self.forcast_track = np.loadtxt(self.forcast_filename)
            new_ts = self.forcast_file_ts
            self.new_forcast_data = True

            self.forcast_start_time = self.forcast_track[0][0]
            self.forcast_dt = self.forcast_track[1][0] - self.forcast_track[0][0]

    def update(self):
        if(not self.load_data()):
            return False

        status = True

        if(self.new_iridium_data == True):
            status &= self.update_iridium_track()
            status &= self.update_iridium_position()
            self.new_iridium_data = False

        if(self.new_forcast_data == True):
            status &= self.update_forecast_track()
            self.new_forcast_data = False

        if(self.forcast_start_time != 0):
            self.update_forcast_position()
        
        return status

    def update_forecast_track(self):
        if(np.size(self.forcast_track)==0):
            return

        layer_name = 'Seabot - Forcast Track'
        layer_list = QgsProject.instance().mapLayersByName(layer_name)

        if(len(layer_list)==0):
            layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', layer_name , "memory")
            pr = layer.dataProvider()
            
            feature = QgsFeature()
            point_list = []
            for i in range(np.shape(self.forcast_track)[0]):
                point_list.append(QgsPoint(self.forcast_track[i][1], self.forcast_track[i][2]))

            feature.setGeometry(QgsGeometry.fromPolyline(point_list))
            pr.addFeatures([feature])
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer)
        # else:
        #     layer = layer_list[0]
        #     pr = layer.dataProvider()

        #     for feature in layer.getFeatures():
        #         geo = feature.geometry()
        #         geo.insertVertex(point, 0)
        #         feature.setGeometry(geo)

        #         pr.addFeatures([feature])
        #         pr.deleteFeatures([feature.id()])
        #         break

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
                feature.setGeometry(geo)

                pr.addFeatures([feature])
                pr.deleteFeatures([feature.id()])
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

            layer.setCustomProperty("labeling/fieldName", "Title" )
            layer.setCustomProperty("labeling/placement", QgsPalLayerSettings.OverPoint)
            layer.setCustomProperty("labeling/fontSize","8" )
            layer.setCustomProperty("labeling/enabled","True" )
            layer.triggerRepaint()

            # add the layer to the canvas
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer)

        else:
            layer = layer_list[0]
            pr = layer.dataProvider()

            for feature in layer.getFeatures():
                pr.changeFeatures({feature.id():self.get_update_map_attribute()}, {feature.id():QgsGeometry.fromPointXY(point)})
                # pr.changeAttributeValues()
                break
                # point = QgsPointXY(self.east,self.north)
                # feature.setGeometry(QgsGeometry.fromPointXY(point))
                # self.update_field(feature)
                # pr.addFeatures([feature])
                # pr.deleteFeatures([feature.id()])
                break

        return True
    
