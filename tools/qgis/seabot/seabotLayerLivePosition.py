from qgis.core import *
import qgis.utils
from PyQt5.QtCore import *
import time
import oyaml as yaml
from os.path import expanduser

class SeabotLayerLivePosition():

    map_file = {}
    last_time = 0.

    new_data = False
    fields = QgsFields()

    first_load = True

    def __init__(self):
        self.fields.append(QgsField('Title', QVariant.String))
        return

    def add_fields_from_yaml(self):
        for key, value in self.map_file.items():
            self.fields.append(QgsField(key, QVariant.Double))

    def get_update_map_attribute(self):
        map = {}
        for i in range(self.fields.size()):
            if self.fields.at(i).name() in self.map_file:
                map[str(i)] = self.map_file[self.fields.at(i).name()]
        return map

    def update_feature(self, feature):
        for i in range(feature.fields().size()):
            key = feature.fields().at(i).name()
            if key in self.map_file:
                feature[key] = self.map_file[key]
            else:
                print("Error : ", key)

    def load_data(self):
        ### GET DATA ### 
        home = expanduser("~")
        with open(home + "/iridium/received/last_received.yaml", 'r') as stream:
            try:
                self.map_file = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return False

            new_time = self.map_file['ts']
            if(self.last_time != new_time):
                self.new_data = True
                self.last_time = new_time

        return True

    def update(self):
        if(not self.load_data()):
            return False
        if(self.new_data == False):
            return True

        status = True
        status &= self.update_track()
        status &= self.update_position()

        self.new_data = False
        return status

    def update_track(self):
        

        ### ADD DATA TO LAYER ###
        layer_name = 'Seabot - Track'

        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(layer_name)
        point = QgsPoint(self.map_file['east'], self.map_file['north'])

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

    def update_position(self):

        ### ADD DATA TO LAYER ###
        layer_name = 'Seabot - Last Position'

        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(layer_name)
        point = QgsPointXY(self.map_file['east'], self.map_file['north'])

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
            feature['Title'] = "Last Position"
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

            layer.setCustomProperty("labeling/fieldName", "imei" )
            layer.setCustomProperty("labeling/placement", QgsPalLayerSettings.OverPoint)
            layer.setCustomProperty("labeling/fontSize","8" )
            layer.setCustomProperty("labeling/enabled","true" )
            layer.triggerRepaint()

            # add the layer to the canvas
            layer.updateExtents()
            QgsProject.instance().addMapLayer(layer)

        else:
            layer = layer_list[0]
            pr = layer.dataProvider()

            # for feature in layer.getFeatures():
            #     pr.changeFeatures(self.get_update_map_attribute(), {feature.id():QgsGeometry.fromPointXY(point)})
            #     # pr.changeAttributeValues()
            #     break
            #     point = QgsPointXY(self.east,self.north)
            #     feature.setGeometry(QgsGeometry.fromPointXY(point))
            #     self.update_field(feature)
            #     pr.addFeatures([feature])
            #     pr.deleteFeatures([feature.id()])
            #     break

        return True
    
