from qgis.core import *
import qgis.utils
from PyQt5.QtCore import *
import time
import yaml
from os.path import expanduser

class SeabotLayerLivePosition():

    ts = 0.
    east = 0.
    north = 0.
    gnss_speed = 0.
    gnss_heading = 0.
    safety_published_frequency = 0.
    safety_depth_limit = 0.
    safety_batteries_limit = 0.
    safety_depressurization = 0.
    enable_mission = 0.
    enable_depth = 0.
    enable_engine = 0.
    enable_flash = 0.
    batteries0 = 0.
    batteries1 = 0.
    batteries2 = 0.
    batteries3 = 0.
    internal_pressure = 0.
    internal_temperature = 0.
    internal_humidity = 0.
    current_waypoint = 0.
    last_cmd_received = 0.
    imei = 0.

    new_data = False

    def __init__(self):
        return

    def update_field(self, feature):
        feature['Title'] = "Last Position"
        feature['ts'] = self.ts
        feature['gnss_speed'] = self.gnss_speed
        feature['gnss_heading'] = self.gnss_heading
        feature['safety_published_frequency'] = self.safety_published_frequency
        feature['safety_depth_limit'] = self.safety_depth_limit
        feature['safety_batteries_limit'] = self.safety_batteries_limit
        feature['safety_depressurization'] = self.safety_depressurization
        feature['enable_mission'] = self.enable_mission
        feature['enable_depth'] = self.enable_depth
        feature['enable_engine'] = self.enable_engine
        feature['enable_flash'] = self.enable_flash
        feature['batteries0'] = self.batteries0
        feature['batteries1'] = self.batteries1
        feature['batteries2'] = self.batteries2
        feature['batteries3'] = self.batteries3
        feature['internal_pressure'] = self.internal_pressure
        feature['internal_temperature'] = self.internal_temperature
        feature['internal_humidity'] = self.internal_humidity
        feature['current_waypoint'] = self.current_waypoint
        feature['last_cmd_received'] = self.last_cmd_received
        feature['imei'] = self.imei

    def load_data(self):
        ### GET DATA ### 
        home = expanduser("~")
        with open(home + "/iridium/received/last_received.yaml", 'r') as stream:
            try:
                data_yaml = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)
                return False

        self.east = data_yaml['east']
        self.north = data_yaml['north']

        if(data_yaml['time'] != self.ts):
            self.new_data = True
        self.ts = data_yaml['time']
        self.gnss_speed = data_yaml['gnss_speed']
        self.gnss_heading = data_yaml['gnss_heading']
        self.safety_published_frequency = data_yaml['safety_published_frequency']
        self.safety_depth_limit = data_yaml['safety_depth_limit']
        self.safety_batteries_limit = data_yaml['safety_batteries_limit']
        self.safety_depressurization = data_yaml['safety_depressurization']
        self.enable_mission = data_yaml['enable_mission']
        self.enable_depth = data_yaml['enable_depth']
        self.enable_engine = data_yaml['enable_engine']
        self.enable_flash = data_yaml['enable_flash']
        self.batteries0 = data_yaml['batteries[0]']
        self.batteries1 = data_yaml['batteries[1]']
        self.batteries2 = data_yaml['batteries[2]']
        self.batteries3 = data_yaml['batteries[3]']
        self.internal_pressure = data_yaml['internal_pressure']
        self.internal_temperature = data_yaml['internal_temperature']
        self.internal_humidity = data_yaml['internal_humidity']
        self.current_waypoint = data_yaml['current_waypoint']
        self.last_cmd_received = data_yaml['last_cmd_received']
        self.imei = int(data_yaml['file_name'][15-6:15])

        return True

    def update_position(self):

        if(not self.load_data()):
            return False
        if(self.new_data == False):
            return True

        ### ADD DATA TO LAYER ###
        layer_name = 'Seabot - Last Position'

        ## Find if layer already exist
        layer_list = QgsProject.instance().mapLayersByName(layer_name)

        if(len(layer_list)==0):
            ### Add New layer Last Position
            layer =  QgsVectorLayer('point?crs=epsg:2154&index=yes', layer_name , "memory")

            fields = QgsFields()
            fields.append(QgsField('Title', QVariant.String))
            fields.append(QgsField('ts', QVariant.Double))
            fields.append(QgsField('gnss_speed', QVariant.Double))
            fields.append(QgsField('gnss_heading', QVariant.Double))
            fields.append(QgsField('safety_published_frequency', QVariant.Double))
            fields.append(QgsField('safety_depth_limit', QVariant.Double))
            fields.append(QgsField('safety_batteries_limit', QVariant.Double))
            fields.append(QgsField('safety_depressurization', QVariant.Double))
            fields.append(QgsField('enable_mission', QVariant.Double))
            fields.append(QgsField('enable_depth', QVariant.Double))
            fields.append(QgsField('enable_engine', QVariant.Double))
            fields.append(QgsField('enable_flash', QVariant.Double))
            fields.append(QgsField('batteries0', QVariant.Double))
            fields.append(QgsField('batteries1', QVariant.Double))
            fields.append(QgsField('batteries2', QVariant.Double))
            fields.append(QgsField('batteries3', QVariant.Double))
            fields.append(QgsField('internal_pressure', QVariant.Double))
            fields.append(QgsField('internal_temperature', QVariant.Double))
            fields.append(QgsField('internal_humidity', QVariant.Double))
            fields.append(QgsField('current_waypoint', QVariant.Double))
            fields.append(QgsField('last_cmd_received', QVariant.Double))
            fields.append(QgsField('imei', QVariant.Double))

            pr = layer.dataProvider()

            pr.addAttributes(fields)
            layer.updateFields()

            # add the first point
            feature = QgsFeature()
            point = QgsPointXY(self.east,self.north)
            feature.setGeometry(QgsGeometry.fromPointXY(point))

            feature.setFields(fields)
            self.update_field(feature)

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

            for feature in layer.getFeatures():
                point = QgsPointXY(self.east,self.north)
                feature.setGeometry(QgsGeometry.fromPointXY(point))
                self.update_field(feature)
                pr.addFeatures([feature])
                pr.deleteFeatures([feature.id()])
                break

        return True
    
