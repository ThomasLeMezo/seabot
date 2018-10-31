from qgis.core import *
import qgis.utils
from PyQt5.QtCore import *

layer_name = 'points'


## Find if layer already exist
layer_list = QgsProject.instance().mapLayersByName(layer_name)

if(len(layer_list)==0):
    layer =  QgsVectorLayer('Point?crs=epsg:2154&index=yes', layer_name , "memory")
else:
    layer = layer_list[0]

pr = layer.dataProvider()

field_title = QgsField('Title', QVariant.String)
field_heading = QgsField('heading', QVariant.Double)
field_speed = QgsField('speed', QVariant.Double)
field_battery1 = QgsField('battery1', QVariant.Double)

fields = QgsFields()
fields.append(field_title)
fields.append(field_heading)
fields.append(field_speed)
fields.append(field_battery1)

pr.addAttributes(fields)
layer.updateFields()

# add the first point
feature = QgsFeature()

point1 = QgsPointXY(145172.5293,6833438.3495)
feature.setGeometry(QgsGeometry.fromPointXY(point1))

feature.setFields(fields)
feature['Title'] = "Point 1"
feature['heading'] = 90.
feature['speed'] = 0.1
feature['battery1'] = 12.4

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
prop.setField("heading")
marker.setDataDefinedAngle(prop) #QgsProperty () 

renderer = QgsSingleSymbolRenderer(marker)
layer.setRenderer(renderer)

layer.setCustomProperty("labeling/fieldName", "Title" )
layer.setCustomProperty("labeling/placement", QgsPalLayerSettings.OverPoint)
layer.setCustomProperty("labeling/fontSize","8" )
layer.setCustomProperty("labeling/enabled","true" )
layer.triggerRepaint()

# add the layer to the canvas
layer.updateExtents()
QgsProject.instance().addMapLayer(layer)
