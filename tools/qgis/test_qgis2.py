from qgis.core import *
import qgis.utils
from PyQt5.QtCore import *
import time

layer_name = 'lines'

## Find if layer already exist
layer_list = QgsProject.instance().mapLayersByName(layer_name)

if(len(layer_list)==0):
    layer =  QgsVectorLayer('linestring?crs=epsg:2154&index=yes', layer_name , "memory")
else:
    layer = layer_list[0]

pr = layer.dataProvider()

# add the first point
feature = QgsFeature()

point1 = QgsPoint(145172.5293,6833438.3495)
point2 = QgsPoint(144172.5293,6833438.3495)
point3 = QgsPoint(144172.5293,6832438.3495)
feature.setGeometry(QgsGeometry.fromPolyline([point1, point2, point3]))

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

# add the layer to the canvas
layer.updateExtents()
QgsProject.instance().addMapLayer(layer)

for feature in layer.getFeatures():
	geo = feature.geometry()
	print(geo.insertVertex(point3, 0))
	feature.setGeometry(geo)

	print(feature.geometry())
	pr.addFeatures([feature])
	pr.deleteFeatures([feature.id()])
	break