from qgis.core import *
import qgis.utils

layer_name = 'test'
layer =  QgsVectorLayer('Point?crs=epsg:2154&index=yes', layer_name , "memory")

symbol = QgsSymbolV2.defaultSymbol(layer.geometryType())

svgStyle = {}
svgStyle['fill'] = '#0000ff'
svgStyle['name'] = determinedIcon
svgStyle['outline'] = '#000000'
svgStyle['outline-width'] = '6.8'
svgStyle['size'] = '6'

symbol_layer = QgsSvgMarkerSymbolLayerV2.create(svgStyle)


# define categories
categories = []
determinedIcon = ""
for unique_value in unique_values:
    # initialize the default symbol for this geometry type
    

    for svgMarker in icon:
        if svgMarker[0] == unique_value:
            determinedIcon = svgMarker[1]

    

    # replace default symbol layer with the configured one
    if symbol_layer is not None:
        symbol.changeSymbolLayer(0, symbol_layer)

    # create renderer object
    category = QgsRendererCategoryV2(unique_value, symbol, str(unique_value))
    # entry for the list of category items
    categories.append(category)

# create renderer object
renderer = QgsCategorizedSymbolRendererV2('attribureName', categories)

# assign the created renderer to the layer
if renderer is not None:
    layer.setRendererV2(renderer)

layer.triggerRepaint()