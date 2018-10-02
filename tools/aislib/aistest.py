import aislib

import time
import serial
from math import *
import yaml
from os.path import expanduser
from pyproj import Proj, transform

outProj = Proj(init='epsg:4326') #WGS84
inProj = Proj(init='epsg:2154') #L93

# sudo socat pty,link=/dev/ttyVout,raw,echo=0 pty,link=/dev/ttyVin,raw,echo=0
ser = serial.Serial('/dev/ttyVin', 115200, rtscts=True, dsrdtr=True)
dt = 0.1
t = 0.0
while 1:
    t+=dt
    
    home = expanduser("~")
    with open(home + "/last_tdt1.yaml", 'r') as stream:
        try:
            data_yaml = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    
    east = data_yaml['east']
    north = data_yaml['north']

    # current_waypoint = data_yaml['current_waypoint']
    longitude,latitude = transform(inProj,outProj,east, north)
    aismsg = aislib.AISPositionReportMessage(
        mmsi = 0,
        status = 8,
        sog = int(round(data_yaml['gnss_speed']*19.4384)), # vitesse en dixieme de noeuds
        pa = 1, # position accuracy (1bit)
        lat = int(round(ceil(latitude)*60+(latitude-ceil(latitude))*100.)), # en 1/10 000 de minutes, Ouest negatif
        lon = int(round(ceil(longitude)*60+(longitude-ceil(longitude))*100.)),
        cog = int(data_yaml['gnss_heading']*10.), # max 3600
        ts = 40, # max 511
        raim = 1,
        comm_state = 82419   
    )
    ais = aislib.AIS(aismsg)
    payload = ais.build_payload(False)
    # print payload
    ser.write(payload+'\n')

    time.sleep(1.0)
