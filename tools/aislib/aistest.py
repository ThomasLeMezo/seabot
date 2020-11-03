import aislib, time, serial, yaml, datetime
from math import *
from os.path import expanduser
from pyproj import Proj, transform
import subprocess
import numpy as np


outProj = Proj(init='epsg:4326') #WGS84
inProj = Proj(init='epsg:2154') #L93

last_east = 0
last_north = 0
traj_forcast = []
id_time = 0

# sudo socat pty,link=/dev/ttyVout,raw,echo=0 pty,link=/dev/ttyVin,raw,echo=0
ser = serial.Serial('/dev/ttyVin', 115200, rtscts=True, dsrdtr=True)
while 1:
    home = expanduser("~")
    with open(home + "/iridium/received/last_received.yaml", 'r') as stream:
        try:
            data_yaml = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)

    
    east = data_yaml['east']
    north = data_yaml['north']
    imei = int(data_yaml['file_name'][15-6:15])
    current_waypoint = data_yaml['current_waypoint']
    ts = int(data_yaml['time'])
    msg_date = datetime.datetime.utcfromtimestamp(ts)

    ## Trajectory
    # if(last_east != east and last_north != north):
    #     last_east = east
    #     last_north = north
    #     subprocess.call(["/home/lemezoth/workspaceQT/invariant-lib/build/build-debug/examples/cpp/seabot/seabot_live/seabot_live", str(ts), str(north), str(east)])
    #     id_time = 0
    #     traj_forcast = np.loadtxt("/home/lemezoth/seabot_forcast.txt")
    #     print(traj_forcast)

    # print(msg_id)

    ship_name = "SEABOT"
    if(imei == 981770):
        ship_name += "1"
    elif(imei == 396100):
        ship_name += "2"
    elif(imei == 392110):
        ship_name += "3"

    # current_waypoint = data_yaml['current_waypoint']
    longitude,latitude = transform(inProj,outProj,east, north)
    # print(longitude, latitude)
    aismsg1 = aislib.AISPositionReportMessage(
        mmsi = imei,
        sog = min(int(round(data_yaml['gnss_speed']*19.4384)), 1022), # vitesse en dixieme de noeuds
        pa = 1,
        lat = int(round(latitude*60e4)),
        lon = int(round(longitude*60e4)),
        cog = min(int(data_yaml['gnss_heading']*10.), 3599)#, # max 3600
        # ts = msg_date.second
    )

    ais1 = aislib.AIS(aismsg1)
    payload1 = ais1.build_payload(False)
    ser.write(payload1+'\n')

    aismsg2 = aislib.AISStaticAndVoyageReportMessage(
            shipname = ship_name,
            mmsi = imei,
            epfd = 3,
            month = msg_date.month,
            day = msg_date.day,
            hour = msg_date.hour,
            minute = msg_date.minute,
            destination = "WAYPOINT " + str(current_waypoint)
        )
    ais2 = aislib.AIS(aismsg2)
    payload2 = ais2.build_payload(False)
    ser.write(payload2+'\n')

    # if(traj_forcast != []):
    #     # Find id_time
    #     t = time.time()
    #     if(t<traj_forcast[0][id_time]):
    #         for i in range(id_time, len(traj_forcast[0])):
    #             if(t>traj_forcast[0][i]):
    #                 id_time = i
    #                 break
    #     # Create msg
    #     forcast_longitude,forcast_latitude = transform(inProj,outProj,traj_forcast[1][id_time], traj_forcast[2][id_time])

    #     aismsg_forcast = aislib.AISPositionReportMessage(
    #         mmsi = 000,
    #         lat = int(round(forcast_latitude*60e4)),
    #         lon = int(round(forcast_longitude*60e4)),
    #     )
    #     ais_forcast = aislib.AIS(aismsg_forcast)
    #     payload_forcast = ais_forcast.build_payload(False)
    #     ser.write(payload_forcast+'\n')

    time.sleep(1.0)
