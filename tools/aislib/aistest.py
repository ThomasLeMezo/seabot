import aislib

import time
import serial
from math import *

# sudo socat pty,link=/dev/ttyVout,raw,echo=0 pty,link=/dev/ttyVin,raw,echo=0
ser = serial.Serial('/dev/ttyVin', 115200, rtscts=True, dsrdtr=True)
dt = 0.1
t = 0.0
while 1:
    t+=dt
    #
    # Tests for Message Type 1
    #
    aismsg = aislib.AISPositionReportMessage(
        mmsi = 0,
        status = 8,
        sog = 75, # vitesse en dixieme de noeuds
        pa = 1, # position accuracy (1bit)
        lat = int(round((48*60+22.437+sin(t+0.2))*10000)), # en 1/10 000 de minutes, Ouest negatif
        lon = -int(round((4*60+29.396+sin(t))*10000)),
        cog = int(abs(atan2(sin(t), sin(t+0.2)))*3600.0/pi), # max 3600
        ts = 40, # max 511
        raim = 1,
        comm_state = 82419   
    )
    ais = aislib.AIS(aismsg)
    payload = ais.build_payload(False)
    print payload
    ser.write(payload+'\n')

    print "nav status: %d" % aismsg.get_attr("status")

    #print aismsg.id
    #print aismsg.repeat

    #bitstr = aismsg.build_bitstream() 
    #print bitstr.bin
    #print bitstr.len

    # # 
    # # Tests for Message Type 24 Format A
    # #
    # del aismsg
    # del aismsg2
    # #print aislib.AISString2Bits('ABC')
    # aismsg = aislib.AISStaticDataReportAMessage(mmsi=237772000,shipname=aislib.AISString2Bits('SEABOT').int)
    # aismsg = aislib.AISStaticDataReportAMessage(mmsi=237772000,shipname='SEABOT')
    # ais = aislib.AIS(aismsg)
    # payload = ais.build_payload(False)
    # print payload
    # ser.write(payload+'\n')

    # aismsg2 = ais.decode(payload)
    # ais2 = aislib.AIS(aismsg2)
    # payload2 = ais2.build_payload(False)
    # assert payload ==  payload2

    # # 
    # # Tests for Message Type 24 Format B
    # #
    # aismsg = aislib.AISStaticDataReportBMessage(mmsi=237772000,shiptype=36,
    #          vendorid='DIY',
    #          callsign='SVXYZ',
    #          to_bow=5,to_stern=5,to_port=1,to_starboard=1)
    # ais = aislib.AIS(aismsg)
    # payload = ais.build_payload(False)
    # print payload
    # ser.write(payload+'\n')

    # aismsg2 = ais.decode(payload)
    # ais2 = aislib.AIS(aismsg2)
    # payload2 = ais2.build_payload(False)
    # assert payload ==  payload2

    # 
    # Tests for Message Type 5
    #
    # print ' Tests for Message Type 5'
    # aismsg = aislib.AISStaticAndVoyageReportMessage(mmsi=0,
    #          imo=0, 
    #          callsign='SEABOT',
    #          shipname='SEABOT 1',
    #          shiptype=36,
    #          to_bow=5,to_stern=5,to_port=1,to_starboard=1, draught=10,
    #          epfd=1, month=5, day=14, hour=20, minute=15,
    #          destination='OUESSANT')
    # ais = aislib.AIS(aismsg)
    # payload = ais.build_payload(False)
    # print payload
    # ser.write(payload+'\n')

    # aismsg2 = ais.decode(payload)
    # ais2 = aislib.AIS(aismsg2)
    # payload2 = ais2.build_payload(False)
    # assert payload ==  payload2
    time.sleep(0.5)
