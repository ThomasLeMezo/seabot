#!/usr/bin/python
# -*- coding: utf-8 -*-

import os


def technical_datas(file, IMEI):
	import subprocess	
	dir_path = os.path.dirname(os.path.realpath(__file__))
	print("file : ", file, " path = ", dir_path)

	call_string = dir_path[:-3]+ "instruments/"+ str(IMEI)+ "/technical_datas/" + str(file)
	print(call_string)
	subprocess.call(["rosrun", "seabot_iridium", "decode_iridium", call_string])
	subprocess.call(["mv", "log_TDT1.txt", "/mnt/webperso/iridium/"])
	return "finit"

global logs
logs = technical_datas(file, IMEI)

