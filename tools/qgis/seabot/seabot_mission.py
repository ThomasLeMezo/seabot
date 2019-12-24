# -*- coding: utf-8 -*-°

import os, time, datetime, sys
import xml.etree.ElementTree as ET

class Seabotwaypoint():

	def __init__(self, time_end, duration, depth, east, north, limit_velocity, approach_velocity, enable_thrusters):
		self.time_end = time_end
		self.duration = duration
		self.depth = depth
		self.east = east
		self.north = north
		self.limit_velocity = limit_velocity
		self.approach_velocity = approach_velocity
		self.enable_thrusters = enable_thrusters

	def __str__(self):
		s = ""
		s += "time_end" + "=" + str(self.time_end) + "\n"
		s += "duration" + "=" + str(self.duration) + "\n"
		s += "depth" + "=" + str(self.depth) + "\n"
		s += "east" + "=" + str(self.east) + "\n"
		s += "north" + "=" + str(self.north) + "\n"
		s += "limit_velocity" + "=" + str(self.limit_velocity) + "\n"
		s += "approach_velocity" + "=" + str(self.approach_velocity) + "\n"
		s += "enable_thrusters" + "=" + str(self.enable_thrusters) + "\n"
		return s

class SeabotMission():

	def __init__(self):
		self.waypoint_list = []
		self.start_time_utc = None
		self.current_wp_id = 0
		self.end_time = None

	def __str__(self):
		s = ""
		for wp in self.waypoint_list:
			s+=wp.__str__()+"\n\n"
		return s

	def add_waypoint(self, wp):
		self.waypoint_list.append(wp)

	def get_current_wp(self):
		d = datetime.datetime.now()
		t = now.timestamp()

		if((len(wyapoint_list)>current_wp_id+1) and (waypoint_list[current_wp_id+1].time_end<t)):
			current_wp_id+=1
		return waypoint_list[current_wp_id+1]

	def load_mission_xml(self, filename):
		self.waypoint_list.clear()
		tree = ET.parse(filename)
		root = tree.getroot()

		child_offset = root.find("offset/start_time_utc")
		self.start_time_utc = datetime.datetime(year=int(child_offset.find("year").text),
									month=int(child_offset.find("month").text),
									day=int(child_offset.find("day").text),
									hour=int(child_offset.find("hour").text),
									minute=int(child_offset.find("min").text))
		self.end_time = self.start_time_utc

		paths = root.find("paths")

		for child in paths:
			self.parse_node(child)

	def parse_node(self, child, depth_offset=0.0):
		if child.tag=="waypoint":
			self.parse_wy(child, depth_offset)
		if child.tag=="loop":
			self.parse_loop(child, depth_offset)

	def parse_wy(self, wp, depth_offset=0.0):
		duration = datetime.timedelta(seconds=float(wp.findtext("duration")))
		self.end_time += duration
		self.waypoint_list.append(Seabotwaypoint(time_end = self.end_time,
											duration=duration,
											depth=float(wp.findtext("depth"))+depth_offset,
											east=int(wp.findtext("east")),
											north=int(wp.findtext("north")),
											limit_velocity=float(wp.findtext("limit_velocity", default="0.02")),
											approach_velocity=float(wp.findtext("approach_velocity", default="1.0")),
											enable_thrusters=True))

	def parse_loop(self, l, depth_offset=0.0):
		n = int(l.attrib["number"])
		dz = float(l.attrib["depth_increment"])
		for i in range(n):
			for child in l:
				self.parse_node(child, depth_offset+i*dz)

if __name__ == '__main__':
    s_m = SeabotMission()
    if(sys.argv[1] != ""):
    	s_m.load_mission_xml(sys.argv[1])
    else:
    	s_m.load_mission_xml("/home/lemezoth/workspaceFlotteur/src/seabot/mission/mission_guerledan.xml")
    print(s_m)