import imaplib
import time
import os

import email
from email.policy import default
from email.utils import parsedate_tz, parsedate

import time
import calendar

import shutil
import smtplib
import struct
import datetime
import subprocess
import yaml
import sqlite3
import sys
import re

import threading, queue

from .seabotDataBase import *

class ImapServer():#threading.Thread):
	serverIMAP = None
	mailbox = 'INBOX'
	is_connected = False
	is_first_connection = True
	running = False
	time_last_check = 0.0 #time.time()
	dt_check = 5.0
	log = "?"
	server_id = -1

	db_connection = DataBaseConnection(init_table=True)

	# ToDo : write database last time access
	# ToDo : write database msg

	# def __init__(self):
		# threading.Thread.__init__(self)
		# self.lock = threading.Lock()
		# self.start()

	def __del__(self):
		# with self.lock:
		self.running = False
		# self.join()

	def start_server(self):
		# with self.lock:
		ImapServer.running = True
		ImapServer.log = str(ImapServer.running)
			# self.log = "Test 1" + str(self.running)
			# self.time_last_check = time.time()
			#self.log = "Test 0" + str(self.running)

	def stop_server(self):
		# with self.lock:
		if(self.running == True):
			ImapServer.running = False
		self.close_server()

	def close_server(self):
		if self.is_connected == True:
			self.is_connected = False
			self.is_first_connection = True
			try:
				self.serverIMAP.close()
				self.serverIMAP.logout()
			except imaplib.IMAP4.error as err:
				print(err, flush=True)

	def __del__(self):
		self.close_server()
 
	def run(self):
		if self.running:
			self.update_imap()
		time.sleep(0.1)

	def set_server_id(self, server_id):
		self.server_id = server_id

	def connect(self):
		print("Try connect")
		try:
			# Retreive data from DB
			login_data = self.db_connection.get_server_data(self.server_id)
			if(len(login_data)==0):
				raise Exception('wrong server_id ', self.server_id)

			self.serverIMAP = imaplib.IMAP4_SSL(login_data["server_ip"], login_data["server_port"])
			rsp = self.serverIMAP.login(login_data["email"], login_data["password"])

			if(rsp[1][0].decode()=="LOGIN completed."):
				rsp, nb_message_inbox = self.serverIMAP.select(mailbox=self.mailbox, readonly=False)
				print("select rsp = ", rsp, " nb_message = ", nb_message_inbox[0].decode())
				self.is_connected = True
				self.is_first_connection = True
			else:
				raise Exception('Failed to select')
			
		except imaplib.IMAP4.error as err:
			print("Error imap ", err)
		except sqlite3.Error as error:
			print("Error sqlite ", error)
		except:
			print("Error ", sys.exc_info())
		
		return "Connected"

	def update_recent(self):
		print("Try update_recent")
		try:
			t = datetime.datetime.now()
			rsp, msgnums = self.serverIMAP.recent()
			if(msgnums[0] != None):
				print(msgnums)
				for num in msgnums[0].split():
					self.download_msg(num.decode())

			self.db_connection.update_last_sync(self.server_id, t)
		except imaplib.IMAP4.error as err:
			self.is_connected = False
			print(err, flush=True)

	def update_first_connection(self):
		print("Try update_first_connection")
		t = datetime.datetime.now()

		# Search for email since last sync date
		date = self.db_connection.get_last_sync(1).strftime("%d-%b-%Y")
		typ, msgnums = self.serverIMAP.search(None, 'SENTSINCE {date}'.format(date=date), 'FROM "sbdservice@sbd.iridium.com"')

		if(msgnums[0] != None):
			for num in msgnums[0].split():
				self.download_msg(num.decode())

		self.is_first_connection = False
		self.db_connection.update_last_sync(self.server_id, t)
		return "Connected"

	def download_msg(self, msgnum):
		if(msgnum == "0"):
			return
		print("Download msg ", msgnum)
		typ, data_msg = self.serverIMAP.fetch(msgnum, '(BODY.PEEK[])')
		# Parse received part (starting with "Received: ")
		mail = email.message_from_bytes(data_msg[0][1], policy=default)

		if(mail["From"]=="sbdservice@sbd.iridium.com"):
			# imei = mail["Subject"].split(": ")[1]
			imei = re.search("SBD Msg From Unit: (.*)",mail["Subject"]).group(1)			
			send_time = calendar.timegm(parsedate(mail["Date"]))

			if mail.get_content_maintype() != 'multipart':
				print("No attachment")
				return

			self.db_connection.add_new_robot(imei) # Add new robot if not existing
			## Extract enclosed file
			for part in mail.iter_attachments():
				if part.get_content_maintype() == 'application':
					# Extract momsn from attached file
					momsn = int(re.search("_(.*)\.", part.get_filename()).group(1))
					
					# Add new entry to the database with the association of the imei and momsn
					message_id = self.db_connection.add_sbd_received(imei, momsn)

					# Test if message is already saved
					if(message_id != None):
						msg_data = part.get_payload(decode=True)
						IridiumMessageParser(msg_data, self.db_connection, message_id, send_time)

	def update_imap(self):
		if(time.time()-self.time_last_check > self.dt_check):
			if(not self.is_connected):
				self.connect()
			if(self.is_connected and self.is_first_connection):
				self.update_first_connection()
			if(self.is_connected and not self.is_first_connection):
				self.update_recent()
			self.time_last_check = time.time()

	def get_log(self):
		return ImapServer.log

class IridiumMessageParser():
	message_type = 0
	message = None
	db = None

	def __init__(self, message_string, db, message_id, send_time):
		self.message = int.from_bytes(message_string, byteorder='little', signed=False)
		self.db = db
		# self.message_type = message_string[-1] & 0x0F
		# Test type of message

		## Assume
		self.save_log_state(message_id, send_time)

	def save_log_state(self, message_id, send_time):
		message_data = self.deserialize_log_state(self.message, send_time)
		self.db.add_sbd_log_state(message_id, message_data)

	def deserialize_data(self, data, nb_bit, start_bit, value_min=None, value_max=None):
		mask = ((1<<nb_bit)-1) << start_bit
		v = (data & mask)>>start_bit
		if(value_min!=None and value_max!=None):
			scale = (value_max-value_min)/(1<<nb_bit-1)
			v=v*scale+value_min
		return v, start_bit+nb_bit

	def deserialize_log_state(self, data, send_time):
		bit_position = 0
		fields = {}
		
		# To be updated
		fields["ts"], bit_position = self.deserialize_data(data, 18, bit_position)
		fields["ts"] = send_time

		fields["east"], bit_position = self.deserialize_data(data, 21, bit_position, 0, 1300000)
		fields["north"], bit_position = self.deserialize_data(data, 21, bit_position, 6000000, 7200000)
		fields["gnss_speed"], bit_position = self.deserialize_data(data, 8, bit_position, 0, 5.0)
		fields["gnss_heading"], bit_position = self.deserialize_data(data, 8, bit_position, 0, 359.0)

		safety, bit_position = self.deserialize_data(data, 8, bit_position)
		safety = round(safety)
		fields["safety_published_frequency"] = (safety >>0) & 0b1
		fields["safety_depth_limit"] = (safety >>1) & 0b1
		fields["safety_batteries_limit"] = (safety >>2) & 0b1
		fields["safety_depressurization"] = (safety >>3) & 0b1
		fields["enable_mission"] = (safety >>4) & 0b1
		fields["enable_depth"] = (safety >>5) & 0b1
		fields["enable_engine"] = (safety >>6) & 0b1
		fields["enable_flash"] = (safety >>7) & 0b1

		fields["battery0"], bit_position = self.deserialize_data(data, 5, bit_position, 9, 12.4)
		fields["battery1"], bit_position = self.deserialize_data(data, 5, bit_position, 9, 12.4)
		fields["battery2"], bit_position = self.deserialize_data(data, 5, bit_position, 9, 12.4)
		fields["battery3"], bit_position = self.deserialize_data(data, 5, bit_position, 9, 12.4)

		fields["pressure"], bit_position = self.deserialize_data(data, 6, bit_position, 680.0, 800.0)
		fields["temperature"], bit_position = self.deserialize_data(data, 6, bit_position, 8.0, 50.0)
		fields["humidity"], bit_position = self.deserialize_data(data, 6, bit_position, 50.0, 100.0)

		fields["waypoint"], bit_position = self.deserialize_data(data, 8, bit_position)
		fields["last_cmd_received"], bit_position = self.deserialize_data(data, 6, bit_position)

		print(fields)
		return fields

if __name__ == '__main__':
	imapServer = ImapServer()
	imapServer.set_server_id(1)
	imapServer.start_server()
	try:
		while True:
			imapServer.run()
			time.sleep(1)
	except KeyboardInterrupt:
		print('interrupted!', flush=True)
