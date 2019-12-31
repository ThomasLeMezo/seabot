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
import datetime
import sys
import re
from os.path import expanduser

import threading, queue


## Database connection to store parameters and sbd messages
class DataBaseConnection():
	sqliteConnection = None
	sqliteCursor = None

	db_file = expanduser("~") + "/.local/share/QGIS/QGIS3/profiles/default/python/plugins/seabot/" + "Seabot_iridium.db"

	sqlite_tables_name = ["ROBOTS", "SBD_LOG_STATE", "CONFIG", "SBD_RECEIVED"]
	sqlite_create_table = ['''CREATE TABLE "'''+sqlite_tables_name[0]+'''" (
										`IMEI`	NUMERIC NOT NULL,
										`NAME`	TEXT,
										PRIMARY KEY(IMEI)
									)''',
							'''CREATE TABLE "'''+sqlite_tables_name[1]+'''" (
								`log_state_id`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
								`message_id`	NUMERIC NOT NULL,
								`ts` DATETIME NOT NULL,
								`east` REAL NOT NULL,
								`north` REAL NOT NULL,
								`gnss_speed` REAL NOT NULL,
								`gnss_heading` REAL NOT NULL,
								`safety_published_frequency` BOOLEAN NOT NULL,
								`safety_depth_limit` BOOLEAN NOT NULL,
								`safety_batteries_limit` BOOLEAN NOT NULL,
								`safety_depressurization` BOOLEAN NOT NULL,
								`enable_mission` BOOLEAN NOT NULL,
								`enable_depth` BOOLEAN NOT NULL,
								`enable_engine` BOOLEAN NOT NULL,
								`enable_flash` BOOLEAN NOT NULL,
								`battery0` REAL NOT NULL,
								`battery1` REAL NOT NULL,
								`battery2` REAL NOT NULL,
								`battery3` REAL NOT NULL,
								`pressure` REAL NOT NULL,
								`temperature` REAL NOT NULL,
								`humidity` REAL NOT NULL,
								`waypoint` INTEGER NOT NULL,
								`last_cmd_received` INTEGER NOT NULL,
								FOREIGN KEY(`message_id`) REFERENCES SBD_RECEIVED (`message_id`)
							)''',
							'''CREATE TABLE "'''+sqlite_tables_name[2]+'''" (
									`config_id`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
									`email`	TEXT NOT NULL,
									`password`	TEXT NOT NULL,
									`server_ip`	TEXT NOT NULL,
									`server_port`	TEXT NOT NULL,
									`last_sync`	timestamp NOT NULL
							)''',
							'''CREATE TABLE "'''+sqlite_tables_name[3]+'''" (
									`message_id`	INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT,
									`IMEI`	NUMERIC NOT NULL,
									`momsn`	NUMERIC NOT NULL,
									FOREIGN KEY(`IMEI`) REFERENCES ROBOTS (`IMEI`)
							)'''
							]

	sqlite_create_table_get_table = '''SELECT name FROM sqlite_master WHERE type='table' and name NOT LIKE 'sqlite_%';'''

	def __init__(self, credential_file=None, init_table=True):
		if(credential_file!=None):
			self.credential_file = credential_file

		# Connection to DB to store iridium messages
		try:
			self.sqliteConnection = sqlite3.connect(self.db_file, 
													detect_types=sqlite3.PARSE_DECLTYPES |
																 sqlite3.PARSE_COLNAMES)
			self.sqliteCursor = self.sqliteConnection.cursor()

			# Querry the list of table names
			self.sqliteCursor.execute(self.sqlite_create_table_get_table)
			records = self.sqliteCursor.fetchall()

			if(init_table):
				# Extract the names and create a list of names
				list_table_name = []
				for name in records:
					list_table_name.append(name[0])
				
				# Test if the table exists, otherwise add a new table
				for i in range(len(self.sqlite_tables_name)):
					if(self.sqlite_tables_name[i] not in list_table_name):
						self.sqliteCursor.execute(self.sqlite_create_table[i])
						self.sqliteConnection.commit()

		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)
			exit()

	def get_email_list(self):
		try:
			self.sqliteCursor.execute('''SELECT email, id FROM CONFIG''')
			records = self.sqliteCursor.fetchall()
			list_email = []
			for data in records:
				table = {}
				table["email"] = data[0]
				table["id"] = data[1]
				list_email.append(table)
			return list_email
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)
			return []

	def save_server(self, email, password, server_ip, server_port):
		try:
			sqlite_insert_config = '''INSERT INTO CONFIG
						  (email, password, server_ip, server_port, last_sync) 
						  VALUES (?, ?, ?, ?, ?);'''

			data_tuple = (email, password, server_ip, server_port, datetime.date.fromtimestamp(0)) 
			self.sqliteCursor.execute(sqlite_insert_config, data_tuple)
			self.sqliteConnection.commit()
			return True
		except:
			print("Error while connecting to sqlite", error)
			return False

	def delete_server(self, id_row):
		try:
			sqlite_delete_config = '''DELETE FROM CONFIG WHERE id=(?);'''
			self.sqliteCursor.execute(sqlite_delete_config, (id_row,))
			self.sqliteConnection.commit()
			return True
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)
			return False

	def get_server_data(self, server_id):
		try:
			sqlite_search_config = '''SELECT * FROM CONFIG where id=(?);'''
			self.sqliteCursor.execute(sqlite_search_config, (server_id,))
			records = self.sqliteCursor.fetchall()
			data = {}
			if(len(records)>0):
				data["id"] = records[0][0]
				data["email"] = records[0][1]
				data["password"] = records[0][2]
				data["server_ip"] = records[0][3]
				data["server_port"] = records[0][4]
				data["last_sync"] = records[0][5]
			return data
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)
			return []

	def update_last_sync(self, server_id, t):
		try:
			self.sqliteCursor.execute("UPDATE CONFIG SET last_sync = ? WHERE id=?", [t,server_id])
			self.sqliteConnection.commit()
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def get_last_sync(self, server_id):
		try:
			self.sqliteCursor.execute("SELECT last_sync from CONFIG WHERE id=?", [server_id])
			row = self.sqliteCursor.fetchone()
			if(len(row)!=0):
				return row[0]
			else:
				return None
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def add_new_robot(self, imei):
		try:
			self.sqliteCursor.execute("SELECT COUNT(1) from ROBOTS WHERE IMEI=(?)", [imei])
			row = self.sqliteCursor.fetchone()
			if(row[0]==0):
				self.sqliteCursor.execute("INSERT INTO ROBOTS (IMEI) VALUES (?)", [imei])
				self.sqliteConnection.commit()
				return False
			else:
				return True
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def add_sbd_received(self, imei, momsn):
		try:
			self.sqliteCursor.execute("SELECT COUNT(1) from SBD_RECEIVED WHERE IMEI= ? and momsn=?", [imei, momsn])
			row = self.sqliteCursor.fetchone()
			if(row[0]==0):
				self.sqliteCursor.execute("INSERT INTO SBD_RECEIVED (IMEI, momsn) VALUES (?, ?)", [imei, momsn])
				self.sqliteConnection.commit()
				return self.sqliteCursor.lastrowid
			else:
				return None
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def add_sbd_log_state(self, message_id, data):
		try:
			sql_insert_log_state = ''' INSERT INTO SBD_LOG_STATE 
			(message_id,
			 ts,
			 east,
			 north,
			 gnss_speed,
			 gnss_heading,
			 safety_published_frequency,
			 safety_depth_limit,
			 safety_batteries_limit,
			 safety_depressurization,
			 enable_mission,
			 enable_depth,
			 enable_engine,
			 enable_flash,
			 battery0,
			 battery1,
			 battery2,
			 battery3,
			 pressure,
			 temperature,
			 humidity,
			 waypoint,
			 last_cmd_received) 
			 VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)'''

			seq = [message_id, 
					data["ts"], 
					data["east"], 
					data["north"], 
					data["gnss_speed"], 
					data["gnss_heading"], 
					data["safety_published_frequency"], 
					data["safety_depth_limit"], 
					data["safety_batteries_limit"], 
					data["safety_depressurization"], 
					data["enable_mission"], 
					data["enable_depth"], 
					data["enable_engine"], 
					data["enable_flash"], 
					data["battery0"], 
					data["battery1"], 
					data["battery2"], 
					data["battery3"], 
					data["pressure"], 
					data["temperature"], 
					data["humidity"], 
					data["waypoint"], 
					data["last_cmd_received"]]

			self.sqliteCursor.execute(sql_insert_log_state, seq)

			self.sqliteConnection.commit()
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

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
