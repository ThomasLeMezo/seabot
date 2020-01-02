import sqlite3
import datetime
import os
from os.path import expanduser

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
								`message_id`	INTEGER NOT NULL,
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

	def get_robot_list(self):
		try:
			self.sqliteCursor.execute('''SELECT imei, name FROM ROBOTS''')
			records = self.sqliteCursor.fetchall()
			list_robots = []
			for data in records:
				table = {}
				table["imei"] = data[0]
				table["name"] = data[1]
				list_robots.append(table)
			return list_robots
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)
			return []

	def get_robot_name(self, imei):
		try:
			self.sqliteCursor.execute('''SELECT name FROM ROBOTS WHERE imei=?''', [imei])
			records = self.sqliteCursor.fetchone()
			return records[0]
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)
			return []

	def update_robot_name(self, name, imei):
		try:
			self.sqliteCursor.execute("UPDATE ROBOTS SET name = ? WHERE imei=?", [name,imei])
			self.sqliteConnection.commit()
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)


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

	def fill_data_log_state(self, row):
		data = {}
		data["log_state_id"] = row[0]
		data["message_id"] = row[1]
		data["ts"] = row[2]
		data["east"] = row[3]
		data["north"] = row[4]
		data["gnss_speed"] = row[5]
		data["gnss_heading"] = row[6]
		data["safety_published_frequency"] = row[7]
		data["safety_depth_limit"] = row[8]
		data["safety_batteries_limit"] = row[9]
		data["safety_depressurization"] = row[10]
		data["enable_mission"] = row[11]
		data["enable_depth"] = row[12]
		data["enable_engine"] = row[13]
		data["enable_flash"] = row[14]
		data["battery0"] = row[15]
		data["battery1"] = row[16]
		data["battery2"] = row[17]
		data["battery3"] = row[18]
		data["pressure"] = row[19]
		data["temperature"] = row[20]
		data["humidity"] = row[21]
		data["waypoint"] = row[22]
		data["last_cmd_received"] = row[23]
		return data

	def get_next_log_state(self, message_id):
		try:
			sql_sentence = '''SELECT *
								FROM SBD_LOG_STATE 
								INNER JOIN SBD_RECEIVED ON (
									SBD_LOG_STATE.message_id = SBD_RECEIVED.message_id 
									AND
									SBD_RECEIVED.IMEI IN (SELECT SBD_RECEIVED.IMEI FROM SBD_RECEIVED WHERE SBD_RECEIVED.message_id=?)
									AND
									SBD_RECEIVED.MOMSN > (SELECT SBD_RECEIVED.MOMSN FROM SBD_RECEIVED WHERE SBD_RECEIVED.message_id=?)
								)
								ORDER BY ts
								LIMIT 1'''
			self.sqliteCursor.execute(sql_sentence, [message_id, message_id])
			row = self.sqliteCursor.fetchone()
			if(row!=None):
				return self.fill_data_log_state(row)
			else:
				return None
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def get_previous_log_state(self, message_id):
		try:
			sql_sentence = '''SELECT *
								FROM SBD_LOG_STATE 
								INNER JOIN SBD_RECEIVED ON (
									SBD_LOG_STATE.message_id = SBD_RECEIVED.message_id 
									AND
									SBD_RECEIVED.IMEI IN (SELECT SBD_RECEIVED.IMEI FROM SBD_RECEIVED WHERE SBD_RECEIVED.message_id=?)
									AND
									SBD_RECEIVED.MOMSN < (SELECT SBD_RECEIVED.MOMSN FROM SBD_RECEIVED WHERE SBD_RECEIVED.message_id=?)
								)
								ORDER BY ts DESC
								LIMIT 1'''
			self.sqliteCursor.execute(sql_sentence, [message_id, message_id])
			row = self.sqliteCursor.fetchone()
			if(row!=None):
				return self.fill_data_log_state(row)
			else:
				return None
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def get_momsn_from_message_id(self, message_id):
		try:
			sql_sentence = '''SELECT SBD_RECEIVED.MOMSN FROM SBD_RECEIVED 
								WHERE SBD_RECEIVED.message_id = ? '''
			self.sqliteCursor.execute(sql_sentence, [message_id])
			row = self.sqliteCursor.fetchone()
			if(row!=None):
				return row[0]
			else:
				return None
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def get_log_state(self, message_id):
		try:
			sql_sentence = '''SELECT *
								FROM SBD_LOG_STATE 
								WHERE SBD_LOG_STATE.message_id = ?
								LIMIT 1'''
			self.sqliteCursor.execute(sql_sentence, [message_id, message_id])
			row = self.sqliteCursor.fetchone()
			if(row!=None):
				return self.fill_data_log_state(row)
			else:
				return None
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def get_last_log_state(self, imei):
		try:
			sql_sentence = '''SELECT *, SBD_RECEIVED.MOMSN
							FROM SBD_LOG_STATE 
							INNER JOIN SBD_RECEIVED ON (
								SBD_LOG_STATE.message_id = SBD_RECEIVED.message_id 
								AND
								SBD_RECEIVED.IMEI = ?
							)
							ORDER BY SBD_RECEIVED.MOMSN DESC
							LIMIT 1'''
			self.sqliteCursor.execute(sql_sentence, [imei])
			row = self.sqliteCursor.fetchone()
			if(row!=None):
				return self.fill_data_log_state(row[0:-2]), row[-1]
			else:
				return None
		except sqlite3.Error as error:
			print("Error while connecting to sqlite", error)

	def get_bounds_momsn(self, imei):
		try:
			sql_sentence = '''SELECT MIN(SBD_RECEIVED.momsn), MAX(SBD_RECEIVED.momsn)
							FROM SBD_RECEIVED
							WHERE SBD_RECEIVED.imei = ?'''
			self.sqliteCursor.execute(sql_sentence, [imei])
			data = self.sqliteCursor.fetchone()
			return data[0], data[1]
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

if __name__ == '__main__':
	db = DataBaseConnection()
	print(db.get_next_log_state(100))
	print(db.get_max_momsn(5))