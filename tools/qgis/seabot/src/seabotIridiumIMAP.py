import imaplib
import os
import email
import time
import shutil
import smtplib
import struct
import datetime
import subprocess
import yaml
from os.path import expanduser

class SeabotIridiumIMAP():
	server_ip = "40.100.175.146"
	server_port = "993"
	login = ""
	password = ""
	credential_file = expanduser("~") + "/iridium/credential.yaml"

	server = None
	mailbox = 'INBOX'

	is_credential_configured = False
	is_connected = False
	is_first_connection = True

	def __init__(self, credential_file=None):
		if(credential_file!=None):
			self.credential_file = credential_file

	def set_credential_file(self, namefile):
		self.credential_file = namefile
		self.is_credential_configured = False

	def configure_credential(self):
		credential_data = []
		try:
			with open(self.credential_file, 'r') as stream:
				try:
					credential_data = yaml.load(stream)
				except yaml.YAMLError as exc:
					print(exc)
					exit(1)
		except:
			print("No credential file")
			return False

		self.server_ip = credential_data["server"]
		self.port = credential_data["port"]
		self.login = credential_data["login"]
		self.password = credential_data["password"]
		self.is_credential_configured = True
		return True

	def connect(self):
		try:
			self.server = imaplib.IMAP4_SSL(self.server_ip, self.server_port)
			rsp = self.server.login(self.login, self.password)
			print(rsp)
			if(rsp[1]=='LOGIN completed.'):
				rsp, nb_message_inbox = self.server.select(mailbox=self.mailbox, readonly=False)
				print(rsp)
			else:
				return False
		except imaplib.IMAP4.error as err:
			print(err)
			return False
		except:
			print("No network")
			return False
		self.is_connected = True
		self.is_first_connection = True
		return True

	def close(self):
		if self.is_connected == True:
			try:
				self.server.close()
				self.server.logout()
			except imaplib.IMAP4.error as err:
				print(err)
				return False
		return True

	def update_recent(self):
		try:
			rsp, id = self.server.recent()
		except imaplib.IMAP4.error as err:
			print(err)
			return False
		return True

	def update_first_connection(self):
		# date = (datetime.date.today() - datetime.timedelta(10)).strftime("%d-%b-%Y")
		# # date = datetime.date(2018,10, 07).strftime("%d-%b-%Y")
		# typ, data = server.search(None, ('UNSEEN' if unseen else 'ALL'), '(SENTSINCE {date})'.format(date=date), '(FROM "sbdservice@sbd.iridium.com")')
		return True

	def update(self):
		try:
			while True:
				if(not self.is_credential_configured):
					self.configure_credential()
				if(not self.is_connected):
					self.connect()
				if(self.is_first_connection):
					self.update_first_connection()
				if(self.is_connected and not self.is_first_connection):
					self.update_recent()
				time.sleep(5)
		except KeyboardInterrupt:
			print('interrupted!')
		self.close()		

if __name__ == '__main__':
	seabotIridiumIMAP = SeabotIridiumIMAP()
	seabotIridiumIMAP.update()

# file = open(expanduser("~") + "/log_imap", "w") 

# file_exist = False
# filepath = expanduser("~") + "/iridium/received/raw/"

# file_list = []

# file.write("Start\n")

# for num in data[0].split():
# 	typ, data_msg = server.fetch(num, '(RFC822)')
# 	mail = email.message_from_string(data_msg[0][1])
		
# 	IMEI = mail["Subject"].split(": ")[1]
# 	file.write(str(IMEI) + "\n")
	
# 	for part in mail.walk():
# 			if part.get_content_maintype() != 'application':
# 				continue

# 			filename = part.get_filename()
			
# 			att_path = os.path.join(filepath, filename)
# 			# att_path = filename

# 			if file_exist and os.path.isfile(att_path) :
# 				continue
			
# 			print(IMEI, filename)
# 			file_list.append([filepath, filename, IMEI])
			
# 			# finally write the stuff
# 			fp = open(att_path, 'wb')
# 			fp.write(part.get_payload(decode=True))
# 			fp.close()

# ### Process files ###
# for d in file_list:
# 	print("Decode trajectory")
# 	subprocess.call(["rosrun", "seabot_iridium", "decode_log_state", d[0], d[1], d[2]])
# 	print("Read data")

# 	with open(expanduser("~") + "/iridium/received/last_received.yaml", 'r') as stream:
# 		try:
# 			pose = yaml.load(stream)
# 		except yaml.YAMLError as exc:
# 			print(exc)
# 			continue
# 	print("Generate trajectory")
# 	subprocess.call([expanduser("~") + "/workspaceQT/invariant-lib/build/build-debug/examples/cpp/seabot/seabot_live/seabot_live", str(pose["ts"]), str(pose["east"]), str(pose["north"])])
# 	file.write(d[0])