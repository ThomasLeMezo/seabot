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

file = open(expanduser("~") + "/log_imap", "w") 

credential_file = expanduser("~") + "/iridium/credential.yaml"

with open(credential_file, 'r') as stream:
    try:
        credential_data = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)
        exit(1)

server = credential_data["server"]
port = credential_data["port"]
login = credential_data["login"]
password = credential_data["password"]

unseen = True
file_exist = False
filepath = expanduser("~") + "/iridium/received/raw/"

server = imaplib.IMAP4_SSL(server, port)
server.login(login, password)

rv, data = server.select()

date = (datetime.date.today() - datetime.timedelta(10)).strftime("%d-%b-%Y")
# date = datetime.date(2018,10, 07).strftime("%d-%b-%Y")

typ, data = server.search(None, ('UNSEEN' if unseen else 'ALL'), '(SENTSINCE {date})'.format(date=date), '(FROM "sbdservice@sbd.iridium.com")')

file_list = []

file.write("Start\n")

for num in data[0].split():
    typ, data = server.fetch(num, '(RFC822)')
    mail = email.message_from_string(data[0][1])
        
    IMEI = mail["Subject"].split(": ")[1]
    file.write(str(IMEI) + "\n")
    
    for part in mail.walk():
            if part.get_content_maintype() != 'application':
                continue

            filename = part.get_filename()
            
            att_path = os.path.join(filepath, filename)
            # att_path = filename

            if file_exist and os.path.isfile(att_path) :
                continue
            
            print(IMEI, filename)
            file_list.append([filepath, filename, IMEI])
            # finally write the stuff
            fp = open(att_path, 'wb')
            fp.write(part.get_payload(decode=True))
            fp.close()

server.close()
server.logout()

### Process files ###
for d in file_list:
    subprocess.call(["rosrun", "seabot_iridium", "decode_log_state", d[0], d[1], d[2]])

    with open(expanduser("~") + "/iridium/received/last_received.yaml", 'r') as stream:
        try:
            pose = yaml.load(stream)
        except yaml.YAMLError as exc:
            print(exc)
            continue

    subprocess.call([expanduser("~") + "/workspaceQT/invariant-lib/build/build-debug/examples/cpp/seabot/seabot_live/seabot_live", str(pose["ts"]), str(pose["east"]), str(pose["north"])])
    file.write(d[0])