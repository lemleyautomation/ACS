import os 
import socket 
import sys
import time

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tag_server:
        tag_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tag_server.bind(('', 8090))
        tag_server.listen()
        while True:
            connection, address = tag_server.accept()
            with connection:
                message = connection.recv(1024)
                if message:
                    message = message.decode('utf-8')
                    if message == "reset knife":
                        os.system("sudo systemctl stop ACS_new.service")
                        time.sleep(1)
                        os.system("sudo systemctl restart ACS_server.service")
                        os.system("sudo systemctl start ACS_new.service")