#!/usr/bin/env python3

from datetime import datetime, timedelta
from socket import socket, AF_INET, SOCK_STREAM
from threading import Thread

addr = "0.0.0.0"
port = 8080
status = b'0'
i=0

# listen and respond to incoming TCP connections
def listen():
    global addr, port, i, status
    s = socket(AF_INET, SOCK_STREAM)
    s.bind((addr, port))
    s.listen(1)
    print("Listening at tcp://{}:{}".format(addr, port))
    while True:
        # accept connection
        conn, caddr = s.accept()
        print("[{}]\tConnection accepted".format(i))
        try:
            key = conn.recv(1)
            # DATE
            if key == b'D':
                data = datetime.now().strftime('%y-%m-%d %H:%M:%S').encode("ascii")
            # ALARM (date + 15s)
            elif key == b'A':
                data = (datetime.now() + timedelta(seconds = 15)).strftime('%H:%M:%S').encode("ascii")
            # STATUS
            elif key == b'S':
                data = status
                status = data
            conn.sendall(data)
            print("[{}]\tSent {} bytes: {}".format(i, len(data), data))
        except Exception as err:
            print("[{}]\tError: {}".format(i, str(err)))
        finally:
            conn.close()
            i += 1

# TODO: replace threads with corroutines
class Listener(Thread):
    def __init__(self):
        Thread.__init__(self)
        self.daemon = True
    def run(self):
        while True:
            listen()

# start listener in background
l = Listener()
l.start()
# read new statuses from terminal
while True:
    s = input()
    if s == "0" or s == "1":
        status = s.encode("ascii")
    print("Status set to '{}'".format(s))
