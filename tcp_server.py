#!/usr/bin/env python3

from datetime import datetime, timedelta
from socket import socket, AF_INET, SOCK_STREAM

addr = "0.0.0.0"
port = 8080
i=0

s = socket(AF_INET, SOCK_STREAM)
s.bind((addr, port))
s.listen(1)
print("Listening at tcp://{}:{}".format(addr, port))
while True:
    conn, caddr = s.accept()
    print("[{}]\tConnection accepted".format(i))
    try:
        key = conn.recv(1)
        # DATE
        if key == b'D':
            data = datetime.now().strftime('%y-%m-%d %H:%M:%S').encode("ascii")
        # ALARM (date + 10s)
        elif key == b'A':
            data = (datetime.now() + timedelta(seconds = 10)).strftime('%H:%M:%S').encode("ascii")
        conn.sendall(data)
        print("[{}]\tSent {} bytes: {}".format(i, len(data), data))
    except Exception as err:
        print("[{}]\tError: {}".format(i, str(err)))
    finally:
        conn.close()
        i += 1
