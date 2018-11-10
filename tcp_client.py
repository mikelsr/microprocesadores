#!/usr/bin/env python3

from datetime import datetime
from socket import socket, AF_INET, SOCK_STREAM

addr = "127.0.0.1"
port = 8080

s = socket(AF_INET, SOCK_STREAM)
s.connect((addr, port))
try:
    s.sendall("D".encode("ascii"))
    data = s.recv(8)
    print(data.decode("ascii"))
finally:
    s.close()

s = socket(AF_INET, SOCK_STREAM)
s.connect((addr, port))
try:
    s.sendall("A".encode("ascii"))
    data = s.recv(8)
    print(data.decode("ascii"))
finally:
    s.close()
