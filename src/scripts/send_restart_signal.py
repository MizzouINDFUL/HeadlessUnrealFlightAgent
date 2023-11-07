import socket
import os
import time

#send the signal "extracted"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost', 12345))

s.sendall('extracted'.encode('utf-8'))

s.close()