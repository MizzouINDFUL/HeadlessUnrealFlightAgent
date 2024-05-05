import socket
import os
import time
import sys

#send the signal "extracted"

port = 12345

if(len(sys.argv) > 1):
    port = int(sys.argv[1])

print("Sending signal to port: ", port)

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost', port))

s.sendall('extracted'.encode('utf-8'))

s.close()