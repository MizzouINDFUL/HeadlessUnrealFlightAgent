import socket
import os
import time
import sys

port = 12345
#check if there is an argument providedd
if (len(sys.argv) > 1):
    port = int(sys.argv[1])

print("listenning for a restart signal at port " + str(port))

#listen for the signal "extracted"
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('localhost', port))
s.listen(1)

while True:
    conn, addr = s.accept()
    data = conn.recv(1024)
    if not data:
        continue
    filename = data.decode('utf-8')
    print('received: ', filename)

    if (filename == 'extracted'):
        print('all topics extracted')

        time.sleep(1)

        #close the socket
        s.close()
        break