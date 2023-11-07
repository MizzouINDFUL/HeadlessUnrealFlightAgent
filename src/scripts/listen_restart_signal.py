import socket
import os
import time

#listen for the signal "extracted"

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('localhost', 12345))
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