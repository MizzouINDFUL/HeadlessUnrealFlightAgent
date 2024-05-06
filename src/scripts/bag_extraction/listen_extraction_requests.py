import socket
import os
import time
import sys

#this should receive socket messages at localhost
#the messages are python file names that will need to be executed in a new tmux pane from inside a running 'ros' container in the following folder: /scripts/bag_extraction/topics/

SESSIONNAME=""
port = 1234
run_in_docker = True
if len(sys.argv) > 1:
    port = int(sys.argv[1])

if len(sys.argv) > 2:
    SESSIONNAME = sys.argv[2]

if len(sys.argv) > 3:
    run_in_docker = sys.argv[3] == 'True'

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('localhost', port))
s.listen(1)
num_topics = 0
num_done = 0
rosbag_done = False
topics_extracted = False
skip = False

while True:

    if skip or (rosbag_done and topics_extracted):
        #close the socket
        s.close()
        print('done extracting topics')

        #terminate all tmux panes except for the first one
        #loop from num_topics to 1
        for i in range(num_topics, 0, -1):
            os.system('tmux select-pane -t {}:Bags-Extract.{}'.format(SESSIONNAME, i))
            time.sleep(0.1)
            os.system('tmux send-keys -t {}:Bags-Extract "exit" Enter'.format(SESSIONNAME, i))
            time.sleep(0.1)

        break

    conn, addr = s.accept()
    data = conn.recv(1024)
    if not data:
        continue
    filename = data.decode('utf-8')
    print('received: ', filename)

    #if filename is 'skip', then skip
    if (filename == 'skip'):
        print('skipping topics')
        skip = True
        continue

    #if filename is a digit, then it is a done message per topic
    if(filename.isdigit()):
        num_done = num_done + 1
        if num_done == num_topics:
            # os.system('tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal \'py print(\\"life done\d\")\'" Enter')
            topics_extracted = True
            continue
        else:
            continue

    if (filename == 'rosbag done'):
        print('rosbag is done playing')
        # os.system('tmux send-keys -t $SESSIONNAME:tellunreal "tellunreal \'py print(\\"life done\d\")\'" Enter')
        rosbag_done = True
        continue

    #this will receive the names of the python files that need to be executed as one string separated by spaces
    #the files are located in the following directory: /scripts/bag_extraction/topics/
    files = filename.split(' ')
    files = [x for x in files if x != '']

    num_topics = len(files)
    print('expecting {} topics'.format(num_topics))

    for filename in files:

        #every file name comes in the format of scriptname:nummessages
        #replace : with a space

        filename = filename.replace(':', ' ')

        # focus of $SESSIONNAME:Bafs-Extract tmux window
        os.system('tmux select-window -t {}:Bags-Extract'.format(SESSIONNAME))
        os.system('tmux split-window -h')
        time.sleep(0.1)
        if run_in_docker:
            os.system('tmux send-keys -t {}:Bags-Extract "docker exec -it {}-airsim-ros /bin/bash" Enter'.format(SESSIONNAME, SESSIONNAME))
        time.sleep(0.1)
        if run_in_docker:
            os.system('tmux send-keys -t {}:Bags-Extract "source /opt/ros/noetic/setup.bash" Enter'.format(SESSIONNAME))
        else:
            os.system('tmux send-keys -t {}:Bags-Extract "source /opt/ros/noetic/setup.bash" Enter'.format(SESSIONNAME))
        time.sleep(0.1)
        if run_in_docker:
            os.system('tmux send-keys -t {}:Bags-Extract "python3 /scripts/bag_extraction/topics/{}; exit" Enter'.format(SESSIONNAME, filename))
        else:
            os.system('tmux send-keys -t {}:Bags-Extract "python3 src/scripts/bag_extraction/topics/{}; exit" Enter'.format(SESSIONNAME, filename))
        os.system('tmux select-pane -t {}:Bags-Extract.0'.format(SESSIONNAME))
        os.system('tmux resize-pane -t {}:Bags-Extract.0 -x 30'.format(SESSIONNAME))
        os.system('tmux select-layout -t {}:Bags-Extract even-horizontal'.format(SESSIONNAME))
        time.sleep(1)
