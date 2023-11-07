import yaml
import time
import socket
import os
import glob

topics = [key for x in yaml.safe_load_all(open('topics.yml')) for key in x]
topic_names = [x['topic'] for x in topics]
topics_and_num_msgs = {x['topic']: x['messages'] for x in topics}

topic_names = [x.replace('/', '_') for x in topic_names]
topic_names = [x + '.py' for x in topic_names]

#do the same replacement in topics_and_num_msgs
topics_and_num_msgs = {x.replace('/', '_') + '.py': topics_and_num_msgs[x] for x in topics_and_num_msgs}

#check if the files exist in the following directory: /scripts/bag_extraction/topics/
#if not, remove them from the list
files = glob.glob('/scripts/bag_extraction/topics/*.py')
files = [os.path.basename(x) for x in files]
files = [x for x in files if x in topic_names]

#in topics_and_num_msgs, remove the topics that don't have a corresponding file
for topic in list(topics_and_num_msgs):
    if topic not in files:
        topics_and_num_msgs.pop(topic)

#replace every filename in files with file:nummsgs

for i in range(len(files)):
    files[i] = files[i] + ':' + str(topics_and_num_msgs[files[i]])

num_files = len(files)

#make it one string spearated by spaces
files = ' '.join(files)

#send all the file names via socket at localhost, 1234

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost', 1234))

s.sendall(files.encode('utf-8'))

s.close()

time.sleep(num_files + 2)
#execute rosbag play ros.bag
os.system('rosbag play ros.bag; sleep 5 && python3 -c "import socket; s = socket.socket(socket.AF_INET, socket.SOCK_STREAM); s.connect((\'localhost\', 1234)); s.sendall(\'rosbag done\'.encode(\'utf-8\')); s.close()"')