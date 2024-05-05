import yaml
import time
import socket
import os
import glob
import sys
import yaml

port = 1234

#if there is an argument passed, it is how many messages are expected in each topic
num_msgs = -1
if len(sys.argv) > 1:
    num_msgs = int(sys.argv[1])

if len(sys.argv) > 2:
    port = int(sys.argv[2])

sessionname =""
#read sessionname from /config.yml
if os.path.exists('/config.yml'):
    with open('/config.yml', 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)
        sessionname = config["sessionname"]
        print(f"Session name found in the config file: {sessionname}")

topics = [key for x in yaml.safe_load_all(open('topics.yml')) for key in x]
topic_names = [x['topic'] for x in topics]
topics_and_num_msgs = {x['topic']: x['messages'] for x in topics}

#in topic names, remove /$SESSIONNAME from the beginning of each topic
topic_names = [x.replace('/' + sessionname, '') for x in topic_names]
topic_names = [x.replace('/', '_') for x in topic_names]
topic_names = [x + '.py' for x in topic_names]

#do the same replacement in topics_and_num_msgs
topics_and_num_msgs = {x.replace('/' + sessionname, ''): topics_and_num_msgs[x] for x in topics_and_num_msgs}
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

# find the minimum number of topics and use that for every other topic
min_num_msgs = 0
if len(topics_and_num_msgs) > 0:
    min_num_msgs = min(topics_and_num_msgs.values())

if num_msgs != -1:
    min_num_msgs = min(min_num_msgs, num_msgs)

#keep a copy of the original topics_and_num_msgs before replacing every value with min_num_msgs
actual_topics_and_num_msgs = topics_and_num_msgs.copy()

#replace every value in topics_and_num_msgs with min_num_msgs
if min_num_msgs != 0:
    for key in topics_and_num_msgs:
        topics_and_num_msgs[key] = min_num_msgs

#replace every filename in files with file:min_num_msgs:actual_num_msgs
for i in range(len(files)):
    file_name = files[i]
    min_msgs = str(topics_and_num_msgs[file_name])
    actual_msgs = str(actual_topics_and_num_msgs[file_name])
    files[i] = f"{file_name}:{min_msgs}:{actual_msgs}"

num_files = len(files)

#make it one string separated by spaces
files = ' '.join(files)

print("sending the following topics: " + files)

#if files is empty, send 'skip'
if files == '':
    files = 'skip'

#send all the file names via socket at localhost
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(('localhost', port))

s.sendall(files.encode('utf-8'))

s.close()

time.sleep(num_files + 2)
#execute rosbag play ros.bag
os.system(f'rosbag play ros.bag; sleep 5 && python3 -c "import socket; s = socket.socket(socket.AF_INET, socket.SOCK_STREAM); s.connect((\'localhost\', {port})); s.sendall(\'rosbag done\'.encode(\'utf-8\')); s.close()"')
