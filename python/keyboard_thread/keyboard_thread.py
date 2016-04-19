
'''
keyboard_thread
extremely simple module that defines a function for reading keyboard input from stdin, then puts it in a queue
The queue is passed at runtime. A lock is also passed; when main releases the lock, the thread acquires it and terminates
'''

import sys
import select
import Queue

def input(queueName,lockName,timeOut=0.01):
    # read from stdin
    read_list = [sys.stdin]
    # call non-blocking
    while(read_list and not lockName.acquire(False)):
        ready = select.select(read_list,[],[],timeOut)[0]
        for file in ready:
            line = file.readline()
            if not line:
                read_list.remove(file)
            elif line.rstrip():
                queueName.put(line)
