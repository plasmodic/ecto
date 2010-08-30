.. ROS shmem infrastructure documentation master file, created by
   sphinx-quickstart on Mon Aug 30 14:27:32 2010.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

ROS shmem infrastructure
========================

This is a message queue that lives in shared memory and supports
multiple publishers and subscribers.

Both must specify the type and key of the message queue.  The first
publisher to construct the queue publishes 

The specifies the type of message and the number of messages
there are to be in the queue at construction time.  



Things that a message queue might have:

- "key" name for message queue
- type of message
- n messages in queue
- number of current publishers
- number of current subscribers
- data rate
- read lock
- write lock


