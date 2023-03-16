#
#   Misuration CLIENT in Python
#   Connects REQ socket to tcp://localhost:5556
#   Sends request for misuration to server, expects the misuration back
#

import zmq

context = zmq.Context()

#  Socket to talk to server
print("Connecting to misuration server…")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5556")


#  Do 10 requests, waiting each time for a response
for request in range(10):
    print("Sending request %s …" % request)
    socket.send(b"IMU")

    #  Get the reply.
    message = socket.recv()
    print("Received reply %s [ %s ]" % (request, message))