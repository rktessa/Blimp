#
#   Hello World server in Python
#   Binds REP socket to tcp://*:5555
#   Expects b"Hello" from client, replies with b"World"
#

import time
import zmq

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://192.168.1.147:5556")

while True:
    #  Wait for next request from client
    message = socket.recv_string()
    tempo, raw_accX, raw_accY, raw_accZ, raw_gyrX, raw_gyrY, raw_gyrZ, raw_magX, raw_magY, raw_magZ = message.split(" ")
    print(f"Received Infos for the execution of the navigation")

    #  Do some 'work'
    #time.sleep(1)
    g = float(raw_accZ)
    #  Send reply back to client
    socket.send_string(f"Accelerazione di gravity %f" % g)