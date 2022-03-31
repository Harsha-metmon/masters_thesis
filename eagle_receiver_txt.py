# Installation guide: https://github.com/zeromq/pyre
# pip install https://github.com/zeromq/pyre/archive/master.zip

try:
    from zyre_pyzmq import Zyre as Pyre
except Exception as e:
    print("using Python native module", e)
    from pyre import Pyre

from pyre import zhelper
import zmq
import json
import logging
import sys
import uuid
import binascii
import struct

try:
    raw_input          # Python 2
except NameError:
    raw_input = input  # Python 3


def receive_eagle(ctx, pipe):
    n = Pyre("EAGLE")
    n.set_header("CHAT_Header1", "example header1")
    n.join("EAGLE")
    n.start()

    poller = zmq.Poller()
    poller.register(pipe, zmq.POLLIN)
    #print(n.socket())
    poller.register(n.socket(), zmq.POLLIN)
    #print(n.socket())

    txtfile = open("eagle_data.txt", "a")
    txtfile.write("Eagle\tTime\tid\tx\ty\tz\tR\tP\tY\n")

    while(True):
        items = dict(poller.poll())
        #print(n.socket(), items)
        if pipe in items and items[pipe] == zmq.POLLIN:
            message = pipe.recv()
            # message to quit
            if message.decode('utf-8') == "$$STOP":
                break
            print("CHAT_TASK: %s" % message)
            n.shouts("CHAT", message.decode('utf-8'))
        else:
            # if n.socket() in items and items[n.socket()] == zmq.POLLIN:
            cmds = n.recv()
            #print("NODE_MSG CONT: %s" % cmds)
            msg_type = cmds.pop(0)
            msg_peer = cmds.pop(0)
            msg_name = cmds.pop(0)
            #print("NODE_MSG TYPE: %s" % msg_type)
            #print("NODE_MSG PEER: %s" % uuid.UUID(bytes=msg_peer))
            #print("NODE_MSG NAME: %s" % msg_name)
            if msg_type.decode('utf-8') == "SHOUT":
                print("NODE_MSG GROUP: %s" % cmds.pop(0))
            elif msg_type.decode('utf-8') == "ENTER":
                headers = json.loads(cmds.pop(0).decode('utf-8'))
                print("NODE_MSG HEADERS: %s" % headers)

            if msg_type == b'ENTER':
                print("Eagle %s entered the network." % msg_name)
            elif msg_type == b'JOIN':
                print("Eagle %s joined the group." % msg_name)
            elif cmds == [b'']:
                print("Eagle %s sending empty message." % msg_name)
            else:
                msg = cmds[0] #msg = cmds.pop(0)??
                decode = 1
                while decode:
                    # decode header
                    header_size_b = msg[:4]
                    (header_size,) = struct.unpack('I', header_size_b)
                    header_b = msg[4:4 + header_size]
                    (msg_id, time) = struct.unpack('II', header_b)
                    # decode data
                    data_size_b = msg[4 + header_size:8 + header_size]
                    (data_size,) = struct.unpack('I', data_size_b)
                    data_b = msg[8 + header_size:8 + header_size + data_size]
                    print('header_size \t', header_size)
                    print('data_size \t', data_size)
                    print('header \t\t', msg_id, '\t', time / 1000, 's')
                    if msg_id == 0:  #if MARKER (see eagle/src/utils/protocol.h)
                        (marker_id, x, y, z, roll, pitch,
                         yaw) = struct.unpack('Hdddddd', data_b)
                        print('data \t\t', 'id', marker_id, '\n\t\t x', x, '\t\ty', y,
                              '\t\tz', z, '\n\t\t r', roll, '\tp', pitch, '\ty', yaw)
                        txtfile.write("%s\t%.6f\t%d\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t%.6f\t\n"
                               % (msg_name, time / 1000, marker_id,
                                  x, y, z, roll, pitch, yaw))


                    elif msg_id == 1: #if OBSTACLE (see eagle/src/utils/protocol.h)
                        (obst_id, shape, x1, y1, x2, y2, x3,
                         y3) = struct.unpack('HIdddddd', data_b)
                        print('data \t\t', 'id', obst_id, '\n\t\t x1', x1, '\t\ty1', y1,
                              '\n\t\t x2', x2, '\t\ty2', y2, '\n\t\t x3', x3, '\t\ty3', y3)
                    if len(msg) > 8 + header_size + data_size:
                        msg = msg[8 + header_size + data_size:]
                    else:
                        decode = 0
    txtfile.close()
    n.stop()


if __name__ == '__main__':
    # Create a StreamHandler for debugging


    logger = logging.getLogger("pyre")
    logger.setLevel(logging.INFO)
    logger.addHandler(logging.StreamHandler())
    logger.propagate = False

    ctx = zmq.Context()
    chat_pipe = zhelper.zthread_fork(ctx, receive_eagle)

    while True:
        try:
            msg = raw_input()
            chat_pipe.send(msg.encode('utf_8'))
        except (KeyboardInterrupt, SystemExit):
            break
    chat_pipe.send("$$STOP".encode('utf_8'))
    print("FINISHED")

