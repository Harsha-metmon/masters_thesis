import multiprocessing
import time
import queue
from eagle_receiver_txt1 import receive_eagle
import control as ct
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi, arctan2, sin, cos, sqrt
import scipy as sp
import scipy.linalg as lg
import math

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
import time

try:
    raw_input          # Python 2
except NameError:
    raw_input = input  # Python 3

# controller calculates control input

def controller(ref, x, y, theta):
    Lf = 0.5
    Lr = 0.5
    vmax = 0.0123
    delt_max = pi / 2
    saturation = 1
    k1 = 0.5
    k2 = 0.5
    k3 = 0.5

    p_err = 3 * [0]
    p_err[0] = ref[0] - x
    p_err[1] = ref[1] - y
    p_err[2] = ref[2] - theta

    vx = k1 * p_err[0]
    vy = k2 * p_err[1]
    omega = k3 * p_err[2]

    numr = (+vy - (omega * Lr * cos(theta)))
    denr = (+vx + (omega * Lr * sin(theta)))
    numf = (+vy + (omega * Lf * cos(theta)))
    denf = (+vx - (omega * Lf * sin(theta)))

    deltar = arctan2(numr, denr) - theta

    deltaf = arctan2(numf, denf) - theta

    Vr = sqrt(vx ** 2 + vy ** 2 + (omega * Lr) ** 2 + (2 * omega * Lr * (vx * sin(theta) - vy * cos(theta))))
    # Vf = (vx - (omega * Lf * sin(theta))) / (cos(deltaf + theta))
    Vf = sqrt(vx ** 2 + vy ** 2 + (omega * Lf) ** 2 + (2 * omega * Lf * (vy * cos(theta) - vx * sin(theta))))

    if saturation:

        f = max(abs(Vr) / vmax, abs(Vf) / vmax)

        if (f >= 1):
            Vr = Vr / f
            Vf = Vf / f

    deltaf = (deltaf) % (2 * pi)
    deltar = (deltar) % (2 * pi)
    #print(math.degrees(deltaf))

    if deltaf > pi:
        deltaf = deltaf - (2 * pi)

    if deltaf < -pi:
        deltaf = deltaf + (2 * pi)

    # print(math.degrees(deltaf))

    if deltar > pi:
        deltar = deltar - (2 * pi)

    if deltar < -pi:
        deltar = deltar + (2 * pi)


    delt=0.05*pi

    if (deltaf > ((pi / 2)+delt) or deltaf < ((-pi / 2)-delt)):

        print(math.degrees(deltaf), 'switchedf')


        Vf = -Vf
        if deltaf > 0:
            deltaf = deltaf - pi

        elif deltaf < 0:
            deltaf = deltaf + pi

    elif (pi/2 <deltaf<(pi/2+delt)) or (-(pi/2)-delt <deltaf<-pi/2):

        print(math.degrees(deltaf), 'clippedf')

        if deltaf>0:
            deltaf=pi/2
        elif deltaf<0:
            deltaf=-pi/2

    # print(math.degrees(deltaf))

    if (deltar > ((pi / 2)+delt) or deltar < ((-pi / 2)-delt)):

        print(math.degrees(deltar), 'switchedr')

        Vr = -Vr
        if deltar > 0:
            deltar = deltar - pi

        elif deltar < 0:
            deltar = deltar + pi
    elif (pi/2 <deltar<(pi/2+delt)) or (-(pi/2)-delt <deltar<-pi/2):

        print(math.degrees(deltar), 'clippedr')

        if deltar>0:
            deltar=pi/2
        elif deltar<0:
            deltar=-pi/2
    '''
    if (deltar > ((pi / 2)) or deltar < ((-pi / 2))):
        Vr = -Vr
        if deltar > 0:
            deltar = deltar - pi
        elif deltar < 0:
            deltar = deltar + pi
    '''

    return p_err, Vf, Vr, deltaf, deltar


def sender(Vf,Vr,deltaf,deltar):
    import socket
    import sys
    import struct
    from time import time, sleep
    from numpy import pi, cos
    import numpy as np

    UDP_IP = "192.168.0.101"
    UDP_PORT = 5000

    tick_min = 0
    tick_max = 200
    vmax = 0.0123
    slope = (tick_max - tick_min) / (2 * vmax)

    deltar_d = round(float(((deltar + pi / 2) * 180 / pi)))
    angr = int(deltar_d)
    deltaf_d = round(float(((deltaf + pi / 2) * 180 / pi)))
    angf = int(deltaf_d)
    ticksf = int(round(float(tick_min + (slope * (Vf + vmax)))))
    ticksr = int(round(float(tick_min + (slope * (Vr + vmax)))))

    print(angr)
    print(angf)
    print(ticksf)
    print(ticksr)

    values = (ticksr, angr, ticksr, angr, ticksf, angf, ticksf, angf)

    MESSAGE = struct.Struct('B B B B B B B B').pack(*values)
    print("UDP target IP: %s" % UDP_IP)
    print("UDP target port: %s" % UDP_PORT)
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


def do_control(q,ref,p_err):

    # feedback gain in each direction
    Ts = 0.75
    t = 0

    [x,y,theta]=[2,2,2]
    while (abs(p_err[0]) >= 0.01 or abs(p_err[1]) >= 0.01 or abs(p_err[2]) >= 0.01):
        # starting from initial state call controller
        try:
            [x, y, theta] = q.get(block=False)
            print([x, y, theta])
        except:
            [x,y,theta]=[x,y,theta]

        [p_err, Vf, Vr, deltaf, deltar] = controller(ref, x, y, theta)
        sender(Vf, Vr, deltaf, deltar)
        time.sleep(0.1)

        t = t + Ts

        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(x, y, "ob", label="trajectory")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.00001)
        plt.axis('tight')

        # print(p_err)

    plt.close()

# main

ref = [0.2,0.6,0.2]

fig = plt.figure()                            
fig.set_size_inches(108.5, 10.5)              
plt.plot(ref[0], ref[1], "or", label="course")

p_err=3*[100]

q = multiprocessing.Queue()
p = multiprocessing.Process(target=do_control, args=(q,ref,p_err))
ctx = zmq.Context()
chat_pipe = zhelper.zthread_fork(ctx, receive_eagle, q)

p.start()

# init ctx

#init thread

# for controls

# do control

p.join()



