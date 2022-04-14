import multiprocessing
import time
import queue
from eagle_receiver_txt1 import receive_eagle
from GUI_rockit import gui_rock

import numpy as np
import matplotlib.pyplot as plt


from rockit import *
from numpy import pi, cos, sin, tan,arctan2,arctan,sqrt
from casadi import vertcat, sumsqr
from rockit import Ocp, DirectMethod, MultipleShooting, FreeTime
from matplotlib import animation
from tkinter import *
import time as time

import control as ct
import scipy as sp

import scipy.linalg as linalg
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

def controller(ref,inp, x, y, theta,ff):

        Lf = 0.5
        Lr = 0.5
        vmax = 0.0123
        delt_max = pi / 2
        saturation = 1

        k1=0.5
        k2=0.5
        k3=0.5

        p_err = 3 * [0]
        p_err[0] = ref[0] - x
        p_err[1] = ref[1] - y
        p_err[2] = ref[2] - theta

        if ff == 0:

            vx = k1 * p_err[0]
            vy = k2 * p_err[1]
            omega = k3 * p_err[2]

        else:
            vx = inp[0]
            vy = inp[1]
            omega = inp[2]

        numr = (+vy - (omega * Lr * cos(theta)))
        denr = (+vx + (omega * Lr * sin(theta)))
        numf = (+vy + (omega * Lf * cos(theta)))
        denf = (+vx - (omega * Lf * sin(theta)))

        deltar = arctan2(numr, denr) - theta

        deltaf = arctan2(numf, denf) - theta

        # Vr = (vx + (omega * Lr * sin(theta))) / (cos(deltar + theta))
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
        # print(math.degrees(deltaf))

        if deltaf > pi:
            deltaf = deltaf - (2 * pi)

        if deltaf < -pi:
            deltaf = deltaf + (2 * pi)

            # print(math.degrees(deltaf))

        if deltar > pi:
            deltar = deltar - (2 * pi)

        if deltar < -pi:
            deltar = deltar + (2 * pi)

        if (deltaf > (pi / 2) or deltaf < (-pi / 2)):
            Vf = -Vf
            if deltaf > 0:
                deltaf = deltaf - pi

            elif deltaf < 0:
                deltaf = deltaf + pi

        # print(math.degrees(deltaf))

        if (deltar > (pi / 2) or deltar < (-pi / 2)):
            Vr = -Vr
            if deltar > 0:
                deltar = deltar - pi
            elif deltar < 0:
                deltar = deltar + pi
        print([p_err, Vf, Vr, deltaf, deltar],'return valuesssssssssssssssssssssss')
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


    values = (ticksr, angr, ticksr, angr, ticksf, angf, ticksf, angf)


    MESSAGE = struct.Struct('B B B B B B B B').pack(*values)
    print("UDP target IP: %s" % UDP_IP)
    print("UDP target port: %s" % UDP_PORT)
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))



def do_feedback(ref,p_err,x,y,theta):
    inp = [None] * 3
    ff=0
    while (abs(p_err[0]) >= 0.01 or abs(p_err[1]) >= 0.01 or abs(p_err[2]) >= 0.01):


        [p_err, Vf, Vr, deltaf, deltar] = controller(ref, inp, x, y, theta,ff)

        sender(Vf, Vr, deltaf, deltar)

        dt=0.4

        time.sleep(dt)




        plt.plot(x, y, "og", label="trajectory")

        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.00001)
        plt.axis('tight')
    return x,y,theta



def do_control(x_in,q,waypoints,p_err):

    ff = 1

    st_t = time.time()

    # feedback gain in each direction
    Ts = 0.1

    total_t = 0
    t_error = 0

    # simulation
    [x,y,theta]=x_in
    plt.plot(waypoints[:, 0], waypoints[:, 1], "m--")


    for j in range(0, len(waypoints-3)):

        ref = waypoints[j, :]
        inp = [Vx_i[j], Vy_i[j], omega_i[j]]

        if diff_t[j]==0:
            dt = diff_t[j-1]
        else:
            dt = diff_t[j]
        t = 0

        # print(ref)
        p_err = 3 * [100]


        # starting from initial state call controller
        try:
            [x, y, theta] = q.get(block=False)

        except:
            [x, y, theta] = [x, y, theta]

        cur_tim = (time.time() - st_t)
        print(x,y,theta,'fromthecontrollerrrrrrrrrrrrrrrrrrrrrrrrrr')
        p_err, Vf, Vr, deltaf, deltar = controller(ref,inp, x, y, theta,ff)

        st_t1 = time.time()
        sender(Vf, Vr, deltaf, deltar)

        #run controller at 10 Hz


        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])

        plt.plot(waypoints[j, 0], waypoints[j, 1], "or", label="course")
        # print(waypoints[j,:])

        plt.plot(x, y, "ob", label="trajectory")
        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.00001)
        plt.axis('tight')

        # print(p_err)

        t_error += abs(p_err[0]) + abs(p_err[1]) + abs(p_err[2])
        total_t += t
        '''
        if (round(cur_tim % 10) == 0):
            x, y, theta = do_feedback(ref, p_err, x, y, theta)
            ff = 1
        '''
        cur_tim2=(time.time()-st_t1)-0.02
        print(cur_tim2,'curtime2')
        time.sleep((dt-cur_tim2) - time.time() % (dt-cur_tim2))
        #time.sleep(dt-cur_tim2)
    plt.close()


#fig = plt.figure()
#fig.set_size_inches(108.5, 10.5)
#plt.plot(ref[0], ref[1], "or", label="course")

p_err=3*[100]

# initial state
ini=[0.27,0.004,0.09]

[x, y, theta] = np.array(ini)

[points,Vx_i,Vy_i,omega_i,diff_t]=gui_rock(ini)

fig = plt.figure()
fig.set_size_inches(108.5, 10.5)

#waypoints=np.array([[0.6,0.7,0.6],[0.3,0.7,0.5],[0.2,0.6,0.2]])
waypoints=np.array(points)

# ang = np.linspace(2 * pi, pi, Nw)
# waypoints = np.array([(np.cos(0.1 * i), np.sin(0.1 * i), ang[i]) for i in range(Nw)])
print(waypoints)

q = multiprocessing.Queue()
p = multiprocessing.Process(target=do_control, args=(ini,q,waypoints,p_err))
ctx = zmq.Context()
chat_pipe = zhelper.zthread_fork(ctx, receive_eagle, q)

p.start()



# init ctx

#init thread

# for controls

# do control

p.join()

