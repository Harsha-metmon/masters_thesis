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


        vxf = k1 * p_err[0]
        vyf = k2 * p_err[1]
        omegaf = k3 * p_err[2]


        vx = inp[0]
        vy = inp[1]
        omega =inp[2]

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

        return p_err, Vf, Vr, deltaf, deltar,vx,vy,omega,vxf,vyf,omegaf



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
    #print("UDP target IP: %s" % UDP_IP)
    #print("UDP target port: %s" % UDP_PORT)
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


def do_feedback(ref,p_err,x,y,theta):
    inp = [None] * 3
    ff=0
    while (abs(p_err[0]) >= 0.05 or abs(p_err[1]) >= 0.05 or abs(p_err[2]) >= 0.05):


        [p_err, Vf, Vr, deltaf, deltar] = controller(ref, inp, x, y, theta,ff)

        sender(Vf, Vr, deltaf, deltar)

        dt=0.1

        time.sleep(dt)

        try:
            [x, y, theta] = q.get(block=False)

        except:
            [x, y, theta] = [x, y, theta]


        plt.plot(x, y, "og", label="trajectory")

        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.00001)
        plt.axis('tight')
    return x,y,theta

def do_control(x_in,q,waypoints,p_err):

    ff = 1



    # feedback gain in each direction

    total_t = 0
    t_error = 0

    t=0
    # simulation
    [x,y,theta]=x_in
    plt.plot(waypoints[:, 0], waypoints[:, 1], "m--")
    textfile = open("a_file.txt", "w")
    for j in range(0, len(waypoints)-1):
        st_1 = time.time()
        ref = waypoints[j, :]
        inp = [Vx_i[j], Vy_i[j], omega_i[j]]


        if diff_t[j]==0:
            dt = diff_t[j-1]
        else:
            dt = diff_t[j]


        # print(ref)
        p_err = 3 * [100]


        # starting from initial state call controller
        try:
            [x, y, theta] = q.get(block=False)

        except:
            [x, y, theta] = [x, y, theta]

        p_err, Vf, Vr, deltaf, deltar,vx,vy,omega,vxf,vyf,omegaf = controller(ref,inp, x, y, theta,ff=1)

        vxl.append(vx)
        vyl.append((vy))
        omegal.append(omega)
        vxfl.append(vxf)
        vyfl.append(vyf)
        omegafl.append(omegaf)
        st_2 = (time.time()-st_1)
        # average value of st_2 is 0.0017
        #print(st_2,'st22222222222222222222222')
        #if j>0:
         #time.sleep(dt-st_2)

        sender(Vf, Vr, deltaf, deltar)
        t_exp.append(t)
        t+=dt

        #textfile.write(str(vx)+ '\n')
        yyy=[vx,vy,omega]
        textfile.write("{0}\t{1}\t{2}\n".format(*yyy))

        #run controller at 10 Hz

        st_3=time.time()

        ax.plot(waypoints[j, 0], waypoints[j, 1], "or", label="course")

        # print(waypoints[j,:])

        ax.plot(x, y, "ob", label="trajectory")
        ax.axis("equal")
        ax.grid(True)
        plt.pause(0.00001)
        ax.axis('tight')

        # print(p_err)

        t_error += abs(p_err[0]) + abs(p_err[1]) + abs(p_err[2])
        total_t += t

        # avg st_4 is 0.17
        st_4=(time.time()-st_3)
        #print(st_4,'st4444444444444444444444444444444444444444444444')

        '''
        if (j%10 == 0):
         x, y, theta = do_feedback(ref, p_err, x, y, theta)
         ff = 1
        '''

        #time.sleep(dt-cur_tim2)
    plt.close()
    #textfile.close()
    quit()

#fig = plt.figure()
#fig.set_size_inches(108.5, 10.5)
#plt.plot(ref[0], ref[1], "or", label="course")

p_err=3*[100]

# initial state
ini=[-0.1454,-0.712, -0.002]



[x, y, theta] = np.array(ini)



[points,Vx_i,Vy_i,omega_i,diff_t]=gui_rock(ini)


ax = plt.axes(xlim=(0.3, -0.3), ylim=(-0.3, 0.3),aspect='equal')
#ax.set_size_inches(108.5, 10.5)


#waypoints=np.array([[0.6,0.7,0.6],[0.3,0.7,0.5],[0.2,0.6,0.2]])
waypoints=np.array(points)


# ang = np.linspace(2 * pi, pi, Nw)
# waypoints = np.array([(np.cos(0.1 * i), np.sin(0.1 * i), ang[i]) for i in range(Nw)])
print(len(waypoints),'lengthofwaypointssssssssssssssssssssssssssssssssssss')
print(len(diff_t),'lennnnnnnnnnnnnnnnnnnnnnnnn')
print(len(Vx_i))

q = multiprocessing.Queue()
p = multiprocessing.Process(target=do_control, args=(ini,q,waypoints,p_err))
ctx = zmq.Context()
chat_pipe = zhelper.zthread_fork(ctx, receive_eagle, q)

manager=multiprocessing.Manager()
vxl = []
vyl = manager.list()
omegal = manager.list()

vxfl = manager.list()
vyfl = manager.list()
omegafl = manager.list()

t_exp=manager.list()


p.start()



# init ctx

#init thread

# for controls

# do control



p.join()

'''
print((type(vxl)),'vxllllllllllllllll')
print((type(t_exp)),'txpppppppppppppp')

vxl=list(vxl)
t_exp=list(t_exp)
print(vxl,'cccccccccccccccccccccccccccccccccc')
print(t_exp,'cccccccccccccccccccccccccccccccccc')

textfile = open("a_file.txt", "w")

for element in vxl:

    textfile.write(element + "\n")

textfile.close()


plt.figure()

plt.plot(t_exp,vxl)
plt.plot(t_exp,vxfl)
plt.title('feedforward,feedback')
plt.show()

'''



