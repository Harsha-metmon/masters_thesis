import multiprocessing
import time
import queue
from eagle_receiver_txt1_multi import receive_eagle
from GUI_rockit_R_multi1 import gui_rock

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

###################

# ref is a 3*3 vector (Nr*3) and inp is (3*3) as well. x,y theta can be vectors or simply x1,y1,theta1 for now.

# Have 3 different controllers for all the robots. Each accepts ref[i,:],inp[i,:] and returns error signal (P_err1 or a vector and Vr1,Vf1,deltaf1,deltar1)

# if all of these are vectors then looping becomes easy but with only 3 robots, we dont need to complicate things right away. so stick to simple things

################

def controller(ref,inp,state,par,ff,gain):

        Lf = 0.45
        Lr = 0.45
        vmax = par[9]
        delt_max = pi / 2
        saturation = 1

        [x,y,theta]=state

        p_err1 = 3 * [0]
        p_err1[0] = ref[0] - x
        p_err1[1] = ref[1] - y

        a1 = ref[2]
        a2 = theta

        if a1 > a2:

            j_d = abs(360 - a1) + a2
            d = abs(a1 - a2)
            if j_d < d:
                a2 += 360

        if a2 > a1:
            j_d = a1 + abs(360 - a2)
            d = abs(a1 - a2)
            if j_d < d:
                a2 = -(abs(360 - a2))

        p_err1[2] = a2 - a1

        vxfb = gain[0] * p_err1[0]
        vyfb = gain[1] * p_err1[1]
        omegafb = gain[2] * p_err1[2]

        Vx = inp[0]
        Vy = inp[1]
        omega = inp[2]

        # Rotate Vx and Vy to robot co-ordinate frame

        vx_r = Vx * np.cos(theta) + Vy * np.sin(theta)
        vy_r = -Vx * np.sin(theta) + Vy * np.cos(theta)

        if vx_r < 0:

            if vy_r < 0:
                vx_r = vx_r+par[0]*vy_r
                vy_r = vy_r+par[1]*vx_r

            elif vy_r > 0:
                vx_r = vx_r+par[2]*vy_r
                vy_r = vy_r+par[3]*vx_r

        elif vx_r > 0:

            if vy_r < 0:
                vx_r = vx_r+par[4]*vy_r
                vy_r = vy_r+par[5]*vx_r

            elif vy_r > 0:
                vx_r = vx_r+par[6]*vy_r
                vy_r = vy_r+par[7]*vx_r

        # Rotate vx_r and vy_r back to world co-ordinate frame

        vx_op = vx_r * np.cos(theta) - vy_r * np.sin(theta)
        vy_op = vx_r * np.sin(theta) + vy_r * np.cos(theta)

        if ff==1:
            vx=vx_op
            vy=vy_op

        elif ff==0:

            vx=vxfb
            vy=vyfb
            omega=omegafb
        else:
            vx=vxfb+vx_op
            vy=vyfb+vy_op
            omega=omegafb+omega

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

        print(inp,'inpppppppppppppppppppp')

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

        delt = par[8] * pi
        #delt=0
        if (deltaf > ((pi / 2) + delt) or deltaf < ((-pi / 2) - delt)):

            Vf = -Vf
            if deltaf > 0:
                deltaf = deltaf - pi

            elif deltaf < 0:
                deltaf = deltaf + pi


        elif (pi / 2 < deltaf < (pi / 2 + delt)) or (-(pi / 2) - delt < deltaf < -pi / 2):

            if deltaf > 0:
                deltaf = pi / 2
            elif deltaf < 0:
                deltaf = -pi / 2


        if (deltar > ((pi / 2) + delt) or deltar < ((-pi / 2) - delt)):

            Vr = -Vr
            if deltar > 0:
                deltar = deltar - pi

            elif deltar < 0:
                deltar = deltar + pi


        elif (pi / 2 < deltar < (pi / 2 + delt)) or (-(pi / 2) - delt < deltar < -pi / 2):

            if deltar > 0:
                deltar = pi / 2
            elif deltar < 0:
                deltar = -pi / 2


        return p_err1,Vf,deltaf,Vr,deltar

####################

#Have 3 different senders as well, each with its own target ip adress

#################


def sender(IP,Vf,Vr,deltaf,deltar):
    import socket
    import sys
    import struct
    from time import time, sleep
    from numpy import pi, cos
    import numpy as np

    #UDP_IP = "192.168.0.101"
    UDP_IP=IP
    UDP_PORT = 5000

    tick_min = 5
    tick_max = 195
    vmax = 0.0185

    slope = (tick_max - tick_min) / (2 * vmax)

    deltar_d = round(float(((deltar + pi / 2) * 180 / pi)))
    angr = int(deltar_d)
    deltaf_d = round(float(((deltaf + pi / 2) * 180 / pi)))
    angf = int(deltaf_d)
    ticksf = int(round(float(tick_min + (slope * (Vf + vmax)))))
    ticksr = int(round(float(tick_min + (slope * (Vr + vmax)))))


    values = (ticksr, angr, ticksr, angr, ticksf, angf, ticksf, angf)

    #print(values,'valueeeeeeeeeeeeeeeeeeeeeeeeeeeeeee')
    #print(Vf,Vr,math.degrees(deltaf),math.degrees(deltar),'valuesbefore')
    MESSAGE = struct.Struct('B B B B B B B B').pack(*values)
    #print("UDP target IP: %s" % UDP_IP)
    #print("UDP target port: %s" % UDP_PORT)
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


def do_control(in_pos,q,waypoints,p_err):

    ip_ad=["192.168.0.102","192.168.0.103"]

    # simulation

    pos=in_pos
    pos_old=pos

    ######################################33
    # write a loop here to plot (or not)
    ######################################

    plt.plot(waypoints[0,0,:], waypoints[0,1, :], "m--")
    plt.plot(waypoints[1,0,:], waypoints[1,1, :], "k--")
    #plt.plot(waypoints[2,:, 0], waypoints[2,:, 1], "y--")

    ######################################################################################

    #problem!!! the number of waypoints could be different for each robot??? or do we assume a global time??  Lets say we design a motion ==> robo1 moves from t=0 to t=10 and then robot 2 moves from
    # t=10==> t==20. So now we can construct a waypoint vector in such a way that (t=0 to 20 and thus we follow a global time and waypoint number is also the same.) but for now we keep things simple
    #assume that they are same or construct cases where they happen to be the same.

    ##########################################################################################

    par_s=[[-0.015,-0.025,-0.015,-0.025,-0.015,-0.025,-0.015,-0.025,0,0.018],[0,-0.03,-0.04,-0.03,0,0.04,-0.04,0.04,0.05,0.018]]

    ff=2

    gain_s=[[0.3,0.3,0.1],[0.3,0.3,0.1]]

    for j in range(0, len(waypoints[0,0,:])-1):

        print(len(waypoints[0,0,:])-1,'waypointnumberrrrrrrrrrrrrrrrrrrr')
        ref = waypoints[:,:,j]

        print(ref,'refffffffffffffffffffffffffffffffffff')

        if diff_t[j]==0:
            dt = diff_t[j-1]
        else:
            dt = diff_t[j]

        flag=0
        # starting from initial state call controller
        try:
            pos = q.get(block=False)
        except:
            pass
            flag=1

        if (flag==1) or len(pos)<2:
            pos=pos_old
        #print(pos,'posssssssssssssssssssss')
        pos_a=np.array(pos)
        state=pos_a[:,2:5]

        for i in range(2):

         p_err[i][:],Vf,deltaf,Vr,deltar = controller(ref[i,:],[Vx_s[i,j],Vy_s[i,j],omega_s[i,j]],state[i,:],par_s[i],ff,gain_s[i])


         sender(ip_ad[i],Vf, Vr, deltaf, deltar)


        time.sleep(dt-0.3)


        ax.plot(ref[0][0], ref[0][1], "or", label="course")
        ax.plot(ref[1][0], ref[1][1], "or", label="course")
        #ax.plot(ref[2][0], ref[2][1], "or", label="course")

        ax.plot(state[0][0], state[0][1], "ob", label="trajectory")
        ax.plot(state[1][0], state[1][1], "ob", label="trajectory")
        #ax.plot(state[2][0], state[2][1], "ok", label="trajectory")

        ax.axis("equal")
        ax.grid(True)
        plt.pause(0.00001)
        ax.axis('tight')

        pos_old=pos


p_err = [ [100]*3 for i in range(2)]


#ini=[0.89338,-1.0262, -0.04780]
q = multiprocessing.Queue()
ctx = zmq.Context()
chat_pipe = zhelper.zthread_fork(ctx, receive_eagle, q)

#######################################################################################

### extracting initial position directly from the eagles. Write a code that runs untill at least one readings of all the tags are extracted.(or not)

#######################################################################################

global in_pos

def some_function(q):

 try:
    global in_pos
    in_pos = q.get(block=False)
    return True
 except:
    pass
    return False


while (True):

    if some_function(q):
        break
    else:
        time.sleep(1)
        continue

in_pos=np.array(in_pos)

ini=in_pos[:,2:5]

#initloc and finloc are vectors

#############################

init_loc = ini.tolist()
#fin_loc = [[0.6,init_loc[0][1],0],[0.6,init_loc[1][1],0],[0.6,init_loc[2][1],0]]

print(init_loc,'dsjadkjhsjdhasjhdasjhgfajshgfkjashgfjaksgfjskhgsajkhfgjsah')

#init_loc = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
#fin_loc = [[0.6, 0, 0], [0.6, 0, 0], [0.6, 0, 0]]

####################################################

#size of the return values waypoints(3, 3, 801) Vx_s (3, 801)....

##################################################

[waypoints,Vx_s,Vy_s,omega_s,diff_t]=gui_rock(init_loc)

###################################################

#print(ini,'iniiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii')

ax = plt.axes(xlim=(0.3, -0.3), ylim=(-0.3, 0.3),aspect='equal')
#ax.set_size_inches(108.5, 10.5)

#############################################################

#transform to world co-ordinate system

#############################
waypoints=np.array(waypoints)

p = multiprocessing.Process(target=do_control, args=(ini,q,waypoints,p_err))


p.start()


p.join()




