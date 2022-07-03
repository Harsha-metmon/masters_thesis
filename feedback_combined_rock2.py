import multiprocessing
import time
import queue
from eagle_receiver_txt1_multi import receive_eagle
from Elke_stationary import gui_rock

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

def controller(ref,inp,state,gain,vmax):

        Lf = 0.45
        Lr = 0.45
        delt_max = pi / 2
        saturation = 1

        [x,y,theta]=state

        p_err1 = 3 * [0]
        p_err1[0] = ref[0] - x
        p_err1[1] = ref[1] - y

        a1 = ref[2]
        a2 = theta

        if a1 > a2:

            j_d = abs(2*pi - a1) + a2
            d = abs(a1 - a2)
            if j_d < d:
                a2 += 2*pi

        if a2 > a1:
            j_d = a1 + abs(2*pi - a2)
            d = abs(a1 - a2)
            if j_d < d:
                a2 = -(abs(2*pi - a2))

        p_err1[2] = a2 - a1

        print(ref[2],theta,a1,a2,'ggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg')

        #Vx = inp[0] * np.cos(ini[2]) - inp[1] * np.sin(ini[2])
        #Vy = inp[0] * np.sin(ini[2]) + inp[1] * np.cos(ini[2])

        vx = gain[0] * p_err1[0]+inp[0]
        vy = gain[1] * p_err1[1]+inp[1]
        omega = gain[2] * p_err1[2]-inp[2]

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


        if deltaf > pi:
            deltaf = deltaf - (2 * pi)

        if deltaf < -pi:
            deltaf = deltaf + (2 * pi)

        if deltar > pi:
            deltar = deltar - (2 * pi)

        if deltar < -pi:
            deltar = deltar + (2 * pi)

        delt = 0.03 * pi

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


def sender(IP,Vf,Vr,deltaf,deltar,vmax):
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

    slope = (tick_max - tick_min) / (2 * vmax)

    deltar_d = round(float(((deltar + pi / 2) * 180 / pi)))
    angr = int(deltar_d)
    deltaf_d = round(float(((deltaf + pi / 2) * 180 / pi)))
    angf = int(deltaf_d)
    ticksf = int(round(float(tick_min + (slope * (Vf + vmax)))))
    ticksr = int(round(float(tick_min + (slope * (Vr + vmax)))))

    values = (ticksr, angr, ticksr, angr, ticksf, angf, ticksf, angf)
    print(IP,values,'senderrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr')

    #print(values,'valueeeeeeeeeeeeeeeeeeeeeeeeeeeeeee')
    #print(Vf,Vr,math.degrees(deltaf),math.degrees(deltar),'valuesbefore')
    MESSAGE = struct.Struct('B B B B B B B B').pack(*values)
    #print("UDP target IP: %s" % UDP_IP)
    #print("UDP target port: %s" % UDP_PORT)
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))



# Find closest point on the reference path compared witch current position
def find_closest_point(pose, reference_path, start_index):
    # reference_path should be numpy array

    # x and y distance from current position (pose) to every point in
    # the reference path starting at a certain starting index

    xlist = reference_path[start_index:,0] - pose[0]
    ylist = reference_path[start_index:,1] - pose[1]
    # Index of closest point by Pythagoras theorem
    index_closest = start_index+np.argmin(np.sqrt(xlist*xlist + ylist*ylist))
    print('find_closest_point results in', index_closest)
    return index_closest

# Return the point on the reference path that is located at a certain distance
# from the current position


def index_last_point_fun(start_index, wp, dist):

    pathpoints = len(wp)

    # Cumulative distance covered
    cum_dist = 0
    # Start looping the index from start_index to end
    for i in range(start_index, pathpoints-1):
        # Update comulative distance covered

        cum_dist += np.linalg.norm(wp[i,0:2] - wp[i+1,0:2])
        # Are we there yet?

        if cum_dist >= dist:
            print('index_last_point results in', i+1)
            return i + 1

    # Desired distance was never covered, -1 for zero-based index

    return pathpoints - 1


gain_s = [[0.2, 0.2, 0.1], [0.1, 0.1, 0.05]]

#gain_s = [[0, 0, 0], [0, 0, 0]]
vmax=[0.0185,0.0185]


def do_control2(in_pos,q,waypoints):

    textfile = open("a_file_m_101", "w")

    ip_ad="192.168.0.102"

    # simulation

    ######################################33

    pos = in_pos[0]
    pos_old = pos

    st_1 = time.time()

    for j in range(0, len(waypoints[0,0,:])-1):


            ref = waypoints[0, :, j]

            inp = [Vx_s[0,j], Vy_s[0,j], omega_s[0,j]]

            if diff_t[j] == 0:
                dt = diff_t[j - 1]

            else:
                dt = diff_t[j]


            flag=0
            flag1=0

            # starting from initial state call controller

            try:
                pos_b = q.get(block=False)
                for i in range(len(pos_b)):
                    if pos_b[i][0]==0:
                        pos=pos_b[i]
                        flag1=1

            except:
                pass
                flag=1

            if (flag==1 or flag1==0):
                pos=pos_old

            pos=np.array(pos)

            state=pos[2:5]

            yyy = [t_s[j],ref[0], state[0], ref[1], state[1], ref[2], state[2]]

            textfile.write("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\n".format(*yyy))

            p_err,Vf,deltaf,Vr,deltar = controller(ref,inp,state,gain_s[0],vmax[0])

            print(ref, 'reffffffffffffffffffffffff1')
            print(state, 'stateeeeeeeeeeeeeeeeeeee1')
            print(p_err, 'perrrrrrrrrrrrrrrrrrrrrrrr')

            sender(ip_ad,Vf, Vr, deltaf, deltar,vmax[0])

            st_4 = (time.time() - st_1)
            time.sleep(dt - st_4)
            st_1 = time.time()

            pos_old=pos

            #####

            if j == (len(waypoints[0, 0, :]) - 1):
                while (abs(p_err[0]) >= 0.1 or abs(p_err[1]) >= 0.1 or abs(p_err[2]) >= 0.025):

                    flag = 0
                    flag1 = 0

                    # starting from initial state call controller

                    try:
                        pos_b = q.get(block=False)
                        for i in range(len(pos_b)):
                            if pos_b[i][0] == 0:
                                pos = pos_b[i]
                                flag1 = 1

                    except:
                        pass
                        flag = 1

                    if (flag == 1 or flag1 == 0):
                        pos = pos_old

                    pos = np.array(pos)

                    state = pos[2:5]

                    inp = [0, 0, 0]

                    p_err, Vf, deltaf, Vr, deltar = controller(ref, inp, state, gain_s[0], vmax[0])

                    sender(ip_ad, Vf, Vr, deltaf, deltar, vmax[0])


def do_control3(in_pos,q,waypoints):


    ip_ad = "192.168.0.103"

    # simulation

    ######################################33

    pos = in_pos[1]
    pos_old = pos

    st_1 = time.time()

    for j in range(0, len(waypoints[1, 0, :]) - 1):

        ref = waypoints[1, :, j]

        inp = [Vx_s[1, j], Vy_s[1, j], omega_s[1, j]]

        if diff_t[j] == 0:
            dt = diff_t[j - 1]

        else:
            dt = diff_t[j]

        flag = 0
        flag1 = 0

        # starting from initial state call controller

        try:
            pos_b = q.get(block=False)
            for i in range(len(pos_b)):
                if pos_b[i][0] == 11:
                    pos = pos_b[i]
                    flag1 = 1

        except:
            pass
            flag = 1

        if (flag == 1 or flag1 == 0):
            pos = pos_old

        pos = np.array(pos)

        state = pos[2:5]

        p_err, Vf, deltaf, Vr, deltar = controller(ref, inp, state, gain_s[1], vmax[1])

        sender(ip_ad, Vf, Vr, deltaf, deltar, vmax[1])

        st_4 = (time.time() - st_1)
        time.sleep(dt - st_4)
        st_1 = time.time()

        pos_old = pos

        ###

        if j==(len(waypoints[1, 0, :])-1):
            while (abs(p_err[0]) >= 0.1 or abs(p_err[1]) >= 0.1 or abs(p_err[2]) >= 0.025):

                flag = 0
                flag1 = 0

                # starting from initial state call controller

                try:
                    pos_b = q.get(block=False)
                    for i in range(len(pos_b)):
                        if pos_b[i][0] == 11:
                            pos = pos_b[i]
                            flag1 = 1

                except:
                    pass
                    flag = 1

                if (flag == 1 or flag1 == 0):
                    pos = pos_old

                pos = np.array(pos)

                state = pos[2:5]

                inp=[0,0,0]

                p_err, Vf, deltaf, Vr, deltar = controller(ref, inp, state, gain_s[1], vmax[1])

                sender(ip_ad, Vf, Vr, deltaf, deltar, vmax[1])



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

ct1=0
ct2=0
flag_d=0

while (ct1==0 or ct2==0 ):

    if some_function(q):
        #print(in_pos,'pppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppppp')
        for i in range (len(in_pos)):

          print(len(in_pos))

          if (in_pos[i][0]==11):
              ct2=1

          elif(in_pos[i][0]==0):
              ct1=1

          elif(in_pos[i][0]!=11 and in_pos[i][0]!=0):
              ind_d=i
              flag_d=1

    else:
        time.sleep(1)
        continue

if flag_d==1:
 del in_pos[ind_d]

print(in_pos,'inposssssssssssssssssssssss')

in_pos=np.array(in_pos)

ini=in_pos[:,2:5]

#initloc and finloc are vectors

init_loc = ini.tolist()

####################################################

#Define waypoints here. From initial and final points.

init = [[0, 0, pi / 2], [0.45, 0, pi / 2]]

init=init_loc

finit = [[0, 0, pi/2], [0.7, 0, pi/2]]

finit = [[0.2, -0.5, pi/2], [0.7, -0.5, pi/2]]

finit = [[0, 0, 0], [0, 0.5, 0]]

finit = [[0.2, -0.5, pi/2], [0.7, -0.5, pi/2]]

finit = [[0, 0, 0], [0, 0.5, 0]]

finit = [[0.8,-1.1,-pi/2], [0.8,0,-pi/2]]

#finit = [[0, 0, 0], [0, 0.5, 0]]
finit=[[0.8,0,-pi/2],[0,0,0]]


'''
N=3
robotN=2
waypoints=np.zeros((robotN,3,N),dtype=float)

for i in range (robotN):

    waypoints[i, :, 0]=np.linspace(init_loc[i][0],fin_loc[i][0],N)
    waypoints[i, :, 1]=np.linspace(init_loc[i][1],fin_loc[i][1],N)
    waypoints[i, :, 2]=np.linspace(init_loc[i][2],fin_loc[i][2],N)
'''
#waypoints=np.array([[[0.7, 0.7, 0]],[[0.7, 0.35, 0]]])

#####################################################

waypoints,Vx_s,Vy_s,omega_s,diff_t,t_s=gui_rock(init,finit)

#################################################
# convert waypoints to world co-ordinates

# from rockit to world/rotation


init_loc=np.array(init_loc)


'''

waypoints_r[:,:,0] = waypoints[:,:,0] * np.cos(init_loc[:,2]) - waypoints[:,:,1] * np.sin(init_loc[:,2])
waypoints_r[:,:,1] = waypoints[:,:,0] * np.sin(init_loc[:,2]) + waypoints[:,:,1] * np.cos(init_loc[:,2])

#rockit to world translation

waypoints[:,:,0]=waypoints_r[:,:,0]+init_loc[:,0]
waypoints[:,:,1]=waypoints_r[:,:,1]+init_loc[:,1]

#in rockit,all angles are theta-ini[2]/add ini[2] to get back the actual theta in world.

waypoints[:,:,2]=waypoints[:,:,2]+init_loc[:,2]

'''

####################################################

p2 = multiprocessing.Process(target=do_control2, args=(in_pos,q,waypoints))
p3 = multiprocessing.Process(target=do_control3, args=(in_pos,q,waypoints))

p2.start()
p3.start()

p2.join()
p3.join()




