import multiprocessing
import time
import queue
from eagle_receiver_txt1 import receive_eagle
from GUI_rockit_R_exp import gui_rock

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

        Lf = 0.45
        Lr = 0.45
        vmax = 0.018
        delt_max = pi / 2
        saturation = 1

        k1=0.05
        k2=0.1
        k3=0.05

        p_err = 3 * [0]
        p_err[0] = ref[0] - x
        p_err[1] = ref[1] - y
        p_err[2] = ref[2] - theta

        vxff = inp[0]
        vyff = inp[1]
        omegaff = inp[2]

        vxfb = k1 * p_err[0]
        vyfb = k2 * p_err[1]
        omegafb = k3 * p_err[2]

        vxc = (k1 * p_err[0]) + inp[0]
        vyc = (k2 * p_err[1]) + inp[1]
        omegac = (k3 * p_err[2]) + inp[2]

        if ff==1:

            vx = inp[0]
            vy = inp[1]
            omega = inp[2]

            Vx = vx * np.cos(ini[2]) - vy * np.sin(ini[2])
            Vy = vx * np.sin(ini[2]) + vy * np.cos(ini[2])


            #Rotate Vx and Vy to robot co-ordinate frame

            vx_r = Vx * np.cos(theta) + Vy * np.sin(theta)
            vy_r = -Vx * np.sin(theta) + Vy * np.cos(theta)

            if vy_r>0:

              if omega>0:

                vx_r = vx_r+0.025*omega
                vy_r = vy_r+0.139*vx_r+0.025*omega

              else:

                 vx_r = vx_r
                 vy_r = vy_r + 0.139 * vx_r-0.025*omega


            else:

              if omega>0:

                  vx_r = vx_r-0.22*vy_r+0.025*omega
                  vy_r = vy_r + 0.139 * vx_r+0.025*omega

              else:

                    vx_r = vx_r - 0.22 * vy_r
                    vy_r = vy_r + 0.139 * vx_r-0.025*omega

            # Rotate vx_r and vy_r back to world co-ordinate frame

            vx = vx_r * np.cos(theta) - vy_r * np.sin(theta)
            vy = vx_r * np.sin(theta) + vy_r * np.cos(theta)



        elif (ff==0):

            vx = k1 * p_err[0]
            vy = k2 * p_err[1]
            omega = k3 * p_err[2]

        else:

            vx1 = inp[0]
            vy1 = inp[1]
            omega1 = inp[2]

            Vx = vx1 * np.cos(ini[2]) - vy1 * np.sin(ini[2])
            Vy = vx1 * np.sin(ini[2]) + vy1 * np.cos(ini[2])

            vx = k1 * p_err[0]+Vx
            vy = k2 * p_err[1]+Vy
            omega = k3 * p_err[2]+omega1

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
        print(Vr,Vf,'velllllllllllllllllllllllllllllllllllllllllllllllllllllllll')
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

        '''
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
        '''

        delt = 0.005 * pi
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


        return p_err,vx,vy,omega,vxff,vyff,omegaff,vxfb,vyfb,omegafb,vxc,vyc,omegac,Vf,deltaf,Vr,deltar



def sender(Vf,Vr,deltaf,deltar):
    import socket
    import sys
    import struct
    from time import time, sleep
    from numpy import pi, cos
    import numpy as np

    UDP_IP = "192.168.0.101"
    UDP_PORT = 5000

    tick_min = 5
    tick_max = 195
    vmax = 0.018

    slope = (tick_max - tick_min) / (2 * vmax)

    deltar_d = round(float(((deltar + pi / 2) * 180 / pi)))
    angr = int(deltar_d)
    deltaf_d = round(float(((deltaf + pi / 2) * 180 / pi)))
    angf = int(deltaf_d)
    ticksf = int(round(float(tick_min + (slope * (Vf + vmax)))))
    ticksr = int(round(float(tick_min + (slope * (Vr + vmax)))))


    values = (ticksr, angr, ticksr, angr, ticksf, angf, ticksf, angf)

    print(values,'valueeeeeeeeeeeeeeeeeeeeeeeeeeeeeee')
    #print(Vf,Vr,math.degrees(deltaf),math.degrees(deltar),'valuesbefore')
    MESSAGE = struct.Struct('B B B B B B B B').pack(*values)
    #print("UDP target IP: %s" % UDP_IP)
    #print("UDP target port: %s" % UDP_PORT)
    sock = socket.socket(socket.AF_INET,  # Internet
                         socket.SOCK_DGRAM)  # UDP
    sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))


def do_control(x_in,q,waypoints,p_err):

    # feedback gain in each direction
    ff=1
    total_t = 0
    t_error = 0

    t=0
    # simulation
    [x,y,theta]=x_in

    plt.plot(waypoints[:, 0], waypoints[:, 1], "m--")

    textfile = open("a_file.txt", "w")
    t_eag=0
    flag=1
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
            [t_eag,x, y, theta] = q.get(block=False)
        except:
            [t_eag,x, y, theta] = [t_eag,x, y, theta]


        print(x,y,theta,x_in[2],'pose of the robottttttttttttttttttt')

        p_err,vx,vy,omega,vxff,vyff,omegaff,vxfb,vyfb,omegafb,vxc,vyc,omegac,Vf,deltaf,Vr,deltar = controller(ref,inp, x, y, theta,ff)

        # derivative

        if j>0:

          if abs(t_eag-t_old)>0:
            vx_e=(x-x_old)/(t_eag-t_old)
            vy_e =(y-y_old)/(t_eag-t_old)
            omega_e=((theta-theta_old))/(t_eag-t_old)
          else:

            vx_e=vx_old
            vy_e=vy_old
            omega_e=omega_old


        st_2 = (time.time()-st_1)
        # average value of st_2 is 0.0017
        #print(st_2,'st22222222222222222222222')
        #if j>0:
         #time.sleep(dt-st_2)

        sender(Vf, Vr, deltaf, deltar)
        t_exp.append(t)
        t+=dt
        time.sleep(dt-0.01)

        #textfile.write(str(vx)+ '\n')
        if j==0:
            vx_e=0
            vy_e=0
            omega_e=0

        yyy=[t,vxff,vyff,omegaff,vxfb,vyfb,omegafb,vxc,vyc,omegac,t_eag,vx_e,vy_e,omega_e]

        textfile.write("{0}\t{1}\t{2}\t{3}\t{4}\t{5}\t{6}\t{7}\t{8}\t{9}\t{10}\t{11}\t{12}\t{13}\n".format(*yyy))

        #run controller at 10 Hz

        st_3=time.time()

        ax.plot(waypoints[j, 0], waypoints[j, 1], "or", label="course")

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
        x_old=x
        y_old=y
        theta_old=theta
        t_old=t_eag
        vx_old=vx_e
        vy_old=vy_e
        omega_old=omega_e
        #time.sleep(dt-cur_tim2)
    plt.close()
    #textfile.close()
    quit()

#fig = plt.figure()
#fig.set_size_inches(108.5, 10.5)
#plt.plot(ref[0], ref[1], "or", label="course")

p_err=3*[100]

# initial state

init_loc = [0,0,0]
fin_loc = [-1,-0.6,0]


[points,Vx_i,Vy_i,omega_i,diff_t,t_s]=gui_rock(init_loc,fin_loc)


#ini=[0.89338,-1.0262, -0.04780]
q = multiprocessing.Queue()
ctx = zmq.Context()
chat_pipe = zhelper.zthread_fork(ctx, receive_eagle, q)


### extracting initial position directly from the eagles


global x_i,y_i,theta_i

def some_function(q):

 try:
    global x_i, y_i, theta_i
    [t_1, x_i, y_i, theta_i] = q.get(block=False)
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

ini=[x_i,y_i,theta_i]
[x, y, theta] = np.array(ini)
#fin=[-0.1797,-0.7306,-0.33283]

print(ini,'iniiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiii')


ax = plt.axes(xlim=(0.3, -0.3), ylim=(-0.3, 0.3),aspect='equal')
#ax.set_size_inches(108.5, 10.5)


#waypoints=np.array([[0.6,0.7,0.6],[0.3,0.7,0.5],[0.2,0.6,0.2]])

waypoints=np.array(points)

waypoints_r=np.zeros((len(points),3),dtype=float)

waypoints_r[:,0] = waypoints[:,0] * np.cos(ini[2]) - waypoints[:,1] * np.sin(ini[2])
waypoints_r[:,1] = waypoints[:,0] * np.sin(ini[2]) + waypoints[:,1] * np.cos(ini[2])


waypoints[:,0]=waypoints_r[:,0]+ini[0]
waypoints[:,1]=waypoints_r[:,1]+ini[1]
waypoints[:,2]=waypoints[:,2]+ini[2]


print(waypoints)
# ang = np.linspace(2 * pi, pi, Nw)
# waypoints = np.array([(np.cos(0.1 * i), np.sin(0.1 * i), ang[i]) for i in range(Nw)])


p = multiprocessing.Process(target=do_control, args=(ini,q,waypoints,p_err))


manager=[]
vxl = []
vyl = []
omegal =[]

vxfl = []
vyfl = []
omegafl =[]

t_exp=[]


p.start()



# init ctx

#init thread

# for controls

# do control



p.join()




