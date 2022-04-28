import math
import multiprocessing
import time
import queue
from eagle_receiver_txt1 import receive_eagle

import numpy as np
import matplotlib.pyplot as plt


from rockit import *
from numpy import pi, cos, sin, tan,arctan2,arctan,sqrt
from casadi import vertcat, sumsqr
from rockit import Ocp, DirectMethod, MultipleShooting, FreeTime
from matplotlib import animation
from tkinter import *
from eagle_receiver_txt2 import eag


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


# GUI+ROCKIT


def gui_rock(init_loc,fin_loc):


    points=[init_loc,fin_loc]

    Lr=0.45
    Lf=0.45
    vmax = 0.018

    def steering_angle(thetas, vxs, vys, omegas):
        # Proportional control inputs are calculated


        numr = (vys - (omegas * Lr * cos(thetas)))
        denr = (vxs + (omegas * Lr * sin(thetas)))
        numf = (vys + (omegas * Lf * cos(thetas)))
        denf = (vxs - (omegas * Lf * sin(thetas)))

        deltar = arctan2(numr, denr) - thetas

        deltaf = arctan2(numf, denf) - thetas

        return deltaf, deltar


    def controller(vx,vy,omega,theta):


        saturation = 1

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

        return Vf, Vr, deltaf, deltar


    def create_ocp_stage(ocp):
        """

        Returns
        -------
        stage : :obj:`~rockit.stage.Stage`
            An ocp stage describing the bouncing ball
         : x,y,theta :obj:`~casadi.MX`
            position variable

        """
        #stage = ocp.stage(t0=FreeTime(0), T=FreeTime(10))
        stage = ocp.stage(t0=FreeTime(0), T=a)
        x = stage.state()
        y = stage.state()
        theta=stage.state()

        omega = stage.control(order=0)
        Vx = stage.control(order=0)
        Vy = stage.control(order=0)

        stage.set_der(x, Vx)
        stage.set_der(y, Vy)
        stage.set_der(theta, omega)

        Vrs = (Vx ** 2 + Vy ** 2 + (omega * Lr) ** 2 + ( 2 * omega * Lr * (Vx * sin(theta) - Vy * cos(theta))))

        Vfs = (Vx ** 2 + Vy ** 2 + (omega * Lf) ** 2 + (2 * omega * Lf * (Vy * cos(theta) - Vx * sin(theta))))

        stage.subject_to(0 <= (Vrs <= vmax ** 2))
        stage.subject_to(0 <= (Vfs <= vmax ** 2))

        stage.subject_to(-7.5 <= (x <= 7.5))
        stage.subject_to(-7.5 <= (y <= 7.5))

        # Minimal time
        #stage.add_objective(stage.T)
        stage.add_objective(stage.integral(Vx ** 2+Vy**2))
        stage.add_objective(stage.integral(omega ** 2))
        #stage.add_objective(stage.integral(x ** 2))

        stage.method(MultipleShooting(N=N-1, M=M, intg='rk'))


        return stage,x,y,theta,Vx,Vy,omega


    ocp = Ocp()

    d1 = sqrt((points[1][0] - points[0][0]) ** 2 + (points[1][1] - points[0][1]) ** 2)

    deg1=abs((points[1][2]-points[0][2]))

    N=21+round((80/22)*float(d1))

    # working factors for k=0.3,0.3,0,3
    factor1 = 60
    factor2 = 0.4

    factor1 = 80
    factor2 = 1

    omg_max=vmax
    # Shoot up the ball

    a = ((d1 / vmax)+(deg1/omg_max))+(factor1*d1)+(factor2*deg1)
    M=int(round(a/(0.5*N)))

    #print(a,d1,deg1,M,'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1')
    # Shoot up the ball
    stage1, x1, y1,theta1,Vx1,Vy1,omega1 = create_ocp_stage(ocp)
    ocp.subject_to(stage1.t0 == 0)  # Stage starts at time 0

    # Initial constraints
    stage1.subject_to(stage1.at_t0(x1) == points[0][0])
    stage1.subject_to(stage1.at_t0(y1) == points[0][1])
    stage1.subject_to(stage1.at_t0(theta1) == points[0][2])

    # Final constraint
    stage1.subject_to(stage1.at_tf(x1) == points[1][0])
    stage1.subject_to(stage1.at_tf(y1) == points[1][1])
    stage1.subject_to(stage1.at_tf(theta1) == points[1][2])

    stage1.set_initial(x1, np.linspace(points[0][0],points[1][0],N))
    stage1.set_initial(y1, np.linspace(points[0][1],points[1][1],N) )
    stage1.set_initial(theta1, np.linspace(points[0][2],points[1][2],N))


    # Pick a solution method
    options = {'ipopt': {"max_iter": 100000}}
    options["expand"] = True
    options["print_time"] = True
    options["error_on_fail"] = True
    ocp.solver('ipopt', options)

    # Solve
    sol = ocp.solve()

    # Plot the 3 bounces

    ts11, x1_s = sol(stage1).sample(x1, grid='integrator')
    ts1, y1_s = sol(stage1).sample(y1, grid='integrator')
    ts1, theta1_s = sol(stage1).sample(theta1, grid='integrator')
    ts1, Vx1_s = sol(stage1).sample(Vx1, grid='integrator')
    ts1, Vy1_s = sol(stage1).sample(Vy1, grid='integrator')
    ts1, omega1_s = sol(stage1).sample(omega1, grid='integrator')


    #print(sol.value(stage1.T)),'ddddddddddddddddddddddddddddddddddddd')

    deltaf1_s=np.zeros((len(Vx1_s),1),dtype=float)


    deltar1_s = np.zeros((len(Vx1_s), 1), dtype=float)


    Vf1_s=np.zeros((len(Vx1_s), 1), dtype=float)


    Vr1_s=np.zeros((len(Vx1_s), 1), dtype=float)


    for k in range((len(Vx1_s))):

        deltaf1_s[k],deltar1_s[k]=steering_angle(theta1_s[k],Vx1_s[k],Vy1_s[k],omega1_s[k])
        Vr1_s[k] = sqrt((Vx1_s[k] ** 2 + Vy1_s[k] ** 2 + (omega1_s[k] * Lr) ** 2 + (2 * omega1_s[k] * Lr * (Vx1_s[k] * sin(theta1_s[k]) - Vy1_s[k] * cos(theta1_s[k])))))
        Vf1_s[k] = sqrt((Vx1_s[k]**2 +Vy1_s[k]**2 + (omega1_s[k]*Lf)**2 +(2 *omega1_s[k]*Lf*(Vy1_s[k] *cos(theta1_s[k]) -Vx1_s[k] *sin(theta1_s[k])))))

        #print(Vr1_s,Vr2_s,Vr3_s,'vvvvvvvvvvvvrrrrrrrrrrrrrrrrrrrrrrrrr')

    plt.figure()

    plt.plot(x1_s, y1_s)

    plt.title('x,y')


    plt.figure()

    plt.plot(ts1, Vx1_s)

    plt.title('t vs xvel')


    plt.figure()

    plt.plot(ts1, Vy1_s)

    plt.title('t vs yvel')


    plt.figure()

    plt.plot(ts1, omega1_s)


    plt.title('t vs omega')

    plt.show(block=True)


    t_s=ts11

    diff_t=np.diff(t_s)
    #print(diff_t,'difffffffffffffffffffffffffffffffffffffffffffff')

    x_d = x1_s
    y_d = y1_s
    theta_d =theta1_s

    deltaf_d = deltaf1_s
    deltar_d = deltar1_s

    #Vf = np.concatenate((Vf1_s,Vf2_s,Vf3_s),axis=None)
    #Vr = np.concatenate((Vr1_s,Vr2_s,Vr3_s),axis=None)

    Vx_i = Vx1_s
    Vy_i = Vy1_s
    omega_i=omega1_s

    Vx_i = Vx1_s
    Vy_i = Vy1_s
    omega_i =omega1_s

    #with np.printoptions(edgeitems=10000):
     #print(len(diff_t),'ts11111111111111111111111111111')

    Vf=(len(x_d)-1)*[None]
    Vr =(len(x_d)-1)*[None]
    deltaf =(len(x_d)-1)*[None]
    deltar =(len(x_d)-1)*[None]

    for i in range(len(x_d)-1):
        Vf[i],Vr[i],deltaf[i],deltar[i]=controller(Vx_i[i],Vy_i[i],omega_i[i],theta_d[i])
        #print([Vf[i],math.degrees(deltaf[i]),Vr[i],math.degrees(deltar[i])])

    points=[ [0]*3 for i in range(len(x_d))]
    for i in range(len(x_d)):
     points[i][0]=x_d[i]
     points[i][1]=y_d[i]
     points[i][2]=theta_d[i]
    return points,Vx_i,Vy_i,omega_i,diff_t,t_s
    #return points,Vx_i,Vy_i,omega_i,diff_t,Vf,Vr,deltaf,deltar

if __name__ == '__main__':



    gui_rock()
