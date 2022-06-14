#     This file is part of rockit.
#
#     rockit -- Rapid Optimal Control Kit
#     Copyright (C) 2019 MECO, KU Leuven. All rights reserved.
#
#     Rockit is free software; you can redistribute it and/or
#     modify it under the terms of the GNU Lesser General Public
#     License as published by the Free Software Foundation; either
#     version 3 of the License, or (at your option) any later version.
#
#     Rockit is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     Lesser General Public License for more details.
#
#     You should have received a copy of the GNU Lesser General Public
#     License along with CasADi; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

"""
Motion planning
===============

Straight line along 45 degree
"""
from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan, arctan, sqrt,arctan2
from casadi import vertcat, sumsqr,DM
from matplotlib import animation
from itertools import combinations
import time
import pdb
import math
import yaml
import queue

from rockit import *
from numpy import pi, cos, sin, tan, arctan2, arctan, sqrt
from matplotlib import animation
from tkinter import *
from functools import partial
import pdb as pdb


def gui_rock(init_loc,fin_loc):


    ##########################################################################################################
    ##########################################################################################################
    ###########################################################################################################

    #steering angle builder

    def steering_angle(thetas,vxs,vys,omegas):

        # Proportional control inputs are calculated
        Lf = 0.45
        Lr = 0.45

        numr = (vys - (omegas * Lr * cos(thetas)))
        denr = (vxs + (omegas * Lr * sin(thetas)))
        numf = (vys + (omegas * Lf * cos(thetas)))
        denf = (vxs - (omegas * Lf * sin(thetas)))

        deltar = arctan2(numr, denr) - thetas

        deltaf = arctan2(numf, denf) - thetas

        return deltaf, deltar

    # Build a plot where you can visualize the init and fin cons

    Init_con = init_loc
    Fin_con=fin_loc

    print(Fin_con,'finnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnconnnnnnnnnnnnnnnnn')

    ###################################################
    # this needs to change, cannot be free time
    ####################################################

    ########################################################################################
    #preprocessing of initcon and fincon to make sure there the shortest theta is traversed.
    #######################################################################################

    vmax = [0.018,0.019]

    robotN = len(init_loc)

    ani = 1

    for i in range(robotN):

        #distance between initial angle and the final angle after the jump

        jump_d=abs(Init_con[i][2]-(2*pi))+Fin_con[i][2]

        #distance between the initial angle and the continous final angle

        dist= abs(Init_con[i][2]-Fin_con[i][2])

        if jump_d<dist:

            if Init_con[i][2]<Fin_con[i][2]:
                Init_con[i][2]+=(2*pi)
            elif(Init_con[i][2]>Fin_con[i][2]):
                Fin_con[i][2]+=(2*pi)

    print(Init_con, Fin_con)
    #print(Init_con[:][2],Fin_con[:][2],'gggggggggggggggggggggggggggggggggggggggggggggggggggggggggggg')

    d=[None]*robotN

    deg=[None]*robotN

    for i in range (robotN):

     d[i] = sqrt((Fin_con[i][0] - Init_con[i][0]) ** 2 + (Fin_con[i][1] - Init_con[i][1]) ** 2)

     deg[i] = abs((Fin_con[i][2] - Init_con[i][2]))

    d1=max(d)
    deg1=max(deg)

    N = 21 + round((80 / 22) * float(d1))

    factor1 = 25
    factor2 = 0.1
    omg_max = vmax
    # Shoot up the ball

    a = ((d1 / vmax[0]) + (deg1 / omg_max[0])) + (factor1 * d1) + (factor2 * deg1)

    M = int(round(a / (0.5 * N)))

    ocp = Ocp(t0=0,T=a)
    #ocp=Ocp(t0=0,T=FreeTime(10))

    x = [None] * robotN
    y = [None] * robotN

    b=[None] * robotN
    theta = [None] * robotN

    deltaf = [None] * robotN
    deltar = [None] * robotN

    in_x= [None] * robotN
    in_y= [None] * robotN
    in_theta= [None] * robotN

    Vx = [None] * robotN
    Vy = [None] * robotN
    omega = [None] * robotN

    Vrs = [None] * robotN
    Vfs = [None] * robotN

    p = [None] * robotN

    Lr = 0.45
    Lf = 0.45
    #vmax = 0.0123

    #max acceleration
    a_max=1
    #max angular acceleration
    alpha_max=1

    for i in range(robotN):
        # kinematic model

        x[i] = ocp.state()
        y[i] = ocp.state()
        theta[i] = ocp.state()

        omega[i] = ocp.control(order=1)
        Vx[i] = ocp.control(order=1)
        Vy[i] = ocp.control(order=1)

        p[i] = vertcat(x[i], y[i])

        ocp.set_der(x[i], Vx[i])
        ocp.set_der(y[i], Vy[i])
        ocp.set_der(theta[i], omega[i])

        # Initial constraints
        ocp.subject_to(ocp.at_t0(x[i]) == Init_con[i][0])
        ocp.subject_to(ocp.at_t0(y[i]) == Init_con[i][1])
        ocp.subject_to(ocp.at_t0(theta[i]) == Init_con[i][2])

        # Final constraint
        ocp.subject_to(ocp.at_tf(x[i]) == Fin_con[i][0])
        ocp.subject_to(ocp.at_tf(y[i]) == Fin_con[i][1])
        ocp.subject_to(ocp.at_tf(theta[i]) == Fin_con[i][2])

        Vrs[i]=(Vx[i]**2+Vy[i]**2+(omega[i]*Lr)**2+(2*omega[i]*Lr*(Vx[i]*sin(theta[i])-Vy[i]*cos(theta[i]))))

        Vfs[i]=(Vx[i]**2 +Vy[i]**2 +(omega[i]*Lf)**2 +(2 *omega[i]*Lf*(Vy[i] *cos(theta[i]) -Vx[i] *sin(theta[i]))))

        in_x[i] = np.linspace(Init_con[i][0], Fin_con[i][0], N)
        in_y[i] = np.linspace(Init_con[i][1], Fin_con[i][1], N)
        in_theta[i] = np.linspace(Init_con[i][2], Fin_con[i][2], N)

        ocp.set_initial(x[i], in_x[i])
        ocp.set_initial(y[i], in_y[i])
        ocp.set_initial(theta[i], in_theta[i])

        ocp.subject_to(0 <= (Vrs[i] <= vmax[i]**2))
        ocp.subject_to(0<= (Vfs[i] <= vmax[i]**2))

        ocp.subject_to(-7.5 <= (x[i] <= 7.5))
        ocp.subject_to(-7.5 <= (y[i] <= 7.5))


    #ocp.add_objective(10* ocp.T)
    ocp.add_objective(2 * ocp.integral(sum(i*i for i in Vx)+sum(i*i for i in Vy)))
    ocp.add_objective(10 * ocp.integral(sum(i*i for i in omega)))
    #ocp.add_objective(1*ocp.integral(y[0]**2+y[1]**2)**2)

    #robot dimensions for collision

    r0=0.5

    #Cominations of all robot pairs for collission avoidance

    Number_C = int(robotN * (robotN - 1) / 2)

    iterable = list(range(0, robotN))

    comb = list(combinations(iterable, 2))

    sft=0.1

    for i in range(Number_C):

        index = list(comb[i])
        a = index[0]
        b = index[1]

        ocp.subject_to(sumsqr(p[a]-p[b])>=(2*r0+sft)**2)

    # Pick a solution method
    options = {'ipopt': {"max_iter": 100000}}
    options["expand"] = True
    options["print_time"] = True
    options["error_on_fail"] = True
    ocp.solver('ipopt', options)

    # Make it concrete for this ocp
    ocp.method(MultipleShooting(N=N-1, M=M, intg='rk'))

    # solve
    sol = ocp.solve()

    # plotting

    x_s = [[None] * (N*M) for _ in range(robotN)]
    y_s = [[None] * (N*M) for _ in range(robotN)]
    theta_s = [[None] * (N*M) for _ in range(robotN)]
    deltaf_s = [[None] * (N*M) for _ in range(robotN)]
    deltar_s = [[None] * (N*M) for _ in range(robotN)]
    Vr_s = [[None] * (N*M) for _ in range(robotN)]
    Vf_s = [[None] * (N*M) for _ in range(robotN)]
    omega_s = [[None] * (N*M) for _ in range(robotN)]
    Vx_s = [[None] * (N*M) for _ in range(robotN)]
    Vy_s = [[None] * (N*M) for _ in range(robotN)]

    robs_s = [[None] * (N*M) for _ in range(robotN)]

    for i in range(robotN):
        ts, x_s[i][:] = sol.sample(x[i], grid='integrator')
        ts, y_s[i][:] = sol.sample(y[i], grid='integrator')
        ts, theta_s[i][:] = sol.sample(theta[i], grid='integrator')
        ts, Vx_s[i][:] = sol.sample(Vx[i], grid='integrator')
        ts, Vy_s[i][:] = sol.sample(Vy[i], grid='integrator')
        ts, omega_s[i][:] = sol.sample(omega[i], grid='integrator')

    diff_t=np.diff(ts)

    #plotting

    for i in range(robotN):
     for k in range(len(Vx_s[0][:])):

        deltaf_s[i][k],deltar_s[i][k]=steering_angle(theta_s[i][k],Vx_s[i][k],Vy_s[i][k],omega_s[i][k])

        Vr_s[i][k] = sqrt((Vx_s[i][k] ** 2 + Vy_s[i][k] ** 2 + (omega_s[i][k] * Lr) ** 2 + (2 * omega_s[i][k] * Lr * (Vx_s[i][k] * sin(theta_s[i][k]) - Vy_s[i][k] * cos(theta_s[i][k])))))

        Vf_s[i][k] = sqrt((Vx_s[i][k]**2 +Vy_s[i][k]**2 +(omega_s[i][k]*Lf)**2 +(2 *omega_s[i][k]*Lf*(Vy_s[i][k] *cos(theta_s[i][k]) -Vx_s[i][k] *sin(theta_s[i][k])))))


    waypoints=np.zeros((robotN,3,((N-1)*M)+1),dtype=float)

    print(np.shape(x_s))

    for i in range (robotN):
       #waypoints=[x_s,y_s,theta_s]
        waypoints[i,0,:]=x_s[i][:]
        waypoints[i,1,:]=y_s[i][:]
        waypoints[i,2,:]=theta_s[i][:]


    print(np.shape(waypoints),np.shape(Vx_s),np.shape(Vy_s))

    Vx_s=np.array(Vx_s)
    Vy_s=np.array(Vy_s)
    omega_s=np.array(omega_s)
    #print(diff_t)

    plt.plot(ts,waypoints[0,0,:])
    plt.show()
    return waypoints,Vx_s,Vy_s,omega_s,diff_t,deltaf_s,deltar_s

if __name__ == '__main__':


    p1=[[0,0,0],[0,2,0]]
    p2=[[0, 0.2, 0], [0, 2.2, 0]]
    p3=[[0.2, 0.2, 0], [0.2, 2.2, 0]]

    Nr=len(p1)

    r0=0.5

    [waypoints1, Vx_s1, Vy_s1, omega_s1, diff_t1,deltaf_s1,deltar_s1] = gui_rock(p1,p2)
    [waypoints2, Vx_s2, Vy_s2, omega_s2, diff_t2,deltaf_s2,deltar_s2] = gui_rock(p2,p3)

    waypoints = np.concatenate((waypoints1, waypoints2), axis=2)

    Vx_s=np.concatenate((Vx_s1, Vx_s2), axis=1)
    Vy_s=np.concatenate((Vy_s1, Vy_s2), axis=1)
    omega_s=np.concatenate((omega_s1, omega_s2), axis=1)
    diff_t=np.concatenate((diff_t1, diff_t2), axis=0)
    deltaf_s=np.concatenate((deltaf_s1,deltaf_s2),axis=1)
    deltar_s=np.concatenate((deltar_s1, deltar_s2),axis=1)

    # Animation
    x_d = waypoints[:,0,:]
    y_d = waypoints[:,1,:]
    theta = waypoints[:,2,:]
    deltaf = deltaf_s
    deltar = deltar_s

    print(np.shape(theta))

    # robot dimension
    ht = 0.3
    wt = 1

    # wheel dimensions
    ht_w = 0.05
    wt_w = 0.1

    hyp = sqrt((0.5 * ht) ** 2 + (0.5 * wt) ** 2)
    ang = arctan(ht / wt)

    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    # np.abs(Init_con[:][1]+Fin_con[:][1]+[7.5])
    xl = np.abs(x_d).max() + 1.5
    yl = np.abs(y_d).max() + 1.5
    ax = plt.axes(xlim=(xl, -xl), ylim=(-yl, yl), aspect='equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_title("robot position")

    line = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for robots
    linewf = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for front wheels
    linewr = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for rear wheels
    line_ex = [ax.plot([], [], lw=1, markersize=0.1)[0] for _ in range(Nr)]  # lines to animate for path followed we only have to plot (x_d[ri,ki]) points
    line_circ = [ax.plot([], [], lw=1, markersize=0.1)[0] for _ in range(Nr)]  # circle around vehicle
    line_in = [ax.scatter([], [], s=50, c='blue', marker='o') for _ in range(Nr)]  # initial position
    line_fin = [ax.scatter([], [], s=500, c='red', marker='+') for _ in range(Nr)]  # final position

    lines = line + linewf + linewr + line_ex + line_circ


    # initialization function: plot the background of each frame
    def init():
        for l in lines:
            l.set_data([], [])
        for l in line_in:
            l.set_offsets([])
        for l in line_fin:
            l.set_offsets([])
        return lines, line_in, line_fin


    # pdb.set_trace()

    # animation function.  This is called sequentially
    def animate(i):
        # Dynamic objects

        """perform animation step"""
        # draw ith rectangle
        points = np.array(
            [[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
        pointsw = np.array([[-0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, +0.5 * ht_w],
                            [-0.5 * wt_w, +0.5 * ht_w]])

        x_r = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_t = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_rw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_tw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')

        # robots
        for k, ln in enumerate(line):
            # xx=[[0]*5]*Nr
            # yy=[[0]*5]*Nr
            for j in range(len(points)):
                x_r[k, j, 0] = points[j, 0] * np.cos(theta[k, i]) - points[j, 1] * np.sin(theta[k, i])
                x_r[k, j, 1] = points[j, 0] * np.sin(theta[k, i]) + points[j, 1] * np.cos(theta[k, i])
                x_t[k, j, :] = x_r[k, j, 0] + x_d[k, i], x_r[k, j, 1] + y_d[k, i]

            xx = x_t[k, :, 0]
            xx = np.concatenate((xx, np.array([x_t[k, 0, 0]])))
            yy = x_t[k, :, 1]
            yy = np.concatenate((yy, np.array([x_t[k, 0, 1]])))

            ln.set_data(xx, yy)

            # wheels

        for k, ln in enumerate(linewf):
            # xxw = [[0] * 5] * Nr
            # yyw = [[0] * 5] * Nr
            for j in range(len(pointsw)):
                x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[k, i] + deltaf[k, i]) - pointsw[j, 1] * np.sin(
                    theta[k, i] + deltaf[k, i])
                x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[k, i] + deltaf[k, i]) + pointsw[j, 1] * np.cos(
                    theta[k, i] + deltaf[k, i])

                # translation
                tx = wt / 2 * cos(theta[k][i])
                ty = wt / 2 * sin(theta[k][i])
                x_tw[k, j, :] = x_rw[k, j, 0] + x_d[k, i] + tx, x_rw[k, j, 1] + y_d[k, i] + ty

            xxw = x_tw[k, :, 0]
            xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
            yyw = x_tw[k, :, 1]
            yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

            ln.set_data(xxw, yyw)

        for k, ln in enumerate(linewr):
            # xxw = [[0] * 5] * Nr
            # yyw = [[0] * 5] * Nr
            for j in range(len(pointsw)):
                x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[k, i] + deltar[k, i]) - pointsw[j, 1] * np.sin(
                    theta[k, i] + deltar[k, i])
                x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[k, i] + deltar[k, i]) + pointsw[j, 1] * np.cos(
                    theta[k, i] + deltar[k, i])

                # translation
                tx = wt / 2 * cos(theta[k][i])
                ty = wt / 2 * sin(theta[k][i])
                x_tw[k, j, :] = x_rw[k, j, 0] + x_d[k, i] - tx, x_rw[k, j, 1] + y_d[k, i] - ty

            xxw = x_tw[k, :, 0]
            xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
            yyw = x_tw[k, :, 1]
            yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

            ln.set_data(xxw, yyw)

        for k, ln in enumerate(line_ex):
            ln.set_data(x_d[k, :i], y_d[k, :i])

        for k, ln in enumerate(line_circ):
            ts = np.linspace(0, 2 * pi, 100)
            ln.set_data(x_d[k, i] + r0 * cos(ts), y_d[k, i] + r0 * sin(ts))

        for k, ln in enumerate(line_in):
            ln.set_offsets([x_d[k, 0], y_d[k, 0]])

        for k, ln in enumerate(line_fin):
            ln.set_offsets([x_d[k, -1], y_d[k, -1]])

        return lines, line_in, line_fin

    frames=len(waypoints[0,0,:])


    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=frames, interval=2, blit=False)

    plt.show()

