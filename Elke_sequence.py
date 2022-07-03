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
    Fin_con = fin_loc

    ani = 1

    ht = 0.25
    wt = 0.9

    vmax = 0.018

    #safety distance
    sft = 0.05

    ind_same = []

    for i in range(len(init_loc)):
        if init_loc[i] == fin_loc[i]:
            ind_same.append(i)

    # create vertices of rectangles for stationary robots
    
    print(len(init_loc),len(ind_same),'indsameeeeeeeeeeeeeeeeeeeeeeeeeeee')

    robotN = len(init_loc)-len(ind_same)

    ob_points = [[None] * 4 for _ in range(len(ind_same))]

    ax =[[None] * len(ind_same) for _ in range(robotN)]
    ay =[[None] * len(ind_same) for _ in range(robotN)]
    b = [[None] * len(ind_same) for _ in range(robotN)]


    for j in range(len(ind_same)):

        indx=ind_same[j]

        # rectangular robots
        vert1 = [-wt / 2, ht / 2]
        vert2 = [-wt / 2, -ht / 2]
        vert3 = [+wt / 2, -ht / 2]
        vert4 = [wt / 2, ht / 2]

        points1_r = [vert1[0] * cos(init_loc[indx][2]) - vert1[1] * sin(init_loc[indx][2]),
                      vert1[0] * sin(init_loc[indx][2]) + vert1[1] * cos(init_loc[indx][2])]

        points2_r = [vert2[0] * cos(init_loc[indx][2]) - vert2[1] * sin(init_loc[indx][2]),
                      vert2[0] * sin(init_loc[indx][2]) + vert2[1] * cos(init_loc[indx][2])]

        points3_r = [vert3[0] * cos(init_loc[indx][2]) - vert3[1] * sin(init_loc[indx][2]),
                      vert3[0] * sin(init_loc[indx][2]) + vert3[1] * cos(init_loc[indx][2])]

        points4_r = [vert4[0] * cos(init_loc[indx][2]) - vert4[1] * sin(init_loc[indx][2]),
                      vert4[0] * sin(init_loc[indx][2]) + vert4[1] * cos(init_loc[indx][2])]


        ob_points[j][:] = [vertcat(points1_r[0] +init_loc[indx][0] , points1_r[1] + init_loc[indx][1]),
                      vertcat(points2_r[0] + init_loc[indx][0], points2_r[1] + init_loc[indx][1]),
                      vertcat(points3_r[0] + init_loc[indx][0], points3_r[1] + init_loc[indx][1]),
                      vertcat(points4_r[0] + init_loc[indx][0], points4_r[1] + init_loc[indx][1])]

    loc_s = []

    for i in ind_same:
        loc_s.append(init_loc[i])


    print(ind_same, 'initialllllllllllllllllllllllllllllllllllllllllll')


    def delete_multiple_element(list_object, indices):
        indices = sorted(indices, reverse=True)
        for idx in indices:
            if idx < len(list_object):
                list_object.pop(idx)

    delete_multiple_element(init_loc,ind_same)
    delete_multiple_element(fin_loc, ind_same)


    print(Fin_con,'finnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnconnnnnnnnnnnnnnnnn')

    ###################################################
    # this needs to change, cannot be free time
    ####################################################

    ########################################################################################
    #preprocessing of initcon and fincon to make sure there the shortest theta is traversed.
    #######################################################################################


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

    print(robotN,'robotN')

    for i in range (robotN):

     d[i] = sqrt((Fin_con[i][0] - Init_con[i][0]) ** 2 + (Fin_con[i][1] - Init_con[i][1]) ** 2)
     print(i,d[i],'wtffffffffffffffffffffffffffffffffffff')
     deg[i] = abs((Fin_con[i][2] - Init_con[i][2]))

    d1=max(d)

    deg1=max(deg)

    N = 21 + round((80 / 22) * float(d1))

    factor1 = 0
    factor2 = 0
    omg_max = vmax

    # Shoot up the ball

    a = ((d1 / vmax) + (deg1 / omg_max)) + (factor1 * d1) + (factor2 * deg1)

    a=a

    print(a,'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')

    M = int(round(a / (0.5 * N)))

    ocp = Ocp(t0=0,T=a)
    #ocp=Ocp(t0=0,T=FreeTime(10))

    x = [None] * robotN
    y = [None] * robotN

    #b=[None] * robotN
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

    p1 = [None] * robotN
    print(p1,'pppppppppppppp')

    vert1_r = [None] * robotN
    vert2_r = [None] * robotN
    vert3_r = [None] * robotN
    vert4_r = [None] * robotN

    robs = [[None] * 4 for _ in range(robotN)]


    Lr = 0.45
    Lf = 0.45
    #vmax = 0.0123

    #max acceleration
    a_max=1
    #max angular acceleration
    alpha_max=1
    print(robotN,'robotttttttttN')
    for i in range(robotN):
        # kinematic model

        x[i] = ocp.state()
        y[i] = ocp.state()
        theta[i] = ocp.state()

        omega[i] = ocp.control(order=1)
        Vx[i] = ocp.control(order=1)
        Vy[i] = ocp.control(order=1)

        print( p1[i],vertcat(x[i], y[i]),'xyyyyyyyyyyyyyyyyyyyyyyyyyyyyyy')

        #p1[i] = vertcat(x[i], y[i])

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

        ocp.subject_to(0 <= (Vrs[i] <= vmax**2))
        ocp.subject_to(0<= (Vfs[i] <= vmax**2))

        ocp.subject_to(-7.5 <= (x[i] <= 7.5))
        ocp.subject_to(-7.5 <= (y[i] <= 7.5))

        # rectangular robots

        vert1 = [-wt / 2, ht / 2]
        vert2 = [-wt / 2, -ht / 2]
        vert3 = [+wt / 2, -ht / 2]
        vert4 = [wt/2, ht / 2]


        vert1_r[i] = [vert1[0] * cos(theta[i]) - vert1[1] * sin(theta[i]),
                      vert1[0] * sin(theta[i]) + vert1[1] * cos(theta[i])]

        vert2_r[i] = [vert2[0] * cos(theta[i]) - vert2[1] * sin(theta[i]),
                      vert2[0] * sin(theta[i]) + vert2[1] * cos(theta[i])]

        vert3_r[i] = [vert3[0] * cos(theta[i]) - vert3[1] * sin(theta[i]),
                      vert3[0] * sin(theta[i]) + vert3[1] * cos(theta[i])]

        vert4_r[i] = [vert4[0] * cos(theta[i]) - vert4[1] * sin(theta[i]),
                      vert4[0] * sin(theta[i]) + vert4[1] * cos(theta[i])]

        robs[i][:] = [vertcat(vert1_r[i][0] + x[i], vert1_r[i][1] + y[i]),
                      vertcat(vert2_r[i][0] + x[i], vert2_r[i][1] + y[i]),
                      vertcat(vert3_r[i][0] + x[i], vert3_r[i][1] + y[i]),
                      vertcat(vert4_r[i][0] + x[i], vert4_r[i][1] + y[i])]


        # obstacle avoidance

        if ob_points:

            for j in range(len(ob_points)):

                ax[i][j] = ocp.control(order=0)
                ay[i][j] = ocp.control(order=0)
                b[i][j] = ocp.control(order=0)

                for p in ob_points[j]:
                    ocp.subject_to(ax[i][j] * p[0] + ay[i][j] * p[1] <= b[i][j])

                for q in robs[i][:]:
                    ocp.subject_to(ax[i][j] * q[0] + ay[i][j] * q[1] >= sft + b[i][j])

                ocp.subject_to(ax[i][j] ** 2 + ay[i][j] ** 2 <= 1)

    #ocp.add_objective(10* ocp.T)
    ocp.add_objective(2 * ocp.integral(sum(i*i for i in Vx)+sum(i*i for i in Vy)))
    ocp.add_objective(2 * ocp.integral(sum(i*i for i in omega)))
    #ocp.add_objective(1 * ocp.integral((x[2]-init_loc[2][0])**2+(y[2]-init_loc[2][1])**2+(theta[2]-init_loc[2][2])**2))
    #ocp.add_objective(1*ocp.integral(y[0]**2+y[1]**2)**2)


    # Cominations of all robot pairs for collission avoidance

    Number_C = int(robotN * (robotN - 1) / 2)

    iterable = list(range(0, robotN))

    comb = list(combinations(iterable, 2))

    ax_r = [None] * Number_C
    ay_r = [None] * Number_C
    b_r = [None] * Number_C

    for i in range(Number_C):

        ax_r[i] = ocp.control(order=0)
        ay_r[i] = ocp.control(order=0)
        b_r[i] = ocp.control(order=0)

        index = list(comb[i])

        a = index[0]
        b = index[1]

        # define here the inter-robot collision constraints

        for p in robs[a][:]:
            ocp.subject_to(ax_r[i] * p[0] + ay_r[i] * p[1] >= sft + b_r[i])

        for q in robs[b][:]:
            ocp.subject_to(ax_r[i] * q[0] + ay_r[i] * q[1] <= b_r[i])

        ocp.subject_to(ax_r[i] ** 2 + ay_r[i] ** 2 <= 1)



    # Pick a solution method
    options = {'ipopt': {"linear_solver":"ma27","max_iter": 100000}} ##mumps
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

    #print(max(Vx_s),max(Vy_s),'maxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxvelllllllllllllllllllllllll')
    print(ts[-1],'horrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr')
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

    if ani==1:

        #Animation

        x_d=np.array(x_s)
        y_d=np.array(y_s)
        theta=np.array(theta_s)
        deltaf=np.array(deltaf_s)
        deltar=np.array(deltar_s)
        Vr=np.array(Vr_s)
        Vf=np.array(Vf_s)
        Vx=np.array(Vx_s)
        Vy=np.array(Vy_s)


        #location of fixed robots
        x_d_s=np.array([i[0] for i in loc_s])
        y_d_s=np.array([i[1] for i in loc_s])
        theta_d_s=np.array([i[2] for i in loc_s])

        print(np.shape(theta))

        #number of dynamic robots

        Nr=robotN

        # Number of static robots

        Ns=len(ind_same)

        #robot dimension
        ht=0.25
        wt=0.9

        #wheel dimensions
        ht_w=0.05
        wt_w=0.1

        hyp=sqrt((0.5*ht)**2+(0.5*wt)**2)
        ang=arctan(ht/wt)

        # First set up the figure, the axis, and the plot element we want to animate
        fig = plt.figure()
        #np.abs(Init_con[:][1]+Fin_con[:][1]+[7.5])
        xl=np.abs(x_d).max()+1.5
        yl=np.abs(y_d).max()+1.5
        ax = plt.axes(xlim=(xl, -xl), ylim=(-yl, yl),aspect='equal')
        ax.set_xlabel('x(m)')
        ax.set_ylabel('y(m)')
        ax.set_title("robot position")

        line = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for robots
        line_s = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Ns)]  # lines to animate for static robots
        linewf = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for front wheels
        linewr = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for rear wheels
        line_ex = [ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #lines to animate for path followed we only have to plot (x_d[ri,ki]) points
        #line_circ=[ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #circle around vehicle
        line_in=[ax.scatter([], [],s=50, c='blue', marker='o') for _ in range(Nr)]  #initial position
        line_fin=[ax.scatter([], [],s=500, c='red', marker='+') for _ in range(Nr)] #final position

        lines=line+linewf+linewr+line_ex

        #initialization function: plot the background of each frame
        def init():
            for l in lines:
                l.set_data([], [])

            points = np.array(
                [[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])

            x_rs = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Ns)], dtype='float')
            x_ts = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Ns)], dtype='float')

            # static robots
            for k, ln in enumerate(line_s):
                # xx=[[0]*5]*Nr
                # yy=[[0]*5]*Nr
                for j in range(len(points)):
                    x_rs[k, j, 0] = points[j, 0] * np.cos(theta_d_s[k]) - points[j, 1] * np.sin(theta_d_s[k])
                    x_rs[k, j, 1] = points[j, 0] * np.sin(theta_d_s[k]) + points[j, 1] * np.cos(theta_d_s[k])
                    x_ts[k, j, :] = x_rs[k, j, 0] + x_d_s[k], x_rs[k, j, 1] + y_d_s[k]

                xx = x_ts[k, :, 0]
                xx = np.concatenate((xx, np.array([x_ts[k, 0, 0]])))
                yy = x_ts[k, :, 1]
                yy = np.concatenate((yy, np.array([x_ts[k, 0, 1]])))

                ln.set_data(xx, yy)

            for l in line_in:
                l.set_offsets([])
            for l in line_fin:
                l.set_offsets([])
            return lines,line_in,line_fin

        #pdb.set_trace()

        # animation function.  This is called sequentially
        def animate(i):
            #Dynamic objects

            """perform animation step"""
            # draw ith rectangle
            points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
            pointsw = np.array([[-0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, +0.5 * ht_w],
                                [-0.5 * wt_w, +0.5 * ht_w]])

            x_r = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)],dtype='float')
            x_t = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
            x_rw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
            x_tw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')

            #robots

            for k, ln in enumerate(line):
                #xx=[[0]*5]*Nr
                #yy=[[0]*5]*Nr
                for j in range(len(points)):
                    x_r[k, j, 0] = points[j, 0] * np.cos(theta[k, i]) - points[j, 1] * np.sin(theta[k, i])
                    x_r[k, j, 1] = points[j, 0] * np.sin(theta[k, i]) + points[j, 1] * np.cos(theta[k, i])
                    x_t[k, j, :] = x_r[k, j, 0] + x_d[k, i], x_r[k, j, 1] + y_d[k, i]

                xx=x_t[k,:,0]
                xx=np.concatenate((xx, np.array([x_t[k,0,0]])))
                yy = x_t[k,:,1]
                yy = np.concatenate((yy, np.array([x_t[k, 0, 1]])))

                ln.set_data(xx,yy)

                #wheels

            for k, ln in enumerate(linewf):
                #xxw = [[0] * 5] * Nr
                #yyw = [[0] * 5] * Nr
                for j in range(len(pointsw)):
                    x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[k, i] + deltaf[k, i]) - pointsw[j, 1] * np.sin(
                        theta[k, i] + deltaf[k, i])
                    x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[k, i] + deltaf[k, i]) + pointsw[j, 1] * np.cos(
                        theta[k, i] + deltaf[k, i])

                    # translation
                    tx = wt/2 * cos(theta[k][i])
                    ty = wt/2* sin(theta[k][i])
                    x_tw[k, j, :] = x_rw[k, j, 0] + x_d[k, i] + tx, x_rw[k, j, 1] + y_d[k, i] + ty

                xxw = x_tw[k, :, 0]
                xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
                yyw = x_tw[k, :, 1]
                yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

                ln.set_data(xxw, yyw)

            for k, ln in enumerate(linewr):
                    #xxw = [[0] * 5] * Nr
                    #yyw = [[0] * 5] * Nr
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
               ln.set_data(x_d[k,:i],y_d[k,:i])


            for k, ln in enumerate(line_in):
                ln.set_offsets([x_d[k, 0],y_d[k, 0]])

            for k, ln in enumerate(line_fin):
                ln.set_offsets([x_d[k, -1], y_d[k, -1]])

            return lines,line_in,line_fin

        anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=(N-1)*M, interval=1, blit=False)

        plt.show()


    Vx_s=np.array(Vx_s)
    Vy_s=np.array(Vy_s)
    omega_s=np.array(omega_s)
    #print(diff_t)

    plt.plot(ts,waypoints[0,0,:])

    plt.show()

    return waypoints,Vx_s,Vy_s,omega_s,diff_t,deltaf_s,deltar_s


if __name__ == '__main__':

    p1 = [[0,0,pi/2],[0.35,0,pi/2],[0.7,0,pi/2]]
    p2 = [[3,3.7,0],[3,3.35,0],[0.7,0,pi/2]]
    p3 = [[3, 3.7, 0], [3, 3.35, 0], [0.7, 0, pi / 2]]
    p4 = [[3,3.7,0],[3,3.35,0],[3,3,0]]

    Nr = len(p1)

    r0 = 0.5


    [waypoints1, Vx_s1, Vy_s1, omega_s1, diff_t1, deltaf_s1, deltar_s1] = gui_rock(p1, p2)


    [waypoints2, Vx_s2, Vy_s2, omega_s2, diff_t2, deltaf_s2, deltar_s2] = gui_rock(p3, p4)

    ############################################################################################################

    # Problem with concatenation:motion 1 has only 2 robots moving//

    # 1) an extra dimension of zeros could be created(dynamic) 2) disable do control1 (not dynamic)
    # 2) 1) get number of robots by len(waypoints)
    #    2) attach a Nrt-Nr init_loc(or the location of stationary object)  vector to waypoint1
    #    3) do the same for waypoint2
    #    4) then concatenate

    ############################################################################################################


    robotN=len(p1)

    Nr_d1=len(waypoints1)
    Nr_s1=robotN-Nr_d1
    st_loc1=[[0,1,2]]


    Nr_d2=len(waypoints2)
    Nr_s2 =robotN-Nr_d2
    st_loc2=[[0, 1, 2],[3,4,5]]


    v_1=np.full((0,3,len(waypoints1[0,0,:])), st_loc1[0])


    v_21=np.full((0,3,len(waypoints2[0,0,:])), st_loc2[0])
    v_22 =np.full((0,3,len(waypoints2[0, 0, :])), st_loc2[1])


    waypoints1c=np.concatenate((waypoints1,v_1),axis=0)
    waypoints2c=np.concatenate((waypoints2,v_21,v_22),axis=0)

    waypoints = np.concatenate((waypoints1c, waypoints2c), axis=2)

    Vx_s = np.concatenate((Vx_s1, Vx_s2), axis=1)
    Vy_s = np.concatenate((Vy_s1, Vy_s2), axis=1)
    omega_s = np.concatenate((omega_s1, omega_s2), axis=1)
    diff_t = np.concatenate((diff_t1, diff_t2), axis=0)
    deltaf_s = np.concatenate((deltaf_s1, deltaf_s2), axis=1)
    deltar_s = np.concatenate((deltar_s1, deltar_s2), axis=1)


    # Animation
    x_d = waypoints[:, 0, :]
    y_d = waypoints[:, 1, :]
    theta = waypoints[:, 2, :]

    deltaf = deltaf_s
    deltar = deltar_s

    #Animation

    print(np.shape(theta))

    #number of dynamic robots

    Nr=robotN

    # Number of static robots

    Ns=len(ind_same)

    #robot dimension
    ht=0.25
    wt=0.9

    #wheel dimensions
    ht_w=0.05
    wt_w=0.1

    hyp=sqrt((0.5*ht)**2+(0.5*wt)**2)
    ang=arctan(ht/wt)

    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    #np.abs(Init_con[:][1]+Fin_con[:][1]+[7.5])
    xl=np.abs(x_d).max()+1.5
    yl=np.abs(y_d).max()+1.5
    ax = plt.axes(xlim=(xl, -xl), ylim=(-yl, yl),aspect='equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_title("robot position")

    line = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for robots
    line_s = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Ns)]  # lines to animate for static robots
    linewf = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for front wheels
    linewr = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for rear wheels
    line_ex = [ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #lines to animate for path followed we only have to plot (x_d[ri,ki]) points
    #line_circ=[ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #circle around vehicle
    line_in=[ax.scatter([], [],s=50, c='blue', marker='o') for _ in range(Nr)]  #initial position
    line_fin=[ax.scatter([], [],s=500, c='red', marker='+') for _ in range(Nr)] #final position

    lines=line+linewf+linewr+line_ex

    #initialization function: plot the background of each frame
    def init():
        for l in lines:
            l.set_data([], [])

        points = np.array(
            [[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])

        x_rs = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Ns)], dtype='float')
        x_ts = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Ns)], dtype='float')

        # static robots
        for k, ln in enumerate(line_s):
            # xx=[[0]*5]*Nr
            # yy=[[0]*5]*Nr
            for j in range(len(points)):
                x_rs[k, j, 0] = points[j, 0] * np.cos(theta_d_s[k]) - points[j, 1] * np.sin(theta_d_s[k])
                x_rs[k, j, 1] = points[j, 0] * np.sin(theta_d_s[k]) + points[j, 1] * np.cos(theta_d_s[k])
                x_ts[k, j, :] = x_rs[k, j, 0] + x_d_s[k], x_rs[k, j, 1] + y_d_s[k]

            xx = x_ts[k, :, 0]
            xx = np.concatenate((xx, np.array([x_ts[k, 0, 0]])))
            yy = x_ts[k, :, 1]
            yy = np.concatenate((yy, np.array([x_ts[k, 0, 1]])))

            ln.set_data(xx, yy)

        for l in line_in:
            l.set_offsets([])
        for l in line_fin:
            l.set_offsets([])
        return lines,line_in,line_fin

    #pdb.set_trace()

    # animation function.  This is called sequentially
    def animate(i):
        #Dynamic objects

        """perform animation step"""
        # draw ith rectangle
        points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
        pointsw = np.array([[-0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, +0.5 * ht_w],
                            [-0.5 * wt_w, +0.5 * ht_w]])

        x_r = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)],dtype='float')
        x_t = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_rw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_tw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')

        #robots

        for k, ln in enumerate(line):
            #xx=[[0]*5]*Nr
            #yy=[[0]*5]*Nr
            for j in range(len(points)):
                x_r[k, j, 0] = points[j, 0] * np.cos(theta[k, i]) - points[j, 1] * np.sin(theta[k, i])
                x_r[k, j, 1] = points[j, 0] * np.sin(theta[k, i]) + points[j, 1] * np.cos(theta[k, i])
                x_t[k, j, :] = x_r[k, j, 0] + x_d[k, i], x_r[k, j, 1] + y_d[k, i]

            xx=x_t[k,:,0]
            xx=np.concatenate((xx, np.array([x_t[k,0,0]])))
            yy = x_t[k,:,1]
            yy = np.concatenate((yy, np.array([x_t[k, 0, 1]])))

            ln.set_data(xx,yy)

            #wheels

        for k, ln in enumerate(linewf):
            #xxw = [[0] * 5] * Nr
            #yyw = [[0] * 5] * Nr
            for j in range(len(pointsw)):
                x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[k, i] + deltaf[k, i]) - pointsw[j, 1] * np.sin(
                    theta[k, i] + deltaf[k, i])
                x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[k, i] + deltaf[k, i]) + pointsw[j, 1] * np.cos(
                    theta[k, i] + deltaf[k, i])

                # translation
                tx = wt/2 * cos(theta[k][i])
                ty = wt/2* sin(theta[k][i])
                x_tw[k, j, :] = x_rw[k, j, 0] + x_d[k, i] + tx, x_rw[k, j, 1] + y_d[k, i] + ty

            xxw = x_tw[k, :, 0]
            xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
            yyw = x_tw[k, :, 1]
            yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

            ln.set_data(xxw, yyw)

        for k, ln in enumerate(linewr):
                #xxw = [[0] * 5] * Nr
                #yyw = [[0] * 5] * Nr
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
           ln.set_data(x_d[k,:i],y_d[k,:i])


        for k, ln in enumerate(line_in):
            ln.set_offsets([x_d[k, 0],y_d[k, 0]])

        for k, ln in enumerate(line_fin):
            ln.set_offsets([x_d[k, -1], y_d[k, -1]])

        return lines,line_in,line_fin

    anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=len(waypoints), interval=1, blit=False)

    plt.show()


Vx_s=np.array(Vx_s)
Vy_s=np.array(Vy_s)
omega_s=np.array(omega_s)
#print(diff_t)

plt.plot(ts,waypoints[0,0,:])
plt.show()






