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


def gui_rock(init_loc):

    ##########################################################################################################
    ##########################################################################################################
    ###########################################################################################################

    # GUI 1
    # This is done first and then another GUI opens for choreography design

    # THIS IS THE FIRST GUI THAT OPENS AND TAKES IN THE PARAMETER VALUSE AND THEN CLOSES#

    def creating():
        root.destroy()

    # Creating GUI window
    root = Tk()
    root.title("Define parameters")

    # Creating mainframe to store widgets
    mainframe = Frame(root, width=700, height=500)
    mainframe.grid()
    mainframe.grid_propagate(1)

    # Defining labels for length and width
    Label(mainframe, text="Length of room:").grid(row=1, column=1)
    Label(mainframe, text="Width of room:").grid(row=3, column=1)
    Label(mainframe, text = "Number of robots:").grid(row=5, column = 1)

    # Defining variables for storing values from GUI
    gui_variable1 = DoubleVar()
    gui_variable2 = DoubleVar()
    gui_variable3=IntVar()

    # Creating two sliders to get user defined length and width
    Slider1 = Scale(mainframe, from_=5, to=20, length=1000, tickinterval=0.5, variable=gui_variable1, orient=HORIZONTAL,
                    resolution=0.5)
    Slider1.set(15)
    Slider1.grid(row=1, column=2)
    Slider2 = Scale(mainframe, from_=5, to=20, length=1000, tickinterval=0.5, variable=gui_variable2, orient=HORIZONTAL,
                    resolution=0.5)
    Slider2.set(15)
    Slider2.grid(row=3, column=2)
    Slider3 = Scale(mainframe, from_=1, to=4, length=600,tickinterval=1,variable=gui_variable3, orient=HORIZONTAL)
    Slider3.set(3)
    Slider3.grid(row = 5, column=2)

    # Executing button that closes GUI and continues with program
    Button(mainframe, text='select number of waypoints per robot', command=creating).grid(row=6, columnspan=3)

    # Waiting for button press
    root.mainloop()

    # Extracting values from sliders
    slider_value1 = gui_variable1.get()
    slider_value2 = gui_variable2.get()
    slider_value3= gui_variable3.get()

    #slider_value3 = len(init_loc)

    ## GUI 2.1 for number of waypoints per robot

    # Creating GUI window
    root = Tk()
    root.title("Define number of waypoints per robot in order")

    # Creating mainframe to store widgets
    mainframe = Frame(root, width=1000, height=1000)
    mainframe.grid()
    mainframe.grid_propagate(1)

    # Defining labels for length and width
    Label(mainframe, text="Define number of waypoints per robot in order:").grid(row=0, column=0)

    # Executing button that closes GUI and continues with program
    Button(mainframe, text='Design choregraphy', command=creating).grid(row=6, columnspan=3)

    gui_var = slider_value3 * [None]
    Slider = slider_value3 * [None]
    for i in range(slider_value3):
        gui_var[i] = IntVar()
        Slider[i] = Scale(master=mainframe, from_=2, to=4, length=300, tickinterval=1,
                          variable=gui_var[i], orient=HORIZONTAL)
        Slider[i].set(2)
        Slider[i].grid(row=i + 1, column=1)

    root.mainloop()

    slider_val = slider_value3 * [None]

    for i in range(slider_value3):
        slider_val[i] = gui_var[i].get()

    # Exiting GUI mainloop, and continuing with the program
    def creating():
        canvas.destroy()

    root = Tk()

    root.counter = 0

    def create_rect(x, y, angle, canvas, tag, fil):  # center coordinates, radius

        if fil == 0:
            fill = 'red'

        elif (fil == 1):
            fill = 'blue'

        elif (fil == 2):
            fill = 'black'

        elif (fil == 3):
            fill = 'yellow'

        else:
            fill = 'white'

        cw = 600
        ch = 600

        xl = slider_value1
        yl = slider_value2

        wt = 1
        ht = 0.3

        # forward transformation to world co-ordinates here

        y = ch - y
        x = (x - cw / 2) * (xl / cw)
        y = (y - ch / 2) * (yl / ch)

        angle = (pi / 180) * angle

        points = np.array(
            [[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])

        x_r = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')
        x_t = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')

        for j in range(len(points)):
            x_r[j, 0] = points[j, 0] * np.cos(angle) - points[j, 1] * np.sin(angle)
            x_r[j, 1] = points[j, 0] * np.sin(angle) + points[j, 1] * np.cos(angle)
            x_t[j, :] = x_r[j, 0] + x, x_r[j, 1] + y

        mm = x_t

        for i in mm:
            i[0] = (cw / xl) * (i[0] + 0.5 * xl)
            i[1] = (ch / yl) * (yl - (i[1] + 0.5 * yl))

        x1, y1 = mm[0]
        x2, y2 = mm[1]
        x3, y3 = mm[2]
        x4, y4 = mm[3]

        return canvas.create_polygon(x1, y1, x2, y2, x3, y3, x4, y4, tags=tag, fill=fill)

    # Number of waypoints

    Nw = slider_val

    Nw = [i - 1 for i in Nw]

    # Number of robots

    Nr = slider_value3

    ang_l = [[[] for j in range(Nw[i])] for i in range(Nr)]

    coord = [[] for i in range(Nr)]

    # canvas dimensions

    cw = 600
    ch = 600

    t_g = [['rect1', 'rect2', 'rect3', 'rect4'], ['rect12', 'rect22', 'rect32', 'rect42'],
           ['rect13', 'rect23', 'rect33', 'rect43'], ['rect14', 'rect24', 'rect34', 'rect44']]

    global m
    m = 0

    global n
    n = 0

    def myfunction(event):

        if root.counter < sum(Nw):
            root.counter += 1
            x, y = event.x, event.y
            global m

            coord[m].append([x, y])
            lp_count = len(coord[m])
            create_rect(x, y, 0, canvas, t_g[m][lp_count - 1], m)

            global n

            n += 1
            if (n == Nw[m] and root.counter < (sum(Nw))):
                m += 1
                n = 0

        if root.counter == sum(Nw):
            print('maximum number of points that can be specified has reached thank you!!')

        return coord

    def get_slider(event, l, k):
        ang[l][k] = gui_variable[l][k].get()

        # kk = len(ang_l[k])
        ang_l[l][k].append(ang[l][k])
        if len(ang_l[l][k]) > 0:
            canvas.delete(t_g[l][k])
        # if len(ang_l[k]) > kk:
        # canvas.delete(t_g[k])
        create_rect(coord[l][k][-2], coord[l][k][-1], ang_l[l][k][-1], canvas, t_g[l][k], l)
        if k == (sum(Nw) - 1):
            canvas.delete(t_g[l][Nw[k]])

    frame_a = Frame()
    Slider = [[[] for j in range(Nw[i])] for i in range(Nr)]
    ang = [[[] for j in range(Nw[i])] for i in range(Nr)]
    # global gui_variable
    gui_variable = [[[] for j in range(Nw[i])] for i in range(Nr)]

    for j in range(Nr):
        for i in range(Nw[j]):
            gui_variable[j][i] = IntVar()
            Slider[j][i] = Scale(master=frame_a, from_=0, to=360, length=300, tickinterval=30,
                                 variable=gui_variable[j][i], orient=HORIZONTAL, command=partial(get_slider, l=j, k=i))
            Slider[j][i].set(0)
            Slider[j][i].grid(row=i + 1, column=j + 1)

    for j in range(Nr):
        for i in range(Nw[j]):
            if not ang[j][i]:
                ang[j][i] = 0

    frame_b = Frame()
    canvas = Canvas(master=frame_b, width=cw, height=ch, bg='grey')
    canvas.create_line(cw / 2 - 20, cw / 2, cw / 2 + 20, cw / 2)
    canvas.create_line(ch / 2, ch / 2 - 20, ch / 2, ch / 2 + 20)

    cw = 600
    ch = 600
    xl = 15
    yl = 15

    for i in range(Nr):
        xx = init_loc[i][0]
        yy = init_loc[i][1]

        xxt = (cw / xl) * (xx + 0.5 * xl)
        yyt = (ch / yl) * (yl - (yy + 0.5 * yl))



        thetat = (180/pi)*init_loc[i][2]
        print(thetat, 'thetatttttttttttttt')
        create_rect(xxt, yyt, thetat, canvas, 'reci', i)
        print(xxt,yyt,i,'looooooooooooooooooooooooooooooooooooofer')

    canvas.bind("<Button-1>", myfunction)
    canvas.pack()

    frame_c = Frame()
    lb = Label(master=frame_c, text="select angles using sliders", fg="red", bg="white", width=100, height=3)

    lb.pack()

    frame_d = Frame()
    lb2 = Label(master=frame_d, text="select points by clicking on grey canvas", fg="green", bg="white", width=100,
                height=3)
    lb2.pack()

    # Swap the order of `frame_a` and `frame_b`
    frame_d.pack(side='top')
    frame_b.pack()
    frame_a.pack()
    frame_c.pack(side='top')

    root.old_coords = 0

    root.mainloop()

    print(coord, 'coordddddddddddddddd')
    print(ang, 'angleeeeeeeeeeeeeeeee')

    #########################################
    # transform points to world co-ordinates
    #########################################

    # forward transformation to world co-ordinates here

    coord_w = [[] for i in range(len(coord))]
    ang_w = [[[] for j in range(len(ang[0]))] for i in range(len(ang))]

    for i in range(len(coord)):
        for j in range(len(coord[i])):
            x, y = coord[i][j]

            y = ch - y
            x = (x - cw / 2) * (xl / cw)
            y = (y - ch / 2) * (yl / ch)

            print(i, j)

            coord_w[i].append([x, y])

            ang_w[i][j] = (pi / 180) * ang[i][j]

    for i in range(Nr):

        coord_w[i].insert(0, init_loc[i][0:2])
        ang_w[i].insert(0, init_loc[i][2])
        for j in range(len(ang_w[0])):
            coord_w[i][j].insert(2, ang_w[i][j])

    print(coord_w,'cooooooooooooooooooooooooooooooooooooooooooooord')

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
    Fin_con = [i[1] for i in coord_w]
    print(Fin_con,'finnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnconnnnnnnnnnnnnnnnn')

    ###################################################
    # this needs to change, cannot be free time
    ####################################################

    ########################################################################################
    #preprocessing of initcon and fincon to make sure there the shortest theta is traversed.
    #######################################################################################

    vmax = 0.018

    robotN = 2

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

    a = ((d1 / vmax) + (deg1 / omg_max)) + (factor1 * d1) + (factor2 * deg1)
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

    vmax=0.018

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

        ocp.subject_to(0 <= (Vrs[i] <= vmax**2))
        ocp.subject_to(0<= (Vfs[i] <= vmax**2))

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

        #ocp.subject_to(sumsqr(p[a]-p[b])>=(2*r0+sft)**2)

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


        print(np.shape(theta))

        Nr=robotN

        #robot dimension
        ht=0.3
        wt=1

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
        linewf = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for front wheels
        linewr = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for rear wheels
        line_ex = [ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #lines to animate for path followed we only have to plot (x_d[ri,ki]) points
        line_circ=[ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #circle around vehicle
        line_in=[ax.scatter([], [],s=50, c='blue', marker='o') for _ in range(Nr)]  #initial position
        line_fin=[ax.scatter([], [],s=500, c='red', marker='+') for _ in range(Nr)] #final position

        lines=line+linewf+linewr+line_ex+line_circ

        #initialization function: plot the background of each frame
        def init():
            for l in lines:
                l.set_data([], [])
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

            for k, ln in enumerate(line_circ):
                ts = np.linspace(0, 2 * pi, 100)
                ln.set_data(x_d[k,i] + r0 * cos(ts),y_d[k,i] + r0 * sin(ts))

            for k, ln in enumerate(line_in):
                ln.set_offsets([x_d[k, 0],y_d[k, 0]])

            for k, ln in enumerate(line_fin):
                ln.set_offsets([x_d[k, -1], y_d[k, -1]])

            return lines,line_in,line_fin

        anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=(N-1)*M, interval=2, blit=False)

        plt.show()


    print(np.shape(waypoints),np.shape(Vx_s),np.shape(Vy_s))

    Vx_s=np.array(Vx_s)
    Vy_s=np.array(Vy_s)
    omega_s=np.array(omega_s)
    #print(diff_t)

    print(diff_t,'dttttttttttttttttttttttttttttttttt')
    plt.plot(ts,waypoints[0,0,:])
    plt.show()
    return waypoints,Vx_s,Vy_s,omega_s,diff_t

if __name__ == '__main__':

    #init=[[0,0,1.8*pi],[0,2,1.8*pi],[0,-2,1.8*pi]]
   # finit = [[0, 0, 0], [0, 2, 0], [0, -2, 0]]

    gui_rock()


