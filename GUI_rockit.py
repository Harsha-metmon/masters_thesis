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


def gui_rock(init_loc):

    #GUI

    # Exiting GUI mainloop, and continuing with the program
    def creating():
        canvas.destroy()


    root = Tk()
    coord = []
    root.counter = 0


    def create_circle(x, y, r, canvas):  # center coordinates, radius
        x0 = x - r
        y0 = y - r
        x1 = x + r
        y1 = y + r
        return canvas.create_oval(x0, y0, x1, y1, fill='red')


    # Number of waypoints

    Nw = 3

    # canvas dimensions

    cw = 600
    ch = 600


    def myfunction(event):



        root.counter += 1
        x, y = event.x, event.y

        if (root.old_coords and root.counter <= Nw):
            x1, y1 = root.old_coords
            canvas.create_line(x, y, x1, y1)

        if root.counter <= Nw:
            root.old_coords = x, y
            coord.append([x, y])
            create_circle(x, y, 10, canvas)

        if root.counter == Nw:
            print('maximum number of points that can be specified has reached thank you!!')

        return coord


    frame_a = Frame()
    Slider = [None] * Nw
    ang = []
    global gui_variable
    gui_variable = [None] * Nw

    for i in range(Nw):
        gui_variable[i] = IntVar()
        Slider[i] = Scale(master=frame_a, from_=0, to=360, length=600, tickinterval=30,
                          variable=gui_variable[i], orient=HORIZONTAL)
        Slider[i].set(0)
        Slider[i].grid(row=i + 1, column=1)

    frame_b = Frame()
    canvas = Canvas(master=frame_b, width=cw, height=ch, bg='grey')
    canvas.create_line(300-20, 300, 300+20,300 )
    canvas.create_line(300 , 300-20, 300, 300+20)
    canvas.bind("<Button-1>", myfunction)
    canvas.pack()

    frame_c=Frame()
    lb = Label(master=frame_c,text="select 4 angles using sliders",fg="red",bg="white",width=100,height=3)

    lb.pack()

    frame_d=Frame()
    lb2 = Label(master=frame_d,text="select 4 points by clicking on grey canvas",fg="green",bg="white",width=100,height=3)
    lb2.pack()

    # Swap the order of `frame_a` and `frame_b`
    frame_d.pack(side='top')
    frame_b.pack()
    frame_a.pack()
    frame_c.pack(side='top')

    root.old_coords = 0

    root.mainloop()

    # degrees to rad conversion

    for i in range(Nw):
        ang_deg = gui_variable[i].get()
        ang_rad = ang_deg * (pi / 180)
        ang.append(ang_rad)

    # mapping from local to world co-ordinates

    # moving the tp left origin canvas co-ordinates to bottom left cs

    # coord=list(coord)
    # print(coord)

    for i in range(Nw):
        coord[i][1] = ch - coord[i][1]
        coord[i][0] = (coord[i][0] - cw / 2) * (15 / cw)
        coord[i][1] = (coord[i][1] - ch / 2) * (15 / ch)


    #call eagle here and extract location

    # points=[[coord[0][0],coord[0][1],ang[0]],[coord[1][0],coord[1][1],ang[1]],[coord[2][0],coord[2][1],ang[2]],[coord[3][0],coord[3][1],ang[3]]]
    coord_1 = [[init_loc[0], init_loc[1], init_loc[2]], [coord[0][0], coord[0][1], ang[0]],
                  [coord[1][0], coord[1][1], ang[1]], [coord[2][0], coord[2][1], ang[2]]]
    print(coord_1,'coooooooooooooooooooords')

    # visualization

    # polygon dimensions

    wt = 1
    ht = 0.3

    points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
    x_r = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')
    x_t = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')

    xl=5
    yl=5

    ax=plt.axes(xlim=(-xl, +xl), ylim=(-yl, yl), aspect='equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_title("Visualize the orientation and position of robot at 4 points")

    for i in range(Nw+1):

        for j in range(len(points)):
            x_r[j, 0] = points[j, 0] * np.cos(coord_1[i][2]) - points[j, 1] * np.sin(coord_1[i][2])
            x_r[j, 1] = points[j, 0] * np.sin(coord_1[i][2]) + points[j, 1] * np.cos(coord_1[i][2])
            x_t[j, :] = x_r[j, 0] + coord_1[i][0], x_r[j, 1] + coord_1[i][1]

        xx = x_t[:, 0]
        xx = np.concatenate((xx, np.array([x_t[0, 0]])))
        yy = x_t[:, 1]
        yy = np.concatenate((yy, np.array([x_t[0, 1]])))

        ax.plot(xx, yy)

    plt.show()

    print("the waypoints selected are: {}".format(coord))
    print("the angles selected are: {}".format(ang))

    # run eagle here and get the lacation to be the first point
    points=coord_1

    #points=[[0,0,0],[0.1,0.1,0.3],[0.1,0.2,0.4],[0.2,0.2,0]]

    Lr=0.5
    Lf=0.5

    def steering_angle(thetas,vxs,vys,omegas):
        # Proportional control inputs are calculated
        Lf = 0.5
        Lr = 0.5

        numr = (vys - (omegas * Lr * cos(thetas)))
        denr = (vxs + (omegas * Lr * sin(thetas)))
        numf = (vys + (omegas * Lf * cos(thetas)))
        denf = (vxs - (omegas * Lf * sin(thetas)))

        deltar = arctan2(numr, denr) - thetas

        deltaf = arctan2(numf, denf) - thetas

        return deltaf, deltar


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

        omega = stage.control()
        Vx = stage.control()
        Vy = stage.control()

        stage.set_der(x, Vx)
        stage.set_der(y, Vy)
        stage.set_der(theta, omega)

        vmax=0.0123
        Lr=0.5
        Lf=0.5

        Vrs = (Vx ** 2 + Vy ** 2 + (omega * Lr) ** 2 + ( 2 * omega * Lr * (Vx * sin(theta) - Vy * cos(theta))))

        Vfs = (Vx ** 2 + Vy ** 2 + (omega * Lf) ** 2 + (2 * omega * Lf * (Vy * cos(theta) - Vx * sin(theta))))

        stage.subject_to(0 <= (Vrs <= vmax ** 2))
        stage.subject_to(0 <= (Vfs <= vmax ** 2))

        stage.subject_to(-7.5 <= (x <= 7.5))
        stage.subject_to(-7.5 <= (y <= 7.5))

        # Minimal time
        stage.add_objective(stage.T)
        stage.add_objective(2*stage.integral(Vx ** 2+Vy**2))
        stage.add_objective(10 * stage.integral(omega ** 2))
        #stage.add_objective(stage.integral(x ** 2))

        stage.method(MultipleShooting(N=N-1, M=M, intg='rk'))


        return stage,x,y,theta,Vx,Vy,omega


    ocp = Ocp()

    d1 = sqrt((points[1][0] - points[0][0]) ** 2 + (points[1][1] - points[0][1]) ** 2)
    d2 = sqrt((points[2][0] - points[1][0]) ** 2 + (points[2][1] - points[1][1]) ** 2)
    d3 = sqrt((points[3][0] - points[2][0]) ** 2 + (points[3][1] - points[2][1]) ** 2)



    deg1=abs((points[1][2]-points[0][2]))
    deg2=abs((points[2][2] - points[1][2]))
    deg3=abs((points[3][2] - points[2][2]))



    N=21+round((80/22)*float(d1))
    print(N,'nnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnn')
    vmax = 0.0123
    factor1=5
    factor2=0.01
    omg_max=2*vmax
    # Shoot up the ball

    a = ((d1 / vmax)+(deg1/omg_max))+(factor1*d1)+(factor2*deg1)
    M=int(round(a/(0.5*N)))

    print(a,d1,deg1,M,'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa1')
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

    N = 21 + round((80 / 22) * float(d2))
    a = ((d2 / vmax)+(deg2/omg_max))+(factor1*d2)+(factor2*deg2)
    M = int(round(a / (0.5 * N)))
    print(a,d2,deg2,N,M,'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa2')
    # After bounce 1
    stage2, x2, y2,theta2,Vx2,Vy2,omega2 = create_ocp_stage(ocp)

    ocp.subject_to(stage2.t0 == stage1.tf)

    stage2.subject_to(stage2.at_t0(x2) == stage1.at_tf(x1))
    stage2.subject_to(stage2.at_t0(y2) == stage1.at_tf(y1))
    stage2.subject_to(stage2.at_t0(theta2) == stage1.at_tf(theta1))

    # Final constraint
    stage2.subject_to(stage2.at_tf(x2) == points[2][0])
    stage2.subject_to(stage2.at_tf(y2) == points[2][1])
    stage2.subject_to(stage2.at_tf(theta2) == points[2][2])

    N = 21 + round((80 / 22) * float(d3))

    a = ((d3 / vmax)+(deg3/omg_max))+(factor1*d3)+(factor2*deg3)
    M = int(round(a / (0.5 * N)))

    print(a,d3,deg3,N,M,'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa3')
    # After bounce 2
    stage3, x3, y3,theta3,Vx3,Vy3,omega3 = create_ocp_stage(ocp)

    ocp.subject_to(stage3.t0 == stage2.tf)

    stage3.subject_to(stage3.at_t0(x3) == stage2.at_tf(x2))
    stage3.subject_to(stage3.at_t0(y3) == stage2.at_tf(y2))
    stage3.subject_to(stage3.at_t0(theta3) == stage2.at_tf(theta2))

    # Final constraint
    stage3.subject_to(stage3.at_tf(x3) == points[3][0])
    stage3.subject_to(stage3.at_tf(y3) == points[3][1])
    stage3.subject_to(stage3.at_tf(theta3) == points[3][2])


    # Pick a solution method
    options = {'ipopt': {"max_iter": 100000}}
    options["expand"] = True
    options["print_time"] = True
    options["error_on_fail"] = True
    ocp.solver('ipopt', options)

    # Solve
    sol = ocp.solve()

    # Plot the 3 bounces
    plt.figure()
    ts11, x1_s = sol(stage1).sample(x1, grid='integrator')
    #print(len(x1_s),'lenxxxxxxxxxxxxxx')
    #print(len(ts11),'tsssssssssssssss1')
    ts1, y1_s = sol(stage1).sample(y1, grid='integrator')
    ts2, x2_s = sol(stage2).sample(x2, grid='integrator')
    ts22, y2_s = sol(stage2).sample(y2, grid='integrator')
    ts3, x3_s = sol(stage3).sample(x3, grid='integrator')
    ts33, y3_s = sol(stage3).sample(y3, grid='integrator')
    ts3, x3_s = sol(stage3).sample(x3, grid='integrator')
    ts1, theta1_s = sol(stage1).sample(theta1, grid='integrator')
    ts2, theta2_s = sol(stage2).sample(theta2, grid='integrator')
    ts3, theta3_s = sol(stage3).sample(theta3, grid='integrator')
    ts1, Vx1_s = sol(stage1).sample(Vx1, grid='integrator')
    #print(len(Vx1_s), 'tsssssssssssssss1')
    ts2, Vx2_s = sol(stage2).sample(Vx2, grid='integrator')
    ts3, Vx3_s = sol(stage3).sample(Vx3, grid='integrator')
    ts1, Vy1_s = sol(stage1).sample(Vy1, grid='integrator')
    ts2, Vy2_s = sol(stage2).sample(Vy2, grid='integrator')
    ts3, Vy3_s = sol(stage3).sample(Vy3, grid='integrator')
    ts1, omega1_s = sol(stage1).sample(omega1, grid='integrator')
    ts2, omega2_s = sol(stage2).sample(omega2, grid='integrator')
    ts3, omega3_s = sol(stage3).sample(omega3, grid='integrator')


    print(sol.value(stage1.T)+sol.value(stage2.T)+sol.value(stage3.T),'ddddddddddddddddddddddddddddddddddddd')



    deltaf1_s=np.zeros((M*(N-1),1),dtype=float)
    deltaf2_s=np.zeros((M*(N-1),1),dtype=float)
    deltaf3_s=np.zeros((M*(N-1),1),dtype=float)

    deltar1_s=np.zeros((M*(N-1),1),dtype=float)
    deltar2_s=np.zeros((M*(N-1),1),dtype=float)
    deltar3_s=np.zeros((M*(N-1),1),dtype=float)

    Vf1_s=np.zeros((M*(N-1),1),dtype=float)
    Vf2_s=np.zeros((M*(N-1),1),dtype=float)
    Vf3_s=np.zeros((M*(N-1),1),dtype=float)

    Vr1_s=np.zeros((M*(N-1),1),dtype=float)
    Vr2_s=np.zeros((M*(N-1),1),dtype=float)
    Vr3_s=np.zeros((M*(N-1),1),dtype=float)
    '''
    for k in range(M*(N-1)):
        print(k)
        deltaf1_s[k],deltar1_s[k]=steering_angle(theta1_s[k],Vx1_s[k],Vy1_s[k],omega1_s[k])

        deltaf2_s[k], deltar2_s[k] = steering_angle(theta2_s[k], Vx2_s[k], Vy2_s[k], omega2_s[k])
        deltaf3_s[k], deltar3_s[k] = steering_angle(theta3_s[k], Vx3_s[k], Vy3_s[k], omega3_s[k])

        Vr1_s[k] = sqrt((Vx1_s[k] ** 2 + Vy1_s[k] ** 2 + (omega1_s[k] * Lr) ** 2 + (2 * omega1_s[k] * Lr * (Vx1_s[k] * sin(theta1_s[k]) - Vy1_s[k] * cos(theta1_s[k])))))
        Vr2_s[k] = sqrt((Vx2_s[k] ** 2 + Vy2_s[k] ** 2 + (omega2_s[k] * Lr) ** 2 + (2 * omega2_s[k] * Lr * (Vx2_s[k] * sin(theta2_s[k]) - Vy2_s[k] * cos(theta2_s[k])))))
        Vr3_s[k] = sqrt((Vx3_s[k] ** 2 + Vy3_s[k] ** 2 + (omega3_s[k] * Lr) ** 2 + (2 * omega3_s[k] * Lr * (Vx3_s[k] * sin(theta3_s[k]) - Vy3_s[k] * cos(theta3_s[k])))))

        Vf1_s[k] = sqrt((Vx1_s[k]**2 +Vy1_s[k]**2 + (omega1_s[k]*Lf)**2 +(2 *omega1_s[k]*Lf*(Vy1_s[k] *cos(theta1_s[k]) -Vx1_s[k] *sin(theta1_s[k])))))
        Vf2_s[k] = sqrt((Vx2_s[k] ** 2 + Vy2_s[k] ** 2 + (omega2_s[k] * Lf) ** 2 + (2 * omega2_s[k] * Lf * (Vy2_s[k] * cos(theta2_s[k]) - Vx2_s[k] * sin(theta2_s[k])))))
        Vf3_s[k] = sqrt((Vx3_s[k] ** 2 + Vy3_s[k] ** 2 + (omega3_s[k] * Lf) ** 2 + (2 * omega3_s[k] * Lf * (Vy3_s[k] * cos(theta3_s[k]) - Vx3_s[k] * sin(theta3_s[k])))))
        '''
        #print(Vr1_s,Vr2_s,Vr3_s,'vvvvvvvvvvvvrrrrrrrrrrrrrrrrrrrrrrrrr')

    plt.plot(x1_s, y1_s)
    plt.plot(x2_s, y2_s)
    plt.plot(x3_s, y3_s)


    plt.show(block=True)


    t_s=np.concatenate((ts11,ts22,ts33),axis=None)

    diff_t=np.diff(t_s)
    print(diff_t,'difffffffffffffffffffffffffffffffffffffffffffff')

    x_d = np.concatenate((x1_s,x2_s,x3_s),axis=None)
    y_d = np.concatenate((y1_s,y2_s,y3_s),axis=None)
    theta_d = np.concatenate((theta1_s,theta2_s,theta3_s),axis=None)

    deltaf_d = np.concatenate((deltaf1_s,deltaf2_s,deltaf3_s),axis=None)
    deltar_d = np.concatenate((deltar1_s,deltar2_s,deltar3_s),axis=None)

    #Vf = np.concatenate((Vf1_s,Vf2_s,Vf3_s),axis=None)
    #Vr = np.concatenate((Vr1_s,Vr2_s,Vr3_s),axis=None)

    Vx_i = np.concatenate((Vx1_s,Vx2_s,Vx3_s),axis=None)
    Vy_i = np.concatenate((Vy1_s,Vy2_s,Vy3_s),axis=None)
    omega_i=np.concatenate((omega1_s,omega2_s,omega3_s),axis=None)


    #with np.printoptions(edgeitems=10000):
     #print(len(diff_t),'ts11111111111111111111111111111')

    points=[ [0]*3 for i in range(len(x_d))]
    for i in range(len(x_d)):
     points[i][0]=x_d[i]
     points[i][1]=y_d[i]
     points[i][2]=theta_d[i]

    return points,Vx_i,Vy_i,omega_i,diff_t

if __name__ == '__main__':


    gui_rock()