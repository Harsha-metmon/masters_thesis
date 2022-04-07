#
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
#
#

"""
Motion planning
===============

Simple motion planning with circular obstacle
"""

from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan,arctan2,arctan,sqrt
from casadi import vertcat, sumsqr
from rockit import Ocp, DirectMethod, MultipleShooting, FreeTime
from matplotlib import animation
from tkinter import *
import time as time

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

N = 4

# canvas dimensions

cw = 600
ch = 600


def myfunction(event):
    root.counter += 1
    x, y = event.x, event.y

    if (root.old_coords and root.counter <= N):
        x1, y1 = root.old_coords
        canvas.create_line(x, y, x1, y1)

    if root.counter <= N:
        root.old_coords = x, y
        coord.append([x, y])
        create_circle(x, y, 10, canvas)

    if root.counter == N:
        print('maximum number of points that can be specified has reached thank you!!')

    return coord


frame_a = Frame()
Slider = [None] * N
ang = []
global gui_variable
gui_variable = [None] * N

for i in range(N):
    gui_variable[i] = IntVar()
    Slider[i] = Scale(master=frame_a, from_=0, to=360, length=600, tickinterval=30,
                      variable=gui_variable[i], orient=HORIZONTAL)
    Slider[i].set(0)
    Slider[i].grid(row=i + 1, column=1)

frame_b = Frame()
canvas = Canvas(master=frame_b, width=cw, height=ch, bg='white')
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

for i in range(N):
    ang_deg = gui_variable[i].get()
    ang_rad = ang_deg * (pi / 180)
    ang.append(ang_rad)

# mapping from local to world co-ordinates

# moving the tp left origin canvas co-ordinates to bottom left cs

# coord=list(coord)
# print(coord)

for i in range(N):
    coord[i][1] = ch - coord[i][1]
    coord[i][0] = (coord[i][0] - cw / 2) * (15 / cw)
    coord[i][1] = (coord[i][1] - ch / 2) * (15 / ch)

# visualization

# polygon dimensions

wt = 1
ht = 0.3


points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
x_r = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')
x_t = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')

xl=7.5
yl=7.5

ax=plt.axes(xlim=(-xl, +xl), ylim=(-yl, yl), aspect='equal')
ax.set_xlabel('x(m)')
ax.set_ylabel('y(m)')
ax.set_title("robot position")

for i in range(N):

    for j in range(len(points)):
        x_r[j, 0] = points[j, 0] * np.cos(ang[i]) - points[j, 1] * np.sin(ang[i])
        x_r[j, 1] = points[j, 0] * np.sin(ang[i]) + points[j, 1] * np.cos(ang[i])
        x_t[j, :] = x_r[j, 0] + coord[i][0], x_r[j, 1] + coord[i][1]

    xx = x_t[:, 0]
    xx = np.concatenate((xx, np.array([x_t[0, 0]])))
    yy = x_t[:, 1]
    yy = np.concatenate((yy, np.array([x_t[0, 1]])))

    ax.plot(xx, yy)

plt.show()

print("the waypoints selected are: {}".format(coord))
print("the angles selected are: {}".format(ang))


points=[[coord[0][0],coord[0][1],ang[0]],[coord[1][0],coord[1][1],ang[1]],[coord[2][0],coord[2][1],ang[2]],[coord[3][0],coord[3][1],ang[3]]]

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
    stage = ocp.stage(t0=FreeTime(0), T=FreeTime(10))
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
    #stage.add_objective(stage.integral(x ** 2))

    stage.method(MultipleShooting(N=20, M=2, intg='rk'))


    return stage,x,y,theta,Vx,Vy,omega


ocp = Ocp()


M=2
N=21

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


# After bounce 1
stage2, x2, y2,theta2,Vx2,Vy2,omega2 = create_ocp_stage(ocp)
stage2.subject_to(stage2.at_t0(x2) == stage1.at_tf(x1))
stage2.subject_to(stage2.at_t0(y2) == stage1.at_tf(y1))
stage2.subject_to(stage2.at_t0(theta2) == stage1.at_tf(theta1))

# Final constraint
stage2.subject_to(stage2.at_tf(x2) == points[2][0])
stage2.subject_to(stage2.at_tf(y2) == points[2][1])
stage2.subject_to(stage2.at_tf(theta2) == points[2][2])



# After bounce 2
stage3, x3, y3,theta3,Vx3,Vy3,omega3 = create_ocp_stage(ocp)
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
ts1, x1_s = sol(stage1).sample(x1, grid='integrator')
ts1, y1_s = sol(stage1).sample(y1, grid='integrator')
ts2, x2_s = sol(stage2).sample(x2, grid='integrator')
ts2, y2_s = sol(stage2).sample(y2, grid='integrator')
ts3, x3_s = sol(stage3).sample(x3, grid='integrator')
ts3, y3_s = sol(stage3).sample(y3, grid='integrator')
ts3, x3_s = sol(stage3).sample(x3, grid='integrator')
ts1, theta1_s = sol(stage1).sample(theta1, grid='integrator')
ts2, theta2_s = sol(stage2).sample(theta2, grid='integrator')
ts3, theta3_s = sol(stage3).sample(theta3, grid='integrator')
ts1, Vx1_s = sol(stage1).sample(Vx1, grid='integrator')
ts2, Vx2_s = sol(stage2).sample(Vx2, grid='integrator')
ts3, Vx3_s = sol(stage3).sample(Vx3, grid='integrator')
ts1, Vy1_s = sol(stage1).sample(Vy1, grid='integrator')
ts2, Vy2_s = sol(stage2).sample(Vy2, grid='integrator')
ts3, Vy3_s = sol(stage3).sample(Vy3, grid='integrator')
ts1, omega1_s = sol(stage1).sample(omega1, grid='integrator')
ts2, omega2_s = sol(stage2).sample(omega2, grid='integrator')
ts3, omega3_s = sol(stage3).sample(omega3, grid='integrator')

#

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

for k in range(M*(N-1)):

    deltaf1_s[k],deltar1_s[k]=steering_angle(theta1_s[k],Vx1_s[k],Vy1_s[k],omega1_s[k])
    deltaf2_s[k], deltar2_s[k] = steering_angle(theta2_s[k], Vx2_s[k], Vy2_s[k], omega2_s[k])
    deltaf3_s[k], deltar3_s[k] = steering_angle(theta3_s[k], Vx3_s[k], Vy3_s[k], omega3_s[k])

    Vr1_s[k] = sqrt((Vx1_s[k] ** 2 + Vy1_s[k] ** 2 + (omega1_s[k] * Lr) ** 2 + (2 * omega1_s[k] * Lr * (Vx1_s[k] * sin(theta1_s[k]) - Vy1_s[k] * cos(theta1_s[k])))))
    Vr2_s[k] = sqrt((Vx2_s[k] ** 2 + Vy2_s[k] ** 2 + (omega2_s[k] * Lr) ** 2 + (2 * omega2_s[k] * Lr * (Vx2_s[k] * sin(theta2_s[k]) - Vy2_s[k] * cos(theta2_s[k])))))
    Vr3_s[k] = sqrt((Vx3_s[k] ** 2 + Vy3_s[k] ** 2 + (omega3_s[k] * Lr) ** 2 + (2 * omega3_s[k] * Lr * (Vx3_s[k] * sin(theta3_s[k]) - Vy3_s[k] * cos(theta3_s[k])))))

    Vf1_s[k] = sqrt((Vx1_s[k]**2 +Vy1_s[k]**2 + (omega1_s[k]*Lf)**2 +(2 *omega1_s[k]*Lf*(Vy1_s[k] *cos(theta1_s[k]) -Vx1_s[k] *sin(theta1_s[k])))))
    Vf2_s[k] = sqrt((Vx2_s[k] ** 2 + Vy2_s[k] ** 2 + (omega2_s[k] * Lf) ** 2 + (2 * omega2_s[k] * Lf * (Vy2_s[k] * cos(theta2_s[k]) - Vx2_s[k] * sin(theta2_s[k])))))
    Vf3_s[k] = sqrt((Vx3_s[k] ** 2 + Vy3_s[k] ** 2 + (omega3_s[k] * Lf) ** 2 + (2 * omega3_s[k] * Lf * (Vy3_s[k] * cos(theta3_s[k]) - Vx3_s[k] * sin(theta3_s[k])))))


plt.plot(x1_s, y1_s)
plt.plot(x2_s, y2_s)
plt.plot(x3_s, y3_s)


plt.show(block=True)

#Animation

# Animation

x_d = np.concatenate((x1_s,x2_s,x3_s),axis=None)
y_d = np.concatenate((y1_s,y2_s,y3_s),axis=None)
theta = np.concatenate((theta1_s,theta2_s,theta3_s),axis=None)

deltaf = np.concatenate((deltaf1_s,deltaf2_s,deltaf3_s),axis=None)
deltar = np.concatenate((deltar1_s,deltar2_s,deltar3_s),axis=None)

Vf = np.concatenate((Vf1_s,Vf2_s,Vf3_s),axis=None)
Vr = np.concatenate((Vr1_s,Vr2_s,Vr3_s),axis=None)

print(np.shape(theta))
print(np.shape(deltaf))
print(np.shape(deltar))

Nr = 1

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
#xl = np.abs(x_d).max() + 1.5
#yl = np.abs(y_d).max() + 1.5


ax = plt.axes(xlim=(-xl, +xl), ylim=(-yl, yl), aspect='equal')
ax.set_xlabel('x(m)')
ax.set_ylabel('y(m)')
ax.set_title("robot position")

line = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for robots
linewf = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for front wheels
linewr = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for rear wheels
line_ex = [ax.plot([], [], lw=1, markersize=0.1)[0] for _ in
           range(Nr)]  # lines to animate for path followed we only have to plot (x_d[ri,ki]) points
line_1 = [ax.scatter([], [], s=500, c='blue', marker='o') for _ in range(Nr)]  # initial position
line_2 = [ax.scatter([], [], s=500, c='blue', marker='1') for _ in range(Nr)]  # initial position
line_3 = [ax.scatter([], [], s=500, c='blue', marker='+') for _ in range(Nr)]  # initial position
line_4 = [ax.scatter([], [], s=500, c='red', marker='*') for _ in range(Nr)]  # final position

lines = line + linewf + linewr + line_ex


# initialization function: plot the background of each frame
def init():
    for l in lines:
        l.set_data([], [])
    for l in line_1:
        l.set_offsets([])
    for l in line_2:
        l.set_offsets([])
    for l in line_3:
        l.set_offsets([])
    for l in line_4:
        l.set_offsets([])
    return lines, line_1, line_2,line_3,line_4


# pdb.set_trace()

# animation function.  This is called sequentially
def animate(i):
    # Dynamic objects

    """perform animation step"""
    # draw ith rectangle
    points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
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
            x_r[k, j, 0] = points[j, 0] * np.cos(theta[i]) - points[j, 1] * np.sin(theta[i])
            x_r[k, j, 1] = points[j, 0] * np.sin(theta[i]) + points[j, 1] * np.cos(theta[i])
            x_t[k, j, :] = x_r[k, j, 0] + x_d[i], x_r[k, j, 1] + y_d[i]

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
            x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[i] + deltaf[i]) - pointsw[j, 1] * np.sin(
                theta[i] + deltaf[i])
            x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[i] + deltaf[i]) + pointsw[j, 1] * np.cos(
                theta[i] + deltaf[i])

            # translation
            tx = wt / 2 * cos(theta[i])
            ty = wt / 2 * sin(theta[i])
            x_tw[k, j, :] = x_rw[k, j, 0] + x_d[i] + tx, x_rw[k, j, 1] + y_d[i] + ty

        xxw = x_tw[k, :, 0]
        xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
        yyw = x_tw[k, :, 1]
        yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

        ln.set_data(xxw, yyw)

    for k, ln in enumerate(linewr):
        # xxw = [[0] * 5] * Nr
        # yyw = [[0] * 5] * Nr
        for j in range(len(pointsw)):
            x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[i] + deltar[i]) - pointsw[j, 1] * np.sin(
                theta[i] + deltar[i])
            x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[i] + deltar[i]) + pointsw[j, 1] * np.cos(
                theta[i] + deltar[i])

            # translation
            tx = wt / 2 * cos(theta[i])
            ty = wt / 2 * sin(theta[i])
            x_tw[k, j, :] = x_rw[k, j, 0] + x_d[i] - tx, x_rw[k, j, 1] + y_d[i] - ty

        xxw = x_tw[k, :, 0]
        xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
        yyw = x_tw[k, :, 1]
        yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

        ln.set_data(xxw, yyw)

    for k, ln in enumerate(line_ex):
        ln.set_data(x_d[:i], y_d[:i])

    for k, ln in enumerate(line_1):
        ln.set_offsets([x_d[0], y_d[0]])

    for k, ln in enumerate(line_2):
        ln.set_offsets([x2_s[0], y2_s[0]])

    for k, ln in enumerate(line_3):
        ln.set_offsets([x3_s[0], y3_s[0]])

    for k, ln in enumerate(line_4):
        ln.set_offsets([x_d[-1], y_d[-1]])

    return lines, line_1, line_2,line_3,line_4


anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=3*M*(N-1), interval=200, blit=False)

plt.show()