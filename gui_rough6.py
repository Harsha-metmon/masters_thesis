import math

import time
import queue

import numpy as np
import matplotlib.pyplot as plt

from rockit import *
from numpy import pi, cos, sin, tan, arctan2, arctan, sqrt

from matplotlib import animation
from tkinter import *
from functools import partial


# GUI+ROCKIT

# GUI

# Exiting GUI mainloop, and continuing with the program
def creating():
    canvas.destroy()


root = Tk()



root.counter = 0


def create_rect(x, y, angle, canvas, tag,fil):  # center coordinates, radius

    if fil==1:
        fill='red'
    else:
        fill='white'


    cw = 600
    ch = 600

    wt = 0.1
    ht = 0.03

    L = 1
    B = 1

    # forward transformation to world co-ordinates here

    y = ch - y
    x = (x - cw / 2) * (B / cw)
    y = (y - ch / 2) * (L / ch)

    angle = (pi / 180) * angle

    points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])

    x_r = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')
    x_t = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')

    xl = 1.5
    yl = 1.5

    for j in range(len(points)):
        x_r[j, 0] = points[j, 0] * np.cos(angle) - points[j, 1] * np.sin(angle)
        x_r[j, 1] = points[j, 0] * np.sin(angle) + points[j, 1] * np.cos(angle)
        x_t[j, :] = x_r[j, 0] + x, x_r[j, 1] + y

    mm = x_t

    for i in mm:
        i[0] = 600 * (i[0] + 0.5)
        i[1] = 600 * (1 - (i[1] + 0.5))

    x1, y1 = mm[0]
    x2, y2 = mm[1]
    x3, y3 = mm[2]
    x4, y4 = mm[3]
    print(mm)
    return canvas.create_polygon(x1, y1, x2, y2, x3, y3, x4, y4, tags=tag, fill=fill)


# Number of waypoints

Nw = 3

ang_l=Nw*[None]
coord=[]

for i in range(Nw):
 ang_l[i] =[]



# canvas dimensions

cw = 600
ch = 600

t_g = ['rect1', 'rect2', 'rect3', 'rect4']


def myfunction(event):
    if root.counter < Nw:
        root.counter += 1
        x, y = event.x, event.y
        coord.append([x, y])
        lp_count=len(coord)
        create_rect(x, y, 0, canvas, t_g[lp_count],0)

    if root.counter == Nw:
        print('maximum number of points that can be specified has reached thank you!!')

    return coord


def get_slider(event,k):
    ang[k] = gui_variable[k].get()
    #kk = len(ang_l[k])
    ang_l[k].append(ang[k])
    if len(ang_l[k])>0:
      canvas.delete(t_g[k])
    #if len(ang_l[k]) > kk:
    #canvas.delete(t_g[k])
    create_rect(coord[k][-2], coord[k][-1], ang_l[k][-1], canvas, t_g[k],1)
    if k==Nw-1:
      canvas.delete(t_g[Nw])

frame_a = Frame()
Slider = [None] * Nw
ang = [None]*Nw
# global gui_variable
gui_variable = [None] * Nw

for i in range(Nw):
    gui_variable[i] = IntVar()
    Slider[i] = Scale(master=frame_a, from_=0, to=360, length=600, tickinterval=30,
                      variable=gui_variable[i], orient=HORIZONTAL, command=partial(get_slider,k=i))
    Slider[i].set(0)
    Slider[i].grid(row=i + 1, column=1)

frame_b = Frame()
canvas = Canvas(master=frame_b, width=cw, height=ch, bg='grey')
canvas.create_line(300 - 20, 300, 300 + 20, 300)
canvas.create_line(300, 300 - 20, 300, 300 + 20)
canvas.bind("<Button-1>", myfunction)
canvas.pack()

frame_c = Frame()
lb = Label(master=frame_c, text="select 4 angles using sliders", fg="red", bg="white", width=100, height=3)

lb.pack()

frame_d = Frame()
lb2 = Label(master=frame_d, text="select 4 points by clicking on grey canvas", fg="green", bg="white", width=100,
            height=3)
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

    ang[i] = ang[i] * (pi / 180)
    

print(ang,'final anglessssssssssssssssssssss')
'''

# mapping from local to world co-ordinates

# moving the tp left origin canvas co-ordinates to bottom left cs

# coord=list(coord)
# print(coord)
w_rm = 1
h_rm = 1
for i in range(Nw):
    coord[i][1] = ch - coord[i][1]
    coord[i][0] = (coord[i][0] - cw / 2) * (w_rm / cw)
    coord[i][1] = (coord[i][1] - ch / 2) * (h_rm / ch)

# call eagle here and extract location

# points=[[coord[0][0],coord[0][1],ang[0]],[coord[1][0],coord[1][1],ang[1]],[coord[2][0],coord[2][1],ang[2]],[coord[3][0],coord[3][1],ang[3]]]
coord_1 = [[coord[0][0], coord[0][1], ang[0]]]
# print(coord_1,'coooooooooooooooooooords')
# print(ang_l,'ang_lllllllllllllllllllllll')

# visualization

# polygon dimensions

wt = 1
ht = 0.3

points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
x_r = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')
x_t = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')

xl = 1.5
yl = 1.5

ax = plt.axes(xlim=(-xl, +xl), ylim=(-yl, yl), aspect='equal')
ax.set_xlabel('x(m)')
ax.set_ylabel('y(m)')
ax.set_title("Visualize the orientation and position of robot at 4 points")

for i in range(Nw):

    for j in range(len(points)):
        x_r[j, 0] = points[j, 0] * np.cos(coord_1[i][2]) - points[j, 1] * np.sin(coord_1[i][2])
        x_r[j, 1] = points[j, 0] * np.sin(coord_1[i][2]) + points[j, 1] * np.cos(coord_1[i][2])
        x_t[j, :] = x_r[j, 0] + coord_1[i][0], x_r[j, 1] + coord_1[i][1]

    xx = x_t[:, 0]
    xx = np.concatenate((xx, np.array([x_t[0, 0]])))
    yy = x_t[:, 1]
    yy = np.concatenate((yy, np.array([x_t[0, 1]])))

    ax.plot(xx, yy)
# print(x_r)

plt.show()

# print("the waypoints selected are: {}".format(coord))
# print("the angles selected are: {}".format(ang))

# run eagle here and get the lacation to be the first point



'''