import math
import yaml
import time
import queue

import numpy as np
import matplotlib.pyplot as plt

from rockit import *
from numpy import pi, cos, sin, tan, arctan2, arctan, sqrt

from matplotlib import animation
from tkinter import *
from functools import partial
import pdb as pdb


# GUI 1
# This is done first and then another GUI opens for choreography design

#THIS IS THE FIRST GUI THAT OPENS AND TAKES IN THE PARAMETER VALUSE AND THEN CLOSES#

def creating():
    root.destroy()

# Creating GUI window
root = Tk()
root.title("Define parameters")

# Creating mainframe to store widgets
mainframe = Frame(root, width = 700, height = 500)
mainframe.grid()
mainframe.grid_propagate(1)

# Defining labels for length and width
Label(mainframe, text = "Length of room:").grid(row=1, column = 1)
Label(mainframe, text = "Width of room:").grid(row=3, column = 1)
Label(mainframe, text = "Number of robots:").grid(row=5, column = 1)
# Defining variables for storing values from GUI
gui_variable1=IntVar()
gui_variable2=IntVar()
gui_variable3=IntVar()

# Creating two sliders to get user defined length and width
Slider1 = Scale(mainframe, from_=1, to=15, length=600,tickinterval=1,variable=gui_variable1, orient=HORIZONTAL)
Slider1.set(1)
Slider1.grid(row =1, column = 2)
Slider2 = Scale(mainframe, from_=1, to=15, length=600,tickinterval=1,variable=gui_variable2, orient=HORIZONTAL)
Slider2.set(1)
Slider2.grid(row = 3, column=2)
Slider3 = Scale(mainframe, from_=1, to=4, length=600,tickinterval=1,variable=gui_variable3, orient=HORIZONTAL)
Slider3.set(1)
Slider3.grid(row = 5, column=2)

# Executing button that closes GUI and continues with program
Button(mainframe, text='select number of waypoints per robot', command=creating).grid(row = 6,columnspan = 3)

# Waiting for button press
root.mainloop()

# Extracting values from sliders
slider_value1= gui_variable1.get()
slider_value2= gui_variable2.get()
slider_value3= gui_variable3.get()

## GUI 2.1 for number of waypoints per robot

# Creating GUI window
root = Tk()
root.title("Define number of waypoints per robot in order")

# Creating mainframe to store widgets
mainframe = Frame(root, width = 1000, height = 1000)
mainframe.grid()
mainframe.grid_propagate(1)

# Defining labels for length and width
Label(mainframe, text = "Define number of waypoints per robot in order:").grid(row=0, column = 0)

# Executing button that closes GUI and continues with program
Button(mainframe, text='Design choregraphy', command=creating).grid(row = 6,columnspan = 3)


gui_var=slider_value3*[None]
Slider=slider_value3*[None]
for i in range(slider_value3):
    gui_var[i] = IntVar()
    Slider[i] = Scale(master=mainframe, from_=2, to=4, length=300, tickinterval=1,
                      variable=gui_var[i], orient=HORIZONTAL)
    Slider[i].set(0)
    Slider[i].grid(row=i + 1, column=1)

root.mainloop()

slider_val=slider_value3*[None]

for i in range(slider_value3):
    slider_val[i] = gui_var[i].get()


# Exiting GUI mainloop, and continuing with the program
def creating():
    canvas.destroy()


root = Tk()



root.counter = 0


def create_rect(x, y, angle, canvas, tag,fil):  # center coordinates, radius

    if fil==0:
        fill='red'

    elif(fil==1):
        fill='blue'

    elif(fil==2):
        fill='black'

    elif(fil==3):
        fill='yellow'

    else:
        fill='white'


    cw = 600
    ch = 600

    xl = slider_value1
    yl = slider_value2

    wt = (cw/xl)*0.01
    ht = (ch/yl)*0.003

    # forward transformation to world co-ordinates here

    y = ch - y
    x = (x - cw / 2) * (xl / cw)
    y = (y - ch / 2) * (yl / ch)

    angle = (pi / 180) * angle

    points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])

    x_r = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')
    x_t = np.array([[0 for m in range(2)] for n in range(4)], dtype='float')

    for j in range(len(points)):
        x_r[j, 0] = points[j, 0] * np.cos(angle) - points[j, 1] * np.sin(angle)
        x_r[j, 1] = points[j, 0] * np.sin(angle) + points[j, 1] * np.cos(angle)
        x_t[j, :] = x_r[j, 0] + x, x_r[j, 1] + y

    mm = x_t

    for i in mm:
        i[0] = (cw/xl) * (i[0] + 0.5*xl)
        i[1] = (ch/yl) * (yl-(i[1] + 0.5*yl))

    x1, y1 = mm[0]
    x2, y2 = mm[1]
    x3, y3 = mm[2]
    x4, y4 = mm[3]
    print(mm)
    return canvas.create_polygon(x1, y1, x2, y2, x3, y3, x4, y4, tags=tag, fill=fill)


# Number of waypoints

Nw = slider_val

#Number of robots

Nr=slider_value3

ang_l=[[[] for j in range(Nw[i])] for i in range(Nr)]
print(ang_l,'angllllllllllllllllllllllllllllllllllllllllllll')

coord=[[] for i in range(Nr)]

# canvas dimensions

cw = 600
ch = 600

t_g = [['rect1', 'rect2', 'rect3', 'rect4'],['rect12', 'rect22', 'rect32', 'rect42'],['rect13', 'rect23', 'rect33', 'rect43'],['rect14', 'rect24', 'rect34', 'rect44']]

global m
m=0

global n
n=0

def myfunction(event):

    if root.counter < sum(Nw):
        root.counter += 1
        x,y = event.x,event.y
        global m

        coord[m].append([x, y])
        lp_count = len(coord[m])
        create_rect(x, y, 0, canvas, t_g[m][lp_count-1], m)

        global n

        n += 1
        if (n==Nw[m] and root.counter<(sum(Nw))):
            m+=1
            n=0

    if root.counter == sum(Nw):
        print('maximum number of points that can be specified has reached thank you!!')

    return coord


def get_slider(event,l, k):
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
                      variable=gui_variable[j][i], orient=HORIZONTAL, command=partial(get_slider,l=j ,k=i))
    Slider[j][i].set(0)
    Slider[j][i].grid(row=i + 1, column=j+1)


for j in range(Nr):
    for i in range(Nw[j]):
        if not ang[j][i]:
            ang[j][i]=0


frame_b = Frame()
canvas = Canvas(master=frame_b, width=cw, height=ch, bg='grey')
canvas.create_line(cw/2 - 20, cw/2, cw/2 + 20, cw/2)
canvas.create_line(ch/2, ch/2 - 20, ch/2, ch /2+ 20)
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

## Writing data to yaml

# format==> we can write a dict ==> number of robots, dimensions of warehouse, pos and orientation of waypoints for each of the robot (a variable).
dictionary = {'room_width': slider_value1, 'room_height': slider_value2, 'number_robots': slider_value3}
dicts = []
angs=[]
keys = range(len(coord))
#print(coord),'coooooooooooooord')

for i in keys:
        dicts.append(coord[i])
        angs.append(ang[i])
data={'choreography 1': [{'Number of robots': slider_value3,
           'room length and width': [slider_value1,slider_value2]},
          {'robot locations': dicts,'robot angles':angs}]}


with open('users.yaml', 'w') as f:
    #data = yaml.dump(dictionary, f)
    data2=yaml.dump(data,f)
#print(yaml.dump(dictionary))
print(yaml.dump(data))





