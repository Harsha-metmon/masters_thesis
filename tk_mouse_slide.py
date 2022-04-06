from tkinter import *
import time as time
from numpy import pi
import numpy as np
import matplotlib.pyplot as plt

# Exiting GUI mainloop, and continuing with the program
def creating():
    canvas.destroy()

root = Tk()
coord=[]
root.counter=0


def create_circle(x, y, r, canvas): #center coordinates, radius
    x0 = x - r
    y0 = y - r
    x1 = x + r
    y1 = y + r
    return canvas.create_oval(x0, y0, x1, y1,fill='red')

# Number of waypoints

N=4

# canvas dimensions

cw=600
ch=600

def myfunction(event):
    root.counter += 1
    x, y = event.x, event.y
    
    if (root.old_coords and root.counter<=N):
        x1, y1 = root.old_coords
        canvas.create_line(x, y, x1, y1)

    if root.counter<=N:
     root.old_coords = x, y
     coord.append((x, y))
     create_circle(x, y, 10, canvas)



    if root.counter == N:
        print('maximum number of points that can be specified has reached thank you!!')
        

    return coord


frame_a = Frame()
Slider=[None]*N
ang=[]
global gui_variable
gui_variable=[None]*N

for i in range(N):


    gui_variable[i] = IntVar()
    Slider[i] = Scale(master=frame_a, from_=0, to=360, length=600, tickinterval=30,
              variable=gui_variable[i], orient=HORIZONTAL)
    Slider[i].set(0)
    Slider[i].grid(row=i + 1, column=1)


frame_b = Frame()
canvas = Canvas(master=frame_b, width=cw, height=ch,bg='white')
canvas.bind("<Button-1>", myfunction)
canvas.pack()

# Swap the order of `frame_a` and `frame_b`
frame_b.pack()
frame_a.pack()

root.old_coords = 0

root.mainloop()

# degrees to rad conversion

for i in range(N):
    ang_deg=gui_variable[i].get()
    ang_rad=ang_deg*(pi/180)
    ang.append(ang_rad)


# mapping from local to world co-ordinates

# moving the tp left origin canvas co-ordinates to bottom left cs

for i in range(N):

 coord[i][1]=ch-coord[i][1]


# visualization

#polygon dimensions

wt=60
ht=20


points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
x_r = np.array([[0 for m in range(2)] for n in range(4)] , dtype='float')
x_t = np.array([[0 for m in range(2)] for n in range(4)] , dtype='float')

ax = plt.axes(aspect='equal')
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
print("the waypoints selected are: {}".format(coord[0][1]))
print("the angles selected are: {}".format(ang))