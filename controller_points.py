import control as ct
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
from numpy import pi, cos, sin, tan,arctan2,arctan,sqrt
from matplotlib import animation
from tkinter import *
import time as time

import scipy.linalg as linalg
import math
import pdb

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
canvas = Canvas(master=frame_b, width=cw, height=ch, bg='grey')
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
ax.set_title("Visualize the orientation and position of robot at 4 points")

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



# simulation

# controller calculates control input

def controller(ref, x, y, theta):
    # Proportional control inputs are calculated
    Lf = 0.5
    Lr = 0.5
    vmax = 0.0123
    delt_max = pi / 2
    saturation = 1

    p_err = 3 * [0]
    p_err[0] = ref[0] - x
    p_err[1] = ref[1] - y
    p_err[2] = ref[2] - theta

    vx = k1 * p_err[0]
    vy = k2 * p_err[1]
    omega = k3 * p_err[2]

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

    if (Vr < 0 or Vf < 0):
        print('danger')
        print('danger')
        print('danger')
        print('danger')

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

    return p_err, Vf, Vr, deltaf, deltar


def sender1(Vf, Vr, deltaf, deltar):
    tick_min = 0
    tick_max = 200
    vmax = 0.0123

    slope = (tick_max - tick_min) / (2 * vmax)

    deltar_d = round(float(((deltar + pi / 2) * 180 / pi)))
    angr = int(deltar_d)
    deltaf_d = round(float(((deltaf + pi / 2) * 180 / pi)))
    angf = int(deltaf_d)
    ticksf = int(round(float(tick_min + (slope * (Vf + vmax)))))
    ticksr = int(round(float(tick_min + (slope * (Vr + vmax)))))

    values1 = (ticksf, ticksr, angf, angr)
    # print(values1)
    return values1


def update(x, y, theta, Vf, Vr, deltaf, deltar, dt):
    lr = 0.5
    lf = 0.5
    p_noise = 0
    m_noise = 0.001
    p_n = p_noise * np.random.rand(3, 1)
    m_n = m_noise * np.random.rand(3, 1)

    A = np.array([[1, 0, lr * sin(theta)], [0, 1, -lr * cos(theta)], [1, 0, -lf * sin(theta)], [0, 1, lf * cos(theta)]],
                 dtype="float")

    b = np.array(
        [Vr * cos(theta + deltar), Vr * sin(theta + deltar), Vf * cos(theta + deltaf), Vf * sin(theta + deltaf)],
        dtype="float")

    [vx, vy, omega], res, rank, sing = np.linalg.lstsq(A, b, rcond=None)
    Ag = np.concatenate((A, np.reshape(b, (A.shape[0], 1))), axis=1)
    rA = np.linalg.matrix_rank(A, tol=None, hermitian=False)

    rAg = np.linalg.matrix_rank(Ag, tol=None, hermitian=False)

    # print(Ag,A)

    x = x + (vx * dt) + p_n[0] + m_n[0]
    y = y + (vy * dt) + p_n[1] + m_n[1]
    theta = theta + (omega * dt) + p_n[2] + m_n[2]
    return x, y, theta, vx, vy, omega


# feedback gain in each direction

k1 = 0.8
k2 = 0.8
k3 = 0.8

Ts = 0.75
N = 200

animation=1

t = np.linspace(0, 200, 200 / Ts)
y_log = np.zeros((3, len(t)))

xd = 0 * np.ones((len(t), 3)).T

# initial state

#[x, y, theta] = np.array([1, 2, 0])
[x, y, theta] = np.array([points[0][0]+0.1, points[0][1]+0.1, points[0][1]])


# simulation

waypoints = np.array(points)

total_t=0
t_error=0


fig=plt.figure()
fig.set_size_inches(108.5, 10.5)
plt.plot(waypoints[:,0],waypoints[:,1],"m--")


#logging

x_log=[]
y_log=[]
theta_log=[]
deltaf_log=[]
deltar_log=[]
val_log=[]

print(np.shape(waypoints))

for j in range(0,len(points)):
    ref = waypoints[j,:]
    print(ref)
    t = 0
    #print(ref)
    p_err = 3 * [100]
    while(abs(p_err[0])>=0.01 or abs(p_err[1])>=0.01 or abs(p_err[2])>=0.01):
        # starting from initial state call controller

        #pdb.set_trace()

        x_log.append(x)
        y_log.append(y)
        theta_log.append(theta)

        [p_err,Vf,Vr,deltaf,deltar] = controller(ref, x, y, theta)



        deltaf_log.append(deltaf)
        deltar_log.append(deltar)


        #print(Vf,Vr,math.degrees(deltaf),math.degrees(deltar))

        #pdb.set_trace()
        [x, y, theta,vx,vy,omega] = update(x, y, theta,Vf, Vr, deltaf, deltar,Ts)
        #print(x,y,theta)
        values1=sender1(Vf,Vr,deltaf,deltar)
        val_log.append(values1)

        print(Vf, Vr, math.degrees(deltaf), math.degrees(deltar))
        print(values1)
        t=t+Ts

        plt.gcf().canvas.mpl_connect('key_release_event',
                                     lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(waypoints[j, 0], waypoints[j, 1], "or", label="course")
        plt.plot(x, y, "ob", label="trajectory")

        plt.axis("equal")
        plt.grid(True)
        plt.pause(0.00001)
        plt.axis('tight')
        #print(p_err)

plt.close()

if animation:

    from matplotlib import animation
    import matplotlib.patches as patches
    import matplotlib as mpl


    fig2 = plt.figure()
    fig2.set_dpi(100)
    fig2.set_size_inches(6, 5.5)

    ax = plt.axes(xlim=(-4, 4), ylim=(-4, 4))
    ax.set_aspect('equal', adjustable='box')
    
    x_d = x_log
    y_d = y_log
    theta = theta_log
    deltaf=deltaf_log
    deltar=deltar_log

    #robot dimension
    height=0.3
    width=1

    #wheel dimension
    w_h=0.04
    w_w=0.1

    vertices=(0,0)
    polygon = patches.Rectangle(vertices,width,height,color="red", alpha=0.50)
    polygon2 = patches.Rectangle(vertices,w_w,w_h,color="blue", alpha=0.50)
    polygon3 = patches.Rectangle(vertices,w_w,w_h,color="green", alpha=0.50)

    def init():

        ax.add_patch(polygon)
        ax.add_patch(polygon2)
        ax.add_patch(polygon3)
        return polygon,polygon2,polygon3

    def animate(i):

       #robot patch
        r=np.sqrt((height/2)**2+(width/2)**2)
        alpha=np.arctan(height/width)
        xc=[r*cos(alpha),r*sin(alpha)]
        xc_w1=[0,height/2]
        xc_w2=[width,height/2]
        x_t=2*[None]
        x_1=2*[None]
        x_2=2*[None]
        midPoint=[[0]*2 for j in range(len(x_d))]

        x_t[0]=xc[0]*cos(theta[i])-xc[1]*sin(theta[i])
        x_t[1]=xc[0]*sin(theta[i])+xc[1]*cos(theta[i])
        x_1[0]=xc_w1[0]*cos(theta[i])-xc_w1[1]*sin(theta[i])
        x_1[1]=xc_w1[0]*sin(theta[i])+xc_w1[1]*cos(theta[i])
        x_2[0]=xc_w2[0]*cos(theta[i])-xc_w2[1]*sin(theta[i])
        x_2[1]=xc_w2[0]*sin(theta[i])+xc_w2[1]*cos(theta[i])
        midPoint[i][:] = [x_d[i]-x_t[0],y_d[i]-x_t[1]]
        r = mpl.transforms.Affine2D().rotate(theta[i])
        r1 = mpl.transforms.Affine2D().rotate(deltar[i])
        r2 = mpl.transforms.Affine2D().rotate(deltaf[i])
        t = mpl.transforms.Affine2D().translate(midPoint[i][0],midPoint[i][1])
        t1 = mpl.transforms.Affine2D().translate(x_1[0],x_1[1])
        t2 = mpl.transforms.Affine2D().translate(x_2[0],x_2[1])

        tra = r +t+ ax.transData
        tra1 =r1+r+t+t1 + ax.transData
        tra2 =r2+r+t+t2 +ax.transData
        polygon.set_transform(tra)
        polygon2.set_transform(tra1)
        polygon3.set_transform(tra2)


        coords = polygon.get_bbox().get_points()
        #print(tra.transform(coords))

        #wheel patches

        ax.add_patch(polygon)
        ax.add_patch(polygon2)
        ax.add_patch(polygon3)
      
        return polygon,polygon2,polygon3

    anim = animation.FuncAnimation(fig2, animate,
                                   init_func=init,
                                   frames=len(x_d),
                                   interval=50,
                                   blit=True)
    #writervideo =animation.FFMpegWriter(fps=100)
    #anim.save('increasingStraightLine.mp4', writer=writervideo)
    plt.show()

    


