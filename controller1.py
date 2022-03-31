import control as ct
import numpy as np
import matplotlib.pyplot as plt
from numpy import pi,arctan2,sin,cos,arctan,sqrt
import scipy as sp

import scipy.linalg as linalg
import math
import pdb

# simulation

# controller calculates control input

def controller(ref, x, y, theta):
    # Proportional control inputs are calculated
    Lf=0.5
    Lr=0.5
    vmax=0.0123
    delt_max=pi/2
    saturation=1


    p_err = 3 * [0]
    p_err[0] = ref[0] - x
    p_err[1] = ref[1] - y
    p_err[2] = ref[2] - theta

    vx = k1 * p_err[0]
    vy = k2 * p_err[1]
    omega = k3 * p_err[2]

    numr=(+vy - (omega * Lr * cos(theta)))
    denr=(+vx + (omega * Lr * sin(theta)))
    numf=(+vy + (omega * Lf * cos(theta)))
    denf=(+vx - (omega * Lf * sin(theta)))

    deltar = arctan2(numr,denr)-theta

    deltaf = arctan2(numf,denf)-theta

    #Vr = (vx + (omega * Lr * sin(theta))) / (cos(deltar + theta))
    Vr=sqrt(vx**2+vy**2+(omega*Lr)**2+(2*omega*Lr*(vx*sin(theta)-vy*cos(theta))))
    #Vf = (vx - (omega * Lf * sin(theta))) / (cos(deltaf + theta))
    Vf = sqrt(vx ** 2 + vy ** 2 + (omega * Lf) ** 2 + (2 * omega * Lf*(vy * cos(theta) - vx * sin(theta))))

    if (Vr<0 or Vf<0):
       print('danger')  
       print('danger') 
       print('danger') 
       print('danger') 
    
     
    if saturation:
   
     f=max(abs(Vr)/vmax,abs(Vf)/vmax)
     
     if(f>=1):
        
        Vr=Vr/f
        Vf=Vf/f   
    
    return p_err,Vf,Vr,deltaf,deltar


def update(x, y, theta, Vf, Vr, deltaf, deltar,dt):
    lr = 0.5
    lf = 0.5
    p_noise=0
    m_noise=0
    p_n=p_noise * np.random.rand(3,1)
    m_n = m_noise * np.random.rand(3,1)

    A = np.array([[1, 0, lr * sin(theta)], [0, 1, -lr * cos(theta)], [1, 0, -lf * sin(theta)], [0, 1, lf * cos(theta)]],dtype="float")

    b = np.array(
        [Vr * cos(theta + deltar), Vr * sin(theta + deltar), Vf * cos(theta + deltaf), Vf * sin(theta + deltaf)],dtype="float")

    [vx,vy,omega], res, rank, sing = np.linalg.lstsq(A, b, rcond=None)
    Ag = np.concatenate((A,np.reshape(b,(A.shape[0],1))),axis=1)
    rA=np.linalg.matrix_rank(A, tol=None, hermitian=False)

    rAg=np.linalg.matrix_rank(Ag, tol=None, hermitian=False)

    #print(Ag,A)

    x = x + (vx * dt)+p_n[0]+m_n[0]
    y = y + (vy * dt)+p_n[1]+m_n[1]
    theta = theta + (omega * dt)+p_n[2]+m_n[2]
    return x, y, theta,vx,vy,omega

# feedback gain in each direction

k1 = 0.5
k2 = 0.5
k3 = 0.5

Ts = 0.75
N = 200

animation=0

t = np.linspace(0, 200, 200 / Ts)
y_log = np.zeros((3, len(t)))

xd = 0 * np.ones((len(t), 3)).T

# initial state

[x, y, theta] = np.array([1, 2, 0])
# simulation

ref = [1.8,2.5,-pi/2]

fig=plt.figure()
fig.set_size_inches(108.5, 10.5)
plt.plot(ref[0], ref[1], "or", label="course")


t = 0
p_err = 3 * [100]

#logging

x_log=[]
y_log=[]
theta_log=[]
deltaf_log=[]
deltar_log=[]


while(abs(p_err[0])>=0.001 or abs(p_err[1])>=0.001 or abs(p_err[2])>=0.01):
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

    t=t+Ts

    plt.gcf().canvas.mpl_connect('key_release_event',
                                 lambda event: [exit(0) if event.key == 'escape' else None])

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

    


