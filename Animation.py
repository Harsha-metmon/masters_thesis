import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import matplotlib.patches as patches
import matplotlib as mpl
from numpy import sin,cos,arctan,pi

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(6, 5.5)

ax = plt.axes(xlim=(-10, 15), ylim=(-10, 15))

x_d = np.linspace(0,10,100)
y_d = np.linspace(0,10,100)
theta = np.linspace(pi/2,0,100)
deltaf=np.linspace(pi/2,0,100)
deltar=np.linspace(0,-pi/2,100)


#robot dimension
height=2
width=6

#wheel dimension
w_h=0.2
w_w=0.6

vertices=(0,0)
vertices1=(width,height/2)
vertices2=(-w_w,height/2)
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
    
    tra = r + t + ax.transData
    tra1 =  r1+r+t+t1 + ax.transData
    tra2 = r2 + r+t+t2 +ax.transData
    polygon.set_transform(tra)
    polygon2.set_transform(tra1)
    polygon3.set_transform(tra2)
    
 
    
    coords = polygon.get_bbox().get_points()
    print(tra.transform(coords))
    
    #wheel patches
    
    
    ax.add_patch(polygon)
    ax.add_patch(polygon2)
    ax.add_patch(polygon3)
    
    return polygon,polygon2,polygon3

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=len(x_d), 
                               interval=200,
                               blit=True)
                               
                               
plt.show()


