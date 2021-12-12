import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation
import matplotlib.patches as patches
import matplotlib as mpl
from numpy import sin,cos,arctan,pi

fig = plt.figure()
fig.set_dpi(100)
fig.set_size_inches(7, 6.5)

ax = plt.axes(xlim=(0, 20), ylim=(0, 20))

x_d = np.linspace(0,10,100)
print(len(x_d))
y_d = np.linspace(0,10,100)
theta = np.linspace(0,pi/2,100)


height=2
width=6
vertices=(0,0)
polygon = patches.Rectangle(vertices,width,height,color="red", alpha=0.50) 

def init():
    
    ax.add_patch(polygon)
    return polygon,

def animate(i):
    
   
    r=np.sqrt((height/2)**2+(width/2)**2)
    alpha=np.arctan(height/width)
    xc=[r*cos(alpha),r*sin(alpha)]
    x_t=2*[None]
    midPoint=[[0]*2 for j in range(len(x_d))]

    x_t[0]=xc[0]*cos(theta[i])-xc[1]*sin(theta[i])
    x_t[1]=xc[0]*sin(theta[i])+xc[1]*cos(theta[i])
    midPoint[i][:] = [x_d[i]-x_t[0],y_d[i]-x_t[1]]
    r = mpl.transforms.Affine2D().rotate(theta[i])
    t = mpl.transforms.Affine2D().translate(midPoint[i][0],midPoint[i][1])
    tra = r + t + ax.transData
    polygon.set_transform(tra)
    ax.add_patch(polygon)
    
    #x = 5 + 3 * np.sin(np.radians(i))
    #y = 5 + 3 * np.cos(np.radians(i))
    #patch.center = (x, y)
    return polygon,

anim = animation.FuncAnimation(fig, animate, 
                               init_func=init, 
                               frames=len(x_d), 
                               interval=20,
                               blit=True)
                               
                               
plt.show()



'''


#function to rotate and translate the standard shape to a new position
def plot_polygon(x_d,theta,height,width):
    vertices=(0,0)
    r=np.sqrt((height/2)**2+(width/2)**2)
    alpha=np.arctan(height/width)
    xc=[r*np.cos(alpha),r*np.sin(alpha)]
    x_t=2*[None]
    x_t[0]=xc[0]*cos(theta)-xc[1]*sin(theta)
    x_t[1]=xc[0]*sin(theta)+xc[1]*cos(theta)
    midPoint = [x_d[0]-x_t[0],x_d[1]-x_t[1]]
    polygon = patches.Rectangle(vertices,width,height,color="red", alpha=0.50) 
    r = mpl.transforms.Affine2D().rotate(theta)
    t = mpl.transforms.Affine2D().translate(midPoint[0],midPoint[1])
    tra = r + t + ax.transData
    polygon.set_transform(tra)
    ax.add_patch(polygon)
    
plot_polygon([3,4],pi/6,2,6)

plt.xlim(-5, 10)
plt.ylim(-5, 10)

plt.grid(True)
plt.show()
'''
