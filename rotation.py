
'''
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.transforms import Affine2D

height = 0.1
width = 1
fig, ax = plt.subplots()
ax.set_aspect('equal')
ax.set_xlim([-width * 1.2, width * 1.2])
ax.set_ylim([-width * 1.2, width * 1.2])
ax.plot(0, 0,  color='r', marker='o', markersize=10)
#point_of_rotation = np.array([0, height/2])          # A
point_of_rotation = np.array([width/2, height/2])  # B
# point_of_rotation = np.array([width/3, height/2])  # C
# point_of_rotation = np.array([width/3, 2*height])  # D

for deg in range(0, 360, 45):
    rec = plt.Rectangle(-point_of_rotation, width=width, height=height, 
                        color=str(deg / 360), alpha=0.9,
                        transform=Affine2D().rotate_deg_around(*(0,0), deg)+ax.transData)
    ax.add_patch(rec)
plt.show()
'''

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
from math import *

#set up the plot
fig = plt.figure()
ax = fig.add_subplot(111)

vertices=(0,0)
width=6
height=2
polygon = patches.Rectangle(vertices,width,height,color="red", alpha=0.50) 

#function to rotate and translate the standard shape to a new position
def plot_polygon(x_d,theta,height,width):
    vertices=(0,0)
    r=np.sqrt((height/2)**2+(width/2)**2)
    alpha=np.arctan(height/width)
    xc=[r*cos(alpha),r*sin(alpha)]
    x_t=2*[None]
    x_t[0]=xc[0]*cos(theta)-xc[1]*sin(theta)
    x_t[1]=xc[0]*sin(theta)+xc[1]*cos(theta)
    midPoint = [x_d[0]-x_t[0],x_d[1]-x_t[1]]
    r = mpl.transforms.Affine2D().rotate(theta)
    t = mpl.transforms.Affine2D().translate(midPoint[0],midPoint[1])
    tra = r + t + ax.transData
    polygon.set_transform(tra)
    coords = polygon.get_patch_transform().transform(polygon.get_path().vertices[:-1])
    tc=tra.transform(coords)
    inv = ax.transData.inverted()
    corner_p=inv.transform((tc))
    print(corner_p)

    ax.add_patch(polygon)
    
    #print(pp)
    
plot_polygon([3,4],0,2,6)


plt.xlim(-5, 10)
plt.ylim(-5, 10)

plt.grid(True)
plt.show()
