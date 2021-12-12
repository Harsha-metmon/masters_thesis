import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib as mpl
fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_xlim(-0.05,1);ax.set_ylim(-0.05,1);
plt.grid('on');

#Rotate rectangle patch object
ts = ax.transData
coords = ts.transform([0.2, 0.5])
tr = mpl.transforms.Affine2D().rotate_deg_around(coords[0], coords[1], 10)
t= ts + tr

rec0 = patches.Rectangle((0.2,0.5),0.25,0.2,alpha=0.5)
ax.add_patch(rec0)

#Rotated rectangle patch
rect1 = patches.Rectangle((0.2,0.5),0.25,0.2,color='blue',alpha=0.5,transform=t)
ax.add_patch(rect1);

#The (desired) point of rotation
ax.scatter([0.0,0.2],[0.0,0.5],c=['g','r'],zorder=10)
txt = ax.annotate('Desired point of rotation',xy=(0.2,0.5),fontsize=16,\
xytext=(0.25,0.35),arrowprops=dict(arrowstyle="->",connectionstyle="arc3,rad=-.2"))
txt2 = ax.annotate('Actual point of rotation',xy=(0.0,0.0),fontsize=16,\
xytext=(0.15,0.15),arrowprops=dict(arrowstyle="->",connectionstyle="arc3,rad=.2"))

plt.show()

