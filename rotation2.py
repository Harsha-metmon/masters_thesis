
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation

x = np.linspace(0,10,10)
y = np.linspace(0,10,10)
yaw = np.linspace(0,3.14,10)
fig = plt.figure()
plt.axis('equal')
plt.grid()
ax = fig.add_subplot(111)
ax.set_xlim(0, 10)
ax.set_ylim(0, 10)

patch = patches.Rectangle((0, 0), 0, 0, fc='y')

def init():
    ax.add_patch(patch)
    return patch,

def animate(i):
    patch.set_width(1.2)
    patch.set_height(1.0)
    patch.set_xy([x[i], y[i]])
    patch._angle = -np.rad2deg(yaw[i])
    return patch,

anim = animation.FuncAnimation(fig, animate,
                               init_func=init,
                               frames=len(x),
                               interval=500,
                               blit=True)
plt.show()

