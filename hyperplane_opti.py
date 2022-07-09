import numpy as np
from numpy import cos,sin,tan
from scipy.optimize import LinearConstraint
from scipy.optimize import NonlinearConstraint
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import time

wt=0.90
ht=0.30

x1=1
y1=1
theta1=np.pi/4

x2=1.9

y2=1

theta2=0

s=0.01

# rectangular robots

vert1 = [-wt / 2, ht / 2]
vert2 = [-wt / 2, -ht / 2]
vert3 = [+wt / 2, -ht / 2]
vert4 = [wt/2, ht / 2]


vert1_r = [vert1[0] * cos(theta1) - vert1[1] * sin(theta1),
              vert1[0] * sin(theta1) + vert1[1] * cos(theta1)]

vert2_r = [vert2[0] * cos(theta1) - vert2[1] * sin(theta1),
              vert2[0] * sin(theta1) + vert2[1] * cos(theta1)]

vert3_r = [vert3[0] * cos(theta1) - vert3[1] * sin(theta1),
              vert3[0] * sin(theta1) + vert3[1] * cos(theta1)]

vert4_r = [vert4[0] * cos(theta1) - vert4[1] * sin(theta1),
              vert4[0] * sin(theta1) + vert4[1] * cos(theta1)]

robs = [[vert1_r[0] + x1, vert1_r[1] + y1],
              [vert2_r[0] + x1, vert2_r[1] + y1],
              [vert3_r[0] + x1, vert3_r[1] + y1],
              [vert4_r[0] + x1, vert4_r[1] + y1]]


vert1_r2 = [vert1[0] * cos(theta2) - vert1[1] * sin(theta2),
              vert1[0] * sin(theta2) + vert1[1] * cos(theta2)]

vert2_r2 = [vert2[0] * cos(theta2) - vert2[1] * sin(theta2),
              vert2[0] * sin(theta2) + vert2[1] * cos(theta2)]

vert3_r2 = [vert3[0] * cos(theta2) - vert3[1] * sin(theta2),
              vert3[0] * sin(theta2) + vert3[1] * cos(theta2)]

vert4_r2 = [vert4[0] * cos(theta2) - vert4[1] * sin(theta2),
              vert4[0] * sin(theta2) + vert4[1] * cos(theta2)]

robs2 = [[vert1_r2[0] + x2, vert1_r2[1] + y2],
              [vert2_r2[0] + x2, vert2_r2[1] + y2],
              [vert3_r2[0] + x2, vert3_r2[1] + y2],
              [vert4_r2[0] + x2, vert4_r2[1] + y2]]




for i in robs:
    co_ef=[robs[0][0],robs[0][1]]



linear_constraint = LinearConstraint([[-robs[0][0], -robs[0][1],1], [-robs[1][0], -robs[1][1],1],[-robs[2][0], -robs[2][1],1],[-robs[3][0], -robs[3][1],1],[robs2[0][0], robs2[0][1],-1],
                                     [robs2[1][0], robs2[1][1],-1],[robs2[2][0], robs2[2][1],-1],[robs2[3][0], robs2[3][1],-1]],[-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf,-np.inf ],
                                     [-s, -s,-s,-s,0,0,0,0])

def obj_f(x):

    #return (x[0]**2+x[0]**2-1)**2
    return 0


def cons_f(x):

    return [x[0]**2,x[1]**2]

def cons_J(x):
    cc=np.array([2 * x[0], 2 * x[1], 0])


    return cc.T

def cons_H(x):

    return np.array([[0, 2,0],[0, 2,0],[0,0,0]])


nonlinear_constraint = NonlinearConstraint(cons_f, -np.inf, 1)

x0 = np.array([0, 0,0])

start=time.time()

res = minimize(obj_f, x0, method='SLSQP',constraints=[linear_constraint,nonlinear_constraint], options={'ftol': 1e-9, 'disp': True})

end=time.time()-start


print(res.x,res.success,end)


ax=plt.axes(xlim=(-3,3),ylim=(-3,3),aspect='equal')


robs_a=np.array(robs)

robs_ax=np.concatenate((robs_a[:,0], np.array([robs_a[0,0]])))
robs_ay=np.concatenate((robs_a[:,1], np.array([robs_a[0,1]])))

#line.set_data(robs_ax,robs_ay)

robs2_a=np.array(robs2)

robs_ax2=np.concatenate((robs2_a[:,0], np.array([robs2_a[0,0]])))
robs_ay2=np.concatenate((robs2_a[:,1], np.array([robs2_a[0,1]])))


aa,bb,cc=res.x

ax.plot(robs_ax,robs_ay)
ax.plot(robs_ax2,robs_ay2)

xx = np.linspace(-3.5,3.5,101)
ax.plot(xx,-aa/bb*xx + cc/bb)


#plt.plot([x[0] for x in robs]+[robs[0][0]],[x[1] for x in robs]+[robs[0][1]],'r-')
plt.show()

