#     This file is part of rockit.
#
#     rockit -- Rapid Optimal Control Kit
#     Copyright (C) 2019 MECO, KU Leuven. All rights reserved.
#
#     Rockit is free software; you can redistribute it and/or
#     modify it under the terms of the GNU Lesser General Public
#     License as published by the Free Software Foundation; either
#     version 3 of the License, or (at your option) any later version.
#
#     Rockit is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#     Lesser General Public License for more details.
#
#     You should have received a copy of the GNU Lesser General Public
#     License along with CasADi; if not, write to the Free Software
#     Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

"""
Motion planning
===============

Straight line along 45 degree
"""
from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan, arctan, sqrt,arctan2
from casadi import vertcat, sumsqr,DM
from matplotlib import animation
from itertools import combinations
import time
import pdb

#steering angle builder

def steering_angle(thetas,vxs,vys,omegas):
    # Proportional control inputs are calculated
    Lf = 0.5
    Lr = 0.5

    numr = (vys - (omegas * Lr * cos(thetas)))
    denr = (vxs + (omegas * Lr * sin(thetas)))
    numf = (vys + (omegas * Lf * cos(thetas)))
    denf = (vxs - (omegas * Lf * sin(thetas)))

    deltar = arctan2(numr, denr) - thetas

    deltaf = arctan2(numf, denf) - thetas

    return deltaf, deltar

# initial and final constraints

N = 21

robotN=4

# Build a plot where you can visualize the init and fin cons

Init_con = [[0, -1.5, pi/3],[1.5, 1.5, pi/2],[-1.5,1.5,0],[1.5,0,pi/2]]
Fin_con = [[0, 1.5, 0],[-1.5, -1.5, 0],[1.5,-1.5,pi/2],[-1.5,0,0]]

ocp = Ocp(T=FreeTime(10.0))

x = [None] * robotN
y = [None] * robotN
ax=[None] * robotN
ay=[None] * robotN
b=[None] * robotN
theta = [None] * robotN
omega = [None] * robotN
deltaf = [None] * robotN
deltar = [None] * robotN
in_x= [None] * robotN
in_y= [None] * robotN
in_theta= [None] * robotN

Vx = [None] * robotN
Vy = [None] * robotN
Vrs = [None] * robotN
Vfs = [None] * robotN

p = [None] * robotN
pt = [None] * robotN
ct = [None] * robotN

sol_states=np.zeros((3*robotN,N))


Lr = 0.5
Lf = 0.5
#vmax = 0.0123

#max acceleration
a_max=1
#max angular acceleration
alpha_max=1

vmax=ocp.parameter()

for i in range(robotN):
    # kinematic model

    x[i] = ocp.state()
    y[i] = ocp.state()
    theta[i] = ocp.state()

    omega[i] = ocp.control(order=1)
    Vx[i] = ocp.control(order=1)
    Vy[i] = ocp.control(order=1)

    p[i] = vertcat(x[i], y[i])
    
    pt[i] = vertcat(x[i], y[i],theta[i])
    
    ct[i]=vertcat(Vx[i],Vy[i],omega[i])
    
    ocp.set_der(x[i], Vx[i])
    ocp.set_der(y[i], Vy[i])
    ocp.set_der(theta[i], omega[i])

    # Initial constraints
    ocp.subject_to(ocp.at_t0(x[i]) == Init_con[i][0])
    ocp.subject_to(ocp.at_t0(y[i]) == Init_con[i][1])
    ocp.subject_to(ocp.at_t0(theta[i]) == Init_con[i][2])

    # Final constraint
    ocp.subject_to(ocp.at_tf(x[i]) == Fin_con[i][0])
    ocp.subject_to(ocp.at_tf(y[i]) == Fin_con[i][1])
    ocp.subject_to(ocp.at_tf(theta[i]) == Fin_con[i][2])

    Vrs[i]=(Vx[i]**2+Vy[i]**2+(omega[i]*Lr)**2+(2*omega[i]*Lr*(Vx[i]*sin(theta[i])-Vy[i]*cos(theta[i]))))

    Vfs[i]=(Vx[i]**2 +Vy[i]**2 +(omega[i]*Lf)**2 +(2 *omega[i]*Lf*(Vy[i] *cos(theta[i]) -Vx[i] *sin(theta[i]))))

    in_x[i] = np.linspace(Init_con[i][0], Fin_con[i][0], N)
    in_y[i] = np.linspace(Init_con[i][1], Fin_con[i][1], N)
    in_theta[i] = np.linspace(Init_con[i][2], Fin_con[i][2], N)

    #ocp.set_initial(x[i], in_x[i])
    #ocp.set_initial(y[i], in_y[i])
    #ocp.set_initial(theta[i], in_theta[i])

    sol_states[i,:]=in_x[i]
    sol_states[i+1,:]=in_y[i]
    sol_states[i+2,:]=in_theta[i]

    ocp.subject_to(0 <= (Vrs[i] <= vmax**2))
    ocp.subject_to(0<= (Vfs[i] <= vmax**2))

    ocp.subject_to(-7.5 <= (x[i] <= 7.5))
    ocp.subject_to(-7.5 <= (y[i] <= 7.5))

    #rate constraints

    #ocp.subject_to(-a_max <= (ocp.der(Vx[i]) <= a_max))
    #ocp.subject_to(--a_max <= (ocp.der(Vy[i]) <= a_max))
    #ocp.subject_to(-alpha_max <= (ocp.der(omega[i]) <= alpha_max))

ocp.add_objective(10* ocp.T)
#ocp.add_objective(1*ocp.integral(y[0]**2+y[1]**2)**2)

#robot dimensions for collision

r0=0.5

#Cominations of all robot pairs for collission avoidance

Number_C = int(robotN * (robotN - 1) / 2)

iterable = list(range(0, robotN))

comb = list(combinations(iterable, 2))

sft=0.1

for i in range(Number_C):

    index = list(comb[i])
    a = index[0]
    b = index[1]

    ocp.subject_to(sumsqr(p[a]-p[b])>=(2*r0+sft)**2)

# Pick a solution method
options = {'ipopt': {"max_iter": 100000}}
options["expand"] = True
options["print_time"] = True
options["error_on_fail"] = True
ocp.solver('ipopt', options)

# Make it concrete for this ocp
ocp.method(MultipleShooting(N=N-1, M=1, intg='rk'))

# solve
ocp.set_value(vmax,0.0123)

# Pick a solution method

ocp.solver('ipopt')

#this part is not scalable yet but could be made so mind you

# Translate problem to function

T = ocp.value(ocp.T)

states = ocp.sample(vertcat(pt[0],pt[1],pt[2],pt[3]),grid='control')[1]

controls = ocp.sample(vertcat(ct[0],ct[1],ct[2],ct[3]),grid='control-')[1]

test = ocp.to_function('test', [vmax, T, states, controls], [ocp.sample(vertcat(ct[0],ct[1],ct[2],ct[3]),grid='control')[0], T, states, controls])

# Initial value

sol_T = 1.0
sol_states = 0
sol_controls = 0

# Solve problem for different values for parameters, initializing with previous solution

start_time=time.time()

signal1, sol_T1, sol_states1, sol_controls1 = test(0.0123, sol_T, sol_states, sol_controls)

cur_time=time.time()

fin_time_1=(cur_time-start_time)

signal2, sol_T2, sol_states2, sol_controls2 = test(0.0123, sol_T1, sol_states1, sol_controls1)

fin_time_2=(time.time()-cur_time)

print(fin_time_1,'fintime1')
print(fin_time_2,'fintime2')

print(sol_T1,'time1')
print(sol_states1,'solstates1')
print(sol_controls1,'solcontrol1')

#plotting
t_s=np.array(signal1.T).T
x_s = sol_states2[[0,3,6,9],:]
y_s = sol_states2[[1,4,7,10],:]
theta_s = sol_states2[[2,5,8,11],:]
Vx_s=sol_controls2[[0,3,6,9],:]
Vy_s=sol_controls2[[1,4,7,10],:]
omega_s=sol_controls2[[2,5,8,11],:]

deltaf_s = DM(robotN,N)
deltar_s = DM(robotN,N)
Vr_s = DM(robotN,N)
Vf_s = DM(robotN,N)


for i in range(robotN):
    for k in range(N-1):
    
        deltaf_s[i,k],deltar_s[i,k]=steering_angle(theta_s[i,k],Vx_s[i,k],Vy_s[i,k],omega_s[i,k])

        Vr_s[i,k] = sqrt((Vx_s[i,k] ** 2 + Vy_s[i,k] ** 2 + (omega_s[i,k] * Lr) ** 2 + (2 * omega_s[i,k] * Lr * (Vx_s[i,k] * sin(theta_s[i,k]) - Vy_s[i,k] * cos(theta_s[i,k])))))

        Vf_s[i, k] = sqrt((Vx_s[i,k]**2 +Vy_s[i,k]**2 +(omega_s[i,k]*Lf)**2 +(2 *omega_s[i,k]*Lf*(Vy_s[i,k] *cos(theta_s[i,k]) -Vx_s[i,k] *sin(theta_s[i,k])))))

#plotting

#Animation

x_d=np.array(x_s)
y_d=np.array(y_s)
theta=np.array(theta_s)
deltaf=np.array(deltaf_s)
deltar=np.array(deltar_s)
Vr=np.array(Vr_s)
Vf=np.array(Vf_s)
Vx=np.array(Vx_s)
Vy=np.array(Vy_s)

print(np.shape(theta))

Nr=robotN

#robot dimension
ht=0.3
wt=1

#wheel dimensions
ht_w=0.05
wt_w=0.1

hyp=sqrt((0.5*ht)**2+(0.5*wt)**2)
ang=arctan(ht/wt)

# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
#np.abs(Init_con[:][1]+Fin_con[:][1]+[7.5])
xl=np.abs(x_d).max()+1.5
yl=np.abs(y_d).max()+1.5
ax = plt.axes(xlim=(xl, -xl), ylim=(-yl, yl),aspect='equal')
ax.set_xlabel('x(m)')
ax.set_ylabel('y(m)')
ax.set_title("robot position")

line = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for robots
linewf = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for front wheels
linewr = [ax.plot([], [],lw=1,markersize=0.3)[0] for _ in range(Nr)] #lines to animate for rear wheels
line_ex = [ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #lines to animate for path followed we only have to plot (x_d[ri,ki]) points
line_circ=[ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #circle around vehicle
line_in=[ax.scatter([], [],s=50, c='blue', marker='o') for _ in range(Nr)]  #initial position
line_fin=[ax.scatter([], [],s=500, c='red', marker='+') for _ in range(Nr)] #final position

lines=line+linewf+linewr+line_ex+line_circ

#initialization function: plot the background of each frame
def init():
    for l in lines:
        l.set_data([], [])
    for l in line_in:
        l.set_offsets([])
    for l in line_fin:
        l.set_offsets([])
    return lines,line_in,line_fin

#pdb.set_trace()

# animation function.  This is called sequentially
def animate(i):
    #Dynamic objects

    """perform animation step"""
    # draw ith rectangle
    points = np.array([[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
    pointsw = np.array([[-0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, +0.5 * ht_w],
                        [-0.5 * wt_w, +0.5 * ht_w]])

    x_r = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)],dtype='float')
    x_t = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
    x_rw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
    x_tw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')

    #robots

    for k, ln in enumerate(line):
        #xx=[[0]*5]*Nr
        #yy=[[0]*5]*Nr
        for j in range(len(points)):
            x_r[k, j, 0] = points[j, 0] * np.cos(theta[k, i]) - points[j, 1] * np.sin(theta[k, i])
            x_r[k, j, 1] = points[j, 0] * np.sin(theta[k, i]) + points[j, 1] * np.cos(theta[k, i])
            x_t[k, j, :] = x_r[k, j, 0] + x_d[k, i], x_r[k, j, 1] + y_d[k, i]

        xx=x_t[k,:,0]
        xx=np.concatenate((xx, np.array([x_t[k,0,0]])))
        yy = x_t[k,:,1]
        yy = np.concatenate((yy, np.array([x_t[k, 0, 1]])))

        ln.set_data(xx,yy)

        #wheels

    for k, ln in enumerate(linewf):
        #xxw = [[0] * 5] * Nr
        #yyw = [[0] * 5] * Nr
        for j in range(len(pointsw)):
            x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[k, i] + deltaf[k, i]) - pointsw[j, 1] * np.sin(
                theta[k, i] + deltaf[k, i])
            x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[k, i] + deltaf[k, i]) + pointsw[j, 1] * np.cos(
                theta[k, i] + deltaf[k, i])
         
            # translation
            tx = wt/2 * cos(theta[k][i])
            ty = wt/2* sin(theta[k][i])
            x_tw[k, j, :] = x_rw[k, j, 0] + x_d[k, i] + tx, x_rw[k, j, 1] + y_d[k, i] + ty

        xxw = x_tw[k, :, 0]
        xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
        yyw = x_tw[k, :, 1]
        yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

        ln.set_data(xxw, yyw)

    for k, ln in enumerate(linewr):
            #xxw = [[0] * 5] * Nr
            #yyw = [[0] * 5] * Nr
            for j in range(len(pointsw)):
                x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[k, i] + deltar[k, i]) - pointsw[j, 1] * np.sin(
                    theta[k, i] + deltar[k, i])
                x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[k, i] + deltar[k, i]) + pointsw[j, 1] * np.cos(
                    theta[k, i] + deltar[k, i])

                # translation
                tx = wt / 2 * cos(theta[k][i])
                ty = wt / 2 * sin(theta[k][i])
                x_tw[k, j, :] = x_rw[k, j, 0] + x_d[k, i] - tx, x_rw[k, j, 1] + y_d[k, i] - ty

            xxw = x_tw[k, :, 0]
            xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
            yyw = x_tw[k, :, 1]
            yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

            ln.set_data(xxw, yyw)

    for k, ln in enumerate(line_ex):
       ln.set_data(x_d[k,:i],y_d[k,:i])

    for k, ln in enumerate(line_circ):
        ts = np.linspace(0, 2 * pi, 100)
        ln.set_data(x_d[k,i] + r0 * cos(ts),y_d[k,i] + r0 * sin(ts))

    for k, ln in enumerate(line_in):
        ln.set_offsets([x_d[k, 0],y_d[k, 0]])

    for k, ln in enumerate(line_fin):
        ln.set_offsets([x_d[k, -1], y_d[k, -1]])

    return lines,line_in,line_fin

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=N, interval=200, blit=False)

plt.show()

#plotting

plt.figure(1)

ax1=plt.subplot(1,2,1)
ax1.plot(t_s, deltaf[0,:].T, 'r--')
legend = plt.legend(['time vs front steering angle,robot1'])
plt.xlabel('time in sec')
plt.ylabel('angle in rad')


ax2=plt.subplot(1,2,2)
ax2.plot(t_s, deltar[0,:].T, 'r--')
legend = plt.legend(['time vs rear steering angle,robot1'])
plt.xlabel('time in sec')
plt.ylabel('angle in rad')


plt.figure(2)

ax1=plt.subplot(1,2,1)
ax1.plot(t_s, deltaf[1,:].T, 'r--')
legend = plt.legend(['time vs front steering angle,robot2'])
plt.xlabel('time in sec')
plt.ylabel('angle in rad')


ax2=plt.subplot(1,2,2)
ax2.plot(t_s, deltar[1,:].T, 'r--')
legend = plt.legend(['time vs rear steering angle,robot2'])
plt.xlabel('time in sec')
plt.ylabel('angle in rad')

plt.figure(3)

ax1=plt.subplot(1,2,1)
ax1.plot(t_s, theta[0,:].T, 'r--')
legend = plt.legend(['time vs yaw angle,robot1'])
plt.xlabel('time in sec')
plt.ylabel('angle in rad')


ax2=plt.subplot(1,2,2)
ax2.plot(t_s, theta[1,:].T, 'r--')
legend = plt.legend(['time vs yaw angle,robot2'])
plt.xlabel('time in sec')
plt.ylabel('angle in rad')

plt.figure(4)

ax1=plt.subplot(1,2,1)
ax1.plot(t_s, Vf[0,:].T, 'r--')
legend = plt.legend(['time vs front wheel vel,robot1'])
plt.xlabel('time in sec')
plt.ylabel('vel in m/s')


ax2=plt.subplot(1,2,2)
ax2.plot(t_s, Vr[0,:].T, 'r--')
legend = plt.legend(['time vs rear wheel vel,robot1'])
plt.xlabel('time in sec')
plt.ylabel('vel in m/s')

plt.figure(5)

ax1=plt.subplot(1,2,1)
ax1.plot(t_s[:-1], Vx[0,:].T, 'r--')
legend = plt.legend(['time vs x vel,robot1'])
plt.xlabel('time in sec')
plt.ylabel('vel in m/s')


ax2=plt.subplot(1,2,2)
ax2.plot(t_s[:-1], Vy[0,:].T, 'r--')
legend = plt.legend(['time vs y vel,robot1'])
plt.xlabel('time in sec')
plt.ylabel('vel in m/s')


plt.show()
