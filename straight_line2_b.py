#
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
#
#

"""
Motion planning
===============
Simple motion planning with circular obstacle
"""

from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan,arctan
from casadi import vertcat, sumsqr
from pylab import *

def moti():

    ocp = Ocp(T=FreeTime(10.0))

    # Bicycle model

    x     = ocp.state()
    y     = ocp.state()
    theta = ocp.state()

    deltaf = ocp.control()
    #Vf      =ocp.control()
    deltar = ocp.control()
    #Vr     =ocp.control()
    V = ocp.control()

    # Distances from the CG to the rear/front wheel centres

    Lr = 0.4
    Lf = 0.6
    vmax=0.0123
    # side slip angle (angle between the heading angle and the frame of the bicycle )

    bet = arctan(((Lf * tan(deltar)) + (Lr * tan(deltaf))) / (Lf + Lr))

    #V=((Vf*cos(deltaf)+Vr*cos(deltar))/(2*cos(bet)))

    # simply the rhs of the state evolution equation for theta
    cc = ((V * cos(bet) * (tan(deltaf) -tan(deltar))) / (Lf + Lr))

    # Kinematics equations (with the robot axis at the cg of the bicycle )

    ocp.set_der(x, V * cos(theta + bet))
    ocp.set_der(y, V * sin(theta + bet))
    ocp.set_der(theta, cc)

    # Initial constraints
    ocp.subject_to(ocp.at_t0(x) == 0)
    ocp.subject_to(ocp.at_t0(y) == 0)
    ocp.subject_to(ocp.at_t0(theta) == 0)
    #ocp.subject_to(ocp.at_t0(deltar) == pi/1.99)
    #ocp.subject_to(ocp.at_t0(deltaf) == pi/1.99)
    
    # final constraints
    ocp.subject_to(ocp.at_tf(x) == 0)
    ocp.subject_to(ocp.at_tf(y) == 4)
    ocp.subject_to(ocp.at_tf(theta) == 0)
    #ocp.subject_to(ocp.at_tf(deltar) == pi/1.99)
    #ocp.subject_to(ocp.at_tf(deltaf) == pi/1.99)

    ocp.set_initial(x, 0)
    ocp.set_initial(y, ocp.t)
    ocp.set_initial(theta, 0)
    ocp.set_initial(V, vmax)
    ocp.set_initial(deltar, pi/1.99)
    ocp.set_initial(deltaf, pi/1.99)
    

    ocp.subject_to(-vmax <= (V <= vmax))
   
    ocp.subject_to(-pi/2 <= (deltaf <= pi/2))
    ocp.subject_to(-pi/2 <= (deltar <= pi/2))
    #ocp.subject_to(((deltaf*deltar) <= 0))

    # Round obstacle
    p0 = vertcat(0.7,0.01)
    r0 = 0.1

    p = vertcat(x,y)
    #ocp.subject_to(sumsqr(p-p0)>=r0**2)

    # Minimal time
    ocp.add_objective(ocp.T)
    ocp.add_objective(10*ocp.integral(x**2))
    #ocp.add_objective(10 * ocp.integral(theta ** 2))

    # Pick a solution method
    ocp.solver('ipopt')

    # Make it concrete for this ocp
    ocp.method(MultipleShooting(N=20,M=1,intg='rk'))

    # solve
    sol = ocp.solve()


    figure()

    ts, xs = sol.sample(x, grid='control')
    ts, ys = sol.sample(y, grid='control')
    ts, thetas = sol.sample(theta, grid='control')

    plot(xs, ys,'bo')

    ts, xs = sol.sample(x, grid='integrator')
    ts, ys = sol.sample(y, grid='integrator')

    plot(xs, ys, 'b.')
    legend = plt.legend(['x vs y'])
    plt.xlabel('x in m')
    plt.ylabel('y in m')

    ts, xs = sol.sample(x, grid='integrator',refine=10)
    ts, ys = sol.sample(y, grid='integrator',refine=10)

    plot(xs, ys, '-')

    ts = np.linspace(0,2*pi,1000)
    plot(p0[0]+r0*cos(ts),p0[1]+r0*sin(ts),'r-')

    tsi, deltar_s = sol.sample(deltar, grid='integrator',refine=50)

    tsi, deltaf_s = sol.sample(deltaf, grid='integrator',refine=50)
    tsi, V_s = sol.sample(V,grid='integrator',refine=50)
    
    tsi, theta_s = sol.sample(theta,grid='integrator',refine=50)
    tsi, bet_s = sol.sample(bet,grid='integrator',refine=50)
    axis('equal')
    show(block=True)

    print(tsi,deltaf_s,deltar_s)
    #print(len(ts),ts)
    #print(len(tsi),tsi)
    figure()
    plot(tsi,deltar_s,'r--')
    plot(tsi,deltaf_s)
    legend = plt.legend(['time vs rear steering angle','time vs front steering angle'])
    plt.xlabel('time in sec')
    plt.ylabel('angle in rad')
    
    figure()

   
    plot(tsi,theta_s)
    
    legend = plt.legend(['time vs yaw angle'])
    plt.xlabel('time in sec')
    plt.ylabel('angle in rad')
    plt.show()
    return tsi, deltar_s, deltaf_s,V_s,bet_s

if __name__ == '__main__':
    moti()
