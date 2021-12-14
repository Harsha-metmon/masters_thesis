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
Motion planning with waypoints
==============================

Simple motion close to waypoints
"""

from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan,arctan
from casadi import vertcat, sumsqr, symvar
from pylab import *

ocp = Ocp(T=FreeTime(10.0))


def moti():

    # Bicycle model

    x     = ocp.state()
    y     = ocp.state()
    theta = ocp.state()

    # forward and rear wheel turning angles plus the heading velocity of the centre of gravity

    deltaf = ocp.control()
    # Vf      =ocp.control()
    deltar = ocp.control()
    # Vr     =ocp.control()
    V= ocp.control()


    # Distances from the CG to the rear/front wheel centres

    Lr = 0.5
    Lf = 0.5
    vmax=0.033

    # side slip angle (angle between the heading angle and the frame of the bicycle )

    bet = arctan(((Lf * tan(deltar)) + (Lr * tan(deltaf))) / (Lf + Lr))

    # V=(Vf*cos(deltaf(i))+Vr*cos(deltar(i)))/(2*cos(beta(i)))

    # simply the rhs of the state evolution equation for theta
    cc = (V * cos(bet) * (tan(deltaf) + tan(deltar)) / (Lf + Lr))

    # Kinematics equations (with the robot axis at the cg of the bicycle )

    ocp.set_der(x, V * cos(theta + bet))
    ocp.set_der(y, V * sin(theta + bet))
    ocp.set_der(theta, cc)


    # Initial constraints
    ocp.subject_to(ocp.at_t0(x)==0)
    ocp.subject_to(ocp.at_t0(y)==0)
    ocp.subject_to(ocp.at_t0(theta)==0)


    ocp.set_initial(x,ocp.t)
    ocp.set_initial(y,0)
    ocp.set_initial(theta,0)
    ocp.set_initial(V,vmax)


    ocp.subject_to(-vmax <= (V<=vmax))
    ocp.subject_to( -pi/2 <= (deltaf<= pi/2))
    ocp.subject_to( -pi/2 <= (deltar<= pi/2))


    # Define a placeholder for concrete waypoints to be defined on edges of the control grid
    waypoints = ocp.parameter(2, grid='control')

    # Minimal time
    ocp.add_objective(ocp.T)
    ocp.add_objective(10*ocp.integral((x-waypoints[0])**2+(y-waypoints[1])**2, grid='control')/ocp.T)

    # Pick a solution method
    ocp.solver('ipopt')

    # Make it concrete for this ocp
    ocp.method(MultipleShooting(N=20,M=4,intg='rk'))


    N=20

    # Give concerte numerical values for waypoints
    waypoints_num = np.array([(i,0 ) for i in range(N)]).T
    ocp.set_value(waypoints, waypoints_num)

    print(waypoints_num)


    # solve
    sol = ocp.solve()


    figure()

    ts, xs = sol.sample(x, grid='control')
    ts, ys = sol.sample(y, grid='control')



    plot(xs, ys,'bo')
    plot(waypoints_num[0,:],waypoints_num[1,:],'kx')

    ts, xs = sol.sample(x, grid='integrator')
    ts, ys = sol.sample(y, grid='integrator')
   
    plot(xs, ys, 'b.')


    ts, xs = sol.sample(x, grid='integrator',refine=10)
    ts, ys = sol.sample(y, grid='integrator',refine=10)

    tsi, deltar_s = sol.sample(deltar, grid='control')
    
    
    
    tsi, deltaf_s = sol.sample(deltaf, grid='control')
    tsi, V_s = sol.sample(V, grid='control')
    tsi, theta_s = sol.sample(theta, grid='control')
    ts, ys = sol.sample(y, grid='integrator',refine=10)

    plot(tsi,theta_s,'r')
    #plot(xs, ys, '-')

    axis('equal')
    show(block=True)

    print(tsi,deltar_s,deltaf_s)
    return tsi,deltar_s,deltaf_s,V_s
    

    

if __name__ == '__main__':
    moti()