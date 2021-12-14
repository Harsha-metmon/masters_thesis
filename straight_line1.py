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

Straight line along x
"""

from rockit import *
import matplotlib.pyplot as plt
import numpy as np
from numpy import pi, cos, sin, tan,arctan
from casadi import vertcat, sumsqr
from pylab import *

def moti():

    ocp = Ocp(T=FreeTime(10.0))

    #initial and final constraints
    
    N=21
    x_i=0
    y_i=0
    theta_i=0
    
    x_f=2
    y_f=0
    theta_f=0
    
    # Bicycle model

    x     = ocp.state()
    y     = ocp.state()
    theta = ocp.state()

    deltaf = ocp.control(order=1)
    #Vf      =ocp.control()
    deltar = ocp.control(order=1)
    #Vr     =ocp.control()
    V = ocp.control()

    # Distances from the CG to the rear/front wheel centres

    Lr = 0.4
    Lf = 0.6
    vmax=0.0123
    
    animation=True
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
    ocp.subject_to(ocp.at_t0(x) == x_i)
    ocp.subject_to(ocp.at_t0(y) == y_i)
    ocp.subject_to(ocp.at_t0(theta) == theta_i)
    #ocp.subject_to(ocp.at_t0(deltar) == pi/1.99)
    #ocp.subject_to(ocp.at_t0(deltaf) == pi/1.99)
    
    # final constraints
    ocp.subject_to(ocp.at_tf(x) == x_f)
    ocp.subject_to(ocp.at_tf(y) == y_f)
    ocp.subject_to(ocp.at_tf(theta) == theta_f)
    #ocp.subject_to(ocp.at_tf(deltar) == pi/1.99)
    #ocp.subject_to(ocp.at_tf(deltaf) == pi/1.99)


    in_x=np.linspace(x_i,x_f,N)
    in_y=np.linspace(y_i,y_f,N)
    in_theta=np.linspace(theta_i,theta_f,N)
    in_V=vmax
    ocp.set_initial(x, in_x)
    ocp.set_initial(y, in_y)
    ocp.set_initial(theta, in_theta)
    ocp.set_initial(V, in_V)
    ocp.set_initial(deltar, 0)
    ocp.set_initial(deltaf, 0)
    

    ocp.subject_to(-vmax <= (V <= vmax))
   
    ocp.subject_to(-pi/2 <= (deltaf <= pi/2))
    ocp.subject_to(-pi/2 <= (deltar <= pi/2))
    ocp.subject_to(-100 <= (ocp.der(deltaf) <= 100))
    ocp.subject_to(-100 <= (ocp.der(deltar) <= 100))
    #ocp.subject_to(((deltaf*deltar) <= 0))

    # Round obstacle
    p0 = vertcat(0.7,0.01)
    r0 = 0.1

    p = vertcat(x,y)
    #ocp.subject_to(sumsqr(p-p0)>=r0**2)

    # Minimal time
    ocp.add_objective(ocp.T)
    #ocp.add_objective(2*ocp.integral(x**2+y**2))
    
    #ocp.add_objective(10 * ocp.integral(theta ** 2))

    # Pick a solution method

    options = {'ipopt': {"max_iter": 3000}}
    options["expand"] = True
    options["print_time"] = True
    ocp.solver('ipopt', options)

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
    #plot(p0[0]+r0*cos(ts),p0[1]+r0*sin(ts),'r-')

    tsi, deltar_s = sol.sample(deltar, grid='integrator',refine=50)
    tsi,x_s=sol.sample(x, grid='integrator',refine=50)
    tsi, y_s = sol.sample(y, grid='integrator', refine=50)

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

    if animation:
        from matplotlib import animation
        import matplotlib.patches as patches
        import matplotlib as mpl

        fig = plt.figure()
        fig.set_dpi(100)
        fig.set_size_inches(6, 5.5)

        ax = plt.axes(xlim=(-2, 2), ylim=(-2, 6))

        x_d = x_s
        y_d = y_s
        theta = theta_s
        deltaf = deltaf_s
        deltar = deltar_s

        # robot dimension
        height = 0.3
        width = 1

        # wheel dimension
        w_h = 0.04
        w_w = 0.1

        vertices = (0, 0)
        polygon = patches.Rectangle(vertices, width, height, color="red", alpha=0.50)
        polygon2 = patches.Rectangle(vertices, w_w, w_h, color="blue", alpha=0.50)
        polygon3 = patches.Rectangle(vertices, w_w, w_h, color="green", alpha=0.50)

        def init():
            ax.add_patch(polygon)
            ax.add_patch(polygon2)
            ax.add_patch(polygon3)
            return polygon, polygon2, polygon3

        def animate(i):
            # robot patch

            r = np.sqrt((height / 2) ** 2 + (width / 2) ** 2)
            alpha = np.arctan(height / width)
            xc = [r * cos(alpha), r * sin(alpha)]
            xc_w1 = [0, height / 2]
            xc_w2 = [width, height / 2]
            x_t = 2 * [None]
            x_1 = 2 * [None]
            x_2 = 2 * [None]
            midPoint = [[0] * 2 for j in range(len(x_d))]

            x_t[0] = xc[0] * cos(theta[i]) - xc[1] * sin(theta[i])
            x_t[1] = xc[0] * sin(theta[i]) + xc[1] * cos(theta[i])
            x_1[0] = xc_w1[0] * cos(theta[i]) - xc_w1[1] * sin(theta[i])
            x_1[1] = xc_w1[0] * sin(theta[i]) + xc_w1[1] * cos(theta[i])
            x_2[0] = xc_w2[0] * cos(theta[i]) - xc_w2[1] * sin(theta[i])
            x_2[1] = xc_w2[0] * sin(theta[i]) + xc_w2[1] * cos(theta[i])
            midPoint[i][:] = [x_d[i] - x_t[0], y_d[i] - x_t[1]]
            r = mpl.transforms.Affine2D().rotate(theta[i])
            r1 = mpl.transforms.Affine2D().rotate(deltar[i])
            r2 = mpl.transforms.Affine2D().rotate(deltaf[i])
            t = mpl.transforms.Affine2D().translate(midPoint[i][0], midPoint[i][1])
            t1 = mpl.transforms.Affine2D().translate(x_1[0], x_1[1])
            t2 = mpl.transforms.Affine2D().translate(x_2[0], x_2[1])

            tra = r + t + ax.transData
            tra1 = r1 + r + t + t1 + ax.transData
            tra2 = r2 + r + t + t2 + ax.transData
            polygon.set_transform(tra)
            polygon2.set_transform(tra1)
            polygon3.set_transform(tra2)

            coords = polygon.get_bbox().get_points()
            # print(tra.transform(coords))

            # wheel patches

            ax.add_patch(polygon)
            ax.add_patch(polygon2)
            ax.add_patch(polygon3)

            return polygon, polygon2, polygon3

        anim = animation.FuncAnimation(fig, animate,
                                       init_func=init,
                                       frames=len(x_d),
                                       interval=20,
                                       blit=True)
        writervideo = animation.FFMpegWriter(fps=60)
        # anim.save('increasingStraightLine.mp4', writer=writervideo)
        plt.show()

    return tsi, deltar_s, deltaf_s,V_s,bet_s

if __name__ == '__main__':
    moti()
