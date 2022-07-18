
import matplotlib.pyplot as plt
import numpy as np
from numpy import sqrt,sin,cos,arctan
from matplotlib import animation


def anim_ate(waypoints,deltaf_s,deltar_s,title):

    # Animation

    x_s = [[k[0] for k in l] for l in waypoints]
    y_s = [[k[1] for k in l] for l in waypoints]
    theta_s = [[k[2] for k in l] for l in waypoints]


    x_d_all = np.array(x_s)

    print(np.shape(x_d_all), 'size')
    lenn = len(x_d_all[0, :])
    frame_red = 1

    x_d = x_d_all[:, 0:-1:round(frame_red)]

    y_d_all = np.array(y_s)

    y_d = y_d_all[:, 0:-1:round(frame_red)]

    theta_all = np.array(theta_s)

    theta = theta_all[:, 0:-1:round(frame_red)]

    deltaf_all = np.array(deltaf_s)

    deltaf = deltaf_all[:, 0:-1:round(frame_red)]

    deltar_all = np.array(deltar_s)

    deltar = deltar_all[:, 0:-1:round(frame_red)]

    Nr = len(x_s)

    # robot dimension
    ht = 0.25
    wt = 0.9

    # wheel dimensions
    ht_w = 0.05
    wt_w = 0.1

    hyp = sqrt((0.5 * ht) ** 2 + (0.5 * wt) ** 2)
    ang = arctan(ht / wt)

    # First set up the figure, the axis, and the plot element we want to animate
    fig = plt.figure()
    # np.abs(Init_con[:][1]+Fin_con[:][1]+[7.5])
    xl = np.abs(x_d).max() + 1.5
    yl = np.abs(y_d).max() + 1.5
    ax = plt.axes(xlim=(xl, -xl), ylim=(-yl, yl), aspect='equal')
    ax.set_xlabel('x(m)')
    ax.set_ylabel('y(m)')
    ax.set_title(title)

    line = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for robots

    linewf = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for front wheels
    linewr = [ax.plot([], [], lw=1, markersize=0.3)[0] for _ in range(Nr)]  # lines to animate for rear wheels
    line_ex = [ax.plot([], [], lw=1, markersize=0.1)[0] for _ in
               range(Nr)]  # lines to animate for path followed we only have to plot (x_d[ri,ki]) points
    # line_circ=[ax.plot([], [],lw=1,markersize=0.1)[0] for _ in range(Nr)] #circle around vehicle
    line_in = [ax.scatter([], [], s=50, c='blue', marker='o') for _ in range(Nr)]  # initial position
    line_fin = [ax.scatter([], [], s=500, c='red', marker='+') for _ in range(Nr)]  # final position

    lines = line + linewf + linewr + line_ex


    # initialization function: plot the background of each frame
    def init():
        for l in lines:
            l.set_data([], [])

        for l in line_in:
            l.set_offsets([])
        for l in line_fin:
            l.set_offsets([])
        return lines, line_in, line_fin


    # pdb.set_trace()

    # animation function.  This is called sequentially
    def animate(i):
        # Dynamic objects

        """perform animation step"""
        # draw ith rectangle
        points = np.array(
            [[-0.5 * wt, -0.5 * ht], [+0.5 * wt, -0.5 * ht], [+0.5 * wt, +0.5 * ht], [-0.5 * wt, +0.5 * ht]])
        pointsw = np.array([[-0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, -0.5 * ht_w], [+0.5 * wt_w, +0.5 * ht_w],
                            [-0.5 * wt_w, +0.5 * ht_w]])

        x_r = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_t = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_rw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')
        x_tw = np.array([[[0 for m in range(2)] for n in range(4)] for o in range(Nr)], dtype='float')

        # robots

        for k, ln in enumerate(line):
            # xx=[[0]*5]*Nr
            # yy=[[0]*5]*Nr
            for j in range(len(points)):
                x_r[k, j, 0] = points[j, 0] * np.cos(theta[k, i]) - points[j, 1] * np.sin(theta[k, i])
                x_r[k, j, 1] = points[j, 0] * np.sin(theta[k, i]) + points[j, 1] * np.cos(theta[k, i])
                x_t[k, j, :] = x_r[k, j, 0] + x_d[k, i], x_r[k, j, 1] + y_d[k, i]

            xx = x_t[k, :, 0]
            xx = np.concatenate((xx, np.array([x_t[k, 0, 0]])))
            yy = x_t[k, :, 1]
            yy = np.concatenate((yy, np.array([x_t[k, 0, 1]])))

            ln.set_data(xx, yy)

            # wheels

        for k, ln in enumerate(linewf):
            # xxw = [[0] * 5] * Nr
            # yyw = [[0] * 5] * Nr
            for j in range(len(pointsw)):
                x_rw[k, j, 0] = pointsw[j, 0] * np.cos(theta[k, i] + deltaf[k, i]) - pointsw[j, 1] * np.sin(
                    theta[k, i] + deltaf[k, i])
                x_rw[k, j, 1] = pointsw[j, 0] * np.sin(theta[k, i] + deltaf[k, i]) + pointsw[j, 1] * np.cos(
                    theta[k, i] + deltaf[k, i])

                # translation
                tx = wt / 2 * cos(theta[k][i])
                ty = wt / 2 * sin(theta[k][i])
                x_tw[k, j, :] = x_rw[k, j, 0] + x_d[k, i] + tx, x_rw[k, j, 1] + y_d[k, i] + ty

            xxw = x_tw[k, :, 0]
            xxw = np.concatenate((xxw, np.array([x_tw[k, 0, 0]])))
            yyw = x_tw[k, :, 1]
            yyw = np.concatenate((yyw, np.array([x_tw[k, 0, 1]])))

            ln.set_data(xxw, yyw)

        for k, ln in enumerate(linewr):
            # xxw = [[0] * 5] * Nr
            # yyw = [[0] * 5] * Nr
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
            ln.set_data(x_d[k, :i], y_d[k, :i])

        for k, ln in enumerate(line_in):
            ln.set_offsets([x_d[k, 0], y_d[k, 0]])

        for k, ln in enumerate(line_fin):
            ln.set_offsets([x_d[k, -1], y_d[k, -1]])

        return lines, line_in, line_fin


    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                   frames=len(x_d[0, :]), interval=200, blit=False)

    plt.show()
