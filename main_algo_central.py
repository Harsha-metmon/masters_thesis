################ function, gui_rock###################################

# main inputs are a bunch of collission pairs of robots(get their start and end points and do ocp on each pair in loop).
#### inputs, a list of start and end points of all intersecting/collision pairs.

###############!!!!!!!!!!!!!!!!!!!!######################
#if a robot is reapeated in multiple pairs, then after collision check/ocp with constraints is called only once per robot-->  ex:[(r1,r2),(r2,r3)] altering the path of r2 in the first ocp
# and then in a small loop, check for collision avoidance of (r2,r3).If (r2,r3) fails, then solve for r2,r3 and then check for the other. If it fails also, then we could solve a 3 robot problem. Or solve a 3
#robot problem to begin with.

################# Alternatively, a simple algorithm==> Go through each pair, if an index/robot is repeated, add the robot to a list. And solve a sub problem for that pair.

from main_algo_0_GUI import GUI
from main_algo_1 import iter_sect
from main_algo_3_ocp import gui_rock
from main_algo_2_col_check import col_check_prelim,col_check_hyper
from main_algo_4_socp import gui_rock_sub
from main_algo_animation import anim_ate
from numpy import sqrt
import numpy as np

###Forget about all the complex shit, solve the simplest problem possible

#####input,an array from GUI rock

points=GUI()
########################################################################
#################### Check for intersection and collect a list of robot pair indices ############################

int_pairs,int_ind=iter_sect(points)

print(int_ind,'indcsss')

#######################################################################
## solve fixed horizon for all the robots induvidually

vmax=0.0185

###determine horizon

d=len(points)*[None]
deg=len(points)*[None]

Init_con=[i[0] for i in points]
Fin_con=[i[1] for i in points]

print(Init_con,'init')
for i in range(len(points)):
    d[i] = sqrt((Fin_con[i][0] - Init_con[i][0]) ** 2 + (Fin_con[i][1] - Init_con[i][1]) ** 2)
    print(i, d[i], 'wtffffffffffffffffffffffffffffffffffff')
    deg[i] = abs((Fin_con[i][2] - Init_con[i][2]))

d1 = max(d)

deg1 = max(deg)

#N = 21 + round((80 / 22) * float(d1))

N=21

factor1 = 0
factor2 = 0

omg_max = vmax

# Shoot up the ball

a = ((d1 / vmax) + (deg1 / omg_max)) + (factor1 * d1) + (factor2 * deg1)

a = 1.1*a

#print(a, 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')

#M = int(round(a / (0.5 * N)))
M=1
###########################################
### call ocp and store all waypoint pairs for collission check

waypoints=[]
deltf_tot=[]
deltr_tot=[]

for i in range(len(points)):
    p1=points[i][0]
    p2=points[i][1]
    wps,df,dr=gui_rock(p1,p2,a,N,M)
    wps=wps.tolist()

    waypoints.append(wps)
    deltf_tot.append(df)
    deltr_tot.append(dr)
print(np.shape(deltr_tot),'realllllllllllllwp')

anim_ate(waypoints,deltf_tot,deltr_tot,'before collision')

#########################################################################
#### Develop a time based collision system

col_ind=[]

for k in range(len(int_ind)):
    wp1=waypoints[int_ind[k][0]]
    wp2=waypoints[int_ind[k][1]]

    for i in range(len(wp1)):
        p1=wp1[i][:]
        p2=wp2[i][:]

        #print(p1,p2,'points')

        if col_check_prelim(p1, p2):
            print('collision may happen')

            if not col_check_hyper(p1, p2):
                print('collision happens between robots',int_ind[k])
                col_ind.append(int_ind[k])
                break

print(col_ind,'colind')

################################################################################
## simply solve a sub problem for the robots in the col_ind, update waypoints for those robots only and start agian by checking for intersections/checking (11:38)

####collect all the robots form the col_ind

flat_col_ind = [x for sublist in col_ind for x in sublist]

print(flat_col_ind,'flattttttt')

# using list comprehension
# to remove duplicated
# from list
col_rob = []
[col_rob.append(x) for x in flat_col_ind if x not in col_rob]

print(col_rob,'colrobbbbbbbbbbb')

############# prepare a list of start and end points for those indices only ################

points_sub=[points[i] for i in col_rob]
print(points_sub,'psub')

######## prepare sub ocp that accepts init,finits(which we alreday have, also supply the horizon lenghth)

Init_sub =[i[0] for i in points_sub]
Finit_sub=[i[1] for i in points_sub]

#### wp_sub is a list (N,3) sized waypints

wp_sub,deltf_sub,deltr_sub=gui_rock_sub(Init_sub,Finit_sub,a,N,M)

print(np.shape(deltf_sub),'wpsubbbbbbbbbbbbbbbbbbb')

#print(np.shape(wp_sub))

anim_ate(wp_sub,deltf_sub,deltr_sub,'sub problem after collision avoidance')

##### replace wp_sub according to their indices in waypoints.

for (ind, rep) in zip(col_rob, wp_sub):
    waypoints[ind] = rep
for (ind, rep) in zip(col_rob, deltf_sub):
    deltf_tot[ind] = rep
for (ind, rep) in zip(col_rob, deltr_sub):
    deltr_tot[ind] = rep

############# run collision check again/or connect the loop/build animation

anim_ate(waypoints,deltf_tot,deltr_tot,'overall motion after collision avoidance')


############### Animations required// all the motion,before colision avoidance/all the motion after colision avoidance./prepare a function that accepts waypoints,deltaf,deltar as inputs
# and outputs the animation when called. Before that, prepare the appropriate vector








