import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
X = pd.read_csv('eagle_data.txt',sep="\t",skiprows=[1])
X.columns = ['time', 'id', 'x','y','z','R','P','Y','Eagle']

E=X['Eagle']
t=X['time'].values
t1=t.tolist()
id=X['id'].values
x=X['x'].values
y=X['y'].values
z=X['z'].values
R=X['R'].values
P=X['P'].values
Y=X['Y'].values

#print(type(x[0]))
#print(t[1]-t[0])

a=x[0]
b=y[0]

c=t[0]
d=Y[0]

print(t)

t=[i - c for i in t]

delta_t=[None]*(len(t)-1)

for i in range(len(t)-1):
   delta_t[i]=t[i+1]-t[i]

x = [i - a for i in x]
y = [i - b for i in y]
Y=[i - d for i in Y]

print(t)
print(delta_t)


x_tot=x[-1]-x[0]
y_tot=y[-1]-y[0]
t_tot=t[-1]-t[0]



distance=np.sqrt(x_tot**2+y_tot**2)

print('dist',distance)

vel=distance/t_tot
print(x_tot,y_tot,t_tot,vel)



plt.figure()

kk=plt.plot(x,y)
legend = plt.legend(['x vs y'])
plt.xlabel('x in m')
plt.ylabel('y in m')

plt.figure()

plt.plot(t,Y)
legend = plt.legend(['time vs yaw angle'])
plt.xlabel('time in secs')
plt.ylabel('angle in radians')


plt.show()


#print(t)

#print(x,y)




