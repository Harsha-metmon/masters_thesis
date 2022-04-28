import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
X = pd.read_csv('a_file.txt',sep="\t",skiprows=list(range(0,1)))

X.columns = ['time','vxff','vyff','omegaff','vxfb','vyfb','omegafb','vxc','vyc','omegac','t_eag','vx_e','vy_e','omega_e']

#t1=t.tolist()
t=X['time'].values
vxff=X['vxff'].values
vyff=X['vyff'].values
omegaff=X['omegaff'].values
vxfb=X['vxfb'].values
vyfb=X['vyfb'].values
omegafb=X['omegafb'].values
vxc=X['vxc'].values
vyc=X['vyc'].values
omegac=X['omegac'].values
t_eag=X['t_eag'].values
vx_e=X['vx_e'].values
vy_e=X['vy_e'].values
omega_e=X['omega_e'].values


c=t_eag[0]



t_exp=[i - c for i in t_eag]


plt.figure()

plt.plot(t,vxff)
plt.xlabel('time in seconds')
plt.ylabel('velocity in m/s')

plt.plot(t_exp,vx_e)
plt.xlabel('time in seconds')
plt.ylabel('velocity in m/s')

plt.plot(t,vxc,'.')
plt.xlabel('time in seconds')
plt.ylabel('velocity in m/s')

plt.plot(t,vxfb)
legend = plt.legend(['t vs vx(feedforward)','t vs vx_actual','t vs vx_combined','t vs vxfb(feedback)'])
plt.xlabel('time in secs')
plt.ylabel('velocity in m/s')


plt.figure()

plt.plot(t,vyff)
plt.xlabel('time in seconds')
plt.ylabel('velocity in m/s')

plt.plot(t_exp,vy_e)
plt.xlabel('time in seconds')
plt.ylabel('velocity in m/s')

plt.plot(t,vyc,'.')
plt.xlabel('time in seconds')
plt.ylabel('velocity in m/s')

plt.plot(t,vyfb)
legend = plt.legend(['t vs vy(feedforward)','t vs vy_actual','t vs vy_combined','t vs vyfb(feedback)'])
plt.xlabel('time in secs')
plt.ylabel('velocity in m/s')



plt.figure()

plt.plot(t,omegaff)
plt.xlabel('time in seconds')
plt.ylabel('velocity in rad/s')

plt.plot(t_exp,omega_e)
plt.xlabel('time in seconds')
plt.ylabel('velocity in rad/s')

plt.plot(t,omegac,'.')
plt.xlabel('time in seconds')
plt.ylabel('velocity in rad/s')

plt.plot(t,omegafb)
legend = plt.legend(['t vs omega(feedforward)','t vs omega_actual','t vs omega_combined','t vs omega(feedback)'])
plt.xlabel('time in secs')
plt.ylabel('velocity in rad/s')


plt.show(block=True)


#print(t)

#print(x,y)




