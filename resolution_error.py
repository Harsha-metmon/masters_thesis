
import matplotlib.pyplot as plt
import numpy as np

vmax=0.0123 
tick_max=200
tick_min=0
angm=np.pi/2

V=np.linspace(-vmax, vmax, 10000, endpoint=True)

deltar_s=np.linspace(-angm, angm, 10000, endpoint=True)
deltaf_s=np.linspace(-angm, angm, 10000, endpoint=True)


Vf_s=V
Vr_s=V


ticksf=[None]*len(V)
ticksr=[None]*len(V)
angr=[None]*len(V)
angf=[None]*len(V)
delr=[None]*len(V)
delf=[None]*len(V)

slope=(tick_max-tick_min)/(2*vmax)

for i in range(len(V)):
  
 
 delr[i] = round(float(((-deltar_s[i]+(np.pi/2)) * 180 / np.pi)))
 angr[i]=int(delr[i])
 delf[i] = round(float(((deltaf_s[i]+(np.pi/2)) * 180 / np.pi)))
 angf[i]=int(delf[i]) 

 ticksf[i]=int(round(float(tick_min+(slope*(Vf_s[i]+vmax)))))
 ticksr[i]=int(round(float(tick_min+(slope*(Vr_s[i]+vmax)))))

print(deltar_s)
print(deltaf_s)
print(angf)

plt.figure()

plt.plot(V,ticksf)

plt.figure()

plt.plot(deltar_s,angr)

plt.show()

