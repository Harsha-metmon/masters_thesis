import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
X = pd.read_csv('eagle_data.txt',sep="\t",skiprows=[1,2,3,4])
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

x = [i - a for i in x]
y = [i - b for i in y]

print(x)



print(y)

plt.plot(x,y)
plt.show()


#print(t)

#print(x,y)




