# After 10 mouse clicks along the trajectory.. the code stores the 10 click locations as co-ordinates and a mapping can be applied to convert them to real world co-ordinates
#also joins the points with a straight line

from tkinter import *
import time as time

def creating():
    root.destroy()

root = Tk()
coord=[]
root.counter=0

# Number of waypoints

N=10

def myfunction(event):

     root.counter+=1
     x, y = event.x, event.y
     if root.old_coords:
        x1, y1 = root.old_coords
        canvas.create_line(x, y, x1, y1)
     root.old_coords = x, y
     coord.append((x,y))
     
     if root.counter==N:
         print('maximum number of points that can be specified has reached thank you!!')
         time.sleep(3)
         creating()
     return coord

c_w=800
c_h=800
canvas = Canvas(root, width=c_w, height=c_h)
canvas.pack()
root.old_coords = 0

root.bind("<Button-1>", myfunction)

root.mainloop()

#convert GUI widget co-ordinates to real world co-ordinates



print("the waypoints selected are: {}".format(coord))



