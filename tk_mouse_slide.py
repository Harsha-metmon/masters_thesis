from tkinter import *
import time as time

# Exiting GUI mainloop, and continuing with the program
def creating():
    canvas.destroy()

root = Tk()
coord=[]
root.counter=0

# Number of waypoints

N=4

def myfunction(event):
    root.counter += 1
    x, y = event.x, event.y

    if (root.old_coords and root.counter<=N):
        x1, y1 = root.old_coords
        canvas.create_line(x, y, x1, y1)

    if root.counter<=N:
     root.old_coords = x, y
     coord.append((x, y))

    if root.counter == N:
        print('maximum number of points that can be specified has reached thank you!!')
        

    return coord


frame_a = Frame()
Slider=[None]*N
ang=[]
gui_variable=[None]*N

for i in range(N):


    gui_variable[i] = IntVar()
    Slider[i] = Scale(master=frame_a, from_=0, to=360, length=600, tickinterval=30,
              variable=gui_variable[i], orient=HORIZONTAL)
    Slider[i].set(0)
    Slider[i].grid(row=i + 1, column=1)


'''
gui_variable = IntVar()
Slider = Scale(master=frame_a, from_=5, to=20, length=600, tickinterval=5,variable=gui_variable, orient=HORIZONTAL)
Slider.set(5)
Slider.pack()

'''

frame_b = Frame()
canvas = Canvas(master=frame_b, width=200, height=200,bg='white')
canvas.bind("<Button-1>", myfunction)
canvas.pack()

# Swap the order of `frame_a` and `frame_b`
frame_b.pack()
frame_a.pack()

root.old_coords = 0

root.mainloop()

for i in range(N):

    ang.append(gui_variable[i].get())

#gvar=gui_variable.get()

print("the waypoints selected are: {}".format(coord))
print("the angles selected are: {}".format(ang))