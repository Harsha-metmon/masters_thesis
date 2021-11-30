
# multithreading +global variable to communicate data from within while loop to outside

import time
import multiprocessing
x=0

def listen():
    while 1:
        global x
        x=x+1
        time.sleep(0.2)



def main():

  while 1:
   global y
   y=x
   #print(y)
   time.sleep(0.2)
  return y

if __name__=="__main__":

    t1=multiprocessing.Process(target=listen)
    t2=multiprocessing.Process(target=main)

    t1.start()
    t2.start()

    while 1:
     print(y)
     time.sleep(0.2)

    t1.join()
    t2.join()


