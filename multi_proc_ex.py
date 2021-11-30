
#share data between 2 processes using array and value
#unlike threads only the global variable's copy which the pocess gets is udated and not the glabal variable itself.
#shared memory is the solution


import time
import multiprocessing
x=0

def listen(x):
    while 1:

        x[0]=x[0]+1
        time.sleep(0.2)



def read(x):

  while 1:


   #print(y)
   time.sleep(0.2)
  return x


def main():
    x = multiprocessing.Array('i',1)

    t1=multiprocessing.Process(target=listen,args=(x,))
    t2=multiprocessing.Process(target=read,args=(x,))

    t1.start()
    t2.start()

    while 1:
     print(x[:])
     time.sleep(0.2)

    t1.join()
    t2.join()


if __name__=="__main__":
    main()
