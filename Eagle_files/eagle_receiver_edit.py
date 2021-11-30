import threading


global pose
pose = [None] * 7
def loti():

    def whatever(pose):
        k=0
        while True:


            k=k+1
            loca=[k] * 7
            global pose
            
            #print(loca)
            pose=loca
        

    whatever(pose)
    

threading.Thread(target=loti,args=(x,)).start()

if __name__ == '__main__':
    # Create a StreamHandler for debugging

    loti()

