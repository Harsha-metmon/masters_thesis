import time
starttime = time.time()
while True:
    print ("tick")
    
    et=(time.time() - starttime)
    et_min=(et % 2.0)
    
    
    a=(2.0 - et_min)
    
    time.sleep(a)
    
    print(a)
    print(et)
    print(et_min)
