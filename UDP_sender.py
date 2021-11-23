
import socket
import sys
import motion_planning
import struct
from time import time, sleep
from numpy import pi
import numpy as np



UDP_IP = "192.168.0.101"
UDP_PORT = 5000

'''
UDP_IP = "127.0.0.1"
UDP_PORT = 5000

'''




#tsi,deltar_s,deltaf_s,V_s=motion_planning_waypoints.moti()
tsi,deltar_s,deltaf_s,V_s=motion_planning.moti()
angr=[None]*len(tsi)
angf=[None]*len(tsi)
for i in range(len(tsi)):
 deltar_s[i] = round(float(((deltar_s[i]+pi/2) * 180 / pi)))
 angr[i]=int(deltar_s[i])
 deltaf_s[i] = round(float(((deltaf_s[i]+pi/2) * 180 / pi)))
 angf[i]=int(deltaf_s[i])
 V_s = [200] * len(tsi)








start=time()
for i in range(len(tsi) - 1):
 inter = tsi[i + 1] - tsi[i]
 values = (V_s[i], angf[i], V_s[i], angf[i], V_s[i], angr[i], V_s[i], angr[i])
 print(values)
 #values = (10,90,10,90,10,90,10,90)
 #MESSAGE = struct.Struct('B B B B B B B B').pack(*values)


 
 #while True:
 #values = (0,90,0,90,0,90,0,90)
 MESSAGE = struct.Struct('B B B B B B B B').pack(*values)
 print("UDP target IP: %s" % UDP_IP)
 print("UDP target port: %s" % UDP_PORT)
 #print("message: %s" % values)
 sock = socket.socket(socket.AF_INET,  # Internet
 socket.SOCK_DGRAM)  # UDP
 sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))
 sleep(inter - time() % inter)  # run every 1 second... you can change that
 
end=time()
print(end-start)