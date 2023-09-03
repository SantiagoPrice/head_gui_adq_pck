#!/usr/bin/env python
import socket
import quaternion
#from scipy.spatial.transform import Rotation as R
import numpy as np
pcname=socket.gethostname()
UDP_IP = socket.gethostbyname(pcname)

print(f"My IP address is {UDP_IP}")
UDP_IP = '192.168.12.41'
#UDP_IP = '127.0.0.20'
UDP_PORT = 18403
#UDP_PORT = 44082
sock = socket.socket(socket.AF_INET, # Internet
                      socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))
while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    #print(data)
    rs= np.array(str(data).replace("\\n'","").split("/")[1:],dtype=float).reshape((3,3))
    q=quaternion.from_rotation_matrix(rs)
    #r = R.from_matrix(rs)
    #print(f"{r.as_euler('xyz', degrees=True)}")
    print(f"{q}")