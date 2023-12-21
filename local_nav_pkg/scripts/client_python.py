from base64 import decode
from cmath import pi
import socket
import struct
from time import sleep
import errno
import sys

HOST = "127.0.0.1"  # The server's hostname or IP address
PORT = 65432  # The port used by the server
buffer_size = 1


# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT)) #Script waits here
print("Socket connected")
s.setblocking(False)

# s.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)

# bufsize = s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF) 
# print ("Buffer size [Before]:%d" %bufsize)

# s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, buffer_size)
# bufsize = s.getsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF) 
# print ("Buffer size [After]:%d" %bufsize)
while(1==1):
    # s.sendall(b"Hello, world. I am dev")
    # print("Sent data to server")
    
    try:
        while(1==1):
            # data = s.recv(4)
            # print(f"Received {data!r}")
            print("Reached try")
            decoded_data = struct.unpack('f', s.recv(4))[0]*180/pi
            print("############################## Decoded data is: ", decoded_data)
        # decoded_data = decoded_data*57.3065
    except socket.error as e:
        err = e.args[0]
        if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
            print("No data available, Hitting continue")
            sleep(7)
            continue
        else:
            # a "real" error occurred
            print("Real error occured: ", e)
            sys.exit(1)
    

