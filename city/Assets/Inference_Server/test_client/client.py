import socket
import os
import struct


HOST = '127.0.0.1'  # The server's hostname or IP address
PORT = 11111        # The port used by the server

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    with open("test4.jpg", 'rb') as f:
        data = f.read()
        img_size = os.fstat(f.fileno()).st_size
        s.sendall(struct.pack("!i", img_size))
        s.sendall(data)
        f.close()
    data = s.recv(1024)

print('Received', repr(data))
