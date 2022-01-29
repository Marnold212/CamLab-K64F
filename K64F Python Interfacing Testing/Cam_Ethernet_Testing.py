# Python acts as Client while each K64F acts as a Server (TCP)
# This means for each detected K64F on chosen TCP PORT the GUI creates a client thread for data transfer 
# On windows machine - Go to "Turn Windows Features On or Off" - enable telnet client 
# In CMD run 
# https://www.geeksforgeeks.org/socket-programming-python/

import socket 
import os 

IP_Address = "131.111.245.115"  # Manually Determined from Checking output on K64F 

# print(IP_Address)

PORT = 5050  # Arbitrary? - check PORT lists 
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

socket.setdefaulttimeout(1)

ADDR = (IP_Address, PORT) 
try:
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # New Socket: 1st arg = Family? ; 2nd arg = Streaming data through socket? 
    client.connect(ADDR)
    with client as s:
        for x in range(100):
            m1 = "HelloThere" # Message must be less than 32 bytes long 
            s.send(m1.encode(FORMAT))
            print(s.recv(len(m1))) # Wait for echo from K64F Ethernet 
            print(len(m1))
except:
    print("Error with connection")



