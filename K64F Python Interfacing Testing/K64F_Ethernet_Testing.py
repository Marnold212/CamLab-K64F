# Python acts as Client while each K64F acts as a Server (TCP)
# This means for each detected K64F on chosen TCP PORT the GUI creates a client thread for data transfer 
# On windows machine - Go to "Turn Windows Features On or Off" - enable telnet client 
# In CMD run 
# https://www.geeksforgeeks.org/socket-programming-python/

import socket 
import os 

IP_Address = []
Mac_Address = []
stream = os.popen('arp -a')  # Pipe output of CMD to stream variable 
output = stream.readlines()  # Read the output from CMD to output 
# print(output)
IP4_List = [x.split() for x in output if "dynamic" in x.split()]
for x in IP4_List:
    IP_Address.append(x[0])
    Mac_Address.append(x[1])
#     print(x)

# print(IP_Address)

PORT = 5050  # Arbitrary? - check PORT lists 
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"


Num_Mbed_TCP_Devices = 0
Valid_IP = []
Valid_Mac_Address = []
for x in range(len(IP_Address)):
    ADDR = (IP_Address[x], PORT) 
    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # New Socket: 1st arg = Family? ; 2nd arg = Streaming data through socket? 
        client.connect(ADDR)
        client.close()
        Valid_IP.append(IP_Address[x])
        Valid_Mac_Address.append(Mac_Address[x])
        Num_Mbed_TCP_Devices += 1
    except:
        continue
        
print(f"Valid IP Addresses on PORT {PORT} : ", Valid_IP)

# Test By sending 2 messages 
for x in range(Num_Mbed_TCP_Devices):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)    # New Socket: 1st arg = Family? ; 2nd arg = Streaming data through socket? 
    ADDR = (Valid_IP[x], PORT)
    client.connect(ADDR)
    with client as s:
        s.send("Hello".encode(FORMAT))
        print(s.recv(len("Hello")))
        s.send("World".encode(FORMAT))
        print(s.recv(len("World"))) 

# client.connect(ADDR)

# with client as s:
#     s.send("Hello".encode(FORMAT))
#     s.send("World".encode(FORMAT))
# while(1):
#     pass