import serial.tools.list_ports as port_list
import serial
import time

ports = list(port_list.comports())
for p in ports:
    print(p)
    # print("0th index = %s "% p[0]) # Returns the Com Port e.g. 'COM4'
    # print("1st index = %s "% p[1]) # Returns the Name of Device e.g. 'mbed Serial Port (COM4)

Num_Devices = len(ports)
print("\nThere are %i COM Ports detected" % Num_Devices)

if Num_Devices == 0:
    print("No COM Ports Detected")
else:
    for i in range(Num_Devices):
        COM_Port = ports[i][0][0:16]   # port[i] is a particular device - port[i][0] is the COM Port of the device 
        if(ports[i][1][0:16] == "mbed Serial Port"):     # port[i] is a particular device - port[i][1] is the description of the device - port[i][1][0:16] are the characters containing the mbed Serial Port description
            print("Attempting to open mbed Serial Port on %s " % COM_Port)
            device = serial.Serial(COM_Port)
            result = device.readline() 
            print(result)    # Return has a "b'  " preface which measns the content is bytes  
            
                    

        else:
            print("The Device connected to port %s is not an mbed Serial Port" % COM_Port)
