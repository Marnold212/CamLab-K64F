import serial.tools.list_ports as port_list
import serial

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
            # device = serial.Serial(COM_Port)
            # result = device.readline()
            # print(result)
            
            serialPort = serial.Serial( port=COM_Port, baudrate=9600, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
            serialString = ""  # Used to hold data coming over UART
    
            while 1:
                # Wait until there is data waiting in the serial buffer
                if serialPort.in_waiting > 0:

                    # Read data out of the buffer until a carraige return / new line is found
                    
                    # serialString = serialPort.read_until()    # Read until expected sequence found - by default '\n'
                    serialString = serialPort.readline()

                    # Print the contents of the serial data
                    try:
                        # print(serialString.decode("Ascii"))
                        print(serialString)
                        # print(serialString.decode("uint8") + "\r\n")   #uint8array
                        # break
                    except:
                        pass
            

        else:
            print("The Device connected to port %s is not an mbed Serial Port" % COM_Port)
