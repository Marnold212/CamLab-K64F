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
            # device = serial.Serial(COM_Port)
            # result = device.readline()
            # print(result)
            
            serialPort = serial.Serial( port=COM_Port, baudrate=9600, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
            serialString = ""  # Used to hold data coming over UART
            Readings = 1    # Just to avoid the statement (%100 == 0)being true initially 
            current_time = time.time()
            while 1:
                if Readings % 101 == 0:
                    new_time = time.time()
                    time_interval = new_time - current_time
                    print("Last 100 readings took %f seconds giving a rate of %f readings/second" % (time_interval, (100.0/time_interval)))
                    current_time = new_time
                    Readings = 1
                # Wait until there is data waiting in the serial buffer
                if serialPort.in_waiting > 0:

                    # Read data out of the buffer until a carriage return / new line is found

                    # Note there is an inherant issue with this where, if a transmitted value has the equivalent hex value of 0a, it triggers the end of line
                    # If we are sending unsigned bytes across channel, inevitably some will have the value of '\n' character 
                    
                    Bytes_Transferred = 8*2 + 1
                    serialString = serialPort.readline()
                    if(len(serialString) > Bytes_Transferred):
                        print("Input too long - error")
                    else:
                        while(len(serialString) < Bytes_Transferred):
                            # print(serialString)
                            serialString += serialPort.readline()
                    serialString = serialString.hex()  # Decode bytes into raw hex values
                    try:
                        # if(serialString[Bytes_Transferred-1] != 10):    # 10 = 0x0a = '\n' LF character in Ascii 
                        if(serialString[-2: ] != '0a'):    # 10 = 0x0a = '\n' LF character in Ascii 
                            print("Transfer Error")
                        else:
                            Readings+=1
                            print("Reading: = %i ; ADC01 = %i ; AD02 = %i ; AD03 = %i ; ADC04 = %i ; AD05 = %i ; AD06 = %i ; AD07 = %i ; AD08 = %i" % (Readings, int(serialString[0:4], 16), int(serialString[4:8], 16), int(serialString[8:12], 16), int(serialString[12:16], 16), int(serialString[16:20], 16), int(serialString[20:24], 16), int(serialString[24:28], 16), int(serialString[28:32], 16)))     # The int('8A01', 16) function converts hex to 16 bit unsigned int - assumes leftmost bit is most significant
                            # print("ADC01 = %i ; AD02 = %i ; AD03 = %i ; ADC04 = %i ; AD05 = %i ; AD06 = %i ; AD07 = %i ; AD08 = %i" % (int(serialString[0:4], 16), int(serialString[4:8], 16), int(serialString[8:12], 16), int(serialString[12:16], 16), int(serialString[16:20], 16), int(serialString[20:24], 16), int(serialString[24:28], 16), int(serialString[28:32], 16)))     # The int('8A01', 16) function converts hex to 16 bit unsigned int - assumes leftmost bit is most significant

                    except:
                        pass

        else:
            print("The Device connected to port %s is not an mbed Serial Port" % COM_Port)
