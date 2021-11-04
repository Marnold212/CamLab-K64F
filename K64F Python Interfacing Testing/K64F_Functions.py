import serial.tools.list_ports as port_list
import serial
import time

# Scans connected USB Devices and attempts to connect to any with name beginning "mbed Serial Port" - may not be best method
# Returns an array with [0]: No. mbed devices connected over USB, [1]: Array corresponding to COM_PORT for each device 
# [2]: 

#### How will we convey COM_PORT - very important for this serial connection 

# def List_All_Mbed_USB_Devices(self):
def List_All_Mbed_USB_Devices():
    ports = list(port_list.comports())
    Num_Serial_Devices = len(ports)
    Num_Mbed_Devices = 0
    COM_PORTS = []
    connectionType = []  # Create a unique value for USB K64F devices which trigger new functions  
    # Say 11 = mbed USB, 10 = mbed ANY, 12 = mbed TCP, 14 = mbed WIFI 
    ID_USB = []
    Baud_Rate = []  # For now assume all operating at 9600 - may change later so might need to add later on 
    # IP = []  # Don't think we need this for USB Serial(Mbed) devices 
    if Num_Serial_Devices > 0:
        for i in range(Num_Serial_Devices):
            COM_Port = ports[i][0][0:16]   # port[i] is a particular device - port[i][0] is the COM Port of the device 
            if(ports[i][1][0:16] == "mbed Serial Port"):     # port[i] is a particular device - port[i][1] is the description of the device - port[i][1][0:16] are the characters containing the mbed Serial Port description
                default_baudrate = 9600 # Assume all boards use default baudrate of 9600 
                
                device = serial.Serial(port=COM_Port, baudrate=default_baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
                
                # In requesting name, we also check we have actually connected to device - and that it is meant to be used for what we are using it for 
                Unique_K64F_ID = "Not Yet Implemented" # Need to query device to obtain name 
                
                COM_PORTS.append(COM_Port)
                connectionType.append(11)
                ID_USB.append(Unique_K64F_ID)

    return(Num_Mbed_Devices, COM_PORTS, connectionType, ID_USB)

mbed_USB_info = List_All_Mbed_USB_Devices()

for i in range(4):
    print(mbed_USB_info[i])