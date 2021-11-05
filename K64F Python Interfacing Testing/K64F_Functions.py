from ctypes.wintypes import HANDLE
import serial.tools.list_ports as port_list
import serial
import time

# Scans connected USB Devices and attempts to connect to any with name beginning "mbed Serial Port" - may not be best method
# Returns an array with [0]: No. mbed devices connected over USB, [1]: Array corresponding to COM_PORT for each device 
# [2]: 

#### How will we convey COM_PORT - very important for this serial connection
#### Don't forget to check if 115200 has any effect on performance or if overhead dominates for such short transfers  

# def List_All_Mbed_USB_Devices(self):
def List_All_Mbed_USB_Devices():
    ports = list(port_list.comports())
    Num_Serial_Devices = len(ports)
    Num_Mbed_Devices = 0
    COM_PORTS = []
    connectionType = []  # Create a unique value for USB K64F devices which trigger new functions  
    # Say 11 = mbed USB, 10 = mbed ANY, 12 = mbed TCP, 14 = mbed WIFI 
    VID_PID = []   # USB VID:PID are the Vendor/Product ID respectively - smae for each K64F Board? - You can determine HIC ID from last 8 digits of Serial Number? 
    # Note that the 0240 at the start of Serial Number refers to the K64F Family 
    ID_USB = []   # ID_USB will be the USB serial number - should be unique
    Baud_Rate = []  # For now assume all operating at 9600 - may change later so might need to add later on 
    # IP = []  # Don't think we need this for USB Serial(Mbed) devices 
    if Num_Serial_Devices > 0:
        for i in range(Num_Serial_Devices):
            COM_Port = ports[i].usb_description()   # ports[i].device outputs COM_PORT    (Note port[i][0][0:16]  is a particular device - port[i][0] is the COM Port of the device)
            if(ports[i][1].startswith("mbed Serial Port")):     # port[i] is a particular device - port[i][1] is the description of the device - port[i][1][0:16] are the characters containing the mbed Serial Port description
                default_baudrate = 9600 # Assume all boards use default baudrate of 9600 
                try:
                    Serial_device = serial.Serial(port=COM_Port, baudrate=default_baudrate, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
                except:
                    raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                # How can we/Do we need to check we have actually connected to device - and that it is meant to be used for what we are using it for 
                if(not Serial_device.readable()):
                    raise Exception ("Issues connecting with mbed Device on %s", COM_Port) # Need to implement proper error handling 
                COM_PORTS.append(COM_Port)
                USB_INFO = ports[i].usb_info().split('=') # USB-PID should be Unique 
                USB_VIDPID = USB_INFO[1].split(' ')[0]
                VID_PID.append(USB_VIDPID)
                USB_Serial_Number = USB_INFO[2].split(' ')[0]
                ID_USB.append(USB_Serial_Number)
                connectionType.append(11)     # Added 10 onto definitions used by LJM library to avoid mixing up - however can change if confusing 
                Serial_device.close()    # Close COM Port communication once info obtained 

    return(Num_Mbed_Devices, COM_PORTS, connectionType, ID_USB, VID_PID)

mbed_USB_info = List_All_Mbed_USB_Devices()

for i in range(5):
    print(mbed_USB_info[i])

def Read_K64F_Hex_Register(serialPort, Address, Expected_Bytes = 4):   # Only use this for reading a single register - size should only be either 1, 2, 3 or 4 for 8-bit up to 32-bit long registers 
    if(len(Address) != 10 or not Address.startswith("0x")):
        raise Exception ("Register Address should be 4 bytes/8 hex digits long and be in format 0x00000000")

    Address = bytes.fromhex(Address[2:]) # Assumes Address is input in format "0x12345678"
    print(b'%b\n' % Address)
    # with serialPort as s:   # with as should allow for some error handling?
    #     print(b'%b\n' % Address)
    #     num1 = s.write(b'%b\n' % Address)   # num1 is number of bytes sent 

    try:
        num1 = serialPort.write(b'%b\n' % Address)   # num1 is number of bytes sent 
    except:
        raise Exception ("Error writing to mbed Serial Device")

    Expected_Bytes = Expected_Bytes + 1 # Note that this includes the EOL character which the user should ignore 
    serialString = "" # Used to hold data coming over UART
    while(1):
        if serialPort.in_waiting > 0:
            # Read data out of the buffer until a carriage return / new line is found
            # Note there is an inherant issue with this where, if a transmitted value has the equivalent hex value of 0a, it triggers the end of line
            # If we are sending unsigned bytes across channel, inevitably some will have the value of '\n' character 
            # serialString = serialPort.readline()
            
            serialString = serialPort.read(size=Expected_Bytes)
            print(serialString)
            # if(len(serialString) > Expected_Bytes):
            #     print("Input too long - error")
            # else:
            #     while(len(serialString) < Expected_Bytes):
            #         # print(serialString)
            #         serialString += serialPort.readline()

            serialString = serialString.hex()  # Decode bytes into raw hex values
            # if(serialString[Expected_Bytes-1] != 10):    # 10 = 0x0a = '\n' LF character in Ascii 
            if(serialString[-2: ] != '0a'):    # 10 = 0x0a = '\n' LF character in Ascii 
                raise Exception ("Issue with Received Data") # Need to implement proper error handling
            else:
                return serialString[:]

# Testing 

Serial_device = serial.Serial(port="COM4", baudrate=9600, bytesize=8, timeout=1, stopbits=serial.STOPBITS_ONE)
Target_Register = "0x40048024"
Received_String = Read_K64F_Hex_Register(Serial_device, Target_Register, 5)
print("No Bytes Sent by Python = %i; Requested Register = %s; Contents of Register(Hex) = %s; N0 Bytes Received By K64F = %i; No Bytes Received By Python = %i" % (5, Target_Register , Received_String[:-4], int(Received_String[-4:-2], 16), len(Received_String)/2.0))

'''
# Functionality to go into findDevices():
log.info("Scanning for additional mbed USB devices.")
mbed_USB_info = List_All_Mbed_USB_Devices()
mbedsUSB = mbed_USB_info[0]
COM_PORTS = mbed_USB_info[1]
connectiontype = mbed_USB_info[2]
ID_USB = mbed_USB_info[3]
for i in range(mbedsUSB):
    if ID_USB[i] not in ID_Existing:
            deviceInformation = {}
            deviceInformation["connect"] = False
            %%%%%%%% 
            
            deviceInformation["id"] = ID_USB[i]
            deviceInformation["connection"] = connectiontype[i]
            deviceInformation["address"] = COM_PORTS[i]    # For now store COM_PORT in address to avoid adding another entry - Eventually use line below for clarity 
            # deviceInformation["COM_PORT"]
            deviceInformation["status"] = True
            self.deviceTableModel.appendRow(deviceInformation)
            self.updateUI.emit(self.configuration)

            # Make a deep copy to avoid pointers in the YAML output.
            acquisitionTable = copy.deepcopy(self.defaultAcquisitionTable)
            newDevice = {
                "id": deviceInformation["id"],
                "connection": deviceInformation["connection"],
                "address": deviceInformation["address"],
                "acquisition": acquisitionTable
            }

            # If no previous devices are configured, add the "devices" key to the configuration.
            name = deviceInformation["name"]
            if "devices" not in self.configuration:
                self.configuration["devices"] = {name: newDevice} 
            else:
                self.configuration["devices"][name] = newDevice 

            # Update acquisition table models and add TableView to TabWidget by emitting the appropriate Signal. Any changes in the table will be reflected immediately in the underlying configuration data.
            self.acquisitionModels[name] = AcquisitionTableModel(self.configuration["devices"][name]["acquisition"])
            self.addAcquisitionTable.emit(name)
            self.updateAcquisitionTabs.emit()
            self.updateUI.emit(self.configuration)
log.info("Found " + str(devicesUSB) + " USB device(s).")

'''       